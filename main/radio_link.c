#include "radio_link.h"

#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "sx1262.h"

enum {
    DATA_LENGTH = 50,
    ALIVE_OFFSET = DATA_LENGTH,
    FRAME_LENGTH = DATA_LENGTH + 1,
    CYCLE_MS = 100,
    RX_WINDOW_MS = (CYCLE_MS * 110 + 99) / 100,
    LINK_EXPIRY_MS = 3 * CYCLE_MS,
    RX_SLICE_MS = 10,
    TX_TIMEOUT_MS = 35,
    SLAVE_TURNAROUND_MS = 20,
    SCAN_CHANNELS = 8,
    SCAN_CHANNEL_SECONDS = 6,
    SCAN_TOTAL_SECONDS = SCAN_CHANNELS * SCAN_CHANNEL_SECONDS,
    SCAN_SAMPLE_MS = 50,
    SCAN_SETTLE_MS = 20,
    SCAN_ACTIVE_RSSI_X2 = -190,
};

_Static_assert(SCAN_TOTAL_SECONDS < 60, "Leave time for scan cleanup and reporting");

typedef struct {
    system_mode_t mode;
    uint32_t generation;
} mode_request_t;

typedef struct {
    uint32_t generation;
    radio_link_status_t status;
} link_sample_t;

typedef struct {
    radio_link_status_t status;
    TickType_t last_rx;
    bool have_counter;
    uint8_t last_counter;
    uint32_t cycle;
    uint32_t last_good_cycle;
    TickType_t slave_window_start;
    uint32_t slave_timed_misses;
} link_state_t;

static const char *const TAG = "radio_link";
static const uint32_t gfsk_channels[SCAN_CHANNELS] = {
    868100000, 868300000, 868500000, 868700000, 868900000, 869425000, 869525000, 869625000,
};
static const uint32_t lora_channels[SCAN_CHANNELS] = {
    867100000, 867300000, 867500000, 867700000, 867900000, 868100000, 868300000, 868500000,
};
static QueueHandle_t requests;
static QueueHandle_t samples;
static uint32_t generation;
static portMUX_TYPE activity_lock = portMUX_INITIALIZER_UNLOCKED;
static unsigned activity;

static void record_activity(unsigned event)
{
    portENTER_CRITICAL(&activity_lock);
    activity |= event;
    portEXIT_CRITICAL(&activity_lock);
}

static const char *mode_name(system_mode_t mode)
{
    switch (mode) {
    case MODE_MASTER: return "MASTER";
    case MODE_SLAVE: return "SLAVE";
    case MODE_SCAN_GFSK: return "GFSK";
    case MODE_SCAN_LORA: return "LORA";
    }
    return "UNKNOWN";
}

#ifdef CONFIG_LINK_LOG_PAYLOAD
static void log_frame(system_mode_t mode, const char *direction,
                      const uint8_t frame[FRAME_LENGTH])
{
    static const char hex[] = "0123456789ABCDEF";
    char data[DATA_LENGTH * 2 + 1];
    for (size_t i = 0; i < DATA_LENGTH; ++i) {
        data[2 * i] = hex[frame[i] >> 4];
        data[2 * i + 1] = hex[frame[i] & 0x0F];
    }
    data[sizeof(data) - 1] = '\0';
    ESP_LOGI(TAG, "%s %s alive=%u data=%s", mode_name(mode), direction,
             (unsigned)frame[ALIVE_OFFSET], data);
}
#else
#define log_frame(mode, direction, frame) ((void)0)
#endif

static void publish_status(const mode_request_t *request, const link_state_t *state)
{
    const link_sample_t sample = {.generation = request->generation, .status = state->status};
    xQueueOverwrite(samples, &sample);
}

static void expire_link(const mode_request_t *request, link_state_t *state)
{
    if (state->status.connected &&
        xTaskGetTickCount() - state->last_rx >= pdMS_TO_TICKS(LINK_EXPIRY_MS)) {
        state->status.connected = false;
        ESP_LOGW(TAG, "%s LINK DOWN: no valid RX for %d ms, missed=%" PRIu32,
                 mode_name(request->mode), LINK_EXPIRY_MS, state->status.missed_packets);
        publish_status(request, state);
    }
}

static void check_slave_rx_timeout(const mode_request_t *request, link_state_t *state)
{
    const TickType_t window = pdMS_TO_TICKS(RX_WINDOW_MS);
    const TickType_t elapsed = xTaskGetTickCount() - state->slave_window_start;
    const uint32_t expired_windows = elapsed / window;
    if (!expired_windows) {
        return;
    }
    state->slave_window_start += expired_windows * window;
    state->slave_timed_misses += expired_windows;
    state->status.missed_packets += expired_windows;
    ESP_LOGW(TAG, "SLAVE RX timeout: %d ms, provisional=%" PRIu32
                  " total=%" PRIu32,
             RX_WINDOW_MS, expired_windows, state->status.missed_packets);
    publish_status(request, state);
}

static esp_err_t send_frame(const uint8_t frame[FRAME_LENGTH])
{
    const esp_err_t result = sx1262_send(frame, FRAME_LENGTH, TX_TIMEOUT_MS);
    if (result == ESP_OK) {
        record_activity(RADIO_LINK_ACTIVITY_TX);
    }
    return result;
}

static bool receive_frame(system_mode_t mode, uint8_t frame[FRAME_LENGTH],
                          sx1262_packet_metrics_t *metrics)
{
    size_t length;
    const esp_err_t result =
        sx1262_receive_continuous(frame, FRAME_LENGTH, &length, metrics, RX_SLICE_MS);
    if (result != ESP_OK) {
        if (result != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "%s RX rejected: %s", mode_name(mode), esp_err_to_name(result));
        }
        return false;
    }
    if (length != FRAME_LENGTH) {
        ESP_LOGW(TAG, "%s RX invalid length=%u", mode_name(mode), (unsigned)length);
        return false;
    }
    const unsigned expected_parity = mode == MODE_MASTER ? 1 : 0;
    if ((frame[ALIVE_OFFSET] & 1U) != expected_parity) {
        ESP_LOGW(TAG, "%s RX wrong counter parity alive=%u", mode_name(mode),
                 (unsigned)frame[ALIVE_OFFSET]);
        return false;
    }
    return true;
}

static bool accept_frame(const mode_request_t *request, link_state_t *state,
                         uint8_t counter, const sx1262_packet_metrics_t *metrics)
{
    uint32_t missed = 0;
    if (!state->have_counter) {
        state->status.missed_packets = 0;
    } else {
        const uint8_t delta = counter - state->last_counter;
        if (request->mode == MODE_MASTER) {
            missed = state->cycle - state->last_good_cycle - 1;
        } else if (delta == 0 || delta >= 128) {
            if (state->status.connected) {
                ESP_LOGW(TAG, "SLAVE RX duplicate or out of order alive=%u previous=%u",
                         (unsigned)counter, (unsigned)state->last_counter);
                return false;
            }
            ESP_LOGI(TAG, "SLAVE counter resync alive=%u previous=%u",
                     (unsigned)counter, (unsigned)state->last_counter);
        } else {
            missed = delta / 2 - 1;
            /* A late consecutive packet supersedes a provisional timeout. */
            state->status.missed_packets -= state->slave_timed_misses;
            state->status.missed_packets += missed;
            if (state->slave_timed_misses > missed) {
                ESP_LOGI(TAG, "SLAVE late RX alive=%u, cancelled %" PRIu32 " provisional miss(es)",
                         (unsigned)counter, state->slave_timed_misses - missed);
            }
        }
    }
    if (missed) {
        ESP_LOGW(TAG, "%s counter gap prev=%u rx=%u missed=%" PRIu32 " total=%" PRIu32,
                 mode_name(request->mode), (unsigned)state->last_counter,
                 (unsigned)counter, missed, state->status.missed_packets);
    }
    if (!state->status.connected) {
        ESP_LOGI(TAG, "%s LINK UP", mode_name(request->mode));
    }
    state->have_counter = true;
    state->last_counter = counter;
    state->last_good_cycle = state->cycle;
    state->last_rx = xTaskGetTickCount();
    state->slave_window_start = state->last_rx;
    state->slave_timed_misses = 0;
    state->status.connected = true;
    state->status.rssi_dbm_x2 = metrics->rssi_dbm_x2;
    record_activity(RADIO_LINK_ACTIVITY_RX);
    publish_status(request, state);
    return true;
}

static bool master_cycle(const mode_request_t *request, link_state_t *state, TickType_t started)
{
    const int64_t started_us = esp_timer_get_time();
    uint8_t tx[FRAME_LENGTH];
    const uint8_t counter = (uint8_t)(2U * state->cycle++);
    esp_fill_random(tx, DATA_LENGTH);
    tx[ALIVE_OFFSET] = counter;
    const esp_err_t result = send_frame(tx);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "MASTER TX failed alive=%u: %s", (unsigned)counter, esp_err_to_name(result));
        return false;
    }
    const esp_err_t rx_result = sx1262_start_continuous_rx();
    if (rx_result != ESP_OK) {
        ESP_LOGW(TAG, "MASTER TX done alive=%u, RX start failed: %s",
                 (unsigned)counter, esp_err_to_name(rx_result));
        return false;
    }
    const int64_t rx_ready_us = esp_timer_get_time() - started_us;
    const int64_t deadline_us = started_us + RX_WINDOW_MS * 1000LL;

    const uint8_t expected = (uint8_t)(counter + 1);
    uint8_t rx[FRAME_LENGTH];
    sx1262_packet_metrics_t metrics;
    mode_request_t pending;
    while (esp_timer_get_time() < deadline_us &&
           xQueuePeek(requests, &pending, 0) != pdPASS) {
        if (!receive_frame(MODE_MASTER, rx, &metrics)) {
            continue;
        }
        if (rx[ALIVE_OFFSET] != expected) {
            ESP_LOGW(TAG, "MASTER RX unexpected alive=%u expected=%u",
                     (unsigned)rx[ALIVE_OFFSET], (unsigned)expected);
            continue;
        }
        size_t i = 0;
        while (i < DATA_LENGTH && (uint8_t)(tx[i] ^ rx[i]) == UINT8_MAX) {
            ++i;
        }
        if (i != DATA_LENGTH) {
            ESP_LOGW(TAG, "MASTER RX inverse mismatch alive=%u byte=%u",
                     (unsigned)expected, (unsigned)i);
            continue;
        }
        const TickType_t elapsed = xTaskGetTickCount() - started;
        accept_frame(request, state, expected, &metrics);
        ESP_LOGI(TAG, "MASTER TX=%u RX=%u RSSI_x2=%d RX_ready_us=%" PRId64 " RTT_ms=%lu",
                 (unsigned)counter, (unsigned)expected, (int)metrics.rssi_dbm_x2, rx_ready_us,
                 (unsigned long)pdTICKS_TO_MS(elapsed));
        log_frame(MODE_MASTER, "TX", tx);
        log_frame(MODE_MASTER, "RX OK", rx);
        return true;
    }
    ESP_LOGW(TAG, "MASTER TX done alive=%u, no valid response alive=%u RX_ready_us=%" PRId64,
             (unsigned)counter, (unsigned)expected, rx_ready_us);
    log_frame(MODE_MASTER, "TX", tx);
    return false;
}

static void slave_receive(const mode_request_t *request, link_state_t *state)
{
    uint8_t rx[FRAME_LENGTH];
    sx1262_packet_metrics_t metrics;
    if (!receive_frame(MODE_SLAVE, rx, &metrics) ||
        !accept_frame(request, state, rx[ALIVE_OFFSET], &metrics)) {
        check_slave_rx_timeout(request, state);
        return;
    }
    const int64_t received_us = esp_timer_get_time();
    uint8_t tx[FRAME_LENGTH];
    for (size_t i = 0; i < DATA_LENGTH; ++i) {
        tx[i] = (uint8_t)~rx[i];
    }
    tx[ALIVE_OFFSET] = (uint8_t)(rx[ALIVE_OFFSET] + 1);
    /* Leave time for the MASTER to switch from TX to continuous RX. */
    vTaskDelay(pdMS_TO_TICKS(SLAVE_TURNAROUND_MS) + 1);
    const esp_err_t result = send_frame(tx);
    const int64_t reply_us = esp_timer_get_time() - received_us;
    ESP_ERROR_CHECK_WITHOUT_ABORT(sx1262_start_continuous_rx());
    log_frame(MODE_SLAVE, "RX OK", rx);
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "SLAVE RX=%u TX_done=%u RSSI_x2=%d reply_us=%" PRId64,
                 (unsigned)rx[ALIVE_OFFSET], (unsigned)tx[ALIVE_OFFSET],
                 (int)metrics.rssi_dbm_x2, reply_us);
        log_frame(MODE_SLAVE, "TX", tx);
    } else {
        ESP_LOGW(TAG, "SLAVE RX=%u RSSI_x2=%d TX failed alive=%u: %s",
                 (unsigned)rx[ALIVE_OFFSET], (int)metrics.rssi_dbm_x2,
                 (unsigned)tx[ALIVE_OFFSET], esp_err_to_name(result));
    }
}

typedef struct {
    int16_t min_x2;
    int16_t max_x2;
    int32_t sum_x2;
    unsigned samples;
    unsigned active;
    unsigned errors;
} scan_result_t;

static void measure_rssi(scan_result_t *result)
{
    int16_t rssi;
    if (sx1262_get_rssi_instant(&rssi) != ESP_OK) {
        ++result->errors;
        return;
    }
    if (!result->samples || rssi < result->min_x2) {
        result->min_x2 = rssi;
    }
    if (!result->samples || rssi > result->max_x2) {
        result->max_x2 = rssi;
    }
    result->sum_x2 += rssi;
    ++result->samples;
    result->active += rssi >= SCAN_ACTIVE_RSSI_X2;
}

static void log_scan_result(const char *label, uint32_t hz, const scan_result_t *result)
{
    if (!result->samples) {
        ESP_LOGW(TAG, "%s freq_hz=%" PRIu32 " samples=0 errors=%u", label, hz, result->errors);
        return;
    }
    ESP_LOGI(TAG, "%s freq_hz=%" PRIu32
                  " samples=%u RSSI_dBm min=%.1f mean=%.1f max=%.1f above_-95dBm=%.1f%% errors=%u",
             label, hz, result->samples, result->min_x2 / 2.0,
             result->sum_x2 / (2.0 * result->samples), result->max_x2 / 2.0,
             100.0 * result->active / result->samples, result->errors);
}

static void scan_channels(const mode_request_t *request, link_state_t *state)
{
    const bool lora = request->mode == MODE_SCAN_LORA;
    const uint32_t *frequencies = lora ? lora_channels : gfsk_channels;
    const char *label = mode_name(request->mode);
    sx1262_config_t config = lora ? sx1262_default_config(frequencies[0]) :
                                  sx1262_default_gfsk_config(frequencies[0]);
    scan_result_t results[SCAN_CHANNELS] = {0};
    const int64_t started = esp_timer_get_time();
    const int64_t end = started + SCAN_TOTAL_SECONDS * 1000000LL;
    const TickType_t sample_delay = pdMS_TO_TICKS(SCAN_SAMPLE_MS);
    state->status.scan_state = SCAN_RUNNING;
    state->status.scan_seconds_left = SCAN_TOTAL_SECONDS;
    publish_status(request, state);
    ESP_LOGI(TAG, "%s scan started: %d channels, %d s, RX BW %s kHz, passive RSSI",
             label, SCAN_CHANNELS, SCAN_TOTAL_SECONDS, lora ? "125" : "187.2");

    bool complete = true;
    mode_request_t pending;
    for (unsigned i = 0; i < SCAN_CHANNELS; ++i) {
        if (xQueuePeek(requests, &pending, 0) == pdPASS) {
            break;
        }
        if (esp_timer_get_time() >= end) {
            complete = false;
            break;
        }
        const int64_t channel_end = started + (i + 1) * SCAN_CHANNEL_SECONDS * 1000000LL;
        config.frequency_hz = frequencies[i];
        esp_err_t result = sx1262_set_config(&config);
        if (result == ESP_OK) {
            result = sx1262_start_continuous_rx();
        }
        if (result == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(SCAN_SETTLE_MS) + 1);
        } else {
            results[i].errors = 1;
            complete = false;
            ESP_LOGW(TAG, "%s channel setup freq_hz=%" PRIu32 ": %s",
                     label, frequencies[i], esp_err_to_name(result));
        }
        while (esp_timer_get_time() < channel_end) {
            if (xQueuePeek(requests, &pending, 0) == pdPASS) {
                break;
            }
            if (result == ESP_OK) {
                measure_rssi(&results[i]);
            }
            const int64_t remaining_us = end - esp_timer_get_time();
            if (remaining_us > 0) {
                state->status.scan_percent = (end - started - remaining_us) * 100 / (end - started);
                state->status.scan_seconds_left = (remaining_us + 999999) / 1000000;
                publish_status(request, state);
            }
            vTaskDelay(sample_delay);
        }
        complete = complete && results[i].samples != 0;
        log_scan_result(label, frequencies[i], &results[i]);
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(sx1262_standby());
    if (xQueuePeek(requests, &pending, 0) == pdPASS) {
        ESP_LOGI(TAG, "%s scan cancelled", label);
        return;
    }
    ESP_LOGI(TAG, "%s scan summary: %s", label, complete ? "complete" : "partial");
    for (unsigned i = 0; i < SCAN_CHANNELS; ++i) {
        log_scan_result(label, frequencies[i], &results[i]);
    }
    state->status.scan_state = complete ? SCAN_DONE : SCAN_ERROR;
    state->status.scan_percent = 100;
    state->status.scan_seconds_left = 0;
    publish_status(request, state);
}

static void radio_task(void *argument)
{
    (void)argument;
    sx1262_config_t config = sx1262_default_gfsk_config(CONFIG_LINK_FREQUENCY_HZ);
    config.tx_power_dbm = CONFIG_LINK_TX_POWER_DBM;
    const esp_err_t result = sx1262_init(&config);
    mode_request_t request = {.mode = MODE_MASTER};
    link_state_t state = {0};
    if (result != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(result);
        state.status.scan_state = SCAN_ERROR;
        for (;;) {
            if (xQueueReceive(requests, &request, portMAX_DELAY) == pdPASS) {
                publish_status(&request, &state);
            }
        }
    }
    ESP_LOGI(TAG, "MASTER started freq=%lu Hz tx=%d dBm frame=%u bytes period=%u ms",
             (unsigned long)config.frequency_hz, (int)config.tx_power_dbm,
             (unsigned)FRAME_LENGTH, (unsigned)CYCLE_MS);
    for (;;) {
        if (xQueueReceive(requests, &request, 0) == pdPASS) {
            state = (link_state_t){.slave_window_start = xTaskGetTickCount()};
            ESP_ERROR_CHECK_WITHOUT_ABORT(sx1262_standby());
            ESP_LOGI(TAG, "%s mode selected", mode_name(request.mode));
            publish_status(&request, &state);
            if (request.mode == MODE_SCAN_GFSK || request.mode == MODE_SCAN_LORA) {
                scan_channels(&request, &state);
                mode_request_t pending;
                xQueuePeek(requests, &pending, portMAX_DELAY);
                continue;
            }
            const esp_err_t link_config = sx1262_set_config(&config);
            if (link_config != ESP_OK) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(link_config);
                mode_request_t pending;
                xQueuePeek(requests, &pending, portMAX_DELAY);
                continue;
            }
        }
        expire_link(&request, &state);
        if (request.mode == MODE_MASTER) {
            const TickType_t started = xTaskGetTickCount();
            mode_request_t pending;
            if (!master_cycle(&request, &state, started) &&
                xQueuePeek(requests, &pending, 0) != pdPASS) {
                ++state.status.missed_packets;
                publish_status(&request, &state);
            }
            expire_link(&request, &state);
            const TickType_t elapsed = xTaskGetTickCount() - started;
            if (elapsed < pdMS_TO_TICKS(CYCLE_MS)) {
                xQueuePeek(requests, &pending, pdMS_TO_TICKS(CYCLE_MS) - elapsed);
            }
        } else {
            slave_receive(&request, &state);
        }
    }
}

esp_err_t radio_link_start(void)
{
    requests = xQueueCreate(1, sizeof(mode_request_t));
    samples = xQueueCreate(1, sizeof(link_sample_t));
    if (requests && samples && xTaskCreate(radio_task, "radio_link", 4096, NULL, 5, NULL) == pdPASS) {
        return ESP_OK;
    }
    if (requests) {
        vQueueDelete(requests);
        requests = NULL;
    }
    if (samples) {
        vQueueDelete(samples);
        samples = NULL;
    }
    return ESP_ERR_NO_MEM;
}

void radio_link_set_mode(system_mode_t mode)
{
    if (requests) {
        const mode_request_t request = {.mode = mode, .generation = ++generation};
        xQueueOverwrite(requests, &request);
    }
}

radio_link_status_t radio_link_get_status(void)
{
    link_sample_t sample;
    if (samples && xQueuePeek(samples, &sample, 0) == pdPASS && sample.generation == generation) {
        return sample.status;
    }
    return (radio_link_status_t){0};
}

unsigned radio_link_take_activity(void)
{
    portENTER_CRITICAL(&activity_lock);
    const unsigned events = activity;
    activity = 0;
    portEXIT_CRITICAL(&activity_lock);
    return events;
}
