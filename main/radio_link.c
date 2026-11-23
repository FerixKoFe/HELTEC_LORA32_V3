#include "radio_link_private.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "sx1262.h"

enum {
    DATA_LENGTH = 50,
    ALIVE_OFFSET = DATA_LENGTH,
    FRAME_LENGTH = DATA_LENGTH + 1,
    CYCLE_MS = 20,
    RX_WINDOW_MS = (CYCLE_MS * 110 + 99) / 100,
    LINK_EXPIRY_MS = 3 * CYCLE_MS,
    RSSI_AVERAGE_MS = 1000,
    RX_SLICE_MS = 2,
    TX_TIMEOUT_MS = 12,
    SLAVE_TURNAROUND_US = 2000,
    UART_LOG_QUEUE_DEPTH = 32,
};

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
    TickType_t rssi_window_start;
    int32_t rssi_sum_x2;
    int32_t rssi_samples;
} link_state_t;

static const char *const TAG = "radio_link";
static QueueHandle_t requests;
static QueueHandle_t samples;
static QueueHandle_t uart_logs;
static SemaphoreHandle_t reply_ready;
static esp_timer_handle_t reply_timer;
static uint32_t generation;
static portMUX_TYPE activity_lock = portMUX_INITIALIZER_UNLOCKED;
static unsigned activity;

typedef struct {
    uint32_t timestamp_ms;
    uint32_t dropped_before;
    char severity;
    char message[192];
} uart_log_t;

void radio_link_queue_log(esp_log_level_t level, const char *format, ...)
{
    static uint32_t dropped;
    if (level > esp_log_level_get(TAG)) {
        return;
    }
    uart_log_t entry = {
        .timestamp_ms = esp_log_timestamp(),
        .dropped_before = dropped,
        .severity = level == ESP_LOG_WARN ? 'W' : 'I',
    };
    va_list args;
    va_start(args, format);
    vsnprintf(entry.message, sizeof(entry.message), format, args);
    va_end(args);
    dropped = xQueueSend(uart_logs, &entry, 0) == pdPASS ? 0 : dropped + 1;
}

static void uart_log_task(void *argument)
{
    (void)argument;
    uart_log_t entry;
    for (;;) {
        if (xQueueReceive(uart_logs, &entry, portMAX_DELAY) != pdPASS) {
            continue;
        }
        if (entry.dropped_before) {
            printf("W (%" PRIu32 ") %s: UART queue dropped %" PRIu32 " line(s)\n",
                   entry.timestamp_ms, TAG, entry.dropped_before);
        }
        printf("%c (%" PRIu32 ") %s: %s\n",
               entry.severity, entry.timestamp_ms, TAG, entry.message);
    }
}

static void reply_timer_elapsed(void *argument)
{
    (void)argument;
    xSemaphoreGive(reply_ready);
}

static void wait_for_reply_slot(int64_t received_us)
{
    const int64_t remaining_us = received_us + SLAVE_TURNAROUND_US - esp_timer_get_time();
    if (remaining_us <= 0) {
        return;
    }
    if (esp_timer_start_once(reply_timer, remaining_us) == ESP_OK) {
        xSemaphoreTake(reply_ready, portMAX_DELAY);
    } else {
        vTaskDelay(pdMS_TO_TICKS(SLAVE_TURNAROUND_US / 1000) + 1);
    }
}

static void record_activity(unsigned event)
{
    portENTER_CRITICAL(&activity_lock);
    activity |= event;
    portEXIT_CRITICAL(&activity_lock);
}

const char *radio_link_mode_name(system_mode_t mode)
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
    radio_link_queue_log(ESP_LOG_INFO, "%s %s alive=%u data=%s", radio_link_mode_name(mode), direction,
             (unsigned)frame[ALIVE_OFFSET], data);
}
#else
#define log_frame(mode, direction, frame) ((void)0)
#endif

void radio_link_publish_status(const mode_request_t *request, const radio_link_status_t *status)
{
    const link_sample_t sample = {.generation = request->generation, .status = *status};
    xQueueOverwrite(samples, &sample);
}

static void publish_rssi_average(const mode_request_t *request, link_state_t *state, TickType_t now)
{
    const TickType_t window = pdMS_TO_TICKS(RSSI_AVERAGE_MS);
    const TickType_t elapsed = now - state->rssi_window_start;
    if (elapsed < window) {
        return;
    }
    state->rssi_window_start += (elapsed / window) * window;
    state->status.rssi_valid = state->rssi_samples != 0;
    if (state->status.rssi_valid) {
        state->status.rssi_dbm_x2 =
            (state->rssi_sum_x2 - state->rssi_samples / 2) / state->rssi_samples;
    }
    state->rssi_sum_x2 = 0;
    state->rssi_samples = 0;
    radio_link_publish_status(request, &state->status);
}

static void expire_link(const mode_request_t *request, link_state_t *state)
{
    if (state->status.connected &&
        xTaskGetTickCount() - state->last_rx >= pdMS_TO_TICKS(LINK_EXPIRY_MS)) {
        state->status.connected = false;
        state->status.rssi_valid = false;
        state->rssi_sum_x2 = 0;
        state->rssi_samples = 0;
        radio_link_queue_log(ESP_LOG_WARN, "%s LINK DOWN: no valid RX for %d ms, missed=%" PRIu32,
                 radio_link_mode_name(request->mode), LINK_EXPIRY_MS, state->status.missed_packets);
        radio_link_publish_status(request, &state->status);
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
    radio_link_queue_log(ESP_LOG_WARN, "SLAVE RX timeout: %d ms, provisional=%" PRIu32
                  " total=%" PRIu32,
             RX_WINDOW_MS, expired_windows, state->status.missed_packets);
    radio_link_publish_status(request, &state->status);
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
            radio_link_queue_log(ESP_LOG_WARN, "%s RX rejected: %s", radio_link_mode_name(mode), esp_err_to_name(result));
        }
        return false;
    }
    if (length != FRAME_LENGTH) {
        radio_link_queue_log(ESP_LOG_WARN, "%s RX invalid length=%u", radio_link_mode_name(mode), (unsigned)length);
        return false;
    }
    const unsigned expected_parity = mode == MODE_MASTER ? 1 : 0;
    if ((frame[ALIVE_OFFSET] & 1U) != expected_parity) {
        radio_link_queue_log(ESP_LOG_WARN, "%s RX wrong counter parity alive=%u", radio_link_mode_name(mode),
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
                radio_link_queue_log(ESP_LOG_WARN, "SLAVE RX duplicate or out of order alive=%u previous=%u",
                         (unsigned)counter, (unsigned)state->last_counter);
                return false;
            }
            radio_link_queue_log(ESP_LOG_INFO, "SLAVE counter resync alive=%u previous=%u",
                     (unsigned)counter, (unsigned)state->last_counter);
        } else {
            missed = delta / 2 - 1;
            /* A late consecutive packet supersedes a provisional timeout. */
            state->status.missed_packets -= state->slave_timed_misses;
            state->status.missed_packets += missed;
            if (state->slave_timed_misses > missed) {
                radio_link_queue_log(ESP_LOG_INFO, "SLAVE late RX alive=%u, cancelled %" PRIu32 " provisional miss(es)",
                         (unsigned)counter, state->slave_timed_misses - missed);
            }
        }
    }
    if (missed) {
        radio_link_queue_log(ESP_LOG_WARN, "%s counter gap prev=%u rx=%u missed=%" PRIu32 " total=%" PRIu32,
                 radio_link_mode_name(request->mode), (unsigned)state->last_counter,
                 (unsigned)counter, missed, state->status.missed_packets);
    }
    if (!state->status.connected) {
        radio_link_queue_log(ESP_LOG_INFO, "%s LINK UP", radio_link_mode_name(request->mode));
    }
    state->have_counter = true;
    state->last_counter = counter;
    state->last_good_cycle = state->cycle;
    state->last_rx = xTaskGetTickCount();
    state->slave_window_start = state->last_rx;
    state->slave_timed_misses = 0;
    state->status.connected = true;
    publish_rssi_average(request, state, state->last_rx);
    state->rssi_sum_x2 += metrics->rssi_dbm_x2;
    ++state->rssi_samples;
    record_activity(RADIO_LINK_ACTIVITY_RX);
    radio_link_publish_status(request, &state->status);
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
        radio_link_queue_log(ESP_LOG_WARN, "MASTER TX failed alive=%u: %s", (unsigned)counter, esp_err_to_name(result));
        return false;
    }
    const esp_err_t rx_result = sx1262_start_continuous_rx();
    if (rx_result != ESP_OK) {
        radio_link_queue_log(ESP_LOG_WARN, "MASTER TX done alive=%u, RX start failed: %s",
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
            radio_link_queue_log(ESP_LOG_WARN, "MASTER RX unexpected alive=%u expected=%u",
                     (unsigned)rx[ALIVE_OFFSET], (unsigned)expected);
            continue;
        }
        size_t i = 0;
        while (i < DATA_LENGTH && (uint8_t)(tx[i] ^ rx[i]) == UINT8_MAX) {
            ++i;
        }
        if (i != DATA_LENGTH) {
            radio_link_queue_log(ESP_LOG_WARN, "MASTER RX inverse mismatch alive=%u byte=%u",
                     (unsigned)expected, (unsigned)i);
            continue;
        }
        const TickType_t elapsed = xTaskGetTickCount() - started;
        accept_frame(request, state, expected, &metrics);
        radio_link_queue_log(ESP_LOG_INFO, "MASTER TX=%u RX=%u RSSI_x2=%d RX_ready_us=%" PRId64 " RTT_ms=%lu",
                 (unsigned)counter, (unsigned)expected, (int)metrics.rssi_dbm_x2, rx_ready_us,
                 (unsigned long)pdTICKS_TO_MS(elapsed));
        log_frame(MODE_MASTER, "TX", tx);
        log_frame(MODE_MASTER, "RX OK", rx);
        return true;
    }
    radio_link_queue_log(ESP_LOG_WARN, "MASTER TX done alive=%u, no valid response alive=%u RX_ready_us=%" PRId64,
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
    wait_for_reply_slot(received_us);
    const esp_err_t result = send_frame(tx);
    const int64_t reply_us = esp_timer_get_time() - received_us;
    ESP_ERROR_CHECK_WITHOUT_ABORT(sx1262_start_continuous_rx());
    log_frame(MODE_SLAVE, "RX OK", rx);
    if (result == ESP_OK) {
        radio_link_queue_log(ESP_LOG_INFO, "SLAVE RX=%u TX_done=%u RSSI_x2=%d reply_us=%" PRId64,
                 (unsigned)rx[ALIVE_OFFSET], (unsigned)tx[ALIVE_OFFSET],
                 (int)metrics.rssi_dbm_x2, reply_us);
        log_frame(MODE_SLAVE, "TX", tx);
    } else {
        radio_link_queue_log(ESP_LOG_WARN, "SLAVE RX=%u RSSI_x2=%d TX failed alive=%u: %s",
                 (unsigned)rx[ALIVE_OFFSET], (int)metrics.rssi_dbm_x2,
                 (unsigned)tx[ALIVE_OFFSET], esp_err_to_name(result));
    }
}

static void radio_task(void *argument)
{
    (void)argument;
    sx1262_config_t config = sx1262_default_gfsk_config(CONFIG_LINK_FREQUENCY_HZ);
    config.tx_power_dbm = CONFIG_LINK_TX_POWER_DBM;
    const esp_err_t result = sx1262_init(&config);
    mode_request_t request = {.mode = MODE_MASTER};
    link_state_t state = {.rssi_window_start = xTaskGetTickCount()};
    if (result != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(result);
        state.status.scan_state = SCAN_ERROR;
        for (;;) {
            if (xQueueReceive(requests, &request, portMAX_DELAY) == pdPASS) {
                radio_link_publish_status(&request, &state.status);
            }
        }
    }
    radio_link_queue_log(ESP_LOG_INFO, "MASTER started freq=%lu Hz tx=%d dBm frame=%u bytes period=%u ms",
             (unsigned long)config.frequency_hz, (int)config.tx_power_dbm,
             (unsigned)FRAME_LENGTH, (unsigned)CYCLE_MS);
    for (;;) {
        if (xQueueReceive(requests, &request, 0) == pdPASS) {
            const TickType_t now = xTaskGetTickCount();
            state = (link_state_t){.slave_window_start = now, .rssi_window_start = now};
            ESP_ERROR_CHECK_WITHOUT_ABORT(sx1262_standby());
            radio_link_queue_log(ESP_LOG_INFO, "%s mode selected", radio_link_mode_name(request.mode));
            radio_link_publish_status(&request, &state.status);
            if (request.mode == MODE_SCAN_GFSK || request.mode == MODE_SCAN_LORA) {
                radio_scan_run(&request, &state.status, requests);
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
        publish_rssi_average(&request, &state, xTaskGetTickCount());
        if (request.mode == MODE_MASTER) {
            const TickType_t started = xTaskGetTickCount();
            mode_request_t pending;
            if (!master_cycle(&request, &state, started) &&
                xQueuePeek(requests, &pending, 0) != pdPASS) {
                ++state.status.missed_packets;
                radio_link_publish_status(&request, &state.status);
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
    uart_logs = xQueueCreate(UART_LOG_QUEUE_DEPTH, sizeof(uart_log_t));
    reply_ready = xSemaphoreCreateBinary();
    esp_err_t result = ESP_ERR_NO_MEM;
    TaskHandle_t logger = NULL;
    if (requests && samples && uart_logs && reply_ready) {
        const esp_timer_create_args_t timer = {
            .callback = reply_timer_elapsed,
            .name = "slave_reply",
        };
        result = esp_timer_create(&timer, &reply_timer);
        if (result == ESP_OK) {
            if (xTaskCreate(uart_log_task, "radio_uart", 4096, NULL, 2, &logger) == pdPASS &&
                xTaskCreate(radio_task, "radio_link", 4096, NULL, 5, NULL) == pdPASS) {
                return ESP_OK;
            }
            result = ESP_ERR_NO_MEM;
        }
    }
    if (logger) {
        vTaskDelete(logger);
    }
    if (reply_timer) {
        esp_timer_delete(reply_timer);
        reply_timer = NULL;
    }
    if (reply_ready) {
        vSemaphoreDelete(reply_ready);
        reply_ready = NULL;
    }
    if (uart_logs) {
        vQueueDelete(uart_logs);
        uart_logs = NULL;
    }
    if (requests) {
        vQueueDelete(requests);
        requests = NULL;
    }
    if (samples) {
        vQueueDelete(samples);
        samples = NULL;
    }
    return result;
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
