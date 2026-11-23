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
#include "freertos/task.h"
#include "sdkconfig.h"
#include "sx1262.h"

enum {
    DATA_LENGTH = 50,
    FRAME_LENGTH = DATA_LENGTH + 1,
    FHSS_DATA_LENGTH = 40,
    FHSS_METADATA_OFFSET = FHSS_DATA_LENGTH + 1,
    FHSS_FRAME_LENGTH = FHSS_METADATA_OFFSET + 8,
    FHSS_SLOT_MS = 20,
    FHSS_SLOT_US = FHSS_SLOT_MS * 1000,
    FHSS_MAX_TX_LATENESS_US = 3000,
    FHSS_RX_GUARD_US = 3000,
    FHSS_SYNC_INTERVAL = 4,
    FHSS_HOLDOVER_US = 6 * FHSS_SLOT_US,
    FHSS_SYNC_HZ = 869525000,
    CYCLE_MS = 20,
    RX_WINDOW_MS = (CYCLE_MS * 110 + 99) / 100,
    RSSI_AVERAGE_MS = 1000,
    RX_SLICE_MS = 2,
    TX_TIMEOUT_MS = 12,
    UART_LOG_QUEUE_DEPTH = 32,
};

typedef struct {
    uint32_t generation;
    radio_link_status_t status;
} link_sample_t;

_Static_assert(FHSS_FRAME_LENGTH <= SX1262_GFSK_DEFAULT_MAX_PAYLOAD_LENGTH,
               "FHSS frame exceeds radio payload limit");

static const uint32_t fhss_channels[] = {868100000, 868300000, 868500000};
_Static_assert(sizeof(fhss_channels) / sizeof(fhss_channels[0]) == FHSS_SYNC_INTERVAL - 1,
               "Each FHSS group must visit every data channel");

typedef struct {
    uint32_t session;
    uint32_t slot;
    int64_t time_us;
    bool synchronized;
} fhss_state_t;

typedef struct {
    radio_link_status_t status;
    TickType_t last_rx;
    TickType_t down_window_start;
    uint8_t last_sequence;
    uint8_t next_sequence;
    fhss_state_t fhss;
    uint32_t fhss_logged_frequency_hz;
    TickType_t rssi_window_start;
    int32_t rssi_sum_x2;
    int32_t rssi_samples;
} link_state_t;

static const char *const TAG = "radio_link";
static QueueHandle_t requests;
static QueueHandle_t samples;
static QueueHandle_t uart_logs;
static uint32_t generation;
static portMUX_TYPE generation_lock = portMUX_INITIALIZER_UNLOCKED;

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

const char *radio_link_mode_name(system_mode_t mode)
{
    switch (mode) {
    case MODE_MASTER: return "MASTER";
    case MODE_SLAVE: return "SLAVE";
    case MODE_M_FHSS: return "M_FHSS";
    case MODE_S_FHSS: return "S_FHSS";
    case MODE_SCAN_GFSK: return "GFSK";
    case MODE_SCAN_LORA: return "LORA";
    }
    return "UNKNOWN";
}

static bool is_fhss(system_mode_t mode)
{
    return mode == MODE_M_FHSS || mode == MODE_S_FHSS;
}

static bool is_master(system_mode_t mode)
{
    return mode == MODE_MASTER || mode == MODE_M_FHSS;
}

static size_t data_length(system_mode_t mode)
{
    return is_fhss(mode) ? FHSS_DATA_LENGTH : DATA_LENGTH;
}

static size_t frame_length(system_mode_t mode)
{
    return is_fhss(mode) ? FHSS_FRAME_LENGTH : FRAME_LENGTH;
}

static unsigned cycle_ms(system_mode_t mode)
{
    return is_fhss(mode) ? FHSS_SLOT_MS : CYCLE_MS;
}

static uint32_t fhss_frequency(uint32_t session, uint32_t slot)
{
    const unsigned position = slot % FHSS_SYNC_INTERVAL;
    if (position == 0) {
        return FHSS_SYNC_HZ;
    }

    // Choose a permutation for each group, visiting each data channel once.
    uint32_t mixed = session ^ (slot / FHSS_SYNC_INTERVAL);
    mixed ^= mixed >> 16;
    mixed *= UINT32_C(0x7feb352d);
    mixed ^= mixed >> 15;
    const unsigned start = mixed % 3;
    const unsigned step = (mixed & 1U) + 1;
    return fhss_channels[(start + (position - 1) * step) % 3];
}

static esp_err_t fhss_retune(system_mode_t mode, link_state_t *state, uint32_t slot)
{
    const uint32_t frequency_hz = fhss_frequency(state->fhss.session, slot);
    const esp_err_t result = sx1262_retune(frequency_hz);
    if (result == ESP_OK && frequency_hz != state->fhss_logged_frequency_hz) {
        state->fhss_logged_frequency_hz = frequency_hz;
        radio_link_queue_log(ESP_LOG_INFO, "%s freq=%" PRIu32 " Hz",
                             radio_link_mode_name(mode), frequency_hz);
    }
    return result;
}

static void write_u32(uint8_t *field, uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i) {
        field[i] = (uint8_t)(value >> (8 * i));
    }
}

static uint32_t read_u32(const uint8_t *field)
{
    uint32_t value = 0;
    for (unsigned i = 0; i < 4; ++i) {
        value |= (uint32_t)field[i] << (8 * i);
    }
    return value;
}

static uint32_t fhss_slot_at(const fhss_state_t *fhss, int64_t now_us)
{
    // Switch halfway between predicted packet arrivals, including after loss.
    return fhss->slot + (now_us - fhss->time_us + FHSS_SLOT_US / 2) / FHSS_SLOT_US;
}

#ifdef CONFIG_LINK_LOG_PAYLOAD
static void log_frame(system_mode_t mode, const char *direction,
                      const uint8_t frame[FRAME_LENGTH])
{
    static const char hex[] = "0123456789ABCDEF";
    char data[DATA_LENGTH * 2 + 1];
    const size_t length = data_length(mode);
    for (size_t i = 0; i < length; ++i) {
        data[2 * i] = hex[frame[i] >> 4];
        data[2 * i + 1] = hex[frame[i] & 0x0F];
    }
    data[length * 2] = '\0';
    radio_link_queue_log(ESP_LOG_INFO, "%s %s seq=%u data=%s", radio_link_mode_name(mode), direction,
             (unsigned)frame[length], data);
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
    const TickType_t now = xTaskGetTickCount();
    const unsigned expiry_ms = 3 * cycle_ms(request->mode);
    if (state->status.connected &&
        now - state->last_rx >= pdMS_TO_TICKS(expiry_ms)) {
        state->status.connected = false;
        ++state->status.link_drops;
        state->down_window_start = now;
        state->status.rssi_valid = false;
        state->rssi_sum_x2 = 0;
        state->rssi_samples = 0;
        radio_link_queue_log(ESP_LOG_WARN, "%s LINK DOWN: no valid RX for %u ms, missed=%" PRIu32,
                 radio_link_mode_name(request->mode), expiry_ms, state->status.missed_packets);
        radio_link_publish_status(request, &state->status);
    }
}

static void record_bad_frame(const mode_request_t *request, link_state_t *state)
{
    expire_link(request, state);
    if (state->status.connected) {
        ++state->status.missed_packets;
        radio_link_publish_status(request, &state->status);
    }
}

static bool receive_frame(const mode_request_t *request, link_state_t *state,
                          uint8_t frame[FRAME_LENGTH],
                          sx1262_packet_metrics_t *metrics)
{
    const system_mode_t mode = request->mode;
    size_t length;
    const esp_err_t result =
        sx1262_receive_continuous(frame, FRAME_LENGTH, &length, metrics, RX_SLICE_MS);
    if (result != ESP_OK) {
        if (result == ESP_ERR_INVALID_CRC || result == ESP_ERR_INVALID_SIZE ||
            result == ESP_ERR_INVALID_RESPONSE) {
            record_bad_frame(request, state);
        }
        if (result != ESP_ERR_TIMEOUT) {
            radio_link_queue_log(ESP_LOG_WARN, "%s RX rejected: %s", radio_link_mode_name(mode), esp_err_to_name(result));
        }
        return false;
    }
    if (length != frame_length(mode)) {
        record_bad_frame(request, state);
        radio_link_queue_log(ESP_LOG_WARN, "%s RX invalid length=%u", radio_link_mode_name(mode), (unsigned)length);
        return false;
    }
    const unsigned expected_parity = is_master(mode) ? 1 : 0;
    if ((frame[data_length(mode)] & 1U) != expected_parity) {
        record_bad_frame(request, state);
        radio_link_queue_log(ESP_LOG_WARN, "%s RX wrong sequence parity seq=%u", radio_link_mode_name(mode),
                 (unsigned)frame[data_length(mode)]);
        return false;
    }
    return true;
}

static bool accept_frame(const mode_request_t *request, link_state_t *state,
                         uint8_t sequence, const sx1262_packet_metrics_t *metrics)
{
    expire_link(request, state);
    if (request->mode == MODE_SLAVE && state->status.connected) {
        const uint8_t delta = sequence - state->last_sequence;
        if (delta == 0 || delta >= 128) {
            radio_link_queue_log(ESP_LOG_WARN, "SLAVE RX duplicate or out of order seq=%u previous=%u",
                     (unsigned)sequence, (unsigned)state->last_sequence);
            record_bad_frame(request, state);
            return false;
        }
    }
    if (!state->status.connected) {
        radio_link_queue_log(ESP_LOG_INFO, "%s LINK UP", radio_link_mode_name(request->mode));
    }
    state->last_sequence = sequence;
    state->last_rx = xTaskGetTickCount();
    state->status.connected = true;
    publish_rssi_average(request, state, state->last_rx);
    state->rssi_sum_x2 += metrics->rssi_dbm_x2;
    ++state->rssi_samples;
    radio_link_publish_status(request, &state->status);
    return true;
}

static void master_cycle(const mode_request_t *request, link_state_t *state, TickType_t started,
                         int64_t fhss_deadline_us)
{
    const bool started_down = !state->status.connected;
    const int64_t started_us = esp_timer_get_time();
    uint8_t tx[FRAME_LENGTH];
    const uint8_t sequence = state->next_sequence;
    state->next_sequence += 2;
    const system_mode_t mode = request->mode;
    const size_t length = data_length(mode);
    esp_fill_random(tx, length);
    tx[length] = sequence;
    if (is_fhss(mode)) {
        write_u32(tx + FHSS_METADATA_OFFSET, state->fhss.session);
        write_u32(tx + FHSS_METADATA_OFFSET + 4, state->fhss.slot);
    }
    const esp_err_t result = sx1262_send(tx, frame_length(mode), TX_TIMEOUT_MS);
    if (result != ESP_OK) {
        radio_link_queue_log(ESP_LOG_WARN, "%s TX failed seq=%u: %s", radio_link_mode_name(mode), (unsigned)sequence, esp_err_to_name(result));
        return;
    }
    const esp_err_t rx_result = sx1262_start_continuous_rx();
    if (rx_result != ESP_OK) {
        radio_link_queue_log(ESP_LOG_WARN, "%s TX done seq=%u, RX start failed: %s",
                 radio_link_mode_name(mode),
                 (unsigned)sequence, esp_err_to_name(rx_result));
        return;
    }
    const int64_t rx_ready_us = esp_timer_get_time() - started_us;
    const int64_t deadline_us = is_fhss(mode) ? fhss_deadline_us :
                                started_us + RX_WINDOW_MS * 1000LL;

    const uint8_t expected = (uint8_t)(sequence + 1);
    uint8_t rx[FRAME_LENGTH];
    sx1262_packet_metrics_t metrics;
    mode_request_t pending;
    while (esp_timer_get_time() < deadline_us &&
           xQueuePeek(requests, &pending, 0) != pdPASS) {
        if (!receive_frame(request, state, rx, &metrics)) {
            continue;
        }
        if (is_fhss(mode) &&
            memcmp(tx + FHSS_METADATA_OFFSET, rx + FHSS_METADATA_OFFSET, 8) != 0) {
            record_bad_frame(request, state);
            continue;
        }
        if (rx[length] != expected) {
            record_bad_frame(request, state);
            radio_link_queue_log(ESP_LOG_WARN, "%s RX unexpected seq=%u expected=%u",
                     radio_link_mode_name(mode),
                     (unsigned)rx[length], (unsigned)expected);
            continue;
        }
        size_t i = 0;
        while (i < length && (uint8_t)(tx[i] ^ rx[i]) == UINT8_MAX) {
            ++i;
        }
        if (i != length) {
            record_bad_frame(request, state);
            radio_link_queue_log(ESP_LOG_WARN, "%s RX inverse mismatch seq=%u byte=%u",
                     radio_link_mode_name(mode),
                     (unsigned)expected, (unsigned)i);
            continue;
        }
        const TickType_t elapsed = xTaskGetTickCount() - started;
        accept_frame(request, state, expected, &metrics);
        radio_link_queue_log(ESP_LOG_INFO, "%s TX=%u RX=%u RSSI_x2=%d RX_ready_us=%" PRId64 " RTT_ms=%lu",
                 radio_link_mode_name(mode),
                 (unsigned)sequence, (unsigned)expected, (int)metrics.rssi_dbm_x2, rx_ready_us,
                 (unsigned long)pdTICKS_TO_MS(elapsed));
        log_frame(mode, "TX", tx);
        log_frame(mode, "RX OK", rx);
        return;
    }
    if (started_down && xQueuePeek(requests, &pending, 0) != pdPASS) {
        ++state->status.down_missed_packets;
        radio_link_publish_status(request, &state->status);
    }
    radio_link_queue_log(ESP_LOG_WARN, "%s TX done seq=%u, no valid response seq=%u RX_ready_us=%" PRId64,
             radio_link_mode_name(mode),
             (unsigned)sequence, (unsigned)expected, rx_ready_us);
    log_frame(mode, "TX", tx);
}

static bool fhss_accept_request(fhss_state_t *fhss, const uint8_t *frame,
                                uint32_t tuned_slot, int64_t received_us)
{
    const uint32_t session = read_u32(frame + FHSS_METADATA_OFFSET);
    const uint32_t slot = read_u32(frame + FHSS_METADATA_OFFSET + 4);
    if (fhss->synchronized) {
        if (slot != tuned_slot || slot == fhss->slot || session != fhss->session) {
            return false;
        }
    } else if (slot % FHSS_SYNC_INTERVAL != 0) {
        return false;
    }
    *fhss = (fhss_state_t){
        .session = session, .slot = slot, .time_us = received_us, .synchronized = true,
    };
    return true;
}

static void slave_receive(const mode_request_t *request, link_state_t *state)
{
    const system_mode_t mode = request->mode;
    const size_t length = data_length(mode);
    uint32_t tuned_slot = 0;
    if (is_fhss(mode)) {
        const int64_t now_us = esp_timer_get_time();
        fhss_state_t *fhss = &state->fhss;
        if (now_us - fhss->time_us >= FHSS_HOLDOVER_US) {
            fhss->synchronized = false;
        }
        if (fhss->synchronized) {
            tuned_slot = fhss_slot_at(fhss, now_us);
        }
        if (fhss_retune(mode, state, tuned_slot) != ESP_OK) {
            vTaskDelay(1);
            return;
        }
    }
    uint8_t rx[FRAME_LENGTH];
    sx1262_packet_metrics_t metrics;
    bool accepted = receive_frame(request, state, rx, &metrics);
    const int64_t received_us = esp_timer_get_time();
    if (accepted && is_fhss(mode) &&
        !fhss_accept_request(&state->fhss, rx, tuned_slot, received_us)) {
        record_bad_frame(request, state);
        accepted = false;
    }
    if (!accepted || !accept_frame(request, state, rx[length], &metrics)) {
        if (!state->status.connected) {
            const TickType_t window = pdMS_TO_TICKS(cycle_ms(mode));
            const TickType_t elapsed = xTaskGetTickCount() - state->down_window_start;
            const uint32_t missed = elapsed / window;
            if (missed) {
                state->down_window_start += missed * window;
                state->status.down_missed_packets += missed;
                radio_link_publish_status(request, &state->status);
            }
        }
        return;
    }
    uint8_t tx[FRAME_LENGTH];
    if (is_fhss(mode)) {
        memcpy(tx + FHSS_METADATA_OFFSET, rx + FHSS_METADATA_OFFSET, 8);
    }
    for (size_t i = 0; i < length; ++i) {
        tx[i] = (uint8_t)~rx[i];
    }
    tx[length] = (uint8_t)(rx[length] + 1);
    const esp_err_t result = sx1262_send(tx, frame_length(mode), TX_TIMEOUT_MS);
    const int64_t reply_us = esp_timer_get_time() - received_us;
    ESP_ERROR_CHECK_WITHOUT_ABORT(sx1262_start_continuous_rx());
    log_frame(mode, "RX OK", rx);
    if (result == ESP_OK) {
        radio_link_queue_log(ESP_LOG_INFO, "%s RX=%u TX_done=%u RSSI_x2=%d reply_us=%" PRId64,
                 radio_link_mode_name(mode),
                 (unsigned)rx[length], (unsigned)tx[length],
                 (int)metrics.rssi_dbm_x2, reply_us);
        log_frame(mode, "TX", tx);
    } else {
        radio_link_queue_log(ESP_LOG_WARN, "%s RX=%u RSSI_x2=%d TX failed seq=%u: %s",
                 radio_link_mode_name(mode),
                 (unsigned)rx[length], (int)metrics.rssi_dbm_x2,
                 (unsigned)tx[length], esp_err_to_name(result));
    }
}

static void fhss_master_cycle(const mode_request_t *request, link_state_t *state)
{
    fhss_state_t *fhss = &state->fhss;
    const int64_t now_us = esp_timer_get_time();
    const int64_t elapsed_us = now_us - fhss->time_us;
    const uint32_t slot = elapsed_us / FHSS_SLOT_US;
    const int64_t phase_us = elapsed_us % FHSS_SLOT_US;

    // Skip overdue transmissions and never retry on a later hop.
    if ((!fhss->synchronized || fhss->slot != slot) && phase_us <= FHSS_MAX_TX_LATENESS_US) {
        fhss->slot = slot;
        fhss->synchronized = true;
        const esp_err_t result = fhss_retune(request->mode, state, slot);
        if (result == ESP_OK) {
            const int64_t deadline_us = now_us - phase_us + FHSS_SLOT_US - FHSS_RX_GUARD_US;
            master_cycle(request, state, xTaskGetTickCount(), deadline_us);
        } else {
            ESP_ERROR_CHECK_WITHOUT_ABORT(result);
        }
        return;
    }
    mode_request_t pending;
    const unsigned wait_ms = (FHSS_SLOT_US - phase_us + 999) / 1000;
    xQueuePeek(requests, &pending, pdMS_TO_TICKS(wait_ms));
}

static void radio_task(void *argument)
{
    (void)argument;
    sx1262_config_t config = sx1262_default_gfsk_config(CONFIG_LINK_FREQUENCY_HZ);
    config.tx_power_dbm = CONFIG_LINK_TX_POWER_DBM;
    const esp_err_t result = sx1262_init(&config);
    mode_request_t request = {.mode = MODE_MASTER};
    const TickType_t started = xTaskGetTickCount();
    link_state_t state = {.rssi_window_start = started, .down_window_start = started};
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
            state = (link_state_t){
                .status = {
                    .missed_packets = state.status.missed_packets,
                    .link_drops = state.status.link_drops,
                    .down_missed_packets = state.status.down_missed_packets,
                },
                .rssi_window_start = now,
                .down_window_start = now,
            };
            ESP_ERROR_CHECK_WITHOUT_ABORT(sx1262_standby());
            radio_link_queue_log(ESP_LOG_INFO, "%s mode selected", radio_link_mode_name(request.mode));
            radio_link_publish_status(&request, &state.status);
            if (request.mode == MODE_SCAN_GFSK || request.mode == MODE_SCAN_LORA) {
                radio_scan_run(&request, &state.status, requests);
                mode_request_t pending;
                xQueuePeek(requests, &pending, portMAX_DELAY);
                continue;
            }
            sx1262_config_t selected_config = config;
            if (is_fhss(request.mode)) {
                selected_config.frequency_hz = FHSS_SYNC_HZ;
                selected_config.gfsk.sync_word[3] = 0x4A;
            }
            const esp_err_t link_config = sx1262_set_config(&selected_config);
            if (link_config != ESP_OK) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(link_config);
                mode_request_t pending;
                xQueuePeek(requests, &pending, portMAX_DELAY);
                continue;
            }
            if (request.mode == MODE_M_FHSS) {
                state.fhss.session = esp_random();
                state.fhss.time_us = esp_timer_get_time();
            }
        }
        expire_link(&request, &state);
        publish_rssi_average(&request, &state, xTaskGetTickCount());
        if (request.mode == MODE_M_FHSS) {
            fhss_master_cycle(&request, &state);
        } else if (request.mode == MODE_MASTER) {
            const TickType_t started = xTaskGetTickCount();
            mode_request_t pending;
            master_cycle(&request, &state, started, 0);
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
    TaskHandle_t logger = NULL;
    if (requests && samples && uart_logs &&
        xTaskCreate(uart_log_task, "radio_uart", 4096, NULL, 2, &logger) == pdPASS &&
        xTaskCreate(radio_task, "radio_link", 4096, NULL, 5, NULL) == pdPASS) {
        return ESP_OK;
    }
    if (logger) {
        vTaskDelete(logger);
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
    return ESP_ERR_NO_MEM;
}

void radio_link_set_mode(system_mode_t mode)
{
    if (requests) {
        portENTER_CRITICAL(&generation_lock);
        const uint32_t next_generation = ++generation;
        portEXIT_CRITICAL(&generation_lock);
        const mode_request_t request = {.mode = mode, .generation = next_generation};
        xQueueOverwrite(requests, &request);
    }
}

radio_link_status_t radio_link_get_status(void)
{
    link_sample_t sample;
    if (samples && xQueuePeek(samples, &sample, 0) == pdPASS) {
        portENTER_CRITICAL(&generation_lock);
        const bool current = sample.generation == generation;
        portEXIT_CRITICAL(&generation_lock);
        if (current) {
            return sample.status;
        }
    }
    return (radio_link_status_t){0};
}
