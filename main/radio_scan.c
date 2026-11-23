#include "radio_link_private.h"

#include <inttypes.h>

#include "esp_timer.h"
#include "freertos/task.h"
#include "sx1262.h"

enum {
    SCAN_CHANNELS = 8,
    SCAN_CHANNEL_SECONDS = 6,
    SCAN_TOTAL_SECONDS = SCAN_CHANNELS * SCAN_CHANNEL_SECONDS,
    SCAN_SAMPLE_MS = 50,
    SCAN_SETTLE_MS = 20,
    SCAN_ACTIVE_RSSI_X2 = -190,
};

_Static_assert(SCAN_TOTAL_SECONDS < 60, "Leave time for scan cleanup and reporting");

static const uint32_t gfsk_channels[SCAN_CHANNELS] = {
    868100000, 868300000, 868500000, 868700000, 868900000, 869425000, 869525000, 869625000,
};
static const uint32_t lora_channels[SCAN_CHANNELS] = {
    867100000, 867300000, 867500000, 867700000, 867900000, 868100000, 868300000, 868500000,
};

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
        radio_link_queue_log(ESP_LOG_WARN, "%s freq_hz=%" PRIu32 " samples=0 errors=%u", label, hz, result->errors);
        return;
    }
    radio_link_queue_log(ESP_LOG_INFO, "%s freq_hz=%" PRIu32
                  " samples=%u RSSI_dBm min=%.1f mean=%.1f max=%.1f above_-95dBm=%.1f%% errors=%u",
             label, hz, result->samples, result->min_x2 / 2.0,
             result->sum_x2 / (2.0 * result->samples), result->max_x2 / 2.0,
             100.0 * result->active / result->samples, result->errors);
}

void radio_scan_run(const mode_request_t *request, radio_link_status_t *status,
                    QueueHandle_t requests)
{
    const bool lora = request->mode == MODE_SCAN_LORA;
    const uint32_t *frequencies = lora ? lora_channels : gfsk_channels;
    const char *label = radio_link_mode_name(request->mode);
    sx1262_config_t config = lora ? sx1262_default_config(frequencies[0]) :
                                  sx1262_default_gfsk_config(frequencies[0]);
    scan_result_t results[SCAN_CHANNELS] = {0};
    const int64_t started = esp_timer_get_time();
    const int64_t end = started + SCAN_TOTAL_SECONDS * 1000000LL;
    const TickType_t sample_delay = pdMS_TO_TICKS(SCAN_SAMPLE_MS);
    status->scan_state = SCAN_RUNNING;
    status->scan_seconds_left = SCAN_TOTAL_SECONDS;
    radio_link_publish_status(request, status);
    radio_link_queue_log(ESP_LOG_INFO, "%s scan started: %d channels, %d s, RX BW %s kHz, passive RSSI",
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
            radio_link_queue_log(ESP_LOG_WARN, "%s channel setup freq_hz=%" PRIu32 ": %s",
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
                status->scan_percent = (end - started - remaining_us) * 100 / (end - started);
                status->scan_seconds_left = (remaining_us + 999999) / 1000000;
                radio_link_publish_status(request, status);
            }
            vTaskDelay(sample_delay);
        }
        complete = complete && results[i].samples != 0;
        log_scan_result(label, frequencies[i], &results[i]);
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(sx1262_standby());
    if (xQueuePeek(requests, &pending, 0) == pdPASS) {
        radio_link_queue_log(ESP_LOG_INFO, "%s scan cancelled", label);
        return;
    }
    radio_link_queue_log(ESP_LOG_INFO, "%s scan summary: %s", label, complete ? "complete" : "partial");
    for (unsigned i = 0; i < SCAN_CHANNELS; ++i) {
        log_scan_result(label, frequencies[i], &results[i]);
    }
    status->scan_state = complete ? SCAN_DONE : SCAN_ERROR;
    status->scan_percent = 100;
    status->scan_seconds_left = 0;
    radio_link_publish_status(request, status);
}
