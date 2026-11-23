#pragma once

#include "radio_link.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct {
    system_mode_t mode;
    uint32_t generation;
} mode_request_t;

const char *radio_link_mode_name(system_mode_t mode);
void radio_link_queue_log(esp_log_level_t level, const char *format, ...)
    __attribute__((format(printf, 2, 3)));
void radio_link_publish_status(const mode_request_t *request, const radio_link_status_t *status);
void radio_scan_run(const mode_request_t *request, radio_link_status_t *status,
                    QueueHandle_t requests);
