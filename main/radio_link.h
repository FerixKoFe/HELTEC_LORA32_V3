#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    MODE_MASTER,
    MODE_SLAVE,
    MODE_M_FHSS,
    MODE_S_FHSS,
    MODE_SCAN_GFSK,
    MODE_SCAN_LORA,
} system_mode_t;

typedef enum {
    SCAN_PENDING,
    SCAN_RUNNING,
    SCAN_DONE,
    SCAN_ERROR,
} scan_state_t;

typedef struct {
    bool connected;
    bool rssi_valid;
    int16_t rssi_dbm_x2;
    uint32_t missed_packets;
    uint32_t link_drops;
    uint32_t down_missed_packets;
    scan_state_t scan_state;
    uint8_t scan_percent;
    uint8_t scan_seconds_left;
} radio_link_status_t;

esp_err_t radio_link_start(void);
void radio_link_set_mode(system_mode_t mode);
radio_link_status_t radio_link_get_status(void);
