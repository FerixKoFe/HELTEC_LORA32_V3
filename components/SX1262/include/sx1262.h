#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define SX1262_MAX_PAYLOAD 255
#define SX1262_SYNC_PRIVATE 0x1424
#define SX1262_SYNC_PUBLIC 0x3444
#define SX1262_GFSK_DEFAULT_PAYLOAD_LENGTH 54

/* Values match the SX1262 LoRa bandwidth field. */
typedef enum {
    SX1262_BW_125_KHZ = 0x04,
    SX1262_BW_250_KHZ = 0x05,
    SX1262_BW_500_KHZ = 0x06,
} sx1262_bandwidth_t;

typedef enum {
    SX1262_MOD_LORA = 0,
    SX1262_MOD_GFSK,
} sx1262_modulation_t;

typedef struct {
    uint32_t bitrate_bps;
    uint32_t deviation_hz;
    uint16_t preamble_bits;
    uint8_t sync_word[4];
    uint8_t fixed_payload_length;
    uint16_t whitening_seed;
} sx1262_gfsk_config_t;

typedef struct {
    uint32_t frequency_hz;
    sx1262_bandwidth_t bandwidth;
    uint8_t spreading_factor; /* SF5 through SF12 */
    uint8_t coding_rate;      /* Denominator 5 through 8 for 4/x */
    int8_t tx_power_dbm;
    uint16_t preamble_symbols;
    uint16_t sync_word;
    bool crc_enabled;
    bool invert_iq;
    uint8_t implicit_payload_length; /* Zero selects explicit headers. */
    sx1262_modulation_t modulation;
    sx1262_gfsk_config_t gfsk;
} sx1262_config_t;

typedef struct {
    int16_t rssi_dbm_x2;
    int8_t snr_db_x4;
    int16_t signal_rssi_dbm_x2;
} sx1262_packet_metrics_t;

/* One board radio. Call from one task or serialize access externally.
 * The caller selects the band and power suitable for the board and region.
 * Send/receive are synchronous, with a timeout of 1..262143 ms.
 */
sx1262_config_t sx1262_default_config(uint32_t frequency_hz);
/* BT 0.5, 117.3 kHz RX BW, 32-bit preamble detector, CRC-16, whitening, boosted RX. */
sx1262_config_t sx1262_default_gfsk_config(uint32_t frequency_hz);
esp_err_t sx1262_init(const sx1262_config_t *config);
esp_err_t sx1262_deinit(void);
esp_err_t sx1262_set_config(const sx1262_config_t *config);
esp_err_t sx1262_get_config(sx1262_config_t *config);
esp_err_t sx1262_set_frequency(uint32_t frequency_hz);
esp_err_t sx1262_get_frequency(uint32_t *frequency_hz);
esp_err_t sx1262_set_tx_power(int8_t dbm);
esp_err_t sx1262_get_tx_power(int8_t *dbm);
esp_err_t sx1262_send(const void *payload, size_t length, uint32_t timeout_ms);
/* On ESP_ERR_INVALID_SIZE, *length contains the required buffer capacity. */
esp_err_t sx1262_receive(void *payload, size_t capacity, size_t *length,
                         sx1262_packet_metrics_t *metrics, uint32_t timeout_ms);
/* Starts continuous RX for passive measurement in either modulation. */
esp_err_t sx1262_start_continuous_rx(void);
esp_err_t sx1262_get_rssi_instant(int16_t *rssi_dbm_x2);
/* Continuous packet reception is available for GFSK. Timeout leaves RX running. */
esp_err_t sx1262_receive_continuous(void *payload, size_t capacity, size_t *length,
                                    sx1262_packet_metrics_t *metrics, uint32_t timeout_ms);
esp_err_t sx1262_standby(void);
esp_err_t sx1262_sleep(void);
