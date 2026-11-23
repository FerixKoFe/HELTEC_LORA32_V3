#include "sx1262.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/* Heltec WiFi LoRa 32 V3.1 board connections. Command framing and board
 * initialization follow the Heltec_ESP32 SX126x reference.
 */
enum {
    PIN_NSS = GPIO_NUM_8, PIN_SCK = GPIO_NUM_9, PIN_MOSI = GPIO_NUM_10,
    PIN_MISO = GPIO_NUM_11, PIN_RESET = GPIO_NUM_12, PIN_BUSY = GPIO_NUM_13,
    PIN_DIO1 = GPIO_NUM_14,
    SET_SLEEP = 0x84, SET_STANDBY = 0x80, SET_TX = 0x83, SET_RX = 0x82,
    SET_PACKET_TYPE = 0x8A, SET_MODULATION = 0x8B, SET_PACKET = 0x8C,
    SET_FREQUENCY = 0x86, CALIBRATE_IMAGE = 0x98, SET_PA = 0x95,
    SET_TX_PARAMS = 0x8E, SET_REGULATOR = 0x96, SET_FALLBACK = 0x93,
    SET_TCXO = 0x97, CALIBRATE = 0x89, SET_RF_SWITCH = 0x9D,
    SET_FIFO_BASE = 0x8F, SET_IRQ = 0x08, GET_IRQ = 0x12, CLEAR_IRQ = 0x02,
    GET_RX_BUFFER = 0x13, GET_PACKET_STATUS = 0x14, GET_RSSI = 0x15, GET_STATUS = 0xC0,
    WRITE_REGISTER = 0x0D, READ_REGISTER = 0x1D, WRITE_FIFO = 0x0E, READ_FIFO = 0x1E,
    IRQ_TX_DONE = 0x001, IRQ_RX_DONE = 0x002, IRQ_HEADER_ERROR = 0x020,
    IRQ_CRC_ERROR = 0x040, IRQ_TIMEOUT = 0x200,
    PACKET_IRQS = IRQ_TX_DONE | IRQ_RX_DONE | IRQ_HEADER_ERROR | IRQ_CRC_ERROR | IRQ_TIMEOUT,
    MAX_TIMEOUT_MS = 262143,
};

static const char *const TAG = "sx1262";
static const spi_host_device_t host = SPI2_HOST;
static spi_device_handle_t radio;
static sx1262_config_t current;
static bool configured;
static bool sleeping;
static bool continuous_rx;
static SemaphoreHandle_t dio1_event;

static esp_err_t wait_busy(void)
{
    const int64_t started = esp_timer_get_time();
    while (gpio_get_level(PIN_BUSY)) {
        const int64_t elapsed = esp_timer_get_time() - started;
        if (elapsed >= 150000) {
            return ESP_ERR_TIMEOUT;
        }
        /* Ordinary SPI commands clear BUSY well before a FreeRTOS tick. */
        if (elapsed < 1000) {
            esp_rom_delay_us(5);
        } else {
            vTaskDelay(1);
        }
    }
    return ESP_OK;
}

static esp_err_t exchange(const uint8_t *tx, uint8_t *rx, size_t size)
{
    spi_transaction_t transaction = {
        .length = size * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t result = spi_device_acquire_bus(radio, portMAX_DELAY);
    if (result != ESP_OK) {
        return result;
    }
    gpio_set_level(PIN_NSS, 0);
    if (sleeping) {
        result = wait_busy();
    }
    if (result == ESP_OK) {
        result = spi_device_polling_transmit(radio, &transaction);
    }
    gpio_set_level(PIN_NSS, 1);
    spi_device_release_bus(radio);
    return result;
}

static esp_err_t command(const uint8_t *tx, size_t size)
{
    ESP_RETURN_ON_ERROR(wait_busy(), TAG, "BUSY timeout");
    ESP_RETURN_ON_ERROR(exchange(tx, NULL, size), TAG, "SPI write failed");
    return tx[0] == SET_SLEEP ? ESP_OK : wait_busy();
}

#define WRITE(...) command((const uint8_t[]){__VA_ARGS__}, sizeof((const uint8_t[]){__VA_ARGS__}))
#define CHECK(expression) ESP_RETURN_ON_ERROR(expression, TAG, "Radio operation failed")

static esp_err_t read_response(const uint8_t *prefix, size_t prefix_size,
                               uint8_t *data, size_t size)
{
    uint8_t tx[SX1262_MAX_PAYLOAD + 4] = {0};
    uint8_t rx[sizeof(tx)];
    memcpy(tx, prefix, prefix_size);
    CHECK(wait_busy());
    CHECK(exchange(tx, rx, prefix_size + size));
    memcpy(data, rx + prefix_size, size);
    return wait_busy();
}

static esp_err_t update_register(uint16_t address, uint8_t mask, uint8_t bits)
{
    const uint8_t prefix[] = {READ_REGISTER, address >> 8, address, 0};
    uint8_t value;
    CHECK(read_response(prefix, sizeof(prefix), &value, 1));
    return WRITE(WRITE_REGISTER, address >> 8, address, (value & ~mask) | bits);
}

static bool valid_config(const sx1262_config_t *config)
{
    if (!config || config->frequency_hz < 150000000 || config->frequency_hz > 960000000 ||
        config->tx_power_dbm < -9 || config->tx_power_dbm > 22) {
        return false;
    }
    if (config->modulation == SX1262_MOD_GFSK) {
        const sx1262_gfsk_config_t *gfsk = &config->gfsk;
        return gfsk->bitrate_bps >= 600 && gfsk->bitrate_bps <= 300000 &&
               gfsk->deviation_hz >= 600 && gfsk->deviation_hz <= 200000 &&
               gfsk->preamble_bits >= 16 &&
               (gfsk->fixed_payload_length || gfsk->max_payload_length) &&
               gfsk->whitening_seed <= 0x1FF;
    }
    return config->modulation == SX1262_MOD_LORA &&
           config->bandwidth >= SX1262_BW_125_KHZ && config->bandwidth <= SX1262_BW_500_KHZ &&
           config->spreading_factor >= 5 && config->spreading_factor <= 12 &&
           config->coding_rate >= 5 && config->coding_rate <= 8 &&
           config->preamble_symbols >= (config->spreading_factor <= 6 ? 12 : 8);
}

sx1262_config_t sx1262_default_gfsk_config(uint32_t frequency_hz)
{
    return (sx1262_config_t){
        .frequency_hz = frequency_hz,
        .tx_power_dbm = 14,
        .modulation = SX1262_MOD_GFSK,
        .crc_enabled = true,
        .gfsk = {
            .bitrate_bps = 100000, .deviation_hz = 25000, .preamble_bits = 32,
            .sync_word = {0x2D, 0xD4, 0xB7, 0xE1},
            .max_payload_length = SX1262_GFSK_DEFAULT_MAX_PAYLOAD_LENGTH,
            .whitening_seed = 0x1FF,
        },
    };
}

sx1262_config_t sx1262_default_config(uint32_t frequency_hz)
{
    return (sx1262_config_t){
        .frequency_hz = frequency_hz, .bandwidth = SX1262_BW_125_KHZ,
        .spreading_factor = 7, .coding_rate = 5, .tx_power_dbm = 14,
        .preamble_symbols = 8, .sync_word = SX1262_SYNC_PRIVATE, .crc_enabled = true,
    };
}

static esp_err_t set_standby(uint8_t clock)
{
    if (!radio) {
        return ESP_ERR_INVALID_STATE;
    }
    if (sleeping) {
        /* Falling NSS wakes the radio. */
        const uint8_t wake[] = {GET_STATUS, 0};
        CHECK(exchange(wake, NULL, sizeof(wake)));
        CHECK(wait_busy());
        sleeping = false;
    }
    const esp_err_t result = WRITE(SET_STANDBY, clock);
    if (result == ESP_OK) {
        continuous_rx = false;
    }
    return result;
}

esp_err_t sx1262_standby(void)
{
    return set_standby(0);
}

esp_err_t sx1262_sleep(void)
{
    CHECK(sx1262_standby());
    CHECK(WRITE(SET_SLEEP, 0x04)); /* Warm start retains configuration. */
    sleeping = true;
    vTaskDelay(pdMS_TO_TICKS(2) + 1);
    return ESP_OK;
}

static uint32_t calibrated_low_hz;
static uint32_t calibrated_high_hz;

static esp_err_t program_frequency(uint32_t hz)
{
    const uint32_t pll = ((uint64_t)hz << 25) / 32000000;
    return WRITE(SET_FREQUENCY, pll >> 24, pll >> 16, pll >> 8, pll);
}

static esp_err_t set_frequency(uint32_t hz)
{
    /* Calibrate over a narrow window around the actual frequency (4 MHz units). */
    const uint32_t mhz = hz / 1000000;
    const uint8_t low = (mhz - 4) / 4;
    const uint8_t high = (mhz + 8) / 4;
    CHECK(WRITE(CALIBRATE_IMAGE, low, high));
    CHECK(program_frequency(hz));
    calibrated_low_hz = low * 4000000U;
    calibrated_high_hz = high * 4000000U;
    return ESP_OK;
}

esp_err_t sx1262_retune(uint32_t frequency_hz)
{
    if (!configured) {
        return ESP_ERR_INVALID_STATE;
    }
    if (frequency_hz < calibrated_low_hz || frequency_hz > calibrated_high_hz) {
        return ESP_ERR_INVALID_ARG;
    }
    if (frequency_hz != current.frequency_hz) {
        CHECK(set_standby(1));
        CHECK(program_frequency(frequency_hz));
        current.frequency_hz = frequency_hz;
    }
    return ESP_OK;
}

static esp_err_t packet_params(const sx1262_config_t *config, uint8_t length)
{
    if (config->modulation == SX1262_MOD_GFSK) {
        const uint16_t preamble = config->gfsk.preamble_bits;
        /* Detector: 16 bits; sync: 32 bits; no address filter. */
        return WRITE(SET_PACKET, preamble >> 8, preamble, 0x05, 32, 0,
                     config->gfsk.fixed_payload_length == 0,
                     length, config->crc_enabled ? 0x02 : 0x01, 1);
    }
    CHECK(WRITE(SET_PACKET, config->preamble_symbols >> 8, config->preamble_symbols,
                config->implicit_payload_length != 0, length, config->crc_enabled,
                config->invert_iq));
    /* SX1262 data sheet workaround for inverted LoRa IQ. */
    return update_register(0x0736, 0x04, config->invert_iq ? 0 : 0x04);
}

static esp_err_t configure_gfsk(const sx1262_gfsk_config_t *config)
{
    const uint32_t bitrate = 1024000000U / config->bitrate_bps;
    const uint32_t deviation = (((uint64_t)config->deviation_hz << 25) + 16000000) / 32000000;
    CHECK(WRITE(SET_MODULATION, bitrate >> 16, bitrate >> 8, bitrate,
                0x09, 0x12, deviation >> 16, deviation >> 8, deviation)); /* BT 0.5, 187.2 kHz */
    CHECK(update_register(0x0889, 0x04, 0x04));
    CHECK(WRITE(WRITE_REGISTER, 0x06, 0xC0, config->sync_word[0], config->sync_word[1],
                config->sync_word[2], config->sync_word[3]));
    /* CRC-16/CCITT-FALSE, seed 0xFFFF and polynomial 0x1021. */
    CHECK(WRITE(WRITE_REGISTER, 0x06, 0xBC, 0xFF, 0xFF, 0x10, 0x21));
    CHECK(update_register(0x06B8, 0x01, config->whitening_seed >> 8));
    CHECK(WRITE(WRITE_REGISTER, 0x06, 0xB9, config->whitening_seed));
    return WRITE(WRITE_REGISTER, 0x08, 0xAC, 0x96); /* RX boosted gain */
}

static uint8_t fixed_payload_length(const sx1262_config_t *config)
{
    return config->modulation == SX1262_MOD_GFSK ?
           config->gfsk.fixed_payload_length : config->implicit_payload_length;
}

static uint8_t receive_payload_limit(const sx1262_config_t *config)
{
    const uint8_t fixed_length = fixed_payload_length(config);
    if (fixed_length) {
        return fixed_length;
    }
    return config->modulation == SX1262_MOD_GFSK ?
           config->gfsk.max_payload_length : SX1262_MAX_PAYLOAD;
}

static esp_err_t configure_lora(const sx1262_config_t *config)
{
    const uint32_t bandwidth_hz = 125000U << (config->bandwidth - SX1262_BW_125_KHZ);
    const bool ldro = ((1U << config->spreading_factor) * 1000U) >= 16U * bandwidth_hz;
    CHECK(WRITE(SET_MODULATION, config->spreading_factor, config->bandwidth,
                config->coding_rate - 4, ldro));
    CHECK(update_register(0x0889, 0x04, config->bandwidth == SX1262_BW_500_KHZ ? 0 : 0x04));
    CHECK(WRITE(WRITE_REGISTER, 0x07, 0x40, config->sync_word >> 8, config->sync_word));
    return WRITE(WRITE_REGISTER, 0x08, 0xAC, 0x94); /* Standard RX gain */
}

esp_err_t sx1262_set_config(const sx1262_config_t *config)
{
    if (!valid_config(config)) {
        return ESP_ERR_INVALID_ARG;
    }
    CHECK(sx1262_standby());
    const bool calibrate = !configured || config->frequency_hz != current.frequency_hz;
    configured = false;
    if (calibrate) {
        CHECK(set_frequency(config->frequency_hz));
    }
    const bool gfsk = config->modulation == SX1262_MOD_GFSK;
    CHECK(WRITE(SET_PACKET_TYPE, gfsk ? 0 : 1));
    CHECK(gfsk ? configure_gfsk(&config->gfsk) : configure_lora(config));
    CHECK(WRITE(SET_TX_PARAMS, config->tx_power_dbm, 0x04)); /* 200 us ramp */
    CHECK(packet_params(config, receive_payload_limit(config)));
    current = *config;
    configured = true;
    return ESP_OK;
}

esp_err_t sx1262_get_config(sx1262_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!configured) {
        return ESP_ERR_INVALID_STATE;
    }
    *config = current;
    return ESP_OK;
}

esp_err_t sx1262_set_frequency(uint32_t frequency_hz)
{
    sx1262_config_t config;
    CHECK(sx1262_get_config(&config));
    config.frequency_hz = frequency_hz;
    return sx1262_set_config(&config);
}

esp_err_t sx1262_get_frequency(uint32_t *frequency_hz)
{
    if (!frequency_hz) {
        return ESP_ERR_INVALID_ARG;
    }
    sx1262_config_t config;
    CHECK(sx1262_get_config(&config));
    *frequency_hz = config.frequency_hz;
    return ESP_OK;
}

esp_err_t sx1262_set_tx_power(int8_t dbm)
{
    sx1262_config_t config;
    CHECK(sx1262_get_config(&config));
    config.tx_power_dbm = dbm;
    return sx1262_set_config(&config);
}

esp_err_t sx1262_get_tx_power(int8_t *dbm)
{
    if (!dbm) {
        return ESP_ERR_INVALID_ARG;
    }
    sx1262_config_t config;
    CHECK(sx1262_get_config(&config));
    *dbm = config.tx_power_dbm;
    return ESP_OK;
}

static void dio1_isr(void *argument)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)argument, &higher_priority_task_woken);
    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t start_dio1_interrupt(void)
{
    const esp_err_t service = gpio_install_isr_service(0);
    if (service != ESP_OK && service != ESP_ERR_INVALID_STATE) {
        return service;
    }
    CHECK(gpio_set_intr_type(PIN_DIO1, GPIO_INTR_POSEDGE));
    SemaphoreHandle_t event = xSemaphoreCreateBinary();
    if (!event) {
        return ESP_ERR_NO_MEM;
    }
    const esp_err_t result = gpio_isr_handler_add(PIN_DIO1, dio1_isr, event);
    if (result != ESP_OK) {
        vSemaphoreDelete(event);
        return result;
    }
    dio1_event = event;
    return ESP_OK;
}

static esp_err_t start_radio(const sx1262_config_t *config)
{
    const gpio_config_t inputs = {
        .pin_bit_mask = (1ULL << PIN_BUSY) | (1ULL << PIN_DIO1),
        .mode = GPIO_MODE_INPUT,
    };
    CHECK(gpio_config(&inputs));
    CHECK(gpio_set_level(PIN_NSS, 1));
    CHECK(gpio_set_direction(PIN_NSS, GPIO_MODE_OUTPUT));
    CHECK(gpio_set_level(PIN_RESET, 0));
    CHECK(gpio_set_direction(PIN_RESET, GPIO_MODE_OUTPUT));
    vTaskDelay(pdMS_TO_TICKS(10) + 1);
    CHECK(gpio_set_level(PIN_RESET, 1));
    vTaskDelay(pdMS_TO_TICKS(10) + 1);
    CHECK(sx1262_standby());
    const uint8_t status_command[] = {GET_STATUS};
    uint8_t status;
    CHECK(read_response(status_command, sizeof(status_command), &status, 1));
    if ((status & 0x70) != 0x20) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    CHECK(WRITE(SET_TCXO, 0x02, 0, 0x01, 0x40)); /* DIO3: 1.8 V, 5 ms */
    CHECK(WRITE(CALIBRATE, 0x3F));
    CHECK(WRITE(SET_RF_SWITCH, 1)); /* DIO2 controls the antenna switch. */
    CHECK(WRITE(SET_REGULATOR, 1)); /* DC-DC */
    CHECK(WRITE(SET_FALLBACK, 0x30)); /* Keep the oscillator on after TX/RX. */
    CHECK(WRITE(SET_PA, 0x04, 0x07, 0x00, 0x01));
    CHECK(WRITE(WRITE_REGISTER, 0x08, 0xE7, 0x38)); /* 160 mA OCP */
    CHECK(update_register(0x08D8, 0x1E, 0x1E)); /* TX clamp workaround */
    CHECK(WRITE(SET_FIFO_BASE, 0, 0));
    CHECK(WRITE(SET_IRQ, PACKET_IRQS >> 8, PACKET_IRQS & 0xFF,
                PACKET_IRQS >> 8, PACKET_IRQS & 0xFF, 0, 0, 0, 0));
    CHECK(sx1262_set_config(config));
    return start_dio1_interrupt();
}

esp_err_t sx1262_init(const sx1262_config_t *config)
{
    if (!valid_config(config)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (radio) {
        return ESP_ERR_INVALID_STATE;
    }
    const spi_bus_config_t bus = {
        .mosi_io_num = PIN_MOSI, .miso_io_num = PIN_MISO, .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = SX1262_MAX_PAYLOAD + 4,
    };
    const spi_device_interface_config_t device = {
        .clock_speed_hz = 4000000, .mode = 0, .spics_io_num = -1, .queue_size = 1,
    };
    CHECK(spi_bus_initialize(host, &bus, SPI_DMA_CH_AUTO));
    esp_err_t result = spi_bus_add_device(host, &device, &radio);
    if (result != ESP_OK) {
        spi_bus_free(host);
        return result;
    }
    result = start_radio(config);
    if (result != ESP_OK) {
        sx1262_deinit();
    }
    return result;
}

esp_err_t sx1262_deinit(void)
{
    if (!radio) {
        return ESP_ERR_INVALID_STATE;
    }
    const esp_err_t sleep_result = sx1262_sleep();
    CHECK(spi_bus_remove_device(radio));
    radio = NULL;
    if (dio1_event) {
        gpio_isr_handler_remove(PIN_DIO1);
        vSemaphoreDelete(dio1_event);
        dio1_event = NULL;
    }
    configured = false;
    sleeping = false;
    continuous_rx = false;
    CHECK(spi_bus_free(host));
    return sleep_result;
}

static esp_err_t prepare_packet(uint8_t length)
{
    CHECK(set_standby(1)); /* STDBY_XOSC for fast TX/RX turnaround. */
    CHECK(WRITE(CLEAR_IRQ, 0x03, 0xFF));
    return packet_params(&current, length);
}

static esp_err_t wait_packet_irq(int64_t deadline, uint16_t done, uint16_t *observed)
{
    *observed = 0;
    for (;;) {
        if (gpio_get_level(PIN_DIO1)) {
            const uint8_t prefix[] = {GET_IRQ, 0};
            uint8_t status[2];
            CHECK(read_response(prefix, sizeof(prefix), status, sizeof(status)));
            const uint16_t irq = (status[0] << 8) | status[1];
            *observed = irq;
            if (irq & (IRQ_HEADER_ERROR | IRQ_CRC_ERROR)) {
                return ESP_ERR_INVALID_CRC;
            }
            if (irq & done) {
                return ESP_OK;
            }
            if (irq & IRQ_TIMEOUT) {
                return ESP_ERR_TIMEOUT;
            }
        }
        const int64_t remaining_us = deadline - esp_timer_get_time();
        if (remaining_us <= 0) {
            return ESP_ERR_TIMEOUT;
        }
        const TickType_t wait_ticks = (remaining_us * configTICK_RATE_HZ + 999999) / 1000000;
        xSemaphoreTake(dio1_event, wait_ticks);
    }
}

static esp_err_t run_packet(uint8_t opcode, uint32_t timeout_ms, uint16_t done)
{
    const uint32_t ticks = timeout_ms * 64;
    const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000 + 20000;
    uint16_t observed;
    esp_err_t result = WRITE(opcode, ticks >> 16, ticks >> 8, ticks);
    if (result == ESP_OK) {
        result = wait_packet_irq(deadline, done, &observed);
    }
    if (result != ESP_OK) {
        /* Explicitly stop on timeout or error. Success already falls back to XOSC. */
        ESP_ERROR_CHECK_WITHOUT_ABORT(set_standby(1));
    }
    return result;
}

esp_err_t sx1262_send(const void *payload, size_t length, uint32_t timeout_ms)
{
    if (!configured) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t fixed_length = fixed_payload_length(&current);
    if (!payload || !length || length > receive_payload_limit(&current) ||
        !timeout_ms || timeout_ms > MAX_TIMEOUT_MS ||
        (fixed_length && length != fixed_length)) {
        return ESP_ERR_INVALID_ARG;
    }
    CHECK(prepare_packet(length));
    uint8_t tx[2 + SX1262_MAX_PAYLOAD] = {WRITE_FIFO, 0};
    memcpy(tx + 2, payload, length);
    CHECK(command(tx, length + 2));
    return run_packet(SET_TX, timeout_ms, IRQ_TX_DONE);
}

static esp_err_t read_packet(void *payload, size_t capacity, size_t *length,
                              sx1262_packet_metrics_t *metrics)
{
    const uint8_t buffer_status[] = {GET_RX_BUFFER, 0};
    uint8_t buffer[2];
    CHECK(read_response(buffer_status, sizeof(buffer_status), buffer, sizeof(buffer)));
    const uint8_t fixed_length = fixed_payload_length(&current);
    const size_t received = fixed_length ? fixed_length : buffer[0];
    if (received > receive_payload_limit(&current)) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (capacity < received) {
        *length = received;
        return ESP_ERR_INVALID_SIZE;
    }
    if (metrics) {
        const uint8_t packet_status[] = {GET_PACKET_STATUS, 0};
        uint8_t status[3];
        CHECK(read_response(packet_status, sizeof(packet_status), status, sizeof(status)));
        if (current.modulation == SX1262_MOD_GFSK) {
            *metrics = (sx1262_packet_metrics_t){
                .rssi_dbm_x2 = -(int16_t)status[2], .signal_rssi_dbm_x2 = -(int16_t)status[1],
            };
        } else {
            *metrics = (sx1262_packet_metrics_t){-(int16_t)status[0], (int8_t)status[1], -(int16_t)status[2]};
        }
    }
    const uint8_t fifo[] = {READ_FIFO, buffer[1], 0};
    CHECK(read_response(fifo, sizeof(fifo), payload, received));
    *length = received;
    return ESP_OK;
}

esp_err_t sx1262_receive(void *payload, size_t capacity, size_t *length,
                         sx1262_packet_metrics_t *metrics, uint32_t timeout_ms)
{
    if (!payload || !capacity || !length || !timeout_ms || timeout_ms > MAX_TIMEOUT_MS) {
        return ESP_ERR_INVALID_ARG;
    }
    *length = 0;
    if (!configured) {
        return ESP_ERR_INVALID_STATE;
    }
    CHECK(prepare_packet(receive_payload_limit(&current)));
    const esp_err_t result = run_packet(SET_RX, timeout_ms, IRQ_RX_DONE);
    if (result != ESP_OK) {
        return result;
    }
    if (current.modulation == SX1262_MOD_LORA && current.implicit_payload_length) {
        /* Stop the RTC timeout after implicit-header LoRa RX (SX1262 errata). */
        CHECK(WRITE(WRITE_REGISTER, 0x09, 0x02, 0));
        CHECK(update_register(0x0944, 0x02, 0x02));
    }
    return read_packet(payload, capacity, length, metrics);
}

esp_err_t sx1262_start_continuous_rx(void)
{
    if (!configured) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!continuous_rx) {
        CHECK(prepare_packet(receive_payload_limit(&current)));
        CHECK(WRITE(SET_RX, 0xFF, 0xFF, 0xFF));
        continuous_rx = true;
    }
    return ESP_OK;
}

esp_err_t sx1262_get_rssi_instant(int16_t *rssi_dbm_x2)
{
    if (!rssi_dbm_x2) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!continuous_rx) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t prefix[] = {GET_RSSI, 0};
    uint8_t raw;
    CHECK(read_response(prefix, sizeof(prefix), &raw, 1));
    *rssi_dbm_x2 = -(int16_t)raw;
    return ESP_OK;
}

esp_err_t sx1262_receive_continuous(void *payload, size_t capacity, size_t *length,
                                    sx1262_packet_metrics_t *metrics, uint32_t timeout_ms)
{
    if (!payload || !capacity || !length || !timeout_ms || timeout_ms > MAX_TIMEOUT_MS) {
        return ESP_ERR_INVALID_ARG;
    }
    *length = 0;
    if (!configured || current.modulation != SX1262_MOD_GFSK) {
        return ESP_ERR_INVALID_STATE;
    }
    CHECK(sx1262_start_continuous_rx());
    uint16_t observed;
    const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    esp_err_t result = wait_packet_irq(deadline, IRQ_RX_DONE, &observed);
    if (result == ESP_OK) {
        result = read_packet(payload, capacity, length, metrics);
    }
    if (observed) {
        CHECK(WRITE(CLEAR_IRQ, observed >> 8, observed));
    }
    return result;
}
