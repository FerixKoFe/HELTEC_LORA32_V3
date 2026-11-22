#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ssd1306.h"

enum {
    OLED_WIDTH = 128,
    OLED_HEIGHT = 64,
    OLED_SDA = GPIO_NUM_17,
    OLED_SCL = GPIO_NUM_18,
    OLED_RESET = GPIO_NUM_21,
    OLED_VEXT = GPIO_NUM_36,
    OLED_ADDRESS = 0x3C,
    I2C_TIMEOUT_MS = 250,
    GLYPH_WIDTH = 5,
    GLYPH_HEIGHT = 7,
    GLYPH_SPACING = 1,
    GLYPH_SCALE = 4,
};

static const char *const TAG = "ssd1306";

/* Each byte is a column, with the least significant bit at the top. */
static const uint8_t hello[][GLYPH_WIDTH] = {
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, /* H */
    {0x7F, 0x49, 0x49, 0x49, 0x41}, /* E */
    {0x7F, 0x40, 0x40, 0x40, 0x40}, /* L */
    {0x7F, 0x40, 0x40, 0x40, 0x40}, /* L */
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, /* O */
};

static void draw_hello(uint8_t *pixels)
{
    const int glyph_count = sizeof(hello) / sizeof(hello[0]);
    const int text_width = (glyph_count * (GLYPH_WIDTH + GLYPH_SPACING)
                            - GLYPH_SPACING) * GLYPH_SCALE;
    const int left = (OLED_WIDTH - text_width) / 2;
    const int top = (OLED_HEIGHT - GLYPH_HEIGHT * GLYPH_SCALE) / 2;

    for (int y = 0; y < GLYPH_HEIGHT * GLYPH_SCALE; ++y) {
        const int display_y = top + y;
        uint8_t *page = pixels + (display_y / 8) * OLED_WIDTH;
        const uint8_t pixel_mask = 1U << (display_y % 8);
        for (int glyph = 0; glyph < glyph_count; ++glyph) {
            const int x = left + glyph * (GLYPH_WIDTH + GLYPH_SPACING) * GLYPH_SCALE;
            for (int column = 0; column < GLYPH_WIDTH * GLYPH_SCALE; ++column) {
                if (hello[glyph][column / GLYPH_SCALE] & (1U << (y / GLYPH_SCALE))) {
                    page[x + column] |= pixel_mask;
                }
            }
        }
    }
}

static esp_err_t write_display(i2c_master_dev_handle_t display)
{
    static const uint8_t setup[] = {
        0x00,       /* Command stream */
        0xAE,       /* Display off while loading RAM */
        0xD5, 0x80, /* Clock divider */
        0xA8, OLED_HEIGHT - 1,
        0xD3, 0x00, /* Display offset */
        0x40,       /* Start line 0 */
        0x8D, 0x14, /* Internal charge pump */
        0x20, 0x00, /* Horizontal addressing */
        0xA1, 0xC8, /* Segment and COM orientation */
        0xDA, 0x12, /* COM pin configuration for 128x64 */
        0x81, 0x7F, /* Contrast */
        0xD9, 0xF1, /* Pre-charge period */
        0xDB, 0x40, /* VCOMH level */
        0xA4, 0xA6, /* Show RAM in normal polarity */
        0x21, 0x00, OLED_WIDTH - 1,
        0x22, 0x00, OLED_HEIGHT / 8 - 1,
    };
    static const uint8_t display_on[] = {0x00, 0xAF};
    uint8_t frame[1 + OLED_WIDTH * OLED_HEIGHT / 8] = {0x40};
    draw_hello(frame + 1);

    ESP_RETURN_ON_ERROR(i2c_master_transmit(display, setup, sizeof(setup), I2C_TIMEOUT_MS),
                        TAG, "Display setup failed");
    ESP_RETURN_ON_ERROR(i2c_master_transmit(display, frame, sizeof(frame), I2C_TIMEOUT_MS),
                        TAG, "Frame transfer failed");
    return i2c_master_transmit(display, display_on, sizeof(display_on), I2C_TIMEOUT_MS);
}

esp_err_t ssd1306_show_hello(void)
{
    const gpio_config_t outputs = {
        .pin_bit_mask = (1ULL << OLED_RESET) | (1ULL << OLED_VEXT),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_set_level(OLED_RESET, 0), TAG, "Could not assert reset");
    ESP_RETURN_ON_ERROR(gpio_set_level(OLED_VEXT, 0), TAG, "Could not enable Vext");
    ESP_RETURN_ON_ERROR(gpio_config(&outputs), TAG, "Could not configure display GPIOs");
    vTaskDelay(pdMS_TO_TICKS(20) + 1);
    ESP_RETURN_ON_ERROR(gpio_set_level(OLED_RESET, 1), TAG, "Could not release reset");
    vTaskDelay(pdMS_TO_TICKS(20) + 1);

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = -1,
        .sda_io_num = OLED_SDA,
        .scl_io_num = OLED_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OLED_ADDRESS,
        .scl_speed_hz = 100000,
    };
    i2c_master_bus_handle_t bus;
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &bus), TAG, "Could not open I2C bus");

    i2c_master_dev_handle_t display;
    esp_err_t result = i2c_master_bus_add_device(bus, &device_config, &display);
    if (result == ESP_OK) {
        result = write_display(display);
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_bus_rm_device(display));
    }
    /* The controller retains its frame after the startup transfer. */
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_del_master_bus(bus));
    return result;
}
