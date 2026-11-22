#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ssd1306.h"

enum {
    OLED_WIDTH = SSD1306_WIDTH,
    OLED_HEIGHT = SSD1306_HEIGHT,
    OLED_SDA = GPIO_NUM_17,
    OLED_SCL = GPIO_NUM_18,
    OLED_RESET = GPIO_NUM_21,
    OLED_VEXT = GPIO_NUM_36,
    OLED_ADDRESS = 0x3C,
    I2C_TIMEOUT_MS = 250,
    GLYPH_WIDTH = 5,
    GLYPH_HEIGHT = 7,
    GLYPH_SPACING = 1,
};

static const char *const TAG = "ssd1306";
static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t display;
static uint8_t frame[1 + OLED_WIDTH * OLED_HEIGHT / 8] = {0x40};

/* Each byte is a column, with the least significant bit at the top. */
static const uint8_t font['m' - ' ' + 1][GLYPH_WIDTH] = {
    ['%' - ' '] = {0x63, 0x13, 0x08, 0x64, 0x63},
    ['-' - ' '] = {0x08, 0x08, 0x08, 0x08, 0x08},
    ['.' - ' '] = {0, 0, 0x60, 0, 0},
    [':' - ' '] = {0, 0, 0x24, 0, 0},
    ['0' - ' '] = {0x3E, 0x51, 0x49, 0x45, 0x3E},
    ['1' - ' '] = {0x00, 0x42, 0x7F, 0x40, 0x00},
    ['2' - ' '] = {0x42, 0x61, 0x51, 0x49, 0x46},
    ['3' - ' '] = {0x21, 0x41, 0x45, 0x4B, 0x31},
    ['4' - ' '] = {0x18, 0x14, 0x12, 0x7F, 0x10},
    ['5' - ' '] = {0x27, 0x45, 0x45, 0x45, 0x39},
    ['6' - ' '] = {0x3C, 0x4A, 0x49, 0x49, 0x30},
    ['7' - ' '] = {0x01, 0x71, 0x09, 0x05, 0x03},
    ['8' - ' '] = {0x36, 0x49, 0x49, 0x49, 0x36},
    ['9' - ' '] = {0x06, 0x49, 0x49, 0x29, 0x1E},
    ['A' - ' '] = {0x7E, 0x09, 0x09, 0x09, 0x7E},
    ['B' - ' '] = {0x7F, 0x49, 0x49, 0x49, 0x36},
    ['C' - ' '] = {0x3E, 0x41, 0x41, 0x41, 0x22},
    ['D' - ' '] = {0x7F, 0x41, 0x41, 0x22, 0x1C},
    ['E' - ' '] = {0x7F, 0x49, 0x49, 0x49, 0x41},
    ['F' - ' '] = {0x7F, 0x09, 0x09, 0x09, 0x01},
    ['G' - ' '] = {0x3E, 0x41, 0x49, 0x49, 0x7A},
    ['H' - ' '] = {0x7F, 0x08, 0x08, 0x08, 0x7F},
    ['I' - ' '] = {0x00, 0x41, 0x7F, 0x41, 0x00},
    ['K' - ' '] = {0x7F, 0x08, 0x14, 0x22, 0x41},
    ['L' - ' '] = {0x7F, 0x40, 0x40, 0x40, 0x40},
    ['M' - ' '] = {0x7F, 0x02, 0x0C, 0x02, 0x7F},
    ['N' - ' '] = {0x7F, 0x04, 0x08, 0x10, 0x7F},
    ['O' - ' '] = {0x3E, 0x41, 0x41, 0x41, 0x3E},
    ['P' - ' '] = {0x7F, 0x09, 0x09, 0x09, 0x06},
    ['R' - ' '] = {0x7F, 0x09, 0x19, 0x29, 0x46},
    ['S' - ' '] = {0x46, 0x49, 0x49, 0x49, 0x31},
    ['T' - ' '] = {0x01, 0x01, 0x7F, 0x01, 0x01},
    ['U' - ' '] = {0x3F, 0x40, 0x40, 0x40, 0x3F},
    ['V' - ' '] = {0x0F, 0x30, 0x40, 0x30, 0x0F},
    ['W' - ' '] = {0x3F, 0x40, 0x38, 0x40, 0x3F},
    ['Y' - ' '] = {0x07, 0x08, 0x70, 0x08, 0x07},
    ['d' - ' '] = {0x38, 0x44, 0x44, 0x48, 0x7F},
    ['m' - ' '] = {0x7C, 0x04, 0x18, 0x04, 0x78},
};

static int text_width(const char *text, uint8_t scale)
{
    if (!text || !*text || scale == 0 || scale > OLED_HEIGHT / GLYPH_HEIGHT) {
        return 0;
    }
    const size_t length = strlen(text);
    const int glyph_step = GLYPH_WIDTH + GLYPH_SPACING;
    if (length > (size_t)(OLED_WIDTH / scale + GLYPH_SPACING) / glyph_step) {
        return 0;
    }
    return ((int)length * glyph_step - GLYPH_SPACING) * scale;
}

esp_err_t ssd1306_draw_text(int left, int top, const char *text, uint8_t scale)
{
    const int width = text_width(text, scale);
    const int text_height = GLYPH_HEIGHT * scale;
    const int glyph_step = GLYPH_WIDTH + GLYPH_SPACING;
    if (!width || left < 0 || left > OLED_WIDTH - width ||
        top < 0 || top > OLED_HEIGHT - text_height) {
        return ESP_ERR_INVALID_ARG;
    }
    for (const char *character = text; *character; ++character) {
        if (*character < ' ' || *character > 'm') {
            return ESP_ERR_INVALID_ARG;
        }
    }

    for (int y = 0; y < text_height; ++y) {
        const int display_y = top + y;
        uint8_t *page = frame + 1 + (display_y / 8) * OLED_WIDTH;
        const uint8_t pixel_mask = 1U << (display_y % 8);
        for (int glyph = 0; text[glyph]; ++glyph) {
            const uint8_t *bitmap = font[text[glyph] - ' '];
            const int x = left + glyph * glyph_step * scale;
            for (int column = 0; column < GLYPH_WIDTH * scale; ++column) {
                if (bitmap[column / scale] & (1U << (y / scale))) {
                    page[x + column] |= pixel_mask;
                }
            }
        }
    }
    return ESP_OK;
}

esp_err_t ssd1306_show_centered_text(const char *text, uint8_t scale)
{
    const int width = text_width(text, scale);
    if (!width) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!display) {
        ESP_RETURN_ON_ERROR(ssd1306_init(), TAG, "OLED initialization failed");
    }
    ssd1306_clear();
    ESP_RETURN_ON_ERROR(ssd1306_draw_text((OLED_WIDTH - width) / 2,
                                        (OLED_HEIGHT - GLYPH_HEIGHT * scale) / 2,
                                        text, scale), TAG, "Text drawing failed");
    ESP_RETURN_ON_ERROR(ssd1306_refresh(), TAG, "Frame transfer failed");
    return ssd1306_set_enabled(true);
}

static esp_err_t send_bytes(const uint8_t *bytes, size_t size)
{
    if (!display) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit(display, bytes, size, I2C_TIMEOUT_MS);
}

esp_err_t ssd1306_set_enabled(bool enabled)
{
    const uint8_t command[] = {0x00, enabled ? 0xAF : 0xAE};
    return send_bytes(command, sizeof(command));
}

esp_err_t ssd1306_set_contrast(uint8_t contrast)
{
    const uint8_t command[] = {0x00, 0x81, contrast};
    return send_bytes(command, sizeof(command));
}

esp_err_t ssd1306_set_inverted(bool inverted)
{
    const uint8_t command[] = {0x00, inverted ? 0xA7 : 0xA6};
    return send_bytes(command, sizeof(command));
}

void ssd1306_clear(void)
{
    memset(frame + 1, 0, sizeof(frame) - 1);
}

esp_err_t ssd1306_set_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t *pixel_byte = frame + 1 + (y / 8) * OLED_WIDTH + x;
    const uint8_t mask = 1U << (y % 8);
    *pixel_byte = on ? *pixel_byte | mask : *pixel_byte & ~mask;
    return ESP_OK;
}

esp_err_t ssd1306_refresh(void)
{
    static const uint8_t address[] = {
        0x00, 0x21, 0x00, OLED_WIDTH - 1, 0x22, 0x00, OLED_HEIGHT / 8 - 1,
    };
    ESP_RETURN_ON_ERROR(send_bytes(address, sizeof(address)), TAG, "Address setup failed");
    return send_bytes(frame, sizeof(frame));
}

esp_err_t ssd1306_set_frame(const uint8_t *pixels, size_t size)
{
    if (!pixels) {
        return ESP_ERR_INVALID_ARG;
    }
    if (size != sizeof(frame) - 1) {
        return ESP_ERR_INVALID_SIZE;
    }
    memcpy(frame + 1, pixels, size);
    return ESP_OK;
}

static esp_err_t configure_display(void)
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
    };
    return send_bytes(setup, sizeof(setup));
}

esp_err_t ssd1306_init(void)
{
    if (bus) {
        return ESP_ERR_INVALID_STATE;
    }
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
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &bus), TAG, "Could not open I2C bus");

    esp_err_t result = i2c_master_bus_add_device(bus, &device_config, &display);
    if (result == ESP_OK) {
        result = configure_display();
    }
    if (result != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(ssd1306_deinit());
    }
    return result;
}

esp_err_t ssd1306_deinit(void)
{
    if (!bus) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t result = ESP_OK;
    if (display) {
        result = ssd1306_set_enabled(false);
        ESP_RETURN_ON_ERROR(i2c_master_bus_rm_device(display), TAG, "Could not remove OLED");
        display = NULL;
    }
    ESP_RETURN_ON_ERROR(i2c_del_master_bus(bus), TAG, "Could not release I2C bus");
    bus = NULL;
    return result;
}

esp_err_t ssd1306_show_hello(void)
{
    return ssd1306_show_centered_text("HELLO", 4);
}
