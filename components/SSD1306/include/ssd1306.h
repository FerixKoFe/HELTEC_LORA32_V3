#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64

/* The board has one display. Serialize access if using multiple tasks. */
esp_err_t ssd1306_init(void);
esp_err_t ssd1306_deinit(void);
esp_err_t ssd1306_set_enabled(bool enabled);
esp_err_t ssd1306_set_contrast(uint8_t contrast);
esp_err_t ssd1306_set_inverted(bool inverted);

/* Drawing updates RAM. Call ssd1306_refresh() to send RAM to the OLED. */
void ssd1306_clear(void);
esp_err_t ssd1306_set_pixel(int x, int y, bool on);
/* Copies a complete page-packed monochrome image into the framebuffer. */
esp_err_t ssd1306_set_frame(const uint8_t *pixels, size_t size);
esp_err_t ssd1306_refresh(void);
/* Draws into RAM. The font includes status labels, digits, spaces, ':' and '-'. */
esp_err_t ssd1306_draw_text(int x, int y, const char *text, uint8_t scale);
/* Clears RAM, draws centered text, and refreshes the display. */
esp_err_t ssd1306_show_centered_text(const char *text, uint8_t scale);
esp_err_t ssd1306_show_hello(void);
