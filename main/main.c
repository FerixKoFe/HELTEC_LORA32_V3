#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ssd1306.h"

#define USER_BUTTON_GPIO GPIO_NUM_0
#define USER_LED_GPIO GPIO_NUM_35

static const TickType_t button_poll_ticks = pdMS_TO_TICKS(10);
static const TickType_t button_debounce_ticks = pdMS_TO_TICKS(30);

void app_main(void)
{
    const gpio_config_t button_config = {
        .pin_bit_mask = 1ULL << USER_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&button_config));

    const gpio_config_t led_config = {
        .pin_bit_mask = 1ULL << USER_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&led_config));
    ESP_ERROR_CHECK(gpio_set_level(USER_LED_GPIO, 0));

    ESP_ERROR_CHECK_WITHOUT_ABORT(ssd1306_show_hello());

    bool led_on = false;
    bool stable_pressed = gpio_get_level(USER_BUTTON_GPIO) == 0;
    bool last_sample = stable_pressed;
    TickType_t last_change = xTaskGetTickCount();

    while (true) {
        vTaskDelay(button_poll_ticks);
        const bool pressed = gpio_get_level(USER_BUTTON_GPIO) == 0;
        const TickType_t now = xTaskGetTickCount();

        if (pressed != last_sample) {
            last_sample = pressed;
            last_change = now;
        }

        if (pressed == stable_pressed || now - last_change < button_debounce_ticks) {
            continue;
        }

        stable_pressed = pressed;
        if (stable_pressed) {
            led_on = !led_on;
            ESP_ERROR_CHECK(gpio_set_level(USER_LED_GPIO, led_on));
        }
    }
}
