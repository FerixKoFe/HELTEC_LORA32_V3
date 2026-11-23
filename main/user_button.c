#include "user_button.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define USER_BUTTON_GPIO GPIO_NUM_0

static const TickType_t button_poll_ticks = pdMS_TO_TICKS(10);
static const TickType_t button_debounce_ticks = pdMS_TO_TICKS(30);
static bool stable_pressed;
static bool last_sample;
static TickType_t last_change;

void user_button_init(void)
{
    const gpio_config_t button_config = {
        .pin_bit_mask = 1ULL << USER_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&button_config));
}

bool user_button_poll_pressed(void)
{
    vTaskDelay(button_poll_ticks);
    const bool pressed = gpio_get_level(USER_BUTTON_GPIO) == 0;
    const TickType_t now = xTaskGetTickCount();

    if (pressed != last_sample) {
        last_sample = pressed;
        last_change = now;
    }
    if (pressed != stable_pressed && now - last_change >= button_debounce_ticks) {
        stable_pressed = pressed;
        return stable_pressed;
    }
    return false;
}

void user_button_sync(void)
{
    stable_pressed = gpio_get_level(USER_BUTTON_GPIO) == 0;
    last_sample = stable_pressed;
    last_change = xTaskGetTickCount();
}
