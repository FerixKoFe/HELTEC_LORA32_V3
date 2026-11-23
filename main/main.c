#include "battery.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "radio_link.h"
#include "ssd1306.h"
#include "status_display.h"
#include "user_button.h"

#define USER_LED_GPIO GPIO_NUM_35

static const TickType_t startup_screen_ticks = pdMS_TO_TICKS(5000);
static const TickType_t activity_flash_ticks = pdMS_TO_TICKS(40);
static const TickType_t display_refresh_ticks = pdMS_TO_TICKS(30);

static const TickType_t battery_poll_ticks = pdMS_TO_TICKS(2000);
static const TickType_t display_poll_ticks = pdMS_TO_TICKS(10);

static QueueHandle_t display_modes;
static TaskHandle_t led_task_handle;

static void led_task(void *argument)
{
    (void)argument;

    const gpio_config_t led_config = {
        .pin_bit_mask = 1ULL << USER_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&led_config));
    ESP_ERROR_CHECK(gpio_set_level(USER_LED_GPIO, 0));

    for (;;) {
        uint32_t mode;
        if (xTaskNotifyWait(0, 0, &mode, portMAX_DELAY) == pdTRUE) {
            ESP_ERROR_CHECK(gpio_set_level(USER_LED_GPIO, mode == MODE_SLAVE));
        }
    }
}

static void button_task(void *argument)
{
    (void)argument;
    user_button_init();
    user_button_sync();
    system_mode_t mode = MODE_MASTER;
    for (;;) {
        if (user_button_poll_pressed()) {
            mode = mode == MODE_SCAN_LORA ? MODE_MASTER : (system_mode_t)(mode + 1);
            radio_link_set_mode(mode);
            xQueueOverwrite(display_modes, &mode);
            xTaskNotify(led_task_handle, (uint32_t)mode, eSetValueWithOverwrite);
        }
    }
}

static void display_task(void *argument)
{
    (void)argument;
    const esp_err_t startup_display = ssd1306_show_centered_text("MASTER", 1);
    ESP_ERROR_CHECK_WITHOUT_ABORT(startup_display);
    bool display_ready = startup_display == ESP_OK;
    const TickType_t startup_screen_since = xTaskGetTickCount();

    bool battery_ready = false;
    if (!battery_is_usb_only()) {
        const esp_err_t battery_result = battery_init();
        ESP_ERROR_CHECK_WITHOUT_ABORT(battery_result);
        battery_ready = battery_result == ESP_OK;
    }
    int battery_cv = 0;
    int shown_battery_cv = 0;
    TickType_t last_battery_sample = xTaskGetTickCount() - battery_poll_ticks;

    bool showing_status = false;
    system_mode_t shown_mode = MODE_MASTER;
    radio_link_status_t shown_link = {0};
    bool tx_active = false;
    bool rx_active = false;
    bool shown_tx = false;
    bool shown_rx = false;
    TickType_t last_tx = 0;
    TickType_t last_rx = 0;
    TickType_t last_display_attempt = xTaskGetTickCount() - display_refresh_ticks;

    while (true) {
        vTaskDelay(display_poll_ticks);
        system_mode_t mode;
        if (xQueuePeek(display_modes, &mode, 0) != pdPASS) {
            continue;
        }
        const TickType_t now = xTaskGetTickCount();
        const unsigned activity = radio_link_take_activity();
        if (activity & RADIO_LINK_ACTIVITY_TX) {
            last_tx = now;
            tx_active = true;
        }
        if (activity & RADIO_LINK_ACTIVITY_RX) {
            last_rx = now;
            rx_active = true;
        }
        tx_active = tx_active && now - last_tx < activity_flash_ticks;
        rx_active = rx_active && now - last_rx < activity_flash_ticks;

        const bool scanning = mode == MODE_SCAN_GFSK || mode == MODE_SCAN_LORA;
        if (showing_status || scanning || now - startup_screen_since >= startup_screen_ticks) {
            if (!scanning && battery_ready &&
                now - last_battery_sample >= battery_poll_ticks) {
                battery_cv = battery_read_centivolts();
                last_battery_sample = now;
            }
            const radio_link_status_t link = radio_link_get_status();
            const bool changed = !showing_status || mode != shown_mode ||
                link.connected != shown_link.connected || link.rssi_valid != shown_link.rssi_valid ||
                link.rssi_dbm_x2 != shown_link.rssi_dbm_x2 ||
                link.missed_packets != shown_link.missed_packets ||
                link.scan_state != shown_link.scan_state ||
                link.scan_percent != shown_link.scan_percent ||
                link.scan_seconds_left != shown_link.scan_seconds_left ||
                tx_active != shown_tx || rx_active != shown_rx ||
                (!scanning && battery_cv != shown_battery_cv);
            if (changed && now - last_display_attempt >= display_refresh_ticks) {
                last_display_attempt = now;
                if (!display_ready) {
                    ssd1306_deinit();
                    const esp_err_t init = ssd1306_init();
                    ESP_ERROR_CHECK_WITHOUT_ABORT(init);
                    if (init != ESP_OK) {
                        continue;
                    }
                }
                esp_err_t result = scanning ? status_display_show_scan(mode, link) :
                                   status_display_show_link(mode, link, tx_active, rx_active, battery_cv);
                if (result == ESP_OK && !display_ready) {
                    result = ssd1306_set_enabled(true);
                    display_ready = result == ESP_OK;
                }
                ESP_ERROR_CHECK_WITHOUT_ABORT(result);
                if (result != ESP_OK) {
                    continue;
                }
                showing_status = true;
                shown_mode = mode;
                shown_link = link;
                shown_tx = tx_active;
                shown_rx = rx_active;
                shown_battery_cv = battery_cv;
            }
        }
    }
}

void app_main(void)
{
    display_modes = xQueueCreate(1, sizeof(system_mode_t));
    ESP_ERROR_CHECK(display_modes ? ESP_OK : ESP_ERR_NO_MEM);
    const system_mode_t initial_mode = MODE_MASTER;
    xQueueOverwrite(display_modes, &initial_mode);

    ESP_ERROR_CHECK_WITHOUT_ABORT(radio_link_start());
    ESP_ERROR_CHECK(xTaskCreate(led_task, "status_led", 2048, NULL, 4, &led_task_handle) == pdPASS ?
                    ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(button_task, "user_button", 3072, NULL, 4, NULL) == pdPASS ?
                    ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(display_task, "oled", 4096, NULL, 3, NULL) == pdPASS ?
                    ESP_OK : ESP_ERR_NO_MEM);
}
