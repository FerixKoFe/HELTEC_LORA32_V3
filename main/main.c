#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_rom_sys.h"
#include "sdkconfig.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ssd1306.h"
#include "radio_link.h"

#define USER_BUTTON_GPIO GPIO_NUM_0
#define USER_LED_GPIO GPIO_NUM_35
#define BATTERY_CTRL_GPIO GPIO_NUM_37
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_0 /* GPIO1 */
#define BATTERY_ADC_ATTEN ADC_ATTEN_DB_0

#ifdef CONFIG_HELTEC_BATTERY_ADC_ACTIVE_HIGH
#define BATTERY_CTRL_ON 1
#else
#define BATTERY_CTRL_ON 0
#endif

static const char *const TAG = "main";
static const TickType_t button_poll_ticks = pdMS_TO_TICKS(10);
static const TickType_t button_debounce_ticks = pdMS_TO_TICKS(30);
static const TickType_t startup_screen_ticks = pdMS_TO_TICKS(5000);
static const TickType_t activity_flash_ticks = pdMS_TO_TICKS(40);

static const TickType_t battery_poll_ticks = pdMS_TO_TICKS(2000);

static adc_oneshot_unit_handle_t battery_adc;
static adc_cali_handle_t battery_calibration;

static esp_err_t init_battery_adc(void)
{
    ESP_RETURN_ON_ERROR(gpio_set_level(BATTERY_CTRL_GPIO, !BATTERY_CTRL_ON),
                        TAG, "Battery control level failed");
    ESP_RETURN_ON_ERROR(gpio_set_direction(BATTERY_CTRL_GPIO, GPIO_MODE_OUTPUT),
                        TAG, "Battery control setup failed");

    const adc_oneshot_unit_init_cfg_t unit = {.unit_id = ADC_UNIT_1};
    const adc_oneshot_chan_cfg_t channel = {
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    const adc_cali_curve_fitting_config_t calibration = {
        .unit_id = ADC_UNIT_1,
        .chan = BATTERY_ADC_CHANNEL,
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit, &battery_adc),
                        TAG, "Battery ADC setup failed");
    esp_err_t err = adc_oneshot_config_channel(battery_adc, BATTERY_ADC_CHANNEL, &channel);
    if (err == ESP_OK) {
        err = adc_cali_create_scheme_curve_fitting(&calibration, &battery_calibration);
    }
    if (err != ESP_OK) {
        adc_oneshot_del_unit(battery_adc);
        battery_adc = NULL;
    }
    return err;
}

static int read_battery_centivolts(void)
{
    enum { sample_count = 8 };
    int sum_mv = 0;
    esp_err_t err = gpio_set_level(BATTERY_CTRL_GPIO, BATTERY_CTRL_ON);
    esp_rom_delay_us(1000); /* Let the switched divider settle. */
    for (int i = 0; i < sample_count && err == ESP_OK; ++i) {
        int raw;
        int mv;
        err = adc_oneshot_read(battery_adc, BATTERY_ADC_CHANNEL, &raw);
        if (err == ESP_OK) {
            err = adc_cali_raw_to_voltage(battery_calibration, raw, &mv);
        }
        if (err == ESP_OK) {
            sum_mv += mv;
        }
    }
    const esp_err_t disable_err = gpio_set_level(BATTERY_CTRL_GPIO, !BATTERY_CTRL_ON);
    if (err != ESP_OK || disable_err != ESP_OK) {
        return 0;
    }

    /* Convert the averaged ADC reading through the 390 kΩ / 100 kΩ divider. */
    const int divisor = sample_count * 100;
    const int centivolts = (sum_mv * 49 + divisor / 2) / divisor;
    return centivolts >= 250 ? centivolts : 0;
}

static int battery_percent(int centivolts)
{
    /* Approximate single-cell LiPo voltage-to-charge curve. */
    static const struct {
        int centivolts;
        int percent;
    } curve[] = {
        {330, 0}, {360, 10}, {370, 20}, {375, 35}, {380, 50},
        {390, 70}, {400, 85}, {410, 95}, {420, 100},
    };

    if (centivolts <= curve[0].centivolts) {
        return 0;
    }
    for (size_t i = 1; i < sizeof(curve) / sizeof(curve[0]); ++i) {
        if (centivolts <= curve[i].centivolts) {
            const int span = curve[i].centivolts - curve[i - 1].centivolts;
            const int offset = centivolts - curve[i - 1].centivolts;
            const int rise = curve[i].percent - curve[i - 1].percent;
            return curve[i - 1].percent + (offset * rise + span / 2) / span;
        }
    }
    return 100;
}

static int rssi_percent(int rssi_dbm_x2)
{
    /* UI scale from -120 dBm (0%) to -50 dBm (100%). */
    const int offset = rssi_dbm_x2 + 240;
    if (offset <= 0) {
        return 0;
    }
    if (offset >= 140) {
        return 100;
    }
    return (offset * 100 + 70) / 140;
}

static void draw_activity_box(int left, bool active)
{
    /* Matches a 5x7 font character at scale 2. */
    for (int y = 0; y < 14; ++y) {
        for (int x = 0; x < 10; ++x) {
            if (active || x == 0 || x == 9 || y == 0 || y == 13) {
                ssd1306_set_pixel(left + x, 2 + y, true);
            }
        }
    }
}

static esp_err_t show_status(system_mode_t mode, radio_link_status_t link,
                             bool tx_active, bool rx_active, int battery_cv)
{
    char rssi[24] = "RSSI: NA";
    if (link.connected) {
        /* Round the half-dBm reading to a whole dBm for the display. */
        const int dbm = (link.rssi_dbm_x2 - 1) / 2;
        snprintf(rssi, sizeof(rssi), "RSSI: %d dBm %d%%", dbm, rssi_percent(link.rssi_dbm_x2));
    }
    ssd1306_clear();
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 2, mode == MODE_MASTER ? "MASTER" : "SLAVE", 2),
                        TAG, "Mode label failed");
    draw_activity_box(SSD1306_WIDTH - 24, tx_active);
    draw_activity_box(SSD1306_WIDTH - 12, rx_active);
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 21, rssi, 1), TAG, "RSSI label failed");
    char missed[24];
    snprintf(missed, sizeof(missed), "MISS: %" PRIu32, link.missed_packets);
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 30, missed, 1), TAG, "Missing packet label failed");
    char battery[24] = "BAT: NA";
    if (battery_cv > 0) {
        snprintf(battery, sizeof(battery), "BAT: %d.%02d V %d%%",
                 battery_cv / 100, battery_cv % 100, battery_percent(battery_cv));
    }
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 39, battery, 1), TAG, "Battery label failed");
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 50, link.connected ? "LINK: UP" : "LINK: DOWN", 2),
                        TAG, "Link label failed");
    return ssd1306_refresh();
}

static esp_err_t show_scan(system_mode_t mode, radio_link_status_t link)
{
    char percent[8];
    char time[18] = "STARTING";
    snprintf(percent, sizeof(percent), "%u%%", (unsigned)link.scan_percent);
    if (link.scan_state != SCAN_PENDING) {
        snprintf(time, sizeof(time), "LEFT: %u S", (unsigned)link.scan_seconds_left);
    }
    ssd1306_clear();
    const char *title = mode == MODE_SCAN_GFSK ? "GFSK SCAN" : "LORA SCAN";
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 2, title, 2), TAG, "Scan title failed");
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 23, percent, 2), TAG, "Scan progress failed");
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 43, time, 1), TAG, "Scan time failed");
    if (link.scan_state == SCAN_DONE || link.scan_state == SCAN_ERROR) {
        const char *result = link.scan_state == SCAN_DONE ? "DONE" : "SCAN ERROR";
        ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 54, result, 1), TAG, "Scan result failed");
    }
    return ssd1306_refresh();
}

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

    bool stable_pressed = gpio_get_level(USER_BUTTON_GPIO) == 0;
    bool last_sample = stable_pressed;
    TickType_t last_change = xTaskGetTickCount();
    system_mode_t mode = MODE_MASTER;
    ESP_ERROR_CHECK_WITHOUT_ABORT(ssd1306_show_centered_text("MASTER", 3));
    const TickType_t startup_screen_since = xTaskGetTickCount();
    ESP_ERROR_CHECK_WITHOUT_ABORT(radio_link_start());

    const esp_err_t battery_init = init_battery_adc();
    ESP_ERROR_CHECK_WITHOUT_ABORT(battery_init);
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

    while (true) {
        vTaskDelay(button_poll_ticks);
        const bool pressed = gpio_get_level(USER_BUTTON_GPIO) == 0;
        const TickType_t now = xTaskGetTickCount();

        if (pressed != last_sample) {
            last_sample = pressed;
            last_change = now;
        }

        if (pressed != stable_pressed && now - last_change >= button_debounce_ticks) {
            stable_pressed = pressed;
            if (stable_pressed) {
                mode = mode == MODE_SCAN_LORA ? MODE_MASTER : (system_mode_t)(mode + 1);
                ESP_ERROR_CHECK(gpio_set_level(USER_LED_GPIO, mode == MODE_SLAVE));
                radio_link_set_mode(mode);
            }
        }
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
            if (!scanning && battery_init == ESP_OK &&
                now - last_battery_sample >= battery_poll_ticks) {
                battery_cv = read_battery_centivolts();
                last_battery_sample = now;
            }
            const radio_link_status_t link = radio_link_get_status();
            if (!showing_status || mode != shown_mode || link.connected != shown_link.connected ||
                link.rssi_dbm_x2 != shown_link.rssi_dbm_x2 ||
                link.missed_packets != shown_link.missed_packets ||
                link.scan_state != shown_link.scan_state ||
                link.scan_percent != shown_link.scan_percent ||
                link.scan_seconds_left != shown_link.scan_seconds_left ||
                tx_active != shown_tx || rx_active != shown_rx ||
                (!scanning && battery_cv != shown_battery_cv)) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(scanning ? show_scan(mode, link) :
                                               show_status(mode, link, tx_active, rx_active, battery_cv));
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
