#include "battery.h"

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "sdkconfig.h"

#define BATTERY_CTRL_GPIO GPIO_NUM_37
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_0 /* GPIO1 */
#define BATTERY_ADC_ATTEN ADC_ATTEN_DB_0

#ifdef CONFIG_HELTEC_BATTERY_ADC_ACTIVE_HIGH
#define BATTERY_CTRL_ON 1
#else
#define BATTERY_CTRL_ON 0
#endif

#ifdef CONFIG_HELTEC_USB_ONLY
static const bool usb_only = true;
#else
static const bool usb_only = false;
#endif

static const char *const TAG = "main";

static adc_oneshot_unit_handle_t battery_adc;
static adc_cali_handle_t battery_calibration;

bool battery_is_usb_only(void)
{
    return usb_only;
}

esp_err_t battery_init(void)
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

int battery_read_centivolts(void)
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

int battery_percent(int centivolts)
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
