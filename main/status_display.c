#include "status_display.h"

#include <inttypes.h>
#include <stdio.h>

#include "battery.h"
#include "esp_check.h"
#include "ssd1306.h"

static const char *const TAG = "main";

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
                ssd1306_set_pixel(left + x, 17 + y, true);
            }
        }
    }
}

esp_err_t status_display_show_link(system_mode_t mode, radio_link_status_t link,
                             bool tx_active, bool rx_active, int battery_cv)
{
    char rssi[16] = "NA";
    if (link.connected && link.rssi_valid) {
        /* Round the half-dBm reading to a whole dBm for the display. */
        const int dbm = (link.rssi_dbm_x2 - 1) / 2;
        snprintf(rssi, sizeof(rssi), "%d  %d%%", dbm, rssi_percent(link.rssi_dbm_x2));
    }
    ssd1306_clear();
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 4, mode == MODE_MASTER ? "MASTER" : "SLAVE", 1),
                        TAG, "Mode label failed");
    draw_activity_box(2, tx_active);
    draw_activity_box(34, rx_active);
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(15, 21, "TX", 1), TAG, "TX label failed");
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(47, 21, "RX", 1), TAG, "RX label failed");
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 39, "RSSI dBm", 1), TAG, "RSSI label failed");
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 50, rssi, 1), TAG, "RSSI value failed");
    char missed[16];
    snprintf(missed, sizeof(missed), "%" PRIu32, link.missed_packets);
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 66, "MISS", 1), TAG, "Missing packet label failed");
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 76, missed, 1), TAG, "Missing packet value failed");
    char battery[16] = "NA";
    if (battery_is_usb_only()) {
        snprintf(battery, sizeof(battery), "USB ONLY");
    } else if (battery_cv > 0) {
        snprintf(battery, sizeof(battery), "%d.%02dV %d%%",
                 battery_cv / 100, battery_cv % 100, battery_percent(battery_cv));
    }
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 91, "BAT", 1), TAG, "Battery label failed");
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 102, battery, 1), TAG, "Battery value failed");
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 118, link.connected ? "LINK: UP" : "LINK: DOWN", 1),
                        TAG, "Link label failed");
    return ssd1306_refresh();
}

esp_err_t status_display_show_scan(system_mode_t mode, radio_link_status_t link)
{
    char percent[8];
    char time[18] = "STARTING";
    snprintf(percent, sizeof(percent), "%u%%", (unsigned)link.scan_percent);
    if (link.scan_state != SCAN_PENDING) {
        snprintf(time, sizeof(time), "LEFT: %u S", (unsigned)link.scan_seconds_left);
    }
    ssd1306_clear();
    const char *title = mode == MODE_SCAN_GFSK ? "GFSK" : "LORA";
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(9, 8, title, 2), TAG, "Scan mode failed");
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(20, 29, "SCAN", 1), TAG, "Scan title failed");
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(9, 49, percent, 2), TAG, "Scan progress failed");
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 76, time, 1), TAG, "Scan time failed");
    if (link.scan_state == SCAN_DONE || link.scan_state == SCAN_ERROR) {
        const char *result = link.scan_state == SCAN_DONE ? "DONE" : "SCAN ERROR";
        ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 94, result, 1), TAG, "Scan result failed");
    }
    return ssd1306_refresh();
}
