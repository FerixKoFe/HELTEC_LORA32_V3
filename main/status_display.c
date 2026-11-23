#include "status_display.h"
#include "radio_link_private.h"

#include <inttypes.h>
#include <stdio.h>

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

esp_err_t status_display_show_link(system_mode_t mode, radio_link_status_t link)
{
    char rssi[16] = "NA";
    if (link.connected && link.rssi_valid) {
        /* Round the half-dBm reading to a whole dBm for the display. */
        const int dbm = (link.rssi_dbm_x2 - 1) / 2;
        snprintf(rssi, sizeof(rssi), "%d  %d%%", dbm, rssi_percent(link.rssi_dbm_x2));
    }
    ssd1306_clear();
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 4, radio_link_mode_name(mode), 1),
                        TAG, "Mode label failed");
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 16, link.connected ? "LINK: UP" : "LINK: DOWN", 1),
                        TAG, "Link label failed");
    char value[16];
    if (link.connected) {
        ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 34, "RSSI dBm", 1), TAG, "RSSI label failed");
        ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 45, rssi, 1), TAG, "RSSI value failed");
    } else {
        snprintf(value, sizeof(value), "%" PRIu32, link.down_missed_packets);
        ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 34, "DOWN MISS", 1), TAG, "Down miss label failed");
        ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 45, value, 1), TAG, "Down miss value failed");
    }
    snprintf(value, sizeof(value), "%" PRIu32, link.missed_packets);
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 65, "MISS", 1), TAG, "Bad packet label failed");
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 76, value, 1), TAG, "Bad packet value failed");
    snprintf(value, sizeof(value), "%" PRIu32, link.link_drops);
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 96, "DROPS", 1), TAG, "Link drops label failed");
    ESP_RETURN_ON_ERROR(ssd1306_draw_text(2, 107, value, 1), TAG, "Link drops value failed");
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
