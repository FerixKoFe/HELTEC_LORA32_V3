#pragma once

#include "radio_link.h"

esp_err_t status_display_show_link(system_mode_t mode, radio_link_status_t link);
esp_err_t status_display_show_scan(system_mode_t mode, radio_link_status_t link);
