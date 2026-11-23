#pragma once

#include <stdbool.h>

#include "esp_err.h"

bool battery_is_usb_only(void);
esp_err_t battery_init(void);
int battery_read_centivolts(void);
int battery_percent(int centivolts);
