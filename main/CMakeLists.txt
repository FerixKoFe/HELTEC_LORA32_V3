idf_component_register(SRCS "status_display.c" "battery.c" "main.c" "battery.c" "user_button.c" "status_display.c"
                         "radio_link.c" "radio_scan.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_adc esp_driver_gpio esp_hw_support esp_timer SSD1306 SX1262)
