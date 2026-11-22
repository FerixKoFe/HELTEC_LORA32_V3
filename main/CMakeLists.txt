idf_component_register(SRCS "main.c" "radio_link.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_adc esp_driver_gpio esp_hw_support esp_timer SSD1306 SX1262)
