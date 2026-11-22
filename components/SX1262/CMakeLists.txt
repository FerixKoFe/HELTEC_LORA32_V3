idf_component_register(SRCS "sx1262.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_driver_gpio esp_driver_spi esp_timer)
