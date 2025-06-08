idf_component_register(SRCS "main.c"
                      INCLUDE_DIRS "."
                      PRIV_REQUIRES esp_wifi
                                   esp_http_client
                                   esp_https_ota
                                   nvs_flash
                                   json
                                   esp_timer
                                   esp_event
                                   mbedtls
                                   esp_driver_spi)