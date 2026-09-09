idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES
                    esp_driver_gpio
                    esp_adc
                    esp_wifi
                    esp_event
                    esp_netif
                    esp_system
                    esp_http_client
                    esp-tls
                    nvs_flash
                    freertos
                    led_strip)
