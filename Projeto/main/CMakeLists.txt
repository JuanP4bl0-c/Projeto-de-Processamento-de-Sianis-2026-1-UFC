idf_component_register(SRCS "main.c"
                            "current_monitor.c"
                            "wifi_manager.c"
                            "mqtt_bridge.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_timer esp_adc freertos esp_wifi esp_event esp_netif nvs_flash mqtt)
