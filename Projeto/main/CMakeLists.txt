idf_component_register(
    SRCS "main.c" "current_monitor.c" "mqtt_bridge.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        mqtt
        esp_adc
        esp_driver_spi
        esp_driver_gpio
        esp_driver_uart
        espressif__esp-dsp
)