#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_err.h"
#include "esp_wifi.h"
#include <stdbool.h>

void wifi_manager_init(const char *ssid, const char *password, wifi_auth_mode_t authmode);
bool wifi_manager_is_connected(void);

#endif // WIFI_MANAGER_H
