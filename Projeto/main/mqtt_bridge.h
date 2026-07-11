#pragma once

#include <stdbool.h>

#include "current_monitor.h"

// ======================= CONFIGURAÇÕES =========================
// #define WIFI_SSID       "UFC_B4_SL3_2"
#define WIFI_SSID       "Visitantes_Adm"
#define WIFI_PASSWORD   ""
#define WIFI_AUTH_MODE  WIFI_AUTH_OPEN
#define MQTT_BROKER_URI "mqtt://10.10.220.31:1883"
// #define MQTT_BROKER_URI "mqtt://10.0.105.207:1883"
// // Precisa bater com TOPICO em Grafico.py


// #define WIFI_SSID "brisa-2504280"
// #define WIFI_PASSWORD "eubcidpn"
// #define MQTT_BROKER_URI "mqtt://192.168.0.6:1883"
// #define WIFI_AUTH_MODE WIFI_AUTH_WPA2_PSK

// =================================================================
#define MQTT_TOPIC_SIGNAL "esp32/sinal"

// Inicializa NVS + WiFi (STA) + cliente MQTT. Chame uma única vez no app_main().
void mqtt_bridge_init(void);

// Retorna true se o cliente MQTT está conectado ao broker no momento.
bool mqtt_bridge_is_connected(void);

// Publica o relatório de análise (RMS, frequência da rede e harmônicas)
// em JSON, no formato consumido por Grafico.py:
// {"rms_mv":.., "mains_hz":.., "rms_raw":.., "harmonicas":[{"f":..,"a":..}, ...]}
void mqtt_bridge_publish_report(const current_monitor_report_t *report);