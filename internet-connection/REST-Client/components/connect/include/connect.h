#ifndef _CONNECT_H_
#define _CONNECT_H_

#include "freertos/event_groups.h"

// Event group bits/flags that are set when an important Wi-Fi event has occurred.
#define WIFI_CONNECTED_BIT BIT0    // Connected to Wi-Fi.
#define IP_GOT_IP_BIT BIT1         // Got IPv4 Address of AP.
#define WIFI_DISCONNECTED_BIT BIT2 // Disconnected from Wi-Fi.
#define WIFI_FAIL_BIT BIT3         // Failed to connect to Wi-Fi after multiple attempts.

// Event group object for communication between Wi-FI event handler and application task.
extern EventGroupHandle_t wifi_evt_group;

/*  To distinguish between connection failures and actual esp_wifi_disconnect() call.
 *  Application must set dont_reconnect to true before calling esp_wifi_disconnect() */
extern bool dont_reconnect;

/**
 * @brief Initialize, configure, and start Wi-Fi station.
 * 
 *        Closely follows STA phases 1-3 in ESP32 Wi-Fi API Guide with exception of FreeRTOS event group.
 */
void wifi_init_sta(void);

#endif