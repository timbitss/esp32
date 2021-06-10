#ifndef _CONNECT_H_
#define _CONNECT_H_

#include "freertos/event_groups.h"

// Event group bits/flags that are set when an important Wi-Fi event has occurred.
#define WIFI_CONNECTED_BIT BIT0 // Connected to Wi-FI.
#define IP_GOT_IP_BIT BIT1      // Got IPv4 Address of AP.
#define WIFI_FAIL_BIT BIT2      // Failed to connect to Wi-Fi.

// Event group object for communication between Wi-FI event handler and application task.
extern EventGroupHandle_t wifi_evt_group; 

/**
 * @brief Initialize, configure, and start Wi-Fi station.
 * 
 *        Closely follows STA phases 1-3 in ESP32 Wi-Fi API Guide with exception of FreeRTOS event group.
 */
void wifi_init_sta(void);

#endif