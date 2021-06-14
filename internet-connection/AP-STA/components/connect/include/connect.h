#ifndef _CONNECT_H_
#define _CONNECT_H_

#include "freertos/event_groups.h"
#include "freertos/semphr.h"

// Storage of target AP's SSID and password in NVS flash:
#define WIFI_NAMESPACE "WIFI_CREDS"       // NVS namespace associated with SSID and Password.
#define SSID_KEY "SSID"                   // Key for SSID item.
#define PWD_KEY "PASSWORD"                // Key for password item.

/* Event group bits/flags that are set when an important Wi-Fi event has occurred.
 * Handle events in user application */
#define WIFI_CONNECTED_BIT BIT0    // Connected to Wi-Fi.
#define IP_GOT_IP_BIT BIT1         // Got IPv4 Address of AP.
#define AP_STARTED_BIT BIT2        // ESP32 AP has started. Stations are now able to connect to ESP32.
#define AP_STA_CONNECTED_BIT BIT3  // Station has connected to ESP32 AP.
#define WIFI_DISCONNECTED_BIT BIT4 // Successful esp_wifi_disconnect() call.
#define WIFI_FAIL_BIT BIT5         // Failed to connect to Wi-Fi after multiple attempts.

// Event group object for communication between Wi-FI callback and application task.
extern EventGroupHandle_t wifi_evt_group;

// Semaphore to retry obtaining wifi credentials from NVS flash.
extern SemaphoreHandle_t test_wifi_creds_sem;

/**
 * @brief Connect to Wi-Fi.
 * 
 *        Starts connect_wifi task.
 */
void connect_to_wifi();

/**
 * @brief Disconnect from Wi-Fi and free up resources.
 */
void disconnect_from_wifi();

#endif