/**
 * @file main.c
 * @author Timothy Nguyen
 * @brief Scan and record available APs 
 * @version 0.1
 * @date 2021-06-07
 */

#include <stdio.h>
#include "freeRTOS/FreeRTOS.h"
#include "freeRTOS/task.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_netif.h"

#define TAG "APPMAIN" // Tag for logging data

#define MAX_APS 20 // Maximum number of access points that ap_records can hold

static esp_err_t event_handler(void *ctx, system_event_t *evt)
{
    return ESP_OK;
}

/**
 * @brief Configure and initialize WiFi station
 */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init()); // Initialize default NVS partition in flash
    ESP_ERROR_CHECK(esp_netif_init()); // Initialize underlying TCP/IP stack
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA)); // set ESP32 in station mode
    ESP_ERROR_CHECK(esp_wifi_start());
}

/**
 * @brief Get authentication mode as a string from lookup table
 * 
 * @param auth_mode Authentication mode as enumerator
 * @return char*    Authentication mode as string
 */
static char *getAuthModeName(wifi_auth_mode_t auth_mode)
{
  static char *names[] = {"OPEN", "WEP", "WPA PSK", "WPA2 PSK", "WPA WPA2 PSK", "MAX"};
  return names[auth_mode];
}

void app_main()
{
    ESP_LOGI(TAG, "Initializing Wi-Fi");
    wifi_init();

    ESP_LOGI(TAG, "Scanning for APs");
    wifi_scan_config_t scan_config = {
        .ssid = 0,    // search for all SSIDs
        .bssid = 0,   // search for all BSSIDs/MACs
        .channel = 0, // search all channels
        .show_hidden = true};
    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true)); // blocking

    uint16_t num_aps = MAX_APS;
    wifi_ap_record_t ap_records[MAX_APS];
    /* As input param, num_aps stores max AP number ap_records can hold. 
       As output param, num_aps receives the actual AP number stored in records. */
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&num_aps, ap_records));
    ESP_LOGI(TAG, "Number of APs found: %u", num_aps);

    // Print out some info about each AP 
    printf("Found %d access points:\n", num_aps);
    printf("\n");
    printf("               SSID              | Channel | RSSI |   Auth Mode \n");
    printf("----------------------------------------------------------------\n");
    for (int i = 0; i < num_aps; i++)
    {
        printf("%32s | %7d | %4d | %12s\n", 
        (char *)ap_records[i].ssid, // SSID aka network name
        ap_records[i].primary, // primary channel of AP
        ap_records[i].rssi, // signal strength
        getAuthModeName(ap_records[i].authmode)); // authentication mode
    }
    printf("----------------------------------------------------------------\n");
}