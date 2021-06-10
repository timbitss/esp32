/**
 * @file main.c
 * @author Timothy Nguyen
 * @brief REST API example.
 * @version 0.1
 * @date 2021-06-07
 */

#include <stdio.h>
#include "freeRTOS/FreeRTOS.h"
#include "freeRTOS/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "rest.h"
#include "connect.h"

// URL to perform GET request on.
#define REQUEST_URL "https://quotes.rest/qod?language=en"

// Tags for logging data and information
#define MAIN_TASK "MAIN_TASK"
#define APP_TASK "APP_TASK"

/**
 * @brief Application task for Wi-Fi applications.
 * 
 * @param param Optional parameters as void ptr.
 */
static void app_task(void *param)
{
    while(1)
    {
        EventBits_t bits = xEventGroupWaitBits(wifi_evt_group, 
                            WIFI_CONNECTED_BIT | IP_GOT_IP_BIT | WIFI_FAIL_BIT, // Bits to wait for.
                            pdTRUE,                // Clear bits on exit if wait condition was met.
                            pdFALSE,               // Wait for any bit to be sit in event group.
                            pdMS_TO_TICKS(10000)); // Timeout period of 10 s.

        if(bits & WIFI_CONNECTED_BIT)
        {
            ESP_LOGI(APP_TASK, "ESP32 Station connected to AP");
        }
        else if(bits & IP_GOT_IP_BIT)
        {
            ESP_LOGI(APP_TASK, "Got IPv4 address of AP.");
            json_parse_t payload = {
                .endpoint = REQUEST_URL,
                .parse_json = 0 // TODO: Create Application-specific JSON parser.
            };
            rest_get(&payload);  
            // TODO: Disconnect Wi-Fi once payload is received with appropriate Wi-Fi event handling.
        }
        else if (bits & WIFI_FAIL_BIT)
        {
            ESP_LOGE(APP_TASK, "Could not connect to Wi-Fi.");
        }
        else
        {
            ESP_LOGE(APP_TASK, "Task timed out.");
        }
    }
}    

void app_main()
{
    // Initialize NVS, required for Wi-Fi driver.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGI(MAIN_TASK, "Erasing contents of NVS flash partition");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Wi-Fi station.
    wifi_init_sta();

    // Create application task.
    xTaskCreatePinnedToCore(app_task, "APP TASK", 4 * 1024, NULL, 5, NULL, APP_CPU_NUM);
}