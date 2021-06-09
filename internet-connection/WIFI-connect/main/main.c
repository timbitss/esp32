/**
 * @file main.c
 * @author Timothy Nguyen
 * @brief Example of connecting to Wi-Fi
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

// Tags for logging data and information
#define WIFI_INIT "WIFI_INIT"
#define APP_MAIN "APP_MAIN"
#define EVENT_HANDLER "EVENT_HANDLER"
#define APP_TASK "APP_TASK"

/* SSID and PWD of target AP.
 * Configure definitions in menuconfig. */
#define SSID CONFIG_WIFI_SSID
#define PWD CONFIG_WIFI_PASSWORD

// FreeRTOS event group bits/flags for communication between event handler and application task.
#define WIFI_CONNECTED_BIT BIT0
#define IP_GOT_IP_BIT BIT1
#define WIFI_FAIL_BIT BIT2

// Maximum number of attempts to connect to Wi-Fi before giving up.
#define MAX_RETRY_ATTEMPTS 10

// To store IPV4 address, netmask, and gateway address.
esp_netif_ip_info_t ipv4_info;

// Event group object for communication between event handler and application task.
EventGroupHandle_t evt_group; 

/**
 * @brief Event handler/call back registered for WIFI and IP events.
 * 
 * @param arg Optional arguments as void ptr.
 * @param event_base Base ID (like last name)
 * @param event_id   Event ID (like first name)
 * @param event_data Data passed along with event as void ptr.
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    static uint8_t attempts = 0;
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        esp_wifi_connect(); 
        break;
    case WIFI_EVENT_STA_CONNECTED:
        xEventGroupSetBits(evt_group, WIFI_CONNECTED_BIT);
        attempts = 0;
        break;
    case IP_EVENT_STA_GOT_IP:
        ipv4_info = ((ip_event_got_ip_t*)event_data)->ip_info;
        xEventGroupSetBits(evt_group, IP_GOT_IP_BIT);
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        attempts++;
        if (attempts <= MAX_RETRY_ATTEMPTS)
        {
            ESP_LOGI(EVENT_HANDLER, "Retrying connection. Attempt #%u", attempts);
            esp_wifi_connect(); // Retry connection.
        }
        else
        {
            xEventGroupSetBits(evt_group, WIFI_FAIL_BIT);
        }
        break;
    default:
        ESP_LOGI(EVENT_HANDLER, "Could not identify event.");
        break;
    }
}

/**
 * @brief Print information related to important events.
 * 
 * @param param 
 */
static void app_task(void *param)
{
    while(1)
    {
        EventBits_t bits = xEventGroupWaitBits(evt_group, 
                            WIFI_CONNECTED_BIT | IP_GOT_IP_BIT | WIFI_FAIL_BIT, // Bits to wait for.
                            pdTRUE,          // Clear bits on exit.
                            pdFALSE,         // Wait for any bit to be sit in event group.
                            portMAX_DELAY ); // Wait forever.

        if(bits & WIFI_CONNECTED_BIT)
        {
            ESP_LOGI(APP_TASK, "ESP32 Station connected to AP, SSID: %s PWD: %s", SSID, PWD);
        }
        else if(bits & IP_GOT_IP_BIT)
        {
            // Defined by the ISO C standard, adjacent string literals are combined into a single one.
            ESP_LOGI(APP_TASK, "IPv4 address: " IPSTR, IP2STR(&ipv4_info.ip)); 
        }
        else if (bits & WIFI_FAIL_BIT)
        {
            ESP_LOGE(APP_TASK, "Could not connect to Wi-Fi.");
        }
        else
        {
            ESP_LOGE(APP_TASK, "Could not identify event from event group.");
        }
    }
}    


/**
 * @brief Initialize, configure, and start Wi-Fi.
 * 
 *        Closely follows STA phases 1-3 in ESP32 Wi-Fi API Guide.
 */
static void wifi_init_sta(void)
{
    // Create an LwIP core task and initialize LwIP-related work.
    ESP_ERROR_CHECK(esp_netif_init());
    // Create the default event loop/task to handle system events (e.g. Wi-Fi events).
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Register event handlers to default event loop.
    esp_event_handler_instance_t instance_any_wifi_evt;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        // Execute handler if any Wi-fi event is posted.
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_wifi_evt));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        // Execute handler if station received IP from connected AP.
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    // Create default WIFI STA instance.
    esp_netif_t *esp_netif_inst = esp_netif_create_default_wifi_sta();
    if (esp_netif_inst == NULL)
    {
        ESP_LOGE(WIFI_INIT, "Could not create default Wi-Fi station instance.");
    }

    /* Create the Wi-Fi driver task and initialize the Wi-Fi driver with default configurations.
     * NOTE: Function must be called before all other Wi-Fi API can be called. */
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));

    /* If the Wi-Fi NVS flash is enabled by menuconfig, all Wi-Fi configurations are stored in flash.
     * Therefore, you only need to call esp_wifi_get_xxx APIs to fetch the configuration stored in flash previously. */
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = SSID,
            .password = PWD
        }
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start()); // Start Wi-Fi station.
    ESP_LOGI(WIFI_INIT, "Initialized Wi-Fi.");
}

void app_main()
{
    // Initialize NVS, required for Wi-Fi driver.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGI(APP_MAIN, "Erasing contents of NVS flash partition");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Create FreeRTOS event group.
    evt_group = xEventGroupCreate();

    // Initialize Wi-Fi station.
    wifi_init_sta();

    // Create application task.
    xTaskCreatePinnedToCore(app_task, "APP TASK", 4 * 1024, NULL, 5, NULL, APP_CPU_NUM);
}