/**
 * @file connect.c
 * @author Timothy Nguyen
 * @brief Create Wi-Fi station and handle Wi-FI events.
 * @version 0.1
 * @date 2021-06-09
 */

#include <stdio.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "connect.h"

#define SSID CONFIG_WIFI_SSID    // SSID of Target AP to connect to.
#define PWD CONFIG_WIFI_PASSWORD // Password of Target AP to connect to.

// Tags for logging data and information.
#define WIFI_EVENT_HANDLER "WIFI_EVENT_HANDLER"
#define WIFI_INIT "WIFI_INIT"

#define MAX_RETRY_ATTEMPTS 10 // Maximum number of attempts to connect to Wi-Fi before giving up.

// Event group object for communication between Wi-FI event handler and application task.
EventGroupHandle_t wifi_evt_group;

// Distinguish between connection failures and actual esp_wifi_disconnect() call.
bool dont_reconnect = false;

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
        ESP_LOGI(WIFI_EVENT_HANDLER, "Attempting to connect to AP, SSID: %s, PWD: %s", SSID, PWD);
        esp_wifi_connect();
        break;
    case WIFI_EVENT_STA_CONNECTED:
        xEventGroupSetBits(wifi_evt_group, WIFI_CONNECTED_BIT);
        attempts = 0;
        break;
    case IP_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_evt_group, IP_GOT_IP_BIT);
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        if (dont_reconnect == true)
        {
            dont_reconnect = false;
            xEventGroupSetBits(wifi_evt_group, WIFI_DISCONNECTED_BIT);
        }
        else
        {
            attempts++;
            if (attempts <= MAX_RETRY_ATTEMPTS)
            {
                ESP_LOGI(WIFI_EVENT_HANDLER, "Retrying connection. Attempt #%u", attempts);
                esp_wifi_connect();
            }
            else
            {
                xEventGroupSetBits(wifi_evt_group, WIFI_FAIL_BIT);
            }
        }
        break;
    case WIFI_EVENT_STA_STOP:
        ESP_LOGI(WIFI_EVENT_HANDLER, "Stopped and freed station control block.");
        break;
    default:
        ESP_LOGE(WIFI_EVENT_HANDLER, "Could not identify event.");
        break;
    }
}

/**
 * @brief Initialize, configure, and start Wi-Fi station.
 * 
 *        Closely follows STA phases 1-3 in ESP32 Wi-Fi API Guide with exception of FreeRTOS event group.
 */
void wifi_init_sta(void)
{
    // Create FreeRTOS event group for Wi-Fi events.
    wifi_evt_group = xEventGroupCreate();

    // Create an LwIP core task and initialize LwIP-related work.
    ESP_ERROR_CHECK(esp_netif_init());

    // Create the default event loop/task where events from the Wi-Fi driver and TCP stack are posted to.
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
                                                        // Execute handler if station received IP address from connected AP.
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
     * NOTE: Function must be called before configuring Wi-Fi connection. */
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));

    /* If the Wi-Fi NVS flash is enabled by menuconfig, all Wi-Fi configurations are stored in flash.
     * Therefore, you only need to call esp_wifi_get_xxx APIs to fetch the configuration stored in flash previously. */
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = SSID,
            .password = PWD}};
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start()); // Start Wi-Fi station.
    ESP_LOGI(WIFI_INIT, "Initialized Wi-Fi.");
}