/**
 * @file connect.c
 * @author Timothy Nguyen
 * @brief Custom Wi-FI API.
 * @version 0.2
 * @date 2021-06-12
 * 
 *       Configure ESP32 as soft-AP credentials in menuconfig prior to flashing.
 */

#include <stdio.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "connect.h"
#include "nvs_flash.h"
#include "freertos/semphr.h"
#include "string.h"

// ESP32 AP credentials. Configure in menuconfig:
#define ESP32AP_SSID CONFIG_ESP32AP_SSID // ESP32 AP SSID
#define ESP32AP_PWD CONFIG_ESP32AP_PWD   // ESP32 AP Password

// Adjust buffer sizes according to application needs:
#define SSID_BUF_SZ 25                 // SSID buffer size.
#define PWD_BUF_SZ 100                 // Password buffer size.
#define CONNECT_WIFI_STACK_SZ 1024 * 4 // Connect_wifi_task stack size.

#define MAX_RETRY_ATTEMPTS 10 // Maximum number of attempts to connect to Wi-Fi before giving up.

// NVS Flash Definitions:
#define WIFI_NAMESPACE "WIFI_CREDS" // NVS namespace associated with SSID and Password.
#define SSID_KEY "SSID"                   // Key for SSID item.
#define PWD_KEY "PASSWORD"                // Key for password item.

// Tags for logging data and information:
#define WIFI_EVENT_HANDLER "WIFI_EVENT_HANDLER"
#define WIFI_INIT "WIFI_INIT"

// Event group object for communication between Wi-FI callback and application task.
EventGroupHandle_t wifi_evt_group;

// Semaphore to retry obtaining wifi credentials from NVS flash.
SemaphoreHandle_t test_wifi_creds_sem;

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
        ESP_LOGI(WIFI_EVENT_HANDLER, "Attempting to connect to AP");
        esp_wifi_connect();
        break;
    case WIFI_EVENT_STA_CONNECTED:
        xEventGroupSetBits(wifi_evt_group, WIFI_CONNECTED_BIT);
        attempts = 0;
        break;
    case IP_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_evt_group, IP_GOT_IP_BIT);
        break;
    case WIFI_EVENT_AP_START:
        xEventGroupSetBits(wifi_evt_group, AP_STARTED_BIT);
        ESP_LOGI(WIFI_EVENT_HANDLER, "ESP32AP SSID: %s, PWD: %s", ESP32AP_SSID, ESP32AP_PWD);
        break;
    case WIFI_EVENT_AP_STACONNECTED:
        xEventGroupSetBits(wifi_evt_group, AP_STA_CONNECTED_BIT);
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
 * @brief Configure and start ESP32 as a soft-AP.
 */
static void wifi_init_ap()
{
    // Create default WIFI AP instance w/ TCP/IP stack.
    esp_netif_t *esp_netif_inst = esp_netif_create_default_wifi_ap();
    if (esp_netif_inst == NULL)
    {
        ESP_LOGE(WIFI_INIT, "Could not create default Wi-Fi AP instance.");
    }

    wifi_config_t wifi_config = {
        .ap = {.ssid = ESP32AP_SSID,
               .password = ESP32AP_PWD,
               .max_connection = 4,
               .authmode = WIFI_AUTH_WPA_WPA2_PSK}};
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config)); // Stored into NVS flash if enabled.
    ESP_ERROR_CHECK(esp_wifi_start());                                  // ! Start Wi-Fi AP.
    ESP_LOGI(WIFI_INIT, "Started Wi-Fi AP.");
}

/**
 * @brief Configure and start ESP32 as Wi-Fi station.
 */
static void wifi_init_sta(const char *ap_ssid, const char *ap_pwd)
{

    // Create default WIFI STA instance.
    esp_netif_t *esp_netif_inst = esp_netif_create_default_wifi_sta();
    if (esp_netif_inst == NULL)
    {
        ESP_LOGE(WIFI_INIT, "Could not create default Wi-Fi station instance.");
    }

    wifi_config_t wifi_config;
    strcpy((char *)wifi_config.ap.ssid, ap_ssid);
    strcpy((char *)wifi_config.ap.password, ap_pwd);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config)); // Stored into NVS flash if enabled.
    ESP_ERROR_CHECK(esp_wifi_start());                                   // Start Wi-Fi station.
    ESP_LOGI(WIFI_INIT, "Started Wi-Fi station.");
}

/**
 * @brief Task to connect to Wi-Fi.
 * 
 * @note  If the AP's SSID and password are not in stored in NVS Flash, set the ESP32 as an AP.
 *        User must then connect to ESP32 and enter AP credentials by accessing IP address on a web browser.
 */
static void connect_wifi_task(void *params)
{
    // Create FreeRTOS event group for Wi-Fi events.
    wifi_evt_group = xEventGroupCreate();

    // Initialize NVS, required for Wi-Fi driver.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGI(WIFI_INIT, "Erasing contents of NVS flash partition");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

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

    /* Create the Wi-Fi driver task and initialize the Wi-Fi driver with default configurations.
     * NOTE: Function must be called before configuring Wi-Fi connection. */
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));

    test_wifi_creds_sem = xSemaphoreCreateBinary();
    bool created_AP = false;

    // Check if SSID and password are stored in NVS flash before connecting to Wi-Fi.
    do
    {
        nvs_handle_t nvs_handle = 0;
        ESP_ERROR_CHECK(nvs_open(WIFI_NAMESPACE, NVS_READWRITE, &nvs_handle));
        char ssid_str[25] = {0};
        char pwd_str[100] = {0};
        esp_err_t err_ssid = nvs_get_str(nvs_handle, SSID_KEY, ssid_str, NULL);
        esp_err_t err_pwd = nvs_get_str(nvs_handle, PWD_KEY, pwd_str, NULL);

        if (err_ssid == ESP_OK && err_pwd == ESP_OK)
        {
            ESP_LOGI(WIFI_INIT, "Connecting to AP, SSID: %s, PWD: %s", ssid_str, pwd_str);
            wifi_init_sta(ssid_str, pwd_str);
            vSemaphoreDelete(test_wifi_creds_sem);
            return;
        }
        else if (created_AP == false)
        {
            ESP_LOGI(WIFI_INIT, "Starting ESP32 as AP.");
            wifi_init_ap();
            created_AP = true;
        }
        else
        {
            ESP_LOGE(WIFI_INIT, "Incorrect Wi-Fi credentials entered, please try again.");
        }
    } while (xSemaphoreTake(test_wifi_creds_sem, portMAX_DELAY));

    vTaskDelete(NULL);
}

/**
 * @brief Connect to wi-fi.
 * 
 *        Starts connect_wifi task.
 */
void connect_to_wifi()
{
    xTaskCreatePinnedToCore(connect_wifi_task, "CONNECT WIFI", CONNECT_WIFI_STACK_SZ, NULL, 5, NULL, APP_CPU_NUM);
}