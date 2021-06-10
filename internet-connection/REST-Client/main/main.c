/**
 * @file main.c
 * @author Timothy Nguyen
 * @brief REST Client example: Get the quote of the day.
 * @version 0.1
 * @date 2021-06-07
 */

#include <stdio.h>
#include "freeRTOS/FreeRTOS.h"
#include "freeRTOS/task.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "rest.h"
#include "connect.h"
#include "cJSON.h"
#include "string.h"
#include "esp_wifi.h"

// URL/endpoint to get JSON string from.
#define REQUEST_URL "http://quotes.rest/qod?language=en"

// Tags for logging data and information
#define MAIN_TASK "MAIN_TASK"
#define APP_TASK "APP_TASK"
#define PARSER "PARSER"

/**
 * @brief Get quote of the day string from JSON string
 * 
 * @param[in] json_string JSON string obtained from REST server.
 * @param[out] output Buffer to store quote of the day.
 * @returns esp_err_t ESP_OK if string was parsed successfuly.
 *                    ESP_xxx otherwise.  
 */
esp_err_t get_qotd(const char *json_string, char *output)
{
    cJSON *json_obj = cJSON_Parse(json_string);
    esp_err_t err = ESP_OK;
    if (json_obj == NULL)
    {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL)
        {
            ESP_LOGE(PARSER, "Error before: %s\n", error_ptr);
        }
        err = ESP_FAIL;
    }
    else
    {
        const cJSON *contents = cJSON_GetObjectItemCaseSensitive(json_obj, "contents");
        const cJSON *quotes = cJSON_GetObjectItemCaseSensitive(contents, "quotes");

        if (cJSON_IsArray(quotes))
        {
            cJSON *quote_element;
            cJSON_ArrayForEach(quote_element, quotes)
            {
                const cJSON *qotd = cJSON_GetObjectItemCaseSensitive(quote_element, "quote");
                if (strlen(qotd->valuestring) < PAYLOAD_SIZE && cJSON_IsString(qotd))
                {
                    strcpy(output, qotd->valuestring);
                }
                else if (!cJSON_IsString(qotd))
                {
                    ESP_LOGE(PARSER, "Qotd does not hold a a string.");
                    err = ESP_ERR_NOT_FOUND;
                }
                else
                {
                    ESP_LOGE(PARSER, "Payload buffer size must be greater than %zu", strlen(qotd->valuestring));
                    err = ESP_ERR_NO_MEM;
                }
            }
        }
        else
        {
            ESP_LOGE(PARSER, "Quotes item is not an array");
            err = ESP_ERR_NOT_FOUND;
        }
    }

    cJSON_Delete(json_obj);
    return err;
}

/**
 * @brief Get and print the quote of the day using a REST service.
 */
void print_qotd(void)
{
    payload_config_t payload = {
        .endpoint = REQUEST_URL,
        .parse_json = &get_qotd // Get QOTD from JSON string.
    };
    rest_get_json(&payload);

    if (payload.err == ESP_OK)
    {
        printf("Quote of the day: %s\n", payload.payload_str); // Print out results
    }
    else
    {
        ESP_LOGE(APP_TASK, "Could not get quote of the day.");
    }
}

/**
 * @brief Application task for Wi-Fi applications.
 * 
 * @param param Optional parameters as void ptr.
 */
static void app_task(void *param)
{
    while (true)
    {
        EventBits_t bits = xEventGroupWaitBits(wifi_evt_group,
                                               // Bits to wait for.  
                                               WIFI_CONNECTED_BIT | IP_GOT_IP_BIT | WIFI_FAIL_BIT | WIFI_DISCONNECTED_BIT, 
                                               // Clear set bits on exit.
                                               pdTRUE,                   
                                               // Wait for any bit to be sit in event group.                                                  
                                               pdFALSE,    
                                               // Timeout period of 10 s.                                                                
                                               pdMS_TO_TICKS(10000));                                                      

        if (bits & WIFI_CONNECTED_BIT)
        {
            ESP_LOGI(APP_TASK, "ESP32 Station connected to AP");
        }
        else if (bits & IP_GOT_IP_BIT)
        {
            ESP_LOGI(APP_TASK, "Got IPv4 address of AP.");
            print_qotd();
            dont_reconnect = true;
            ESP_ERROR_CHECK(esp_wifi_disconnect()); 
        }
        else if (bits & WIFI_DISCONNECTED_BIT) // Successful esp_wifi_disconnect() call.
        {
            ESP_LOGI(APP_TASK, "ESP32 disconnected from Wi-Fi.");
            ESP_ERROR_CHECK(esp_wifi_stop());  // Free up resources.
            ESP_ERROR_CHECK(esp_wifi_deinit());
            vTaskDelete(NULL); 
        }
        else if (bits & WIFI_FAIL_BIT) // Bit only set after multiple reconnection attempts.
        {
            ESP_LOGE(APP_TASK, "Could not connect to Wi-Fi.");
            vTaskDelete(NULL);
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