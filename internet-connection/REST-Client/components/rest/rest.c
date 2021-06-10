/**
 * @file rest.c
 * @author Timothy Nguyen
 * @brief REST API example for ESP32.
 * @version 0.1
 * @date 2021-06-09
 */

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_err.h"
#include "rest.h"

#define TAG "REST" // Tag for logging data and information.

#define BUFFER_SIZE 1024 * 2 // Size of receive buffer.

/**
 * @brief Handle HTTP events based on event ID.
 * 
 * @param evt Pointer to esp_http_client_event_t struct with information about latest HTTP event.
 * @return esp_err_t ESP error code. Always returns ESP_OK.
 */
static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    static char rx_buffer[BUFFER_SIZE] = {0}; // Buffer to store body of server's response.
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER: // Occurs when receiving each header from server.
        ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        printf("%.*s", evt->data_len, (char *)evt->data);
        if (rx_buffer[0] == '\0') 
        {
            strncpy(rx_buffer, (char *)evt->data, evt->data_len);
            ESP_LOGD(TAG, "Bytes filled in rx_buffer: %u", strlen(rx_buffer));
        }
        else
        {
            strncat(rx_buffer, (char *)evt->data, evt->data_len);
            ESP_LOGD(TAG, "Bytes filled in rx_buffer: %u", strlen(rx_buffer));
        }
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
        printf("%s\n", rx_buffer);
        // Parse JSON string after all packets have been received. 
        payload_config_t* payload_config = (payload_config_t*)evt->user_data;
        payload_config->err = payload_config->parse_json(rx_buffer, payload_config->payload_str);
        memset((char*) rx_buffer, 0, BUFFER_SIZE); // Clear rx_buffer.
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

/**
 * @brief Get JSON string from web server and parse it.
 * 
 * @param[in,out] payload_config As an input, payload_config holds URL and application-specific parser function.
 *                               As an output, payload_config holds payload string and result of parsing JSON.
 */
void rest_get_json(payload_config_t* payload_config)
{
    esp_http_client_config_t config = {
        .url = payload_config->endpoint,
        .event_handler = http_event_handler,
        .user_data = payload_config
        };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    /* Perform HTTP transaction (GET by default).
     * Blocking, by default, until HTTP_EVENT_ON_FINISH has been handled. */
    esp_err_t err = esp_http_client_perform(client); 

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Response status code = %d", esp_http_client_get_status_code(client));
    }
    else
    {
        ESP_LOGE(TAG, "Error getting JSON string at %s", payload_config->endpoint);
    }

    ESP_ERROR_CHECK(esp_http_client_close(client));
    ESP_ERROR_CHECK(esp_http_client_cleanup(client));
}