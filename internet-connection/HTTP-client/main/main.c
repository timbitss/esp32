/**
 * @file main.c
 * @author Timothy Nguyen
 * @brief Example of using HTTP Client API to make HTTP requests from ESP32 
 * @version 0.1
 * @date 2021-06-07
 */

#include <stdio.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "protocol_examples_common.h"
#include "esp_log.h"

#define TAG "APPMAIN" // for logging data

esp_err_t _http_event_handle(esp_http_client_event_t *evt)
{
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
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER");
        printf("%.*s", evt->data_len, (char *)evt->data);
        break;
    case HTTP_EVENT_ON_DATA: 
        ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        //if (!esp_http_client_is_chunked_response(evt->client)) // Print data sent in packets.
        //{
            printf("%.*s", evt->data_len, (char *)evt->data);
        //}
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

void app_main()
{
    nvs_flash_init();
    tcpip_adapter_init();
    esp_event_loop_create_default();
    example_connect();

    esp_http_client_config_t client_conf = {
        .url = "http://www.tcpipguide.com/free/index.htm", // IMPORTANT: adding URL overrides most configuration fields
        .event_handler = _http_event_handle
    };
    esp_http_client_handle_t client = esp_http_client_init(&client_conf); // Start HTTP session with client config.
    /* Perform HTTP GET request to read contents of index.html. 
       Blocking by default. */
    esp_err_t err = esp_http_client_perform(client);                      

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "HTTP Response: Status = %d, content_length = %d", // 200 = OK, request has succeeded
                 esp_http_client_get_status_code(client),     
                 esp_http_client_get_content_length(client)); 
    }
    esp_http_client_cleanup(client); // End HTTP session
}