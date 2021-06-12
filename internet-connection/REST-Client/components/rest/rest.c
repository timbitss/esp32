/**
 * @file rest.c
 * @author Timothy Nguyen
 * @brief REST API example.
 * @version 0.1
 * @date 2021-06-09
 * @note Prior to usage, the ESP32 must be connected to Wi-FI as STA.
 * 
 *       ESP32 is set up as an HTTP client in order to make REST transactions.
 *       Includes SMS messaging using Twilio REST API.
 */

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "esp_log.h"
#include "esp_err.h"
#include "rest.h"

#define TAG "REST" // Tag for logging data and information.

/* Twilio REST API parameters. 
 * User must configure in menuconfig before flashing project to enable SMS functionality. */
#define TWILIO_SID CONFIG_TWILIO_SID               // SID of Twilio account.
#define TWILIO_AUTH_TOKEN CONFIG_TWILIO_AUTH_TOKEN // Authentification token of Twilio account.
#define FROM_PHONE_NUM CONFIG_FROM_PHONE_NUM       // Source Twilio phone number in E.164 format.
#define TO_PHONE_NUM CONFIG_TO_PHONE_NUM           // Destination phone number in E.164 format.

// Twilio SMS endpoint.
#define TWILIO_SMS_ENDPOINT "https://api.twilio.com/2010-04-01/Accounts/" TWILIO_SID "/Messages.json"

/**
 * @brief Handle HTTP events based on event ID.
 * 
 * @param evt Pointer to esp_http_client_event_t struct with information about latest HTTP event.
 * @return esp_err_t ESP error code. Always returns ESP_OK.
 */
static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    static char rx_buffer[HTTP_RESPONSE_SIZE] = {0}; // Buffer to store response body.
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADERS_SENT:
        ESP_LOGI(TAG, "HTTP_EVENT_HEADERS_SENT");
        break;
    case HTTP_EVENT_ON_HEADER: // Occurs when receiving each header from server.
        ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);

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
        printf("%s\n", rx_buffer); // Print out HTTP response body.
        rest_config_t *rest_config = (rest_config_t *)evt->user_data;

        if (rest_config->HTTP_method == HTTP_METHOD_GET)
        {
            rest_config->parse_json(rx_buffer, rest_config->payload_str);
        }

        memset((char *)rx_buffer, 0, HTTP_RESPONSE_SIZE); // Clear rx_buffer.
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

/**
 * @brief Execute REST/HTTP transaction.
 * 
 * @note GET transaction assumes parsing of JSON.
 *       POST transaction assumes sending of SMS message via Twilio REST API.
 * 
 * @param[in,out] rest_config_t  Input: rest_config holds HTTP configuration settings.
 *                               Output: rest_config will hold parsed string if HTTP GET request.
 * 
 * @return HTTP response status code.
 */
int rest_execute(rest_config_t *rest_config)
{
    esp_http_client_config_t client_config = {.event_handler = http_event_handler,
                                              .user_data = rest_config};

    esp_http_client_handle_t client;

    if (rest_config->HTTP_method == HTTP_METHOD_GET)
    {
        client_config.method = HTTP_METHOD_GET;
        client_config.url = rest_config->endpoint;
        client = esp_http_client_init(&client_config);
    }
    else // POST request
    {
        client_config.method = HTTP_METHOD_POST;
        client_config.url = TWILIO_SMS_ENDPOINT;
        client_config.auth_type = HTTP_AUTH_TYPE_BASIC;
        client_config.username = TWILIO_SID;
        client_config.password = TWILIO_AUTH_TOKEN;
        client = esp_http_client_init(&client_config);

        esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");
        char post_buf[300] = {0};
        sprintf(post_buf, "%s=%s&%s=%s&%s=%s", "Body", rest_config->payload_str,
                                               "From", FROM_PHONE_NUM,
                                               "To", TO_PHONE_NUM);
        esp_http_client_set_post_field(client, (const char *)post_buf, strlen(post_buf));
    }

    /* Perform HTTP transaction.
     * Blocking, by default, until HTTP_EVENT_ON_FINISH has been handled. */
    ESP_ERROR_CHECK(esp_http_client_perform(client));

    int status = esp_http_client_get_status_code(client);
    ESP_ERROR_CHECK(esp_http_client_close(client));
    ESP_ERROR_CHECK(esp_http_client_cleanup(client));
    return status;
}
