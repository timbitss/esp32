/**
 * @file server.c
 * @author Timothy Nguyen
 * @brief ESP32 as a light-weight web server.
 * @version 0.1
 * @date 2021-06-11
 * 
 *      Server expects a single boolean JSON item named "LED_switch" if client posts to /temperature. 
 */

#include "sys/param.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "server.h"
#include "tmp102.h"
#include "cJSON.h"
#include "driver/gpio.h"

#define TAG "SERVER" // For logging data and information.

#define BUF_SIZE 150 // Size of rx buffer for POST requests.

#define LED_SWITCH_NUM GPIO_NUM_15 // LED GPIO number corresponding to LED_switch JSON key.

/**
 * @brief Handler for GET requests at root endpoint with wildcard.
 * 
 * @param req HTTP request data structure.
 * @return esp_err_t ESP error code.
 */
static esp_err_t root_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "URI %s was hit", req->uri);
    const char resp[] = "Hello World";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}


/**
 * @brief Handler for GET requests at /temperature.
 * 
 * @param req HTTP request data structure.
 * @return esp_err_t ESP error code.
 */
static esp_err_t temperature_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "URI %s was hit", req->uri);
    char resp[20] = {0};
    sprintf(resp, "Temperature: %.2f", tmp102_get_temp());
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/**
 * @brief Handler for POST requests at /led
 * 
 * @param req HTTP request data structure.
 * @return esp_err_t ESP error code.
 */
static esp_err_t led_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "URI %s was hit", req->uri);
    char rx_buf[BUF_SIZE] = {0};
  
    // Will copy first BUF_SIZE - 1 characters if content_len > BUF_SIZE
    int ret;
    {
        ret = httpd_req_recv(req, rx_buf, req->content_len); 
    } while ( ret == HTTPD_SOCK_ERR_TIMEOUT ); // Retry if HTTP socket timed out or was interrupted.
    
    // Set GPIO pin based on boolean value of "LED_switch" key
    cJSON *json = cJSON_Parse(rx_buf);
    const cJSON *led_switch = cJSON_GetObjectItemCaseSensitive(json, "LED_switch");
    if(cJSON_IsBool(led_switch))
    {
        gpio_set_level(LED_SWITCH_NUM, cJSON_IsTrue(led_switch));
        ESP_ERROR_CHECK(httpd_resp_set_status(req, "200 OK"));
        ESP_ERROR_CHECK(httpd_resp_send(req, NULL, 0)); // End transaction.
    }
    else
    {
        ESP_LOGE(TAG, "LED_switch is not a boolean item.");
        ESP_ERROR_CHECK(httpd_resp_set_status(req, "400 Bad Request"));
        ESP_ERROR_CHECK(httpd_resp_send(req, NULL, 0)); // End transaction.
    }
  
    return ESP_OK;
}

/**
 * @brief Registers endpoint handlers and starts web server.
 * 
 * @return httpd_handle_t Handler to HTTP server instance.
 *                        If server failed to start, handler returned will be NULL.
 */
httpd_handle_t start_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    config.uri_match_fn = httpd_uri_match_wildcard;

    // Register endpoint handler for GET requests at root endpoint with wildcard.
    httpd_uri_t root_endpoint_config = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = root_handler,
        .user_ctx = NULL
    };

    // Register endpoint handler for GET requests at /temperature endpoint.
    httpd_uri_t temperature_endpoint_config = {
        .uri = "/temperature",
        .method = HTTP_GET,
        .handler = temperature_handler,
        .user_ctx = NULL};

    // Register endpoint handler for POST requests at /led endpoint.
    httpd_uri_t led_endpoint_config = {
        .uri = "/led",
        .method = HTTP_POST,
        .handler = led_handler,
        .user_ctx = NULL};

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_register_uri_handler(server, &temperature_endpoint_config);
        httpd_register_uri_handler(server, &led_endpoint_config);
        httpd_register_uri_handler(server, &root_endpoint_config);
    }

    ESP_LOGI(TAG, "Created web server and registered endpoints.");
    return server;
}

/**
 * @brief Stop web server.
 */
void stop_server(httpd_handle_t server)
{
    if (server)
    {
        ESP_LOGI(TAG, "Closing web server.");
        httpd_stop(server);
    }
}