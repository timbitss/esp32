/**
 * @file server.c
 * @author Timothy Nguyen
 * @brief ESP32 as a light-weight HTTP web server.
 * @version 0.1
 * @date 2021-06-11
 * @note Prior to usage, the ESP32 must be connected to Wi-FI as STA or be acting as an AP.
 * 
 * If on a web brower:
 *       - GET request at /api/temperature returns temperature as JSON string.
 *       - POST request at /api/led with "LED_switch":<bool> JSON item turns LED on or off.
 *       - GET request at the root endpoint loads up an interactive webpage.
 */

#include "sys/param.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "server.h"
#include "tmp102.h"
#include "cJSON.h"
#include "driver/gpio.h"
#include "esp_spiffs.h"
#include "connect.h"

#define TAG "SERVER" // For logging data and information.

// Configure size of buffers based on application needs:
#define BUF_SIZE 150      // Size of rx buffer for POST requests.
#define LINE_BUF_SIZE 256 // Size of buffer for reading each line of file.
#define PATH_BUF_SIZE 600 // Size of path.

/**
 * @brief Handler for GET requests at root endpoint + wildcard.
 * 
 * @param req HTTP request data structure.
 * @return esp_err_t ESP error code.
 */
static esp_err_t root_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "URI %s was hit", req->uri);

    // Send back file located in SPIFFS.
    const esp_vfs_spiffs_conf_t spiffs_conf =
        {
            .base_path = "/spiffs",  // File path prefix must be prepended if stdlib functions are used.
            .partition_label = NULL, // Finds first SPIFFS partition label.
            .max_files = 5,          // Max files open at the same time.
            .format_if_mount_failed = true};
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&spiffs_conf));

    char file_path[PATH_BUF_SIZE] = {0};
    if (strcmp(req->uri, "/") == 0)
    {
        sprintf(file_path, "/spiffs/index.html");
    }
    else
    {
        sprintf(file_path, "/spiffs%s", req->uri);
    }

    FILE *fp = fopen(file_path, "r");
    if (fp == NULL)
    {
        ESP_LOGE(TAG, "File not found.");
        httpd_resp_send_404(req);
    }
    else
    {
        char *ext = strrchr(file_path, '.');
        if(strcmp(ext,".css") == 0)
        {
            ESP_LOGI(TAG, "Setting mime type to css");
            httpd_resp_set_type(req, "text/css");
        }

        char line_buf[LINE_BUF_SIZE] = {0};
        while (fgets(line_buf, LINE_BUF_SIZE, fp) != NULL)
        {
            httpd_resp_send_chunk(req, line_buf, HTTPD_RESP_USE_STRLEN); // Provide each line one by one.
        }
        httpd_resp_send_chunk(req, NULL, 0);
        fclose(fp);
    }

    ESP_ERROR_CHECK(esp_vfs_spiffs_unregister(NULL));
    return ESP_OK;
}

/**
 * @brief Handler for POST requests at /setwifi.
 * 
 * @param req HTTP request data structure.
 * @return esp_err_t ESP error code.
 */
static esp_err_t set_wifi_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "URI %s was hit", req->uri);
    
    // TODO: Test if AP works. 
    // TODO: Then write code to extract SSID and password from string, then place in NVS flash (set and commit).
    // TODO: Add redirection.
  
    xSemaphoreGive(test_wifi_creds_sem); // Signal connect_wifi_task to retry Wi-Fi connection.
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

    // Send back temperature as JSON string.
    char resp[50] = {0};
    sprintf(resp, "{\"Temperature\":%.2f}", tmp102_get_temp()); // Format temperature as JSON string.
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/**
 * @brief Handler for POST requests at /led.
 * 
 * @param req HTTP request data structure.
 * @return esp_err_t ESP error code.
 */
static esp_err_t led_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "URI %s was hit", req->uri);

    // Set GPIO pin based on boolean value of "LED_switch" key.
    char rx_buf[BUF_SIZE] = {0};
    int ret;
    {
        ret = httpd_req_recv(req, rx_buf, req->content_len);
    } while (ret == HTTPD_SOCK_ERR_TIMEOUT); 

    cJSON *json = cJSON_Parse(rx_buf);
    const cJSON *led_switch = cJSON_GetObjectItemCaseSensitive(json, "LED_switch");
    if (cJSON_IsBool(led_switch))
    {
        gpio_set_level(LED_SWITCH_NUM, cJSON_IsTrue(led_switch));
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
 *                        NULL if web server failed to start.
 */
httpd_handle_t start_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    config.uri_match_fn = httpd_uri_match_wildcard;

    // Register endpoint handler for POST requests at /setwifi endpoint.
    httpd_uri_t setwifi_endpoint_config = {
        .uri = "/setwifi",
        .method = HTTP_POST,
        .handler = set_wifi_handler,
        .user_ctx = NULL};

    // Register endpoint handler for GET requests at root endpoint + wildcard.
    httpd_uri_t root_endpoint_config = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = root_handler,
        .user_ctx = NULL};

    // Register endpoint handler for GET requests at /api/temperature endpoint.
    httpd_uri_t temperature_endpoint_config = {
        .uri = "/api/temperature",
        .method = HTTP_GET,
        .handler = temperature_handler,
        .user_ctx = NULL};

    // Register endpoint handler for POST requests at /api/led endpoint.
    httpd_uri_t led_endpoint_config = {
        .uri = "/api/led",
        .method = HTTP_POST,
        .handler = led_handler,
        .user_ctx = NULL};

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_register_uri_handler(server, &setwifi_endpoint_config);
        httpd_register_uri_handler(server, &temperature_endpoint_config);
        httpd_register_uri_handler(server, &led_endpoint_config);
        httpd_register_uri_handler(server, &root_endpoint_config); // ! Register wildcard endpoint last.
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