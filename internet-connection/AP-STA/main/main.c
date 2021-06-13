/**
 * @file main.c
 * @author Timothy Nguyen
 * @brief ESP32 switching between AP and STA example.
 * @version 0.1
 * @date 2021-06-11
 */

#include <stdio.h>
#include "freeRTOS/FreeRTOS.h"
#include "freeRTOS/task.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "connect.h"
#include "string.h"
#include "esp_wifi.h"
#include "server.h"
#include "tmp102.h"
#include "driver/gpio.h"

#define TAG "APP" // Tag for logging data and information.

#define GPIO_OUT_BIT_MASK (1ULL << GPIO_NUM_15) // Bit mask of GPIO pins to use as output.

/**
 * @brief Set up ESP32 as web server. 
 */
void create_server(void)
{
    httpd_handle_t server = start_server();
    if (server == NULL)
    {
        ESP_LOGE(TAG, "Could not create server.");
    }
    else
    {
        vTaskDelay(pdMS_TO_TICKS(600 * 1000)); // Open web server for 10 minutes before closing.
        stop_server(server);
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
        WIFI_CONNECTED_BIT | IP_GOT_IP_BIT | AP_STARTED_BIT | WIFI_FAIL_BIT | WIFI_DISCONNECTED_BIT,
                                               // Clear set bits on exit.
                                               pdTRUE,
                                               // Wait for any bit to be sit in event group.
                                               pdFALSE,
                                               // Timeout period of 10 s.
                                               pdMS_TO_TICKS(10000));

        if (bits & WIFI_CONNECTED_BIT)
        {
            ESP_LOGI(TAG, "ESP32 Station connected to AP");
        }
        else if (bits & IP_GOT_IP_BIT)
        {
            ESP_LOGI(TAG, "Got IPv4 address of AP.");
            create_server();
            dont_reconnect = true;
            ESP_ERROR_CHECK(esp_wifi_disconnect());
        }
        else if (bits & AP_STARTED_BIT)
        {
            ESP_LOGI(TAG, "ESP32 AP has started.");
            create_server();
        }
        else if (bits & AP_STA_CONNECTED_BIT)
        {
            ESP_LOGI(TAG, "A station has connected to the ESP32.");
        }
        else if (bits & WIFI_DISCONNECTED_BIT) // Successful esp_wifi_disconnect() call.
        {
            ESP_LOGI(TAG, "ESP32 station disconnected from Wi-Fi.");
            ESP_ERROR_CHECK(esp_wifi_stop()); // Free up resources.
            ESP_ERROR_CHECK(esp_wifi_deinit());
            ESP_ERROR_CHECK(esp_netif_deinit());
            vTaskDelete(NULL);
        }
        else if (bits & WIFI_FAIL_BIT) // Bit only set after multiple reconnection attempts.
        {
            ESP_LOGE(TAG, "Could not connect to Wi-Fi.");
            vTaskDelete(NULL);
        }
        else
        {
            ESP_LOGE(TAG, "Task timed out.");
        }
    }
}

void app_main()
{
    // Configure GPIO15 as GPIO output pin.
    gpio_config_t io_config;
    io_config.pin_bit_mask = GPIO_OUT_BIT_MASK;
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.pull_down_en = 0;
    io_config.pull_up_en = 0;
    io_config.intr_type = 0;
    ESP_ERROR_CHECK(gpio_config(&io_config));

    // Check if Wi-Fi configured in flash, if so, configure Wi-FI as STA, otherwise as AP.

    // Initialize Wi-Fi station.
    wifi_init_sta();

    // Initialize TMP102 temperature sensor driver.
    tmp102_init();

    // Create application task.
    xTaskCreatePinnedToCore(app_task, "APP TASK", 4 * 1024, NULL, 5, NULL, APP_CPU_NUM);
}