/**
 * @file main.c
 * @author Timothy Nguyen
 * @brief ESP32 switching between AP and STA example.
 * @version 0.1
 * @date 2021-06-11
 * 
 *       If the AP's SSID and password are not in stored in NVS Flash, the program sets the ESP32 as an AP.
 *       The user must then connect to the ESP32 and enter AP credentials through a web browser.
 * 
 * @note  Configure credentials for ESP32 as soft-AP in menuconfig prior to flashing.
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
 * @brief Application task for Wi-Fi applications.
 * 
 * @param param Optional parameters as void ptr.
 */
static void app_task(void *param)
{
    wifi_evt_group = xEventGroupCreate();
    bool created_HTTP_server();
    while (true)
    {
        EventBits_t bits = xEventGroupWaitBits(wifi_evt_group,
                                               // Bits to wait for.
        WIFI_CONNECTED_BIT | IP_GOT_IP_BIT | AP_STARTED_BIT | WIFI_FAIL_BIT | WIFI_DISCONNECTED_BIT | AP_STA_CONNECTED_BIT,
                                               // Clear set bits on exit.
                                               pdTRUE,
                                               // Wait for any bit to be sit in event group.
                                               pdFALSE,
                                               // Time out after 10 seconds.
                                               pdMS_TO_TICKS(10000));

        if (bits & WIFI_CONNECTED_BIT)
        {
            ESP_LOGI(TAG, "ESP32 Station connected to AP");
        }
        else if (bits & IP_GOT_IP_BIT)
        {
            ESP_LOGI(TAG, "Got IPv4 address of AP.");
            http_start_server();
        }
        else if (bits & AP_STARTED_BIT)
        {
            http_start_server();
            ESP_LOGI(TAG, "ESP32 AP has started.");
            ESP_LOGI(TAG, "Please connect to ESP32 and use web browser to set Wi-Fi credentials.");
        }
        else if (bits & AP_STA_CONNECTED_BIT)
        {
            ESP_LOGI(TAG, "A station has connected to the ESP32.");
        }
        else if (bits & WIFI_DISCONNECTED_BIT) // Successful esp_wifi_disconnect() call.
        {
            ESP_LOGI(TAG, "ESP32 station disconnected from Wi-Fi.");
            tmp102_stop();
            vTaskDelete(NULL);
        }
        else if (bits & WIFI_FAIL_BIT) // Bit only set after multiple reconnection attempts.
        {
            ESP_LOGE(TAG, "Could not connect to Wi-Fi.");
            disconnect_from_wifi();
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

    // Create application task.
    xTaskCreatePinnedToCore(app_task, "APP TASK", 4 * 1024, NULL, 5, NULL, APP_CPU_NUM);

    // Initialize TMP102 temperature sensor driver.
    tmp102_init();

    // Connect to Wi-Fi.
    connect_to_wifi();
}

