/**
 * @file main.c
 * @author Timothy Nguyen
 * @brief Light-sleep example with timer and GPIO wakeup
 * @version 0.1
 * @date 2021-06-05
 * 
 * @note GPIO wakeup only works for light-sleep mode.
 */

#include <stdio.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp32/rom/uart.h"

// Tags for logging data and information
#define TAG "APPMAIN"

// GPIO Wakeup Pin
#define GPIO_WAKEUP_PIN GPIO_NUM_19

void app_main()
{
    while (true)
    {
        ESP_LOGI(TAG, "Enabling RTC timer wakeup source.");
        ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(5000000)); // Enable RTC timer wake up after 5 seconds

        // Configure GPIO as wakeup source on high level
            ESP_LOGI(TAG, "Enabling GPIO wakeup source.");
            gpio_config_t gpio_conf = {
                .pin_bit_mask = (1 << GPIO_WAKEUP_PIN),
                .mode = GPIO_MODE_INPUT,
                .intr_type = GPIO_INTR_HIGH_LEVEL,
                .pull_down_en = GPIO_PULLDOWN_ENABLE,
                .pull_up_en = GPIO_PULLUP_DISABLE};
            ESP_ERROR_CHECK(gpio_config(&gpio_conf));
            ESP_ERROR_CHECK(gpio_wakeup_enable(GPIO_WAKEUP_PIN, GPIO_INTR_HIGH_LEVEL));
            ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());

        ESP_LOGI(TAG, "Going to sleep");
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_ERROR_CHECK(esp_light_sleep_start());

        esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause(); // Get cause of wakeup

        switch (wakeup_cause)
        {
        case ESP_SLEEP_WAKEUP_GPIO:
            ESP_LOGI(TAG, "Wake up caused by GPIO");
            break;
        case ESP_SLEEP_WAKEUP_TIMER:
            ESP_LOGI(TAG, "Wake up caused by timer");
            break;
        default:
            ESP_LOGE(TAG, "Unknown wakeup source");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}