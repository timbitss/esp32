/**
 * @file main.c
 * @author Timothy Nguyen
 * @brief Deep-sleep example with timer and EXT1 wakeup
 * @version 0.1
 * @date 2021-06-05
 * 
 * @note Deep sleep mode powers off digital core (CPUs, internal SRAM and ROM), and hence reboots after being woken up.
 */

#include <stdio.h>
#include "esp_log.h"
#include "driver/rtc_io.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp32/rom/uart.h"

// Tags for logging data and information
#define TAG "APPMAIN"

// RTC pins used in EXT1 wakeup source as a bit mask
#define RTC_GPIO_MASK (1ULL << GPIO_NUM_13) | (1ULL << GPIO_NUM_15)

/* Count of wakeups.
   RTC_DATA_ATTR attribute places object in RTC slow memory to retain count. */
RTC_DATA_ATTR static uint32_t count = 0;

void app_main()
{
    // Check which wakeup source triggered reset from deep-sleep
    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
    switch (wakeup_cause)
    {
    case ESP_SLEEP_WAKEUP_UNDEFINED:
        ESP_LOGI(TAG, "Reset was not caused by exit from deep sleep");
        break;
    case ESP_SLEEP_WAKEUP_EXT1:
    {
        uint64_t bit_mask = esp_sleep_get_ext1_wakeup_status();
        if (bit_mask & (1 << GPIO_NUM_13))
            ESP_LOGI(TAG, "GPIO13 caused wake up, count: %u", ++count);
        else
            ESP_LOGI(TAG, "GPIO15 caused wake up, count %u", ++count);
        break;
    }
    case ESP_SLEEP_WAKEUP_TIMER:
        ESP_LOGI(TAG, "Timer caused wake up, count %u", ++count);
        break;
    default:
        ESP_LOGE(TAG, "Unknown wakeup source");
        break;
    }

    ESP_LOGI(TAG, "Enabling RTC timer wakeup from deep sleep.");
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(5000000)); // wakeup after 5 seconds

    ESP_LOGI(TAG, "Enabling EXT1 wakeup from deep sleep.");
    if (rtc_gpio_is_valid_gpio(GPIO_NUM_13) && rtc_gpio_is_valid_gpio(GPIO_NUM_15))
    {
        rtc_gpio_pulldown_en(GPIO_NUM_15);
        rtc_gpio_pullup_dis(GPIO_NUM_15);
        rtc_gpio_pulldown_en(GPIO_NUM_13);
        rtc_gpio_pullup_dis(GPIO_NUM_13);
        ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(RTC_GPIO_MASK, ESP_EXT1_WAKEUP_ANY_HIGH));
    }
    else
    {
        ESP_LOGE(TAG, "RTC pins required for EXT1 wakeup.");
    }

    ESP_LOGI(TAG, "Going to deep sleep...");
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_deep_sleep_start();
}