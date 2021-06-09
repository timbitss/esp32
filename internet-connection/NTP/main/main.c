/**
 * @file main.c
 * @author Timothy Nguyen
 * @brief Example of updating ESP32 system time over NTP.
 * @version 0.1
 * @date 2021-06-07
 * 
 * Press touchbutton to wake up ESP32 from deep-sleep and display time.
 * 
 * @note Must configure SSID and SSID password in project config prior to flashing to target.
 */

#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "esp_sntp.h"
#include "driver/rtc_io.h"

// Tags for logging information and/or data
#define APP_MAIN "APP_MAIN"
#define SNTP_INIT "SNTP_INIT"
#define NOTIF_CB "NOTIF_CB"
#define UPDATE_TIME "UPDATE_TIME"
#define TIME "TIME"
#define EXT1 "EXT1"

// RTC pads to use as wakeup source as a bit mask
#define RTC_GPIO_MASK (1 << GPIO_NUM_13) | (1 << GPIO_NUM_15)

// For time verification
#define CURRENT_YEAR 2021

/* Number of times ESP32 has booted.
   RTC_DATA_ATTR attribute places object in RTC slow memory to retain count through deep sleep. */
RTC_DATA_ATTR static uint32_t boot_count = 0;

/**
 * @brief Callback to notify that system time has been synchronized over NTP.
 * 
 * @param tv Time since epoch in ms and us.
 */
static void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(NOTIF_CB, "System time synchronized to NTP server.");
}

/**
 * @brief Initialize SNTP and send request.
 * 
 * @pre ESP32 must be connected to Wi-Fi
 */
static void sntp_initialize()
{
    ESP_LOGI(SNTP_INIT, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL); // Synchronize every hour by default
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init(); // Sends out request immediately
}

/**
 * @brief Update system time over NTP.
 * 
 *        Time is only synced once in this example.
 * 
 * @param[out] Store UTC time since epoch after synchronization process is completed.
 */
static void update_sys_time(time_t *now)
{
    ESP_LOGI(UPDATE_TIME, "Connecting to Wi-Fi");
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    sntp_initialize();
    while (sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED)
    {
        ESP_LOGI(UPDATE_TIME, "Waiting for sync to be completed.");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    sntp_stop();

    ESP_LOGI(UPDATE_TIME, "Disconnecting from Wi-Fi");
    ESP_ERROR_CHECK(example_disconnect());

    time(now);
}

/**
 * @brief Print time.
 * 
 *        If system time is off, connect to Wi-Fi and update over NTP. 
 */
static void print_time()
{
    char buffer[50];
    time_t now;
    time(&now);                            // Get time since epoch
    struct tm *timeinfo = localtime(&now); // Convert to local time.

    /* Update system time over NTP if tm_year attribute is off
       NOTE: tm_year member holds number of years since 1900 */
    if (timeinfo->tm_year < (CURRENT_YEAR - 1900))
    {
        strftime(buffer, sizeof(buffer), "%c", timeinfo); // Format time into readable form and place into char buffer.
        ESP_LOGI(TIME, "System time is off, updating over NTP: %s", buffer);
        update_sys_time(&now);
        localtime_r(&now, timeinfo);
    }

    strftime(buffer, sizeof(buffer), "%c", timeinfo);
    ESP_LOGI(TIME, "%s", buffer);
}

/**
 * @brief Enable EXT1 wakeup source from deep sleep if any selected pin is high.
 */
static void enable_EXT1_wakeup()
{
    ESP_LOGI(EXT1, "Enabling EXT1 wakeup from deep sleep.");
    if (rtc_gpio_is_valid_gpio(GPIO_NUM_13) && rtc_gpio_is_valid_gpio(GPIO_NUM_15))
    {
        if (boot_count == 1)
        {
            rtc_gpio_pulldown_en(GPIO_NUM_15);
            rtc_gpio_pullup_dis(GPIO_NUM_15);
            rtc_gpio_pulldown_en(GPIO_NUM_13);
            rtc_gpio_pullup_dis(GPIO_NUM_13);
        }
        ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(RTC_GPIO_MASK, ESP_EXT1_WAKEUP_ANY_HIGH));
    }
    else
    {
        ESP_LOGE(APP_MAIN, "RTC pins required for wakeup.");
    }
}

/**
 * @brief Debounce GPIO pin.
 * 
 * @param gpio_num Number of GPIO pin to debounce
 */
static void debounce(gpio_num_t gpio_num);

/**
 * @brief Find wakeup source and debounce GPIO if necessary.
 */
static void get_wakeup_cause()
{
    // Check which wakeup source triggered reset from deep-sleep
    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
    switch (wakeup_cause)
    {
    case ESP_SLEEP_WAKEUP_UNDEFINED:
        ESP_LOGI(APP_MAIN, "Reset was not caused by exit from deep sleep");
        break;
    case ESP_SLEEP_WAKEUP_EXT1:
    {
        uint64_t bit_mask = esp_sleep_get_ext1_wakeup_status();
        if (bit_mask & (1 << GPIO_NUM_13))
        {
            ESP_LOGI(APP_MAIN, "GPIO13 caused wake up");
            debounce(GPIO_NUM_13);
        }
        else
        {
            ESP_LOGI(APP_MAIN, "GPIO15 caused wake up");
            debounce(GPIO_NUM_15);
        }
        break;
    }
    case ESP_SLEEP_WAKEUP_TIMER:
        ESP_LOGI(APP_MAIN, "Timer caused wake up");
        break;
    default:
        ESP_LOGE(APP_MAIN, "Unknown wakeup source");
        break;
    }
}
void app_main()
{
    ++boot_count;
    ESP_LOGI(APP_MAIN, "Boot count: %d", boot_count);

    // Check which wakeup source triggered reset from deep-sleep
    get_wakeup_cause();

    // Set local timezone
    setenv("TZ", "PST8PDT", 1);
    tzset();

    print_time();

    enable_EXT1_wakeup();

    ESP_LOGI(APP_MAIN, "Going to deep sleep...");
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_deep_sleep(10 * 1000000ULL); // wake up after 10 seconds if no buttons have been pressed
}

static void debounce(gpio_num_t gpio_num)
{
    do
    {
        vTaskDelay(pdMS_TO_TICKS(20));
    } while (rtc_gpio_get_level(gpio_num) == 1);
}