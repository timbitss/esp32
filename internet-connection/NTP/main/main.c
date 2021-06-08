/**
 * @file main.c
 * @author Timothy Nguyen
 * @brief Example of using NTP to update ESP32 system time.
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

// RTC pad to use as wakeup source
#define RTC_PAD_NUM GPIO_NUM_13

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
    sntp_setoperatingmode(SNTP_OPMODE_POLL);  // Synchronize every hour by default
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init(); // Sends out request immediately
}

/**
 * @brief Update system time over NTP.
 * 
 *        Time is only synced once in this example.
 * 
 * @param now Store time after synchronization is completed.
 */
static void update_sys_time(time_t now)
{
    ESP_LOGI(UPDATE_TIME, "Connecting to Wi-Fi");
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect()); 

    sntp_initialize(); 
    while(sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED)
    {
        ESP_LOGI(UPDATE_TIME, "Waiting for sync to be completed.");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    sntp_stop();

    ESP_LOGI(UPDATE_TIME, "Disconnecting from Wi-Fi");
    ESP_ERROR_CHECK(example_disconnect());

    time(&now);
}

/**
 * @brief Print time.
 * 
 *        If system time is off, connect to Wi-Fi and update over NTP. 
 * 
 * @param time Time before epoch
 */
static void print_time()
{
  time_t now;
  time(&now);
  struct tm *timeinfo = localtime(&time);
  if(timeinfo->tm_year < 2021)
  {
    ESP_LOGI(APP_MAIN, "System time is off. Updating over NTP.");  
    update_sys_time(&time);
    localtime_r(&time, timeinfo);
  }
  char buffer[50];
  strftime(buffer, sizeof(buffer), "%c", timeinfo); // Store date and time representation into char buffer.
  ESP_LOGI(TIME, "Time: %s", buffer);
}

void app_main()
{
    
    ++boot_count;
    ESP_LOGI(APP_MAIN, "Boot count: %d", boot_count);

    // Set local timezone once
    if(boot_count == 1)
    {
        setenv("TZ", "PST8PDT", 1);
        tzset();
    }

    print_time();

}