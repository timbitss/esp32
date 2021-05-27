/**
 * @file main.c
 * @author Timothy Nguyen 
 * @brief Reading and write an int32_t value into custom NVS partition named 'nvs2'
 * @version 0.1
 * @date 2021-05-26
 * 
 * @note Custom partition table is located in partition2.csv under root project directory. 
 *       Must update configuration through idf.py menuconfig to use the custom partition table.
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#define TAG "NVS"

/**
 * @note NVS items stored as key-value pairs
 * 
 */
void app_main()
{
   
    ESP_LOGI(TAG, "Initializing custom NVS partition labelled 'nvs2'...");
    static const char* custom_label = "nvs2";
    ESP_ERROR_CHECK(nvs_flash_init_partition(custom_label));

    ESP_LOGI(TAG, "Opening handle for nvs2 NVS partition...");
    nvs_handle_t nvs_handle;
    ESP_ERROR_CHECK(nvs_open_from_partition(custom_label, "storage", NVS_READWRITE, &nvs_handle));

    int32_t val = 0;
    const char* key = "my_val";
    ESP_LOGI(TAG, "Retrieving value from key %s", key);
    esp_err_t err = nvs_get_i32(nvs_handle, key, &val);
    switch(err)
    {
        case ESP_OK:
            ESP_LOGI(TAG, "Successfully retrieved value %d from key %s", val, key);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGE(TAG, "%s key does not exist within namespace, setting key with value %d", key, val);
            nvs_set_i32(nvs_handle, key, val);
            nvs_commit(nvs_handle); // Write pending changes to non-volatile storage
            nvs_close(nvs_handle); // Close handle and free any allocated resources
            ESP_LOGI(TAG, "Resetting ESP32");
            esp_restart();
            break;
        default:
            ESP_LOGE(TAG, "Error %s", esp_err_to_name(err));
            break;
    }

    /* Print some info regarding nvs partition.*/
    nvs_stats_t stats;
    nvs_get_stats(custom_label, &stats);
                                                                                     // namespace count within partition
    ESP_LOGI(TAG, "Count: UsedEntries = (%d), FreeEntries = (%d), AllEntries = (%d), namespace count = (%d)",
                   stats.used_entries, stats.free_entries, stats.total_entries, stats.namespace_count);

    nvs_close(nvs_handle);
}   
