/**
 * @file main.c
 * @author Timothy Nguyen 
 * @brief Store ID_t data structs (known as blobs) into custom NVS partition named 'nvs2'
 * @version 0.1
 * @date 2021-05-26
 * 
 * @note Custom partition table is located in partition2.csv. 
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

//-------------------------------------------------------------------------------------------------

#define TAG "NVS"

#define MIN_AGE 18

//-------------------------------------------------------------------------------------------------

// To store person's name and age
typedef struct{
    char name[20];
    int age;
}ID_t;

// Pool of random names 
const char* names[10] = {"Timothy", "Sophie", "Dave", "Ann", "Dominic", "Richard", "Linda", "Bob", "John", "Rick"};

//-------------------------------------------------------------------------------------------------

/**
 * @note NVS items stored as key-value pairs
 * 
 */
void app_main()
{
   
    ESP_LOGI(TAG, "Initializing custom NVS partition labelled 'nvs2'...");
    static const char* custom_nvs = "nvs2";
    ESP_ERROR_CHECK(nvs_flash_init_partition(custom_nvs));

    ESP_LOGI(TAG, "Opening nvs2 partition handle under ID_storage namespace...");
    nvs_handle_t nvs_handle;
    ESP_ERROR_CHECK(nvs_open_from_partition(custom_nvs, "ID_storage", NVS_READWRITE, &nvs_handle));

    /* Retrieve values of 10 keys from flash */
    char id_key[16]; // Maximum key size is 15 characters/bytes, excluding zero terminator
    size_t required_size = sizeof(ID_t); 
    ID_t id;

    for(int i = 0; i < 10; i++){
        sprintf(id_key, "person %c", i+'0'); 
        esp_err_t err = nvs_get_blob(nvs_handle, id_key, (void*)&id, &required_size);
        switch(err)
        {
            case ESP_ERR_NVS_NOT_FOUND:      // Key is not found, create ID and write into nvs
                ESP_LOGE(TAG, "%s", esp_err_to_name(err));
                ID_t id_temp;
                sprintf(id_temp.name, "%s", names[i]);
                id_temp.age = esp_random() % 50 + MIN_AGE;
                ESP_ERROR_CHECK(nvs_set_blob(nvs_handle, id_key, (void*)&id_temp, required_size));
                ESP_ERROR_CHECK(nvs_commit(nvs_handle));
                break;
            case ESP_ERR_NVS_INVALID_LENGTH: // Length is not sufficient to store data
                ESP_LOGE(TAG, "%s", esp_err_to_name(err));
                break;
            case ESP_OK:
                ESP_LOGI(TAG, "Key/ID: %s, Name: %s, Age: %d", id_key, id.name, id.age);
                break;
            default:
                ESP_LOGE(TAG, "Unknown error.");
                break; 
        }

    }

    /* Print some info regarding nvs partition.
       2 used entries: 1 entry for namespace, 1 entry for key-value pair */
    nvs_stats_t stats;
    nvs_get_stats(custom_nvs, &stats);
                                                                                     // namespace count within partition
    ESP_LOGI(TAG, "Count: UsedEntries = (%d), FreeEntries = (%d), AllEntries = (%d), namespace count = (%d)",
                   stats.used_entries, stats.free_entries, stats.total_entries, stats.namespace_count);

    nvs_close(nvs_handle);
    
}   
