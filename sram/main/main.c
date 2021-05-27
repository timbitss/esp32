/**
 * @file main.c
 * @author Timothy Nguyen 
 * @brief Testing ESP32 Memory API
 * @version 0.1
 * @date 2021-05-26
 * 
 * 
 */

#include <stdio.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "Memory" // tag for logging

void vTask(void* pvParameters)
{
    // Should print ~2048 bytes of free stack space
    ESP_LOGI(TAG, "Smallest amount of free stack space in child task %d B", uxTaskGetStackHighWaterMark(NULL)); 
    vTaskDelete(NULL);
}

/**
 * @brief Each RTOS task has its own stack, dynamically-allocated from DRAM heap when task is created
 * 
 */
void app_main(void)
{   
    ESP_LOGI(TAG, "Smallest amount of free stack space since app_main was created %d B", uxTaskGetStackHighWaterMark(NULL)); 
    ESP_LOGI(TAG, "Free DRAM heap at start-up: %d KB", heap_caps_get_free_size(MALLOC_CAP_8BIT)/1024); 
    
    /* Dynamically-allocate largest possible block from heap, then free afterwards */ 
    size_t largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "Largest free contiguous block of memory able to be allocated: %d KB", largest_block/1024);
    void* ptr = malloc(largest_block);

    if(ptr == NULL)
    {
        ESP_LOGI(TAG, "Could not allocate block of memory.");
    }
    else
    {
        ESP_LOGI(TAG, "Allocated block of memory");
        ESP_LOGI(TAG, "DRAM heap now available: %d KB", heap_caps_get_free_size(MALLOC_CAP_8BIT)/1024); // Expect to be smaller
        ESP_LOGI(TAG, "Freeing memory");
        free(ptr);
        ESP_LOGI(TAG, "DRAM heap now available: %d KB", heap_caps_get_free_size(MALLOC_CAP_8BIT)/1024); 
    }
    
    // Expect to be lower since function calls were made, growing the stack.
    ESP_LOGI(TAG, "Smallest amount of free stack space since app_main was created %d B", uxTaskGetStackHighWaterMark(NULL)); 

    // Create child task.
    ESP_LOGI(TAG, "Creating child task with 2048 bytes of stack space allocated from heap\n");
    xTaskCreate(vTask, "Child Task", 1024 * 2, NULL, 2, NULL);
    ESP_LOGI(TAG, "DRAM heap available after task creation: %d KB", heap_caps_get_free_size(MALLOC_CAP_8BIT)/1024);
}
