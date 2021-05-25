#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <math.h>

// Pitstop synchronization bits
#define RACECAR (1UL << 0UL)
#define TIRE1 (1UL << 1UL)
#define TIRE2 (1UL << 2UL)
#define TIRE3 (1UL << 3UL)
#define TIRE4 (1UL << 4UL)
#define FUEL (1UL << 5UL)

#define MAXDELAY 5000UL
#define MINDELAY pdMS_TO_TICKS(2000)

//-------------------------------------------------------------------

static EventGroupHandle_t xPitstopEventGroup; // Event group object to implement synchronization

//-------------------------------------------------------------------

// Pitstop crew
void vPitstop(void* pvParameters)
{

    const EventBits_t xTaskSyncBit = (EventBits_t)pvParameters; 
    const TickType_t xDelay = pdMS_TO_TICKS(esp_random() % MAXDELAY) + MINDELAY;
    const EventBits_t xAllSyncBits = (TIRE1|TIRE2|TIRE3|TIRE4|FUEL);
    const uint32_t taskNum = (uint32_t)log2((double)xTaskSyncBit);

    while(true)
    {
        // Wait for racecar to arrive
        printf("Task %u Waiting for racecar to arrive.\n", taskNum);
        xEventGroupWaitBits(xPitstopEventGroup, (EventBits_t)RACECAR, pdTRUE, pdTRUE, portMAX_DELAY);

        // Complete job and wait at sync point
        vTaskDelay(xDelay); 
        printf("Task %u completed -- waiting at sync point.\n", taskNum);
        xEventGroupSync(xPitstopEventGroup, xTaskSyncBit, xAllSyncBits, portMAX_DELAY);
    }
}

// Racecar periodically enters pitstop for repairs.
void vRacecar(void* pvParameters){
    const TickType_t xDelay = pdMS_TO_TICKS(esp_random() % MAXDELAY) + MINDELAY;
    const EventBits_t xAllSyncBits = (TIRE1|TIRE2|TIRE3|TIRE4|FUEL);

    while(true){
        printf("Racecar driving around track\n");
        vTaskDelay(xDelay); 
        printf("Racecar entering pitstop\n");
        xEventGroupSetBits(xPitstopEventGroup, RACECAR); // racecar has entered pitstop, signal pitstop crew
        xEventGroupSync(xPitstopEventGroup, 0, xAllSyncBits, portMAX_DELAY); // wait for pitstop crew to finish
        printf("Racecar exiting pitstop\n");
    }
    
}

//-------------------------------------------------------------------

void app_main(void)
{
    // Create event group to demonstrate synchronization
    xPitstopEventGroup = xEventGroupCreate();

    // Create pitstop tasks and racecar
    xTaskCreatePinnedToCore(vPitstop, "Tire1", 8192, (void*) TIRE1, 2, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(vPitstop, "Tire2", 8192, (void*) TIRE2, 2, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(vPitstop, "Tire3", 8192, (void*) TIRE3, 2, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(vPitstop, "Tire4", 8192, (void*) TIRE4, 2, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(vPitstop, "Fuel", 8192, (void*) FUEL, 2, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(vRacecar, "Racecar", 8192, NULL, 2, NULL, APP_CPU_NUM);
  
    // No need to start scheduler for ESP32
}