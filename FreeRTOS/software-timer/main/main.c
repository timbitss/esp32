#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_system.h"

// Timer IDs
#define AUTORELOAD_ID 0U
#define ONESHOT_ID 1U

// Timer periods
#define AUTORELOAD_PERIOD pdMS_TO_TICKS(1000)
#define ONESHOT_PERIOD pdMS_TO_TICKS(4000)

// Number of autoreload executions before stopping timer
#define EXECUTIONLIMIT 5

//--------------------------------------------------------------------------------------------

// Stop and delete auto-reload timer after 5 hits
void vTimerCallback( TimerHandle_t xTimer )
{
    uint32_t timer_ID = (uint32_t)pvTimerGetTimerID(xTimer);
    static uint8_t auto_count = 0;

    if(timer_ID == AUTORELOAD_ID)
    {
        auto_count++;
        printf("Autoreload timer %u hit at %lld ms\n", auto_count, esp_timer_get_time()/1000); // us to ms
        if(auto_count == EXECUTIONLIMIT){
            xTimerStop(xTimer, 0);
            xTimerDelete(xTimer, 0);
        }
    }
    else
    {
        printf("Oneshot timer hit at %lld ms\n", esp_timer_get_time()/1000);
        xTimerDelete(xTimer, 0);
    }
}


//--------------------------------------------------------------------------------------------

void app_main(void)
{
    // Create autoreload and one shot timer with specified period.
    TimerHandle_t xPeriodic, xOneShot; 
    xPeriodic = xTimerCreate("Periodic", AUTORELOAD_PERIOD, pdTRUE, (void*)AUTORELOAD_ID, vTimerCallback);
    xOneShot = xTimerCreate("OneShot", ONESHOT_PERIOD, pdFAIL, (void*)ONESHOT_ID, vTimerCallback);

    // Start timers
    if(xTimerStart(xPeriodic, 0) == pdFAIL || xTimerStart(xOneShot, 0) == pdFAIL) 
        printf("Timers could not be started\n.");
}