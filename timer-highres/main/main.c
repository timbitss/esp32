#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "esp_system.h"

// Timer IDs
#define AUTORELOAD_ID 0U
#define ONESHOT_ID 1U

// Timer periods
#define AUTORELOAD_PERIOD 100ULL // 100 us period

// Number of autoreload executions before stopping timer
#define EXECUTIONLIMIT 5

//--------------------------------------------------------------------------------------------

static void vTimerCallback( void* arg ) 
{
   // do nothing
}

//--------------------------------------------------------------------------------------------

void app_main(void)
{
    // Create autoreload and one shot timer period.
    const esp_timer_create_args_t periodic_args = {vTimerCallback, (void*) AUTORELOAD_ID, 0, "PERIODIC"};
    const esp_timer_create_args_t oneshot_args = {vTimerCallback, (void*) ONESHOT_ID, 0, "ONESHOT"};
    esp_timer_handle_t periodic_timer, oneshot_timer; 

    ESP_ERROR_CHECK(esp_timer_create(&periodic_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_args, &oneshot_timer));

    // Start timers
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, AUTORELOAD_PERIOD));
    ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, ONESHOT_PERIOD));

    // esp_timer_start_once(oneshot_timer, ONESHOT_PERIOD);
    printf("Started timers. Time since boot = %lld\n", esp_timer_get_time());

    // Print out debug info: name period time_of_next_alarm times_armed times_triggered total_callback_run_time
    // CONFIG_ESP_TIMER_PROFILING option should be defined in menuconfig
    for(uint8_t i = 0; i < 5; ++i) 
    {
        ESP_ERROR_CHECK(esp_timer_dump(stdout));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Delete timers
    esp_timer_delete(oneshot_timer);
    esp_timer_stop(periodic_timer);
    esp_timer_delete(periodic_timer); 
}