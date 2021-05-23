#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

//-------------------------------------------------------------------

typedef enum{
    TEMP,     // temperature sensor reading (C)
    HUMIDITY, // humidity sensor reading (%)
    PRESSURE, // pressure sensor reading (Pa)
} ID_t;

typedef struct{
    ID_t sensor;
    uint32_t reading;
} DATA_t;

//-------------------------------------------------------------------

static QueueHandle_t xQueue;

//-------------------------------------------------------------------

// Read data from sensors using queue
void vCentral(void* pvParameters)
{
    const TickType_t xTimeout = pdMS_TO_TICKS(100);
    DATA_t data;
    while(true)
    {
        if(xQueueReceive(xQueue, &data, xTimeout) == pdTRUE) // will always read from queue once data is available
        { 
            switch(data.sensor){
            case TEMP:
                printf("Temp: %d C\n", data.reading);
                break;
            case HUMIDITY:
                printf("Humidity: %d %%\n", data.reading);
                break;
            case PRESSURE:
                printf("Pressure: %d Pa\n", data.reading);
                break;     
            default:
                printf("Could not interpret data.\n");
                break;
            }
        }
        else
        {
            printf("No new data arrived in 100 ms.\n"); // Should never reach this line
        }
    }
}

// Sends sensor readings to central task
void vSensor(void* pvParameters)
{
    DATA_t data = *((DATA_t*)pvParameters);
    while(true)
    {
        if(xQueueSendToBack(xQueue, &data, 0) == pdFAIL)
            printf("Could not send message.\n"); // should never print since queue will at max have 1 item
    }
}


//-------------------------------------------------------------------

void app_main(void)
{
    // Create queue with space for 5 DATA_t structures.
    xQueue = xQueueCreate(5, sizeof(DATA_t));
    
    /* Create tasks. Note that central task has higher priority than sensor tasks.
       Central task will pre-empt sensor tasks when data is available on queue. 
       Therefore, queue will at most have 1 data item inside. 
       IMPORTANT!!! Objects defined in app_main and passed by reference must be declared static 
       to be valid for the program's lifetime. */
    static const DATA_t sensor_Info[3] = {{TEMP, 25}, {HUMIDITY, 43}, {PRESSURE, 30}};
    xTaskCreatePinnedToCore(vSensor, "Temperature sensor", 10000, &(sensor_Info[0]), 2, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(vSensor, "Humidity sensor", 10000, &(sensor_Info[1]), 2, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(vSensor, "Pressure sensor", 10000, &(sensor_Info[2]), 2, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(vCentral, "Central", 10000, NULL, 3, NULL, APP_CPU_NUM);

    // No need to start scheduler for ESP32

}