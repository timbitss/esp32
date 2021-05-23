#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

//-------------------------------------------------------------------

static SemaphoreHandle_t xBinarySemaphore; 

//-------------------------------------------------------------------

// Waits for HTTP message, then signals vTask 
void vListenForHTTP(void* pvParameters)
{
 while(true){
   printf("Received http message\n");
   xSemaphoreGive(xBinarySemaphore);
   printf("Processed http message\n");
   vTaskDelay(pdMS_TO_TICKS(5000));
 }
}

// Waits for signal from vTask, then does something
void vTask(void* pvParameters)
{
 while(true){
   xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
   printf("Doing something with http\n");
 }
}

//-------------------------------------------------------------------

void app_main(void)
{
  // Create binary semaphore and tasks
  xBinarySemaphore = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(vListenForHTTP, "ListenToHTTP", 8000, NULL, 2, NULL, APP_CPU_NUM);
  xTaskCreatePinnedToCore(vTask, "Task1", 8000, NULL, 1, NULL, APP_CPU_NUM);

  // No need to start scheduler for ESP32

}