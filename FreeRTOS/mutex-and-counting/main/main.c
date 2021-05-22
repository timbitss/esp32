#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

//-------------------------------------------------------------------

static SemaphoreHandle_t xMutex; 
static SemaphoreHandle_t xCounting;

static uint32_t num = 0;

//-------------------------------------------------------------------

// Increment num
void increment()
{
  xSemaphoreTake(xMutex, portMAX_DELAY);
  num++;
  xSemaphoreGive(xMutex);
}

// Increment num 800000 times
void vTask(void* pvParameters)
{
  for(uint32_t i = 0; i < 800000UL; i++){
    increment();
  }
  xSemaphoreGive(xCounting);
  vTaskDelete(NULL);
}

//-------------------------------------------------------------------

void app_main(void)
{
  xMutex = xSemaphoreCreateMutex(); 
  xCounting = xSemaphoreCreateCounting(4, 0);

  xTaskCreatePinnedToCore(vTask, "Task1", 8000, NULL, 2, NULL, APP_CPU_NUM);
  xTaskCreatePinnedToCore(vTask, "Task2", 8000, NULL, 2, NULL, APP_CPU_NUM);
  xTaskCreatePinnedToCore(vTask, "Task3", 8000, NULL, 2, NULL, APP_CPU_NUM);
  xTaskCreatePinnedToCore(vTask, "Task4", 8000, NULL, 2, NULL, APP_CPU_NUM);

  // No need to start scheduler for ESP32

  // Wait for 4 tasks to complete
  for(uint8_t i = 0; i < 4; i++)
    xSemaphoreTake(xCounting, portMAX_DELAY);

  printf("Final value: %d\n", num); // should equal 3200000
}