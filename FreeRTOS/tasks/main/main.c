#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

//-------------------------------------------------------------------

#define TAG "app_main" // tag for logging

//-------------------------------------------------------------------

// Prints out passed parameter
void vTask(void* pvParameters)
{
  while (1)
  {
    printf("%s\n", (char *) pvParameters);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

//-------------------------------------------------------------------

void app_main(void)
{

  // Creates two tasks with unique parameters. 
  // ESP32's FreeRTOS port distributes tasks between two cores unless task is pinned to core.
  // This form of parallelism is known as symmetric multiprocessing (SMP).
  static const char *pcTextForTask1 = "Task 1 is running\n"; 
  static const char *pcTextForTask2 = "Task 2 is running\n";
  TaskHandle_t task1, task2;
  xTaskCreate(vTask, "Task 1", 2048, (void*)pcTextForTask1, 2, &task1); 
  configASSERT(task1);
  xTaskCreate(vTask, "Task 2", 2048, (void*)pcTextForTask2, 2, &task2);
  configASSERT(task2);
  ESP_LOGI(TAG, "Tasks created.");

  // FreeRTOS scheduler already started on application startup. 
  // vTaskStartScheduler();

  // Delete tasks after ~ 5 seconds.
  vTaskDelay(pdMS_TO_TICKS(5000));
  vTaskDelete(task1);
  vTaskDelete(task2);
  ESP_LOGI(TAG, "Tasks deleted.");

  // Should never reach this point if scheduler is active.
}