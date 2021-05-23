#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

//-------------------------------------------------------------------

#define APP_MAIN "app_main" // tags for logging
#define SENDER "sender"
#define RECEIVER "receiver"

//-------------------------------------------------------------------

static TaskHandle_t receive_task;
 
//-------------------------------------------------------------------

// Receives notifications
// Note that receiver only reads message from sender that last sent message. 
void vReceiver(void* pvParameters)
{
  while(1)
  {
    uint32_t notif_val = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 
    ESP_LOGI(RECEIVER, "Message received from sender %d", notif_val);                                     
  }

}

// Sends notification to receiver task every 3 seconds
void vSender(void* pvParameters)
{
  while (1)
  {
    xTaskNotify(receive_task, *((uint32_t*)pvParameters), eSetValueWithOverwrite); 
    ESP_LOGI(SENDER, "Message sent from sender %d", *((uint32_t*)pvParameters));
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

//-------------------------------------------------------------------

void app_main(void)
{
  // Create 1 receive task
  xTaskCreate(vReceiver, "Receiver", 10000, NULL, 2, &receive_task);
  ESP_LOGI(APP_MAIN, "Created receiver tasks.");

  // Create 4 sender tasks with unique ID
  static const uint32_t sender_ID[4] = {0, 1, 2, 3};
  xTaskCreate(vSender, "Sender0", 8000, (void*)sender_ID, 2, NULL);
  xTaskCreate(vSender, "Sender1", 8000, (void*)(sender_ID+1), 2, NULL);
  xTaskCreate(vSender, "Sender2", 8000, (void*)(sender_ID+2), 2, NULL);
  xTaskCreate(vSender, "Sender3", 8000, (void*)(sender_ID+3), 2, NULL);
  ESP_LOGI(APP_MAIN, "Created sender tasks.");

  // Scheduler already started, does not need to be called.
  // vTaskStartScheduler();

}