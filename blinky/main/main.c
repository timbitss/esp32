#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

//--------------------------------------------------------------

#define GPIO_OUT_BIT_MASK (1ULL << GPIO_NUM_22)  

#define ON 1
#define OFF 0

#define TAG "main"

//--------------------------------------------------------------

void app_main(void)
{
  // setup GPIO22 as GPIO output pin
  gpio_config_t io_config;      
  io_config.pin_bit_mask = GPIO_OUT_BIT_MASK;
  io_config.mode = GPIO_MODE_OUTPUT;
  io_config.pull_down_en = 0;
  io_config.pull_up_en = 0;
  io_config.intr_type = 0;
  if(gpio_config(&io_config) == ESP_ERR_INVALID_ARG) ESP_LOGE(TAG, "Invalid argument to GPIO config function.");
  
  // blinky w/ 250 ms intervals
  uint8_t led_out = 0;
  const TickType_t interval = pdMS_TO_TICKS(250);
  while(true){
    gpio_set_level(GPIO_NUM_22, led_out);
    vTaskDelay(interval);
    led_out ^= 1;
  }

}
