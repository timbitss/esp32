/**
 * @file main.c
 * @author Timothy Nguyen
 * @brief Simple GPIO Posedge-Triggered Interrupts with counter using FreeRTOS queue
 * @version 0.1
 * @date 2021-05-28
 * 
 * @note Includes button debouncing
 */

#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"

#define GPIO_INPUT_PINS ((1ULL << GPIO_NUM_21) | (1ULL << GPIO_NUM_27))  // Input pins to read button presses 

/* Tags for ESP Logging */
#define ISR "ISR"
#define TASK "TASK"
#define APPMAIN "APPMAIN"

/* Length of queue */
#define QUEUE_LENGTH 10UL

/* ISR uses queue to notify vTaskCounter which GPIO pin has triggered interrupt */
QueueHandle_t xQueue;

/**
 * @brief Increments and prints button press count
 * 
 * @param gpio_num number correspdonding to GPIO pin
 */
static void incr_count(uint32_t gpio_num);

/**
 * @brief ISR handler for posedge-triggered GPIO pins
 * 
 *        Sends arg to counter task that keeps track of button pushes through a queue.
 * 
 * @param arg GPIO number
 */
static void isr_handler_GPIO(void* args)
{
    gpio_num_t pin_num = (gpio_num_t)args; 
    xQueueSendFromISR(xQueue, &pin_num, NULL);
}

/**
 * @brief Keeps track of button pushes
 * 
 * @param pvParameters NULL
 */
static void vTaskCounter(void* pvParameters)
{
    static gpio_num_t gpio_num = 0;
    xQueue = xQueueCreate(QUEUE_LENGTH, sizeof(gpio_num_t));

    while(true)
    {
        if(xQueueReceive(xQueue, &gpio_num, pdMS_TO_TICKS(5000)) != pdPASS) // wait 5 seconds for new item in queue
            ESP_LOGI(TASK, "Still waiting...");
        else
        {
            /* Wait for button to be released before counting button press */
            gpio_intr_disable(gpio_num); 
            do{
              vTaskDelay(pdMS_TO_TICKS(20)); 
            }while(gpio_get_level(gpio_num) == 1);
            incr_count(gpio_num);
            gpio_intr_enable(gpio_num);
        }
    }
}

void app_main(void)
{

    ESP_LOGI(APPMAIN, "Setting input pins to generate interrupts on posedge w/ software pulldown"); 
    gpio_config_t io_config = {
        .pin_bit_mask = GPIO_INPUT_PINS,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE       
    };
    ESP_ERROR_CHECK(gpio_config(&io_config));

    ESP_LOGI(APPMAIN, "Installing GPIO ISR handler service, then adding only one ISR handler for all GPIOS");
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_NUM_21, isr_handler_GPIO, (void*)GPIO_NUM_21));
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_NUM_27, isr_handler_GPIO, (void*)GPIO_NUM_27));

    ESP_LOGI(APPMAIN, "Creating counter task, pinned to APP_CPU.");
    if(xTaskCreatePinnedToCore(vTaskCounter, TASK, 8 * 1024, NULL, 2, NULL, APP_CPU_NUM) != pdPASS)
        ESP_LOGE(APPMAIN, "Could not create counter task.");

}

static void incr_count(uint32_t gpio_num)
{
    static uint32_t count21 = 0, count27 = 0; // keep track of GPIO21 and GPIO27 button presses
    switch(gpio_num)
    {
        case GPIO_NUM_21:
            count21++;
            ESP_LOGI(TASK, "GPIO21 trigger count: %u", count21);
            break;
        case GPIO_NUM_27:
            count27++;
            ESP_LOGI(TASK, "GPIO27 trigger count: %u", count27);
            break;    
        default:
            ESP_LOGE(TASK, "Could not interepret queue item");
            break;
    }
} 