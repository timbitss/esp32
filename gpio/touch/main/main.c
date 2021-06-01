/**
 * @file main.c
 * @author Timothy Nguyen
 * @brief Touch-triggered interrupts example
 * @version 0.1
 * @date 2021-05-31
 * 
 *  An interrupt is triggered when a pad's count of charge/discharge cycles is below a certain threshold.
 *  This is because a larger capacitance will take longer to charge and discharge (dV/dt = I/C).
 * 
 * @note The pulse count of TOUCH1 (GPIO0) stays at zero and could not be used.
 *       This could be due to a possible hardware connection error with the ESP32-PICO-KIT.
 * 
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/touch_sensor.h"
#include "driver/touch_pad.h"
#include "esp_log.h"

/* For touch sensor drivers */
#define ZERO_THRESHOLD    0      // Initialize GPIOs with zero interrupt threshold            
#define FILTER_PERIOD_MS  10     // IIR Filter Sampling Period (ms)
#define TOUCH_PADS_USED   3     // Number of touch pads used

/* For logging data */
#define APPMAIN     "APPMAIN"
#define TASK        "TASK"
#define ISR         "ISR"

/* Interrupt thresholds of each touch pad */
static uint16_t pad_thresh[TOUCH_PADS_USED] = {0};

/* Touch task handle */
TaskHandle_t touch_task_handle;

/* true = touch pad pressed, false = touch pad not pressed */
static bool pad_touched[TOUCH_PADS_USED] = {0}; 

/**
 * @brief Prints out which touch pads were pressed when notified by ISR
 * 
 * @param args Optional arguments
 */
void touch_task(void* args)
{
    while(1){
        if(xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(5000)) == pdFALSE){
            ESP_LOGI(TASK, "Waiting for touch pad press..."); // timed out after 5 s
        }
        else{ 
            for(uint8_t i = 0; i < TOUCH_PADS_USED; i++){
                if(pad_touched[i] == true){
                    ESP_LOGI(TASK, "Pad #%u pressed!\n", i);
                    pad_touched[i] = false;
                    volatile uint16_t touched_val = 0;
                    /* Wait for button to released */
                    do{
                        vTaskDelay(pdMS_TO_TICKS(20));
                        touch_pad_read_filtered((touch_pad_t)i, &touched_val);  
                    } while(touched_val < pad_thresh[i]);
                }
            }
            
            touch_pad_clear_status(); // clear touch status register
            touch_pad_intr_enable();  // enable interrupts again
        }   
    }
}

/**
 * @brief Update which touch pad(s) was pressed, then notify task
 * 
 * @param args Optional arguments
 */
void touch_intr_handler(void* args)
{
    touch_pad_intr_disable(); // disable touch pad interrupts before processing 
    uint32_t status = touch_pad_get_status();
    for(uint8_t i = 0; i < TOUCH_PADS_USED; i++)
    {
        if(status & (1 << i)) 
            pad_touched[i] = true;
    }
    xTaskNotifyFromISR(touch_task_handle, 0, eNoAction, NULL);
}

/**
 * @brief Set each touch pad interrupt thresholds to equal 2/3 of untouched pulse count.
 * 
 * @note  Do not use touchpads here since it will skew the threshold values. 
 */
void set_thresholds()
{
    touch_pad_filter_start(FILTER_PERIOD_MS);    // Filter pulse count readings first using IIR Filter
    vTaskDelay(pdMS_TO_TICKS(FILTER_PERIOD_MS)); // wait 10 ms before reading filtered value due to hysteresis

    for(uint8_t i = 0; i < TOUCH_PADS_USED; i++)
    {
        if(i != 1){
            uint16_t untouched_count = 0;
            ESP_ERROR_CHECK(touch_pad_read_filtered((touch_pad_t)i, &untouched_count));
            pad_thresh[i] = untouched_count * 2 / 3;
            ESP_ERROR_CHECK(touch_pad_set_thresh((touch_pad_t)i, pad_thresh[i]));
        }
    }

    /* Trigger interrupts when pulse count is below threshold */
    touch_pad_set_trigger_mode(TOUCH_TRIGGER_BELOW); 
}


void app_main()
{
    ESP_LOGI(APPMAIN, "Initializing touch pad module");
    ESP_ERROR_CHECK(touch_pad_init());

    // For touch-triggered interrupts, let hardware periodically 
    // measure the total capacitance on the touch sensor channel using an FSM.
    // Measurement and sleep time can optionally be adjusted.
    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));
    
    // Set range within which the touch pads are charged / discharged.
    // Larger range leads to increased power consumption, E = (1/2)*C*V^2
    // Can also manipulate speed of charging and discharging and measurement/sampling period. 
    ESP_ERROR_CHECK(touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V)); 
    
    // Enable touch sensor functionality for GPIOs with zero threshold initially.
    ESP_LOGI(APPMAIN, "Enabling touch pad functionality for GPIOs");
    ESP_ERROR_CHECK(touch_pad_config(TOUCH_PAD_NUM0, ZERO_THRESHOLD)); 
    ESP_ERROR_CHECK(touch_pad_config(TOUCH_PAD_NUM1, ZERO_THRESHOLD)); 
    ESP_ERROR_CHECK(touch_pad_config(TOUCH_PAD_NUM2, ZERO_THRESHOLD)); 
    
    // We establish touch detection interrupt thresholds. 
    ESP_LOGI(APPMAIN, "Starting IIR filter and setting touch sensor thresholds");
    set_thresholds();
    
    // Now, we can register ISR and enable touch-triggered interrupts.
    ESP_LOGI(APPMAIN, "Registering ISRs");
    ESP_ERROR_CHECK(touch_pad_isr_register(touch_intr_handler, NULL));
    ESP_ERROR_CHECK(touch_pad_intr_enable());

    // Create task that 
    ESP_LOGI(APPMAIN, "Creating task");
    xTaskCreatePinnedToCore(touch_task, TASK, 1024 * 4, NULL, 2, &touch_task_handle, APP_CPU_NUM);

    while(1){
        printf("0: (%u), 1: (%u) 2: (%u) \n", pad_touched[0], pad_touched[1], pad_touched[2]);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}