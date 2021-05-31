/**
 * @file main.c
 * @author Timothy Nguyen
 * @brief Fade LEDs on and off using LEDC/PWM peripheral with different periods
 * @version 0.1
 * @date 2021-05-30
 * 
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"

/** For configuring high-speed LEDC peripherals */
#define PWM_FREQ            5000UL                  // PWM frequency in Hz
#define HS_TIMER_SEL        LEDC_TIMER_0            // Timer to drive high-speed channels
#define HS_LED_CHANNEL_0    LEDC_CHANNEL_0          // High-speed channel to drive first LED [0,7] 
#define HS_LED_CHANNEL_1    LEDC_CHANNEL_1          // High-speed channel to drive second LED [0,7]
#define FADE_PERIOD_0_MS    2000                    // Period first LED fades on and off
#define FADE_PERIOD_1_MS    500                     // Period second LED fades on and off


/* For logging data */
#define APPMAIN "APPMAIN"

void app_main()
{
    ESP_LOGI(APPMAIN, "Configuring timer for high-speed LEDC channels");
    ledc_timer_config_t ledc_timer_conf = {
       .speed_mode = LEDC_HIGH_SPEED_MODE,
       .duty_resolution = LEDC_TIMER_10_BIT, // PWM resolution of 0.1% (100% / 1024 possibly duty cycle values)
       .timer_num = HS_TIMER_SEL,            // arbitrary clock module (0-3)
       .freq_hz = PWM_FREQ,
       .clk_cfg = LEDC_AUTO_CLK              // let LEDC driver automatically choose clock source
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_conf));
    
    ESP_LOGI(APPMAIN, "Configuring high-speed channel group");
    static const ledc_channel_config_t ledc_channel_conf[2] = {
        {
        .gpio_num = GPIO_NUM_15,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = HS_LED_CHANNEL_0,
        .intr_type = false,
        .timer_sel = HS_TIMER_SEL,
        .duty = 0,  // start at 0% duty cycle
        .hpoint = 0 // value in timer counter where PWM signal is latched high, see Figure 14-4 on pg.383 for details
        },
        {
        .gpio_num = GPIO_NUM_9,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = HS_LED_CHANNEL_1,
        .intr_type = false,
        .timer_sel = HS_TIMER_SEL,
        .duty = 0,  // start at 0% duty cycle 
        .hpoint = 0 // value in timer counter when PWM signal is latched high, see Figure 14-4 on pg.383 for details
        }
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_conf[0]));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_conf[1]));
    
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(APPMAIN, "Configuring fade for LEDs");
    ledc_fade_func_install(0);
    while(true) // fade on and off with defined fade period
    {
        ledc_set_fade_time_and_start(LEDC_HIGH_SPEED_MODE, HS_LED_CHANNEL_0, 1023, FADE_PERIOD_0_MS/2, LEDC_FADE_NO_WAIT);
        ledc_set_fade_time_and_start(LEDC_HIGH_SPEED_MODE, HS_LED_CHANNEL_1, 1023, FADE_PERIOD_1_MS/2, LEDC_FADE_NO_WAIT);
        ledc_set_fade_time_and_start(LEDC_HIGH_SPEED_MODE, HS_LED_CHANNEL_0, 0, FADE_PERIOD_0_MS/2, LEDC_FADE_NO_WAIT);
        ledc_set_fade_time_and_start(LEDC_HIGH_SPEED_MODE, HS_LED_CHANNEL_1, 0, FADE_PERIOD_1_MS/2, LEDC_FADE_NO_WAIT);
    }    

}