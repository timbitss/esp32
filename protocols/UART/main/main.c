/**
 * @file main.c
 * @author Timothy Nguyen
 * @brief UART Pattern Detection Example
 * @version 0.1
 * @date 2021-06-02
 * 
 * @note UART0 port is connected to USB-to-UART bridge. Best to use UART1 or UART2.
 */

#include <stdio.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

// Tags for data logging
#define APP_MAIN "APP_MAIN"
#define UART_TASK "UART_TASK"

// UART Serial Info
#define RX_PIN      GPIO_NUM_10  // RX pin on ESP32
#define TX_PIN      GPIO_NUM_9   // TX pin on ESP32
#define UART_NUM    UART_NUM_1   // UART controller num [0,2]
#define BUF_SZ      1024         // Size of TX(optional)/RX ring buffers and receive buffer in bytes
#define QUEUE_SZ    10           // Number of items in UART event queue and pattern queue

// Pattern detection
#define PATTERN     '+'          // Pattern character 
#define NUM_CHARS   3            // Trigger interrupt when number of consecutive and identical characters is detected
#define BETWEEN_TO  10000        // Maximum gap time between pattern chars (in baud-rate)
#define PRE_IDLE    0            // Minimum idle time before first pattern char (in baud-rate)
#define POST_IDLE   0            // Minimum idle time after last pattern char (in baud-rate)

// Queue to store UART events
QueueHandle_t uart_queue;

/**
 * @brief Handles UART events.
 * 
 * @param args Optional arguments as a void ptr.
 */
void UART_Task(void *args)
{
    uart_event_t event;
    uint8_t* rx_buf = (uint8_t *)malloc(BUF_SZ);
    while (true)
    {
        xQueueReceive(uart_queue, &event, (portTickType)portMAX_DELAY); // wait for event from queue indefinitely
        memset(rx_buf, 0, BUF_SZ);                                      // clear buffer for new read
        switch (event.type)
        {
        case UART_DATA:
            ESP_LOGI(UART_TASK, "UART Data Event, Data Size: %d", event.size);
            uart_read_bytes(UART_NUM, rx_buf, event.size, pdMS_TO_TICKS(100));
            ESP_LOGI(UART_TASK, "Received: %s", rx_buf);
            break;
        case UART_BREAK:
            ESP_LOGI(UART_TASK, "Break Event.");
            break;
        case UART_BUFFER_FULL:
            ESP_LOGI(UART_TASK, "SW RX Ring Buffer Full."); // consider increasing RX ring buffer size
            break;
        case UART_FIFO_OVF:
            ESP_LOGI(UART_TASK, "HW RX FIFO Overflow"); // consider adding HW flow control
            break;
        case UART_FRAME_ERR:
            ESP_LOGI(UART_TASK, "RX Frame Error");
            break;
        case UART_PARITY_ERR:
            ESP_LOGI(UART_TASK, "Parity Event");
            break;
        case UART_DATA_BREAK:
            ESP_LOGI(UART_TASK, "TX Data and Break Event.");
            break;
        case UART_PATTERN_DET:
        {
            size_t buffered_size;
            uart_get_buffered_data_len(UART_NUM, &buffered_size);
            int pos = uart_pattern_pop_pos(UART_NUM); // get position of UART pattern (succeeds data string)
            ESP_LOGI(UART_TASK, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
            if (pos == -1)
            {
                uart_flush(UART_NUM); // detected pattern may not be found in the rx buffer due to overflow.
            }
            else
            {
                // Read data, then pattern from RX ring buffer 
                uart_read_bytes(UART_NUM, rx_buf, pos, pdMS_TO_TICKS(100)); 
                uint8_t pat[NUM_CHARS + 1];
                memset(pat, 0, sizeof(pat));
                uart_read_bytes(UART_NUM, pat, NUM_CHARS, pdMS_TO_TICKS(100));
                ESP_LOGI(UART_TASK, "Data: %s, Pattern: %s", (char*)rx_buf, (char*)pat);
            }
            break;
        }
        case UART_EVENT_MAX:
            ESP_LOGI(UART_TASK, "UART Event Max Index");
            break;
        default:
            ESP_LOGE(UART_TASK, "Could not identify UART event.");
            break;
        }
    }
}

void app_main()
{
    // Configure UART parameters
    ESP_LOGI(APP_MAIN, "Configuring UART1");
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config)); // RTS and CTS pins not used

    // Set UART pins
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Install UART driver with event queue and RX+TX(optional) ring buffers
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SZ, BUF_SZ, QUEUE_SZ, &uart_queue, 0));

    //Enable UART pattern detection.
    ESP_LOGI(APP_MAIN, "Enabling pattern detection.");
    ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(UART_NUM, PATTERN, NUM_CHARS, BETWEEN_TO, POST_IDLE, PRE_IDLE));

    // Allocate a new queue to save the positions of detected patterns in the rx buffer.
    // Theoretically, should only have one pattern position in queue.
    ESP_ERROR_CHECK(uart_pattern_queue_reset(UART_NUM, QUEUE_SZ));

    ESP_LOGI(APP_MAIN, "Creating UART task");
    xTaskCreatePinnedToCore(UART_Task, "UART Task", 1024 * 2, NULL, 12, NULL, APP_CPU_NUM);
}