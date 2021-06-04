/**
 * @file main.c
 * @author Timothy Nguyen
 * @brief I2C Example Interfacing with TMP102 Temperature Sensor
 * @version 0.1
 * @date 2021-06-02
 * 
 * @note Internal SW pull-ups are enabled for SDA and SCL lines
 */

#include <stdio.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// I2C Configuration Parameters
#define I2C_PORT_NUM I2C_NUM_0        // I2C Controller Num [0,1]
#define I2C_MASTER_SDA_IO GPIO_NUM_18 // I2C Master SDA Pin
#define I2C_MASTER_SCL_IO GPIO_NUM_19 // I2C Master SCL Pin Num
#define I2C_MASTER_FREQ_HZ 100000     // SCL frequency (Hz)

// Tags for Data Logging
#define INIT "INIT"           // For i2c_master_init()
#define TEMP_TASK "TEMP_TASK" // For task that reads temperature every econd

// TMP102 I2C Specific Parameters
#define TEMP_DEVICE_ADDR 0x48 // TMP102 device address
#define TEMP_LEN 2            // 12-bit binary temperature value is stored in two registers
#define TEMP_RES 0.0625       // Temperature resolution per count

// General I2C Parameters
#define READ_BIT 0b1  // Request read operation
#define WRITE_BIT 0b0 // Request write operation
#define ACK_EN true   // Enable master ack check
#define ACK_BIT 0b0   // Receiver sets SDA low to acknowledge reception of data
#define NACK_BIT 0b1  // Receiver sets SDA high to terminate reception of data

/**
 * @brief I2C master initialization.
 */
static esp_err_t i2c_master_init(void)
{
    ESP_LOGI(INIT, "Initializing ESP32 as I2C Master.");
    int i2c_port = I2C_PORT_NUM;
    i2c_config_t conf =
        {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = I2C_MASTER_FREQ_HZ,
        };
    ESP_ERROR_CHECK(i2c_param_config(i2c_port, &conf));
    return i2c_driver_install(i2c_port, conf.mode, 0, 0, 0);
}

/**
 * @brief Get raw digital temperature from TMP102 through I2C bus.
 * 
 * @param raw_temp Array to store 12-bit temperature 
 */
static void get_raw(uint8_t raw_temp[2])
{
    // Create command link
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);                                                    // Start bit
    i2c_master_write_byte(i2c_cmd, (TEMP_DEVICE_ADDR << 1) | (READ_BIT), ACK_EN); // Request to read from temp. sensor
    /* The TMP102 pointer register already points to the temperature register.
       Therefore, we can read the 12-bit temperature value w/o additional configuration */
    i2c_master_read(i2c_cmd, raw_temp, TEMP_LEN, ACK_BIT);
    i2c_master_stop(i2c_cmd);
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_PORT_NUM, i2c_cmd, pdMS_TO_TICKS(1000)));
}

/**
 * @brief Read temperature from TMP102 sensor every second.
 * 
 * @param args Optional arguments as void ptr.
 * @note Negative temperature readings not included. 
 */
void i2c_read_temp_task(void *args)
{
    // Read and convert digital temperature format every second
    while (true)
    {
        uint8_t raw_temp[2] = {0};
        get_raw(raw_temp);
        ESP_LOGI(TEMP_TASK, "Raw -> MSB: 0x%02x, LSB: 0x%02x", raw_temp[0], raw_temp[1]); // MSB read first
        uint16_t data = (raw_temp[0] << 4) | (raw_temp[1] >> 4);
        float temperature = (data * TEMP_RES);
        ESP_LOGI(TEMP_TASK, "Temp (Celsius): %f", temperature); 
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreatePinnedToCore(i2c_read_temp_task, "TEMP task", 2048, NULL, 7, NULL, APP_CPU_NUM);
}