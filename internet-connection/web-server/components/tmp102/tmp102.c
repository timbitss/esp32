/**
 * @file main.c
 * @author Timothy Nguyen
 * @brief TMP102 Temperature Sensor I2C API. 
 * @version 0.1
 * @date 2021-06-11
 * 
 * @note I2C Controller 0 is initialized with i2c_temp_controller_init().
 *       Negative temperatures are currently not supported.
 */

#include <stdio.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// I2C Configuration Parameters:
#define I2C_MASTER_SDA_IO GPIO_NUM_18 // I2C Master SDA Pin
#define I2C_MASTER_SCL_IO GPIO_NUM_19 // I2C Master SCL Pin Num
#define I2C_MASTER_FREQ_HZ 100000     // SCL frequency (Hz)
#define I2C_PORT_NUM I2C_NUM_0        // I2C Controller Num [0,1]

#define STACK_DEPTH 2048 // Stack depth for i2c_read_temp_task()

// Tags for Data Logging:
#define INIT "INIT"           // For i2c_master_init()
#define TEMP_TASK "TEMP_TASK" // For task that reads temperature every econd

// TMP102 I2C Specific Parameters:
#define TEMP_DEVICE_ADDR 0x48 // TMP102 device address
#define TEMP_LEN 2            // 12-bit binary temperature value is stored in two registers
#define TEMP_RES 0.0625       // Temperature resolution in degrees Celsius per count.

// General I2C Parameters:
#define READ_BIT 0b1  // Request read operation.
#define WRITE_BIT 0b0 // Request write operation.
#define ACK_EN true   // Enable master ack check.
#define ACK_BIT 0b0   // Receiver sets SDA low to acknowledge reception of data.
#define NACK_BIT 0b1  // Receiver sets SDA high to terminate reception of data.

static float temperature_c = 0; // Holds temperature in degrees celsius for get_temp().

/**
 * @brief Get the temperature.
 *        
 * @note tmp102_init() must be called once prior to calling this function.
 * 
 * @return float Temperature in degrees celsius.
 */
float tmp102_get_temp(void)
{
    return temperature_c;
}

static void i2c_read_temp_task(void *args);

/**
 * @brief Initialize I2C Controller 0 and create a task to read the temperature periodically.
 */
void tmp102_init(void)
{
    const i2c_port_t i2c_port = I2C_PORT_NUM;
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
    ESP_ERROR_CHECK(i2c_driver_install(i2c_port, conf.mode, 0, 0, 0));
    ESP_LOGI(INIT, "Initialized I2C Controller 0 as Master.");

    xTaskCreatePinnedToCore(i2c_read_temp_task, "I2C Temperature", STACK_DEPTH, NULL, 5, NULL, APP_CPU_NUM);
}

/**
 * @brief Get raw digital temperature from TMP102 over I2C bus.
 * 
 * @param raw_temp Array to store 12-bit raw temperature. 
 */
static void get_raw_temp(uint8_t raw_temp[2])
{
    // Create I2C command link.
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);                                                    // Take control of I2C bus.
    i2c_master_write_byte(i2c_cmd, (TEMP_DEVICE_ADDR << 1) | (READ_BIT), ACK_EN); // Request to read from temp. sensor

    /* The TMP102 pointer register already points to the temperature register.
       Therefore, we can read the 12-bit temperature value w/o additional configuration. */
    i2c_master_read(i2c_cmd, raw_temp, TEMP_LEN, ACK_BIT);
    i2c_master_stop(i2c_cmd); // Release control of I2C bus.
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_PORT_NUM, i2c_cmd, pdMS_TO_TICKS(1000)));
}

/**
 * @brief Task to read and update temperature every second. 
 *        I2C controller must be initialized prior to calling this function.
 * 
 * @note  Negative temperature readings are not supported currently. 
 * 
 * @param args Optional arguments as void ptr.
 */
static void i2c_read_temp_task(void *args)
{
    while (true)
    {
        uint8_t raw_temp[2] = {0};
        get_raw_temp(raw_temp);
        // ESP_LOGI(TEMP_TASK, "Raw -> MSB: 0x%02x, LSB: 0x%02x", raw_temp[0], raw_temp[1]); // MSB read first by convention.
        uint16_t data = (raw_temp[0] << 4) | (raw_temp[1] >> 4);
        temperature_c = (data * TEMP_RES);
        // ESP_LOGI(TEMP_TASK, "Temp (Celsius): %f", temperature_c);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
