/**
 * @file MinIMU9.cpp
 * @author Timothy Nguyen
 * @brief Pololu MinIMU-9 v5 Drivers for the ESP32-Pico-D4.
 * @date 2021-08-16
 * 
 * The IMU board features an LSM6DS33 accelerometer and gyrometer and an LIS3MD magnetometer.
 */


#include "MinIMU9.h"
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "MinIMU9";

/* I2C Device Addresses */
#define LSM6DS33_DEVICE_ADDR 0b1101011 // Accelerometer and gyrometer.
#define LIS3MD_DEVICE_ADDR 0b0011110   // Magnetometer.

/**
 * @brief Configure and initialize the I2C Port and IMU device.  
 * 
 * @param i2c_port_num I2C Port Number.
 * @param i2c_conf     I2C Configuration Structure.
 */
MinIMU9::MinIMU9(i2c_port_t i2c_port_num, const i2c_config_t *i2c_conf) : i2c_port{i2c_port_num}
{
    /* Initialize I2C Port */
    ESP_ERROR_CHECK(i2c_param_config(i2c_port_num, i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_port_num, i2c_conf->mode, 0, 0, 0));
    ESP_LOGI(TAG, "Initialized I2C Port #%d", i2c_num);

    /* Create I2C Command Queue */
    i2c_cmd_handle = i2c_cmd_link_create();

    /* Initialize LSM6DS33 */
    Init_LSM6();
    ESP_LOGI(TAG, "Initialized LSM6DS33");
}

/**
 * @brief Delete I2C driver and command queue.
 */
MinIMU9::~MinIMU9()
{
    i2c_driver_delete(i2c_port);
    i2c_cmd_link_delete(i2c_cmd_handle);
    ESP_LOGI(TAG, "Deleted I2C Port #%d Driver and command queue", i2c_port);
}

/**
 * @brief Configure and turn on LSM6DS33's accelerometer and gyrometer.
 */
void MinIMU9::Init_LSM6()
{
    LSM6_register_write_byte(LSM6_reg_addr::CTRL1_XL, 0x60); // Acc = 416Hz (High-Performance mode).
    LSM6_register_write_byte(LSM6_reg_addr::CTRL2_G, 0x60);  // Gyro = 416Hz (High-Performance mode).
    LSM6_register_write_byte(LSM6_reg_addr::CTRL3_C, 0x44);  // Enable Block Data Update (BDU) feature.
}

bool MinIMU9::LSM6_register_read(LSM6_reg_addr reg_addr, uint8_t *data, size_t len)
{
    i2c_master_start(i2c_cmd_handle);                                                            // Start transmission.
    i2c_master_write_byte(i2c_cmd_handle, (LSM6DS33_DEVICE_ADDR << 1) | I2C_MASTER_WRITE, true); // Request write operation.
    i2c_master_write_byte(i2c_cmd_handle, (uint8_t)reg_addr, true);                              // Indicate register to read from.
    i2c_master_start(i2c_cmd_handle);                                                            // Repeated start condition.
    i2c_master_write_byte(i2c_cmd_handle, (LSM6DS33_DEVICE_ADDR << 1) | I2C_MASTER_READ, true);  // Request read operation.
    i2c_master_read(i2c_cmd_handle, data, len - 1, I2C_MASTER_ACK);                              // Read variable length of data.
    i2c_master_read(i2c_cmd_handle, data, 1, I2C_MASTER_NACK);
    i2c_master_stop(i2c_cmd_handle);                                                             // Stop transmission.
    i2c_master_cmd_begin(i2c_port, i2c_cmd_handle, pdMS_TO_TICKS(50));                           // Timeout after 50 ms.
    return true;
}

bool MinIMU9::LSM6_register_write_byte(LSM6_reg_addr reg_addr, uint8_t data)
{
    i2c_master_start(i2c_cmd_handle);                                                            // Start transmission.
    i2c_master_write_byte(i2c_cmd_handle, (LSM6DS33_DEVICE_ADDR << 1) | I2C_MASTER_WRITE, true); // Request write operation.
    i2c_master_write_byte(i2c_cmd_handle, (uint8_t)reg_addr, true);                              // Indicate register to write to.
    i2c_master_write_byte(i2c_cmd_handle, data, true);                                           // Write data byte to register.
    i2c_master_stop(i2c_cmd_handle);                                                             // Stop transmission.
    i2c_master_cmd_begin(i2c_port, i2c_cmd_handle, pdMS_TO_TICKS(50));                           // Timeout after 50 ms.
    return true;
}
