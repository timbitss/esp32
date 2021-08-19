/**
 * @file MinIMU9.cpp
 * @author Timothy Nguyen
 * @brief Pololu MinIMU-9 v5 Drivers for the ESP32-Pico-D4.
 * @date 2021-08-16
 * 
 * The IMU board features an LSM6DS33 accelerometer and gyrometer and an LIS3MD magnetometer.
 * 
 * Inclination sensing using an accelerometer resources: https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf
 *                                                       https://stanford.edu/class/ee267/notes/ee267_notes_imu.pdf.
 */

#include <cassert>
#include <cmath>
#include "MinIMU9.h"
#include "esp_log.h"
#include "driver/i2c.h"

#define PI 3.14159265F

static const char *TAG = "MinIMU9";

static inline int sgn(float x);

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
    ESP_LOGI(TAG, "Initialized I2C Port #%d", i2c_port_num);

    /* Initialize LSM6DS33 */
    Init_LSM6();
    ESP_LOGI(TAG, "Initialized LSM6DS33");
}

/**
 * @brief Delete I2C driver.
 */
MinIMU9::~MinIMU9()
{
    i2c_driver_delete(i2c_port);
    ESP_LOGI(TAG, "Deleted I2C Port #%d Driver.", i2c_port);
}

/**
 * @brief Check if correct value is read from WHO_AM_I register.
 * 
 * @return true Value of WHO_AM_I register matches expected contents.
 * @return false Value of WHO_AM_I register does not match expected contents.
 */
bool MinIMU9::Test_LSM6()
{
    static constexpr uint8_t LSM6_WHO_AM_I_REG = 0x69; // Expected value.
    uint8_t read_buf = 0;
    Register_read(I2C_device_addr::LSM6DS33_DEVICE_ADDR, (uint8_t)LSM6_reg_addr::WHO_AM_I, &read_buf, 1);
    if (read_buf == LSM6_WHO_AM_I_REG)
    {
        ESP_LOGI(TAG, "Correct value read from LSM6 WHO_AM_I Register");
        return true;
    }
    else
    {
        ESP_LOGI(TAG, "Incorrect value read from LSM6 WHO_AM_I Register: %x", read_buf);
        return false;
    }
}

/**
 * @brief Read and store IMU measurements in respective vectors.
 */
void MinIMU9::Read()
{
    static constexpr uint8_t bytes_to_read = 12;            // 2 bytes * 6 axes.
    static constexpr float xl_sensitivity_factor = 0.061;   // [mg/LSB], for FS = +-2 g.
    static constexpr float gyro_sensitivity_factor = 4.375; // [mdps/LSB], for FS = +-250 dps.
    uint8_t read_buf[bytes_to_read] = {0};

    Register_read(I2C_device_addr::LSM6DS33_DEVICE_ADDR, (uint8_t)LSM6_reg_addr::OUTX_L_G, read_buf, bytes_to_read);

    // Perform concatenation and scaling.
    gyro.x = (int16_t)(read_buf[1] << 8 | read_buf[0]) * gyro_sensitivity_factor / 1000;
    gyro.y = (int16_t)(read_buf[3] << 8 | read_buf[2]) * gyro_sensitivity_factor / 1000;
    gyro.z = (int16_t)(read_buf[5] << 8 | read_buf[4]) * gyro_sensitivity_factor / 1000;
    xl.x = (int16_t)(read_buf[7] << 8 | read_buf[6]) * xl_sensitivity_factor / 1000;
    xl.y = (int16_t)(read_buf[9] << 8 | read_buf[8]) * xl_sensitivity_factor / 1000;
    xl.z = (int16_t)(read_buf[11] << 8 | read_buf[10]) * xl_sensitivity_factor / 1000;
}

/**
 * @brief Calculate pitch around the y-axis.
 * 
 * @return float Pitch angle [-90°, +90°].
 * 
 * IMPORTANT: Either roll or pitch angle must be restricted to [-90°, +90°], but not both! See AN3461 pg. 11.
 */
float MinIMU9::Calc_Pitch_Angle()
{
    return atan2f(xl.x, sqrt(xl.y * xl.y + xl.z * xl.z)) * (180.0 / PI);
} 

/**
 * @brief Calculate roll around the x-axis. 
 * 
 * @return float Roll angle [-180°, +180°].
 * 
 * IMPORTANT: Either roll or pitch angle must be restricted to [-90°, +90°], but not both! See AN3461 pg. 11.
 */
float MinIMU9::Calc_Roll_Angle()
{
    return atan2f(xl.y, sgn(xl.z) * sqrt(xl.x * xl.x + xl.z * xl.z)) * (180.0 / PI);
}  

/**
 * @brief Configure and turn on LSM6DS33's accelerometer and gyrometer.
 * 
 * @note Sign required in the denominator for atan2() to work properly.
 */
void MinIMU9::Init_LSM6()
{
    // Acc: ODR = 416Hz (High-Performance mode), FS = +-2 g.
    Register_write_byte(I2C_device_addr::LSM6DS33_DEVICE_ADDR, (uint8_t)LSM6_reg_addr::CTRL1_XL, 0x60);

    // Gyro: ODR = 416Hz (High-Performance mode), FS = +-250 dps.
    Register_write_byte(I2C_device_addr::LSM6DS33_DEVICE_ADDR, (uint8_t)LSM6_reg_addr::CTRL2_G, 0x60);

    // Enable Block Data Update (BDU) feature.
    Register_write_byte(I2C_device_addr::LSM6DS33_DEVICE_ADDR, (uint8_t)LSM6_reg_addr::CTRL3_C, 0x44);
}

/**
 * @brief Read variable number of bytes from I2C device.
 * 
 * @param device_addr Device address.
 * @param reg_addr    Register address.
 * @param data        Location to store data.
 * @param len         Number of bytes to read.  
 * @return true       Read was successful.
 * @return false      Read was unsuccessful.
 */
bool MinIMU9::Register_read(I2C_device_addr device_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{
    // Create I2C command queue.
    i2c_cmd_handle = i2c_cmd_link_create();
    assert(i2c_cmd_handle != NULL);

    // Append R/~W bit to device address.
    uint8_t device_addr_write = (uint8_t)device_addr << 1;
    uint8_t device_addr_read = device_addr_write | 0x01;

    // Add I2C commands to queue.
    ESP_ERROR_CHECK(i2c_master_start(i2c_cmd_handle));              // Start transmission.
    i2c_master_write_byte(i2c_cmd_handle, device_addr_write, true); // Request write operation
    i2c_master_write_byte(i2c_cmd_handle, reg_addr, true);          // Indicate register to read from.
    i2c_master_start(i2c_cmd_handle);                               // Repeated start condition.
    i2c_master_write_byte(i2c_cmd_handle, device_addr_read, true);  // Request read operation.
    if (len > 1)                                                    // Read variable length of data.
    {
        i2c_master_read(i2c_cmd_handle, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read(i2c_cmd_handle, data + len - 1, 1, I2C_MASTER_NACK);
    i2c_master_stop(i2c_cmd_handle); // Stop transmission.

    // Execute transaction.
    ESP_ERROR_CHECK(i2c_master_cmd_begin(i2c_port, i2c_cmd_handle, pdMS_TO_TICKS(1000)));

    // Delete command queue.
    i2c_cmd_link_delete(i2c_cmd_handle);
    return true;
}

/**
 * @brief Write a byte into an I2C register.
 * 
 * @param device_addr I2C device address.
 * @param reg_addr    Register addresss.
 * @param data        Byte to write to register.
 * @return true       Write was successful.
 * @return false      Write was not successful.
 */
bool MinIMU9::Register_write_byte(I2C_device_addr device_addr, uint8_t reg_addr, uint8_t data)
{
    // Create I2C command queue
    i2c_cmd_handle = i2c_cmd_link_create();
    assert(i2c_cmd_handle != NULL);

    // Append ~W bit to device address.
    uint8_t device_addr_write = (uint8_t)device_addr << 1;

    // Add I2C commands to queue.
    ESP_ERROR_CHECK(i2c_master_start(i2c_cmd_handle));              // Start transmission.
    i2c_master_write_byte(i2c_cmd_handle, device_addr_write, true); // Request write operation
    i2c_master_write_byte(i2c_cmd_handle, (uint8_t)reg_addr, true); // Indicate register to write to.
    i2c_master_write_byte(i2c_cmd_handle, data, true);              // Write data byte to register.
    i2c_master_stop(i2c_cmd_handle);                                // Stop transmission.

    // Execute transaction.
    ESP_ERROR_CHECK(i2c_master_cmd_begin(i2c_port, i2c_cmd_handle, pdMS_TO_TICKS(1000)));

    // Delete command queue.
    i2c_cmd_link_delete(i2c_cmd_handle);
    return true;
}

static inline int sgn(float x)
{
    return (x >= 0) ? 1.0F : -1.0F;
}