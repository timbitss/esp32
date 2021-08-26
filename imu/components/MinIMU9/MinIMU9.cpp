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
 *                                                       https://www.thierry-lequeu.fr/data/AN3461.pdf
 * 
 * Roll and tilt measurements follow rotation sequence Ryxz (yaw -> roll -> pitch).
 */

#include <cassert>
#include <cmath>
#include <algorithm>
#include "MinIMU9.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "RCFilter.h"

#define PI 3.14159265F

using std::max; using std::min;

static const char *TAG = "MinIMU9";

static inline int sgn(float x);

/**
 * @brief Configure and initialize the I2C Port and IMU device.  
 * 
 * @param i2c_port_num I2C Port Number.
 * @param i2c_conf     I2C Configuration Structure.
 * @param _xl_weight   Accelerometer weight for complementary filter.
 * @param _gyro_weight Gyroscope weight for complementary filter. 
 * 
 * @note _xl_weight and _gyro_weight must sum to 1.
 */
MinIMU9::MinIMU9(i2c_port_t i2c_port_num, const i2c_config_t *i2c_conf, float _xl_weight, float _gyro_weight) 
: i2c_port{i2c_port_num}, pitch{0.0f}, roll{0.0f}, heading{0.0f}
{
    /* Initialize I2C Port */
    ESP_ERROR_CHECK(i2c_param_config(i2c_port_num, i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_port_num, i2c_conf->mode, 0, 0, 0));
    ESP_LOGI(TAG, "Initialized I2C Port #%d", i2c_port_num);

    /* Initialize LSM6DS33 */
    Init_LSM6();
    ESP_LOGI(TAG, "Initialized LSM6DS33");

    /* Initialize LIS3MD */
    Init_LIS3();
    ESP_LOGI(TAG, "Initialized LIS3MD");

    if(_xl_weight + _gyro_weight == 1.0f)
    {
        xl_weight = _xl_weight;
        gyro_weight = _gyro_weight;
    }
    else
    {
        ESP_LOGW(TAG, "Weights for complementary filter do not add to 1.");
        // Assign default weights.
        xl_weight = 0.02f;
        gyro_weight = 0.98f;
    }

    ESP_LOGI(TAG, "MinIMU9 object constructed.");
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
 * @brief Check if correct value is read from the LSM6's WHO_AM_I register.
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
 * @brief Check if correct value is read from the LIS3's WHO_AM_I register.
 * 
 * @return true Value of WHO_AM_I register matches expected contents.
 * @return false Value of WHO_AM_I register does not match expected contents.
 */
bool MinIMU9::Test_LIS3()
{
    static constexpr uint8_t LIS3_WHO_AM_I_REG = 0x3D; // Expected value.
    uint8_t read_buf = 0;
    Register_read(I2C_device_addr::LIS3MD_DEVICE_ADDR, (uint8_t)LIS3_reg_addr::WHO_AM_I, &read_buf, 1);
    if (read_buf == LIS3_WHO_AM_I_REG)
    {
        ESP_LOGI(TAG, "Correct value read from LIS3MD WHO_AM_I Register");
        return true;
    }
    else
    {
        ESP_LOGI(TAG, "Incorrect value read from LIS3MD WHO_AM_I Register: %x", read_buf);
        return false;
    }
}

/**
 * @brief Read and store IMU measurements in respective vectors.
 */
void MinIMU9::Read()
{
    Read_LSM6();
    Read_LIS3();
}

/**
 * @brief Calculate pitch, roll, and heading from IMU data.
 */
void MinIMU9::Calc_Orientation()
{
    Calc_Pitch_Fusion();
    Calc_Roll_Fusion();
    Calc_Heading();
}

/**
 * @brief Calibrate magnetometer to account for hard and soft iron sources.
 */
void MinIMU9::Calibrate_Mag()
{
    ESP_LOGI(TAG, "Performing magnetometer calibration...");
    ESP_LOGI(TAG, "Please rotate the device 360° on a horizontal service.");
    float max_x = 0.0f, min_x = 0.0f, max_y = 0.0f, min_y = 0.0f;
    static constexpr uint16_t samples_to_take = 500;

    // Find maximum and minimum X and Y magnetic readings.
    for(uint16_t i = 0; i < samples_to_take; i++)
    {
        Read();
        min_x = min(min_x, mag.x);
        min_y = min(min_y, mag.y);
        max_x = max(max_x, mag.x);
        max_y = max(max_y, mag.y);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Determine scale factors and zero offset values.
    float A = (max_y - min_y) / (max_x - min_x);
    float B = 1.0f / A;
    x_sf = A > 1.0f ? A : 1.0f;
    y_sf = B > 1.0f ? B : 1.0f;
    x_off = ((max_x - min_x) / 2.0f - max_x) * x_sf;
    y_off = ((max_y - min_y) / 2.0f - max_y) * y_sf;
 

    ESP_LOGI(TAG, "Magnetometer calibration completed. " 
                   "x_sf: %.3f, y_sf: %.3f, x_off: %.3f, y_off: %.3f",
                   x_sf, y_sf, x_off, y_off);
}

/**
 * @brief Calculate pitch around the earth's y-axis using the accelerometer.
 * 
 * @return float Pitch angle [-180°, +180°].
 * 
 * IMPORTANT: Either roll or pitch angle must be restricted to [-90°, +90°], but not both! See AN3461 pg. 11.
 */
float MinIMU9::Calc_Pitch_Xl()
{
    return atan2f(xl.x, sgn(-xl.z) * sqrt(xl.y * xl.y + xl.z * xl.z)) * (180.0 / PI);
} 

/**
 * @brief Calculate roll around the earth's x-axis using the accelerometer. 
 *
 * @return float Roll angle [-90°, +90°].
 * 
 * IMPORTANT: Either roll or pitch angle must be restricted to [-90°, +90°], but not both! See AN3461 pg. 11.
 */
float MinIMU9::Calc_Roll_Xl()
{
    return atan2f(-xl.y,  sqrt(xl.x * xl.x + xl.z * xl.z)) * (180.0 / PI);
}  

/**
 * @brief Calculate pitch around the earth's y-axis from gyroscope for sensor fusion.
 * 
 * @return float Pitch angle in degrees.
 */
float MinIMU9::Calc_Pitch_Gyro_Fusion()
{
    static bool first_calc = true;
    static uint32_t t0 = 0;
    
    if(first_calc)
    {
        first_calc = false;
        t0 = esp_timer_get_time() / 1000; 
        return 0.0f;  // First sample, assume Pitch[0] = 0° (at rest).
    }
    else
    {
        // Integrate angular rate.
        uint32_t tnew = esp_timer_get_time() / 1000;
        uint32_t dt = tnew - t0;
        t0 = tnew;
        return pitch + gyro.y * dt / 1000.0f; // Feed back previous pitch estimation.
    }
}

/**
 * @brief Calculate roll around the earth's x-axis from gyroscope for sensor fusion.
 * 
 * @return float Roll angle in degrees.
 */
float MinIMU9::Calc_Roll_Gyro_Fusion()
{
    static bool first_calc = true;
    static uint32_t t0 = 0; 
  
    if(first_calc)
    {
        first_calc = false;
        t0 = esp_timer_get_time() / 1000; 
        return 0.0f; // First sample, assume Roll[0] = 0° (at rest).
    }
    else
    {
        // Integrate angular rate.
        uint32_t tnew = esp_timer_get_time() / 1000;
        uint32_t dt = tnew - t0;
        t0 = tnew;
        return roll + gyro.x * dt / 1000.0f; // Feed back previous roll estimation.
    } 
}

/**
 * @brief Calculate pitch angle by fusing gyroscope and accelerometer estimations.
 */
void MinIMU9::Calc_Pitch_Fusion()
{
    pitch = xl_weight * Calc_Pitch_Xl() + gyro_weight * Calc_Pitch_Gyro_Fusion(); 
}

/**
 * @brief Calculate roll angle by fusing gyroscope and accelerometer estimations.
 */
void MinIMU9::Calc_Roll_Fusion()
{
    roll = xl_weight * Calc_Roll_Xl() + gyro_weight * Calc_Roll_Gyro_Fusion();
}

/**
 * @brief Calculate heading, the angle between the longitudinal axis (the plane body) 
 *        and true north, from the magnetometer.
 * 
 * @return float Heading angle [0°, +359°].
 */
void MinIMU9::Calc_Heading()
{
    float pitch_rad = pitch * (PI / 180.0f);
    float roll_rad = roll * (PI / 180.0f);

    // Transform magnetic readings to Earth's horizontal plane aka tilt compensation.
    float X_h = mag.x * cosf(pitch_rad) + mag.y * sinf(roll_rad) * sinf(pitch_rad) - mag.z * cos(roll_rad) * sin(pitch_rad);
    float Y_h = mag.y * cosf(roll_rad) + mag.z * sinf(roll_rad);

    X_h = x_sf * X_h + x_off;
    Y_h = y_sf * Y_h + y_off;

    // Calculate heading relative to magnetic north in degrees.
    heading = atan2f(Y_h, X_h) * (180.0f / PI);
    if(heading < 0)
    {
        heading += 360; 
    }

    // Add declination angle to get heading relative to true north.
    // See https://www.ngdc.noaa.gov/geomag/faqgeom.shtml#How_do_I_correct_my_compass_bearing_to_true_bearing
    // static constexpr float declination = 15.99f; // In Vancouver, BC.
    // heading += declination;
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

    // Flip sign of rotation in Y and Z direction.
    Register_write_byte(I2C_device_addr::LSM6DS33_DEVICE_ADDR, (uint8_t)LSM6_reg_addr::ORIENT_CFG_G, 0x18);

    // Enable Block Data Update (BDU) feature.
    Register_write_byte(I2C_device_addr::LSM6DS33_DEVICE_ADDR, (uint8_t)LSM6_reg_addr::CTRL3_C, 0x44);

    // Enable and HPF for gyrosope to minimize drift.
    Register_write_byte(I2C_device_addr::LSM6DS33_DEVICE_ADDR, (uint8_t)LSM6_reg_addr::CTRL7_G, 0xC0);

    // Reset HPF.
    Register_write_byte(I2C_device_addr::LSM6DS33_DEVICE_ADDR, (uint8_t)LSM6_reg_addr::CTRL7_G, 0xC8);
}

/**
 * @brief Initialize LIS3MD magnetometer. 
 * 
 * FS = +- 4 gauss by default.
 */
void MinIMU9::Init_LIS3()
{   
    // Sets UHP mode on the X/Y axes, ODR at 80 Hz and activates temperature sensor. 
    Register_write_byte(I2C_device_addr::LIS3MD_DEVICE_ADDR, (uint8_t)LIS3_reg_addr::CTRL_REG1, 0xFC);

    // Sets UHP mode on the Z-axis.
    Register_write_byte(I2C_device_addr::LIS3MD_DEVICE_ADDR, (uint8_t)LIS3_reg_addr::CTRL_REG4, 0x0C);

    // Enable Block Data Update (BDU) feature.
    Register_write_byte(I2C_device_addr::LIS3MD_DEVICE_ADDR, (uint8_t)LIS3_reg_addr::CTRL_REG5, 0x40);

    // Enable magnetometer in continuous-conversion mode.
    Register_write_byte(I2C_device_addr::LIS3MD_DEVICE_ADDR, (uint8_t)LIS3_reg_addr::CTRL_REG3, 0x00);
}

/**
 * @brief Read and store accelerometer and gyroscope data from the LSM6DS33.
 */
void MinIMU9::Read_LSM6()
{
    static constexpr uint8_t bytes_to_read = 12;             // 2 bytes * 6 axes.
    static constexpr float xl_sensitivity_factor = 0.061f;   // [mg/LSB], for FS = +-2 g.
    static constexpr float gyro_sensitivity_factor = 8.75f;  // [mdps/LSB], for FS = +-250 dps.
    uint8_t read_buf[bytes_to_read] = {0};

    Register_read(I2C_device_addr::LSM6DS33_DEVICE_ADDR, (uint8_t)LSM6_reg_addr::OUTX_L_G, read_buf, bytes_to_read);

    // Perform concatenation and scaling.
    gyro.x = (int16_t)(read_buf[1] << 8 | read_buf[0]) * gyro_sensitivity_factor / 1000;
    gyro.y = (int16_t)(read_buf[3] << 8 | read_buf[2]) * gyro_sensitivity_factor / 1000;
    gyro.z = (int16_t)(read_buf[5] << 8 | read_buf[4]) * gyro_sensitivity_factor / 1000;
    xl.x = (int16_t)(read_buf[7] << 8 | read_buf[6]) * xl_sensitivity_factor / 1000;
    xl.y = -(int16_t)(read_buf[9] << 8 | read_buf[8]) * xl_sensitivity_factor / 1000;   // Note sign change.
    xl.z = -(int16_t)(read_buf[11] << 8 | read_buf[10]) * xl_sensitivity_factor / 1000; // Note sign change.
}

/**
 * @brief Read and store magnetometer data from the LIS3MD.
 */
void MinIMU9::Read_LIS3()
{
    static constexpr uint8_t bytes_to_read = 6;  // 2 bytes * 6 axes.
    // From the datasheet: 
    // "In order to read multiple bytes, it is necessary to assert the most significant bit of the subaddress."
    static constexpr uint8_t starting_sub_addr = uint8_t(LIS3_reg_addr::OUT_X_L) | 0x80;
    static constexpr float mag_sensitivity_factor = 1.0f/6842.0f; // [gauss/LSB], for FS = +-4 gauss.
    uint8_t read_buf[bytes_to_read] = {0};

    Register_read(I2C_device_addr::LIS3MD_DEVICE_ADDR, starting_sub_addr, read_buf, bytes_to_read);

    // Perform concatenation and scaling.
    mag.x = (int16_t)(read_buf[1] << 8 | read_buf[0]) * mag_sensitivity_factor;
    mag.y = -(int16_t)(read_buf[3] << 8 | read_buf[2]) * mag_sensitivity_factor; // Note sign change.
    mag.z = -(int16_t)(read_buf[5] << 8 | read_buf[4]) * mag_sensitivity_factor; // Note sign change.
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
    return (x >= 0) ? 1 : -1;
}