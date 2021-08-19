/**
 * @file MinIMU9.h
 * @author Timothy Nguyen
 * @brief Pololu MinIMU-9 v5 Drivers for the ESP32-Pico-D4.
 * @date 2021-08-16
 * 
 * The IMU board features an LSM6DS33 accelerometer and gyrometer and an LIS3MD magnetometer.
 */

#pragma once

#include <cstdint>
#include "driver/i2c.h"

/* Raw IMU measurements in three axes. */
template<typename T> struct IMU_vector
{
    T x, y, z;
    IMU_vector(): x{0}, y{0}, z{0}{}
};

class MinIMU9
{
public:
    MinIMU9(i2c_port_t i2c_port_num, const i2c_config_t *i2c_conf); 
    ~MinIMU9();

    bool Test_LSM6(); 
    void Read();
    float Calc_Pitch_Angle(); // Angle of x-axis relative to ground [-90°, +90°].  
    float Calc_Roll_Angle();  // Angle of y-axis relative to ground [-180°, +180°].

    // Last values read from IMU. 
    IMU_vector<float> xl;   // [g] 
    IMU_vector<float> gyro; // [dps]
    IMU_vector<float> mag; 

private:
    // I2C Device Addresses 
    enum class I2C_device_addr
    {
        LSM6DS33_DEVICE_ADDR = 0b1101011, // Accelerometer and gyrometer.
        LIS3MD_DEVICE_ADDR = 0b0011110    // Magnetometer.
    };
    // LSM6DS33 Register addresses. 
    enum class LSM6_reg_addr
    {
        FUNC_CFG_ACCESS   = 0x01,

        FIFO_CTRL1        = 0x06,
        FIFO_CTRL2        = 0x07,
        FIFO_CTRL3        = 0x08,
        FIFO_CTRL4        = 0x09,
        FIFO_CTRL5        = 0x0A,
        ORIENT_CFG_G      = 0x0B,

        INT1_CTRL         = 0x0D,
        INT2_CTRL         = 0x0E,
        WHO_AM_I          = 0x0F,
        CTRL1_XL          = 0x10,
        CTRL2_G           = 0x11,
        CTRL3_C           = 0x12,
        CTRL4_C           = 0x13,
        CTRL5_C           = 0x14,
        CTRL6_C           = 0x15,
        CTRL7_G           = 0x16,
        CTRL8_XL          = 0x17,
        CTRL9_XL          = 0x18,
        CTRL10_C          = 0x19,

        WAKE_UP_SRC       = 0x1B,
        TAP_SRC           = 0x1C,
        D6D_SRC           = 0x1D,
        STATUS_REG        = 0x1E,

        OUT_TEMP_L        = 0x20,
        OUT_TEMP_H        = 0x21,
        OUTX_L_G          = 0x22,
        OUTX_H_G          = 0x23,
        OUTY_L_G          = 0x24,
        OUTY_H_G          = 0x25,
        OUTZ_L_G          = 0x26,
        OUTZ_H_G          = 0x27,
        OUTX_L_XL         = 0x28,
        OUTX_H_XL         = 0x29,
        OUTY_L_XL         = 0x2A,
        OUTY_H_XL         = 0x2B,
        OUTZ_L_XL         = 0x2C,
        OUTZ_H_XL         = 0x2D,

        FIFO_STATUS1      = 0x3A,
        FIFO_STATUS2      = 0x3B,
        FIFO_STATUS3      = 0x3C,
        FIFO_STATUS4      = 0x3D,
        FIFO_DATA_OUT_L   = 0x3E,
        FIFO_DATA_OUT_H   = 0x3F,
        TIMESTAMP0_REG    = 0x40,
        TIMESTAMP1_REG    = 0x41,
        TIMESTAMP2_REG    = 0x42,

        STEP_TIMESTAMP_L  = 0x49,
        STEP_TIMESTAMP_H  = 0x4A,
        STEP_COUNTER_L    = 0x4B,
        STEP_COUNTER_H    = 0x4C,

        FUNC_SRC          = 0x53,

        TAP_CFG           = 0x58,
        TAP_THS_6D        = 0x59,
        INT_DUR2          = 0x5A,
        WAKE_UP_THS       = 0x5B,
        WAKE_UP_DUR       = 0x5C,
        FREE_FALL         = 0x5D,
        MD1_CFG           = 0x5E,
        MD2_CFG           = 0x5F,
    };
    /* LIS3MDL Register addresses */
    enum class LIS3_reg_addr
    {
      WHO_AM_I    = 0x0F,

      CTRL_REG1   = 0x20,
      CTRL_REG2   = 0x21,
      CTRL_REG3   = 0x22,
      CTRL_REG4   = 0x23,
      CTRL_REG5   = 0x24,

      STATUS_REG  = 0x27,
      OUT_X_L     = 0x28,
      OUT_X_H     = 0x29,
      OUT_Y_L     = 0x2A,
      OUT_Y_H     = 0x2B,
      OUT_Z_L     = 0x2C,
      OUT_Z_H     = 0x2D,
      TEMP_OUT_L  = 0x2E,
      TEMP_OUT_H  = 0x2F,
      INT_CFG     = 0x30,
      INT_SRC     = 0x31,
      INT_THS_L   = 0x32,
      INT_THS_H   = 0x33,
    };

    void Init_LSM6();
    bool Register_read(I2C_device_addr device_addr, uint8_t reg_addr, uint8_t *data, size_t len);
    bool Register_write_byte(I2C_device_addr device_addr, uint8_t reg_addr, uint8_t data);

    i2c_port_t i2c_port;
    i2c_cmd_handle_t i2c_cmd_handle;
};


