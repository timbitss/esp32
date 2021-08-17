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
};

class MinIMU9
{
public:
    MinIMU9(i2c_port_t i2c_port_num, const i2c_config_t *i2c_conf); 
    ~MinIMU9();

    bool Test_LSM6(); 
    void Read();      
    const IMU_vector<float>* Get_xl();
    const IMU_vector<float>* Get_gyro();
    const IMU_vector<float>* Get_mag();

private:
    /* LSM6DS33 Register addresses. */
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

    void Init_LSM6();
    bool LSM6_register_read(LSM6_reg_addr reg_addr, uint8_t *data, size_t len);
    bool LSM6_register_write_byte(LSM6_reg_addr reg_addr, uint8_t data);

    IMU_vector<float> xl;  
    IMU_vector<float> gyro;
    IMU_vector<float> mag;
    i2c_port_t i2c_port;
    i2c_cmd_handle_t i2c_cmd_handle;
};

