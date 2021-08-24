#include <stdio.h>
#include "esp_log.h"
#include "MinIMU9.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#define TAG "main" // tag for logging

extern "C" void app_main(void);

void app_main(void)
{
    constexpr int sda_io_num = GPIO_NUM_21;
    constexpr int scl_io_num = GPIO_NUM_22;
    constexpr uint32_t i2c_clk_speed = 50000; // 50 kHz

    i2c_port_t port_num = I2C_NUM_0;
    i2c_config_t i2c_conf;
    i2c_conf.mode = I2C_MODE_MASTER;
    i2c_conf.master.clk_speed = i2c_clk_speed;
    i2c_conf.scl_io_num = scl_io_num;
    i2c_conf.sda_io_num = sda_io_num;
    i2c_conf.scl_pullup_en = false;
    i2c_conf.sda_pullup_en = false;

    constexpr uint32_t sample_time_ms = 10;
    constexpr uint32_t sampling_freq_hz = 1.0 / (sample_time_ms / 1000.0f);
    constexpr float cutoff_freq_hz = 0.5f;
    constexpr float xl_weight = 0.02f;
    constexpr float gyro_weight = 0.98f;

    MinIMU9 imu(port_num, &i2c_conf, sampling_freq_hz, cutoff_freq_hz, xl_weight, gyro_weight);

    imu.Test_LSM6();
    imu.Test_LIS3();

    // Read IMU every 10 ms.
    while(1)
    {
        imu.Read(MinIMU9::Device::BOTH);
        printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\r\n", 
            imu.Calc_Pitch(MinIMU9::Sensor::ACCEL), imu.Calc_Pitch(MinIMU9::Sensor::GYRO), imu.Calc_Pitch(MinIMU9::Sensor::FUSION),
            imu.Calc_Roll(MinIMU9::Sensor::ACCEL), imu.Calc_Roll(MinIMU9::Sensor::GYRO),   imu.Calc_Roll(MinIMU9::Sensor::FUSION),
            imu.Get_Mag_X(), imu.Get_Mag_Y(), imu.Get_Mag_Z());
        vTaskDelay(pdMS_TO_TICKS(sample_time_ms));
    }
}
