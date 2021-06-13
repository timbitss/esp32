#ifndef _TMP102_H_
#define _TMP102_H_

// I2C Configuration Parameters:
#define I2C_MASTER_SDA_IO GPIO_NUM_18 // I2C Master SDA Pin
#define I2C_MASTER_SCL_IO GPIO_NUM_19 // I2C Master SCL Pin Num
#define I2C_MASTER_FREQ_HZ 100000     // SCL frequency (Hz)
#define I2C_PORT_NUM I2C_NUM_0        // I2C Controller Num [0,1]

#define STACK_DEPTH 2048 // Stack depth for i2c_read_temp_task()

/**
 * @brief Initialize I2C Controller 0 and create a task to read the temperature periodically.
 */
void tmp102_init(void);

/**
 * @brief Get the temperature.
 * 
 * @note tmp102_init() must be called once prior to calling this function.
 * 
 * @return float: Temperature in degrees celsius.
 */
float tmp102_get_temp(void);

/**
 * @brief Delete read_temp_task.
 */
void tmp102_stop(void);

#endif