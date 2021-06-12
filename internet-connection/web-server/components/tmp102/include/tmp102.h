#ifndef _TMP102_H_
#define _TMP102_H_

/**
 * @brief Initialize I2C Controller 0 and create a task to read the temperature periodically.
 */
void tmp102_init(void);

/**
 * @brief Get the temperature.
 * 
 * @note tmp102_init() must be called once prior to calling this function.
 * 
 * @return float Temperature in degrees celsius.
 */
float tmp102_get_temp();

#endif