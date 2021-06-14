#ifndef _SERVER_H_
#define _SERVER_H_

#include "esp_http_server.h"

#define LED_SWITCH_NUM GPIO_NUM_15 // LED GPIO number referencing to LED_switch JSON key.

/**
 * @brief Registers endpoint handlers and starts web server.
 */
void http_start_server(void);

/**
 * @brief Stop web server.
 */
void http_stop_server(void);

#endif 