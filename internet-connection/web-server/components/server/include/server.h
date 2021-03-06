#ifndef _SERVER_H_
#define _SERVER_H_

#include "esp_http_server.h"

#define LED_SWITCH_NUM GPIO_NUM_15 // LED GPIO number referencing to LED_switch JSON key.

/**
 * @brief Start web server.
 * 
 * @return httpd_handle_t Handler to HTTP server instance.
 *                        If server failed to start, handler returned will be NULL.
 */
httpd_handle_t start_server(void);

/**
 * @brief Stop web server.
 */
void stop_server(httpd_handle_t server);

#endif 