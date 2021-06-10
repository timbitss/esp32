#ifndef _REST_H_
#define _REST_H_

#include "esp_err.h"

#define PAYLOAD_SIZE 200  // Size of payload string
#define ENDPOINT_SIZE 100 // Size of endpoint (URL)

/**
 * @brief REST Client configuration settings and payload data.
 */
typedef struct
{
    // Configure prior to calling rest_get().
    char endpoint[ENDPOINT_SIZE];                                   // URL to get JSON string from.
    esp_err_t (*parse_json)(const char *json_string, char *output); // Application-specific JSON string parser.

    // Data obtained after calling rest_get() with structure.
    char payload_str[PAYLOAD_SIZE];                                 // Payload string parsed from JSON string.
    esp_err_t err;                                                  // Result of parsing JSON.
} payload_config_t;

/**
 * @brief Get JSON string from web server and parse it.
 * 
 * @param[in,out] payload_config As an input, payload_config holds URL and application-specific parser function.
 *                               As an output, payload_config holds payload string and result of parsing JSON.
 */
void rest_get_json(payload_config_t *payload_config);

#endif