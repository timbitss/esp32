#ifndef _REST_H_
#define _REST_H_

#include "esp_err.h"
#include "esp_http_client.h"

// Adjust buffer sizes based on application needs.
#define HTTP_RESPONSE_SIZE 1024 * 2 // Size of HTTP response body buffer.
#define PAYLOAD_SIZE 200            // Max size of payload string.
#define ENDPOINT_SIZE 100           // Max size of endpoint string (URL).

/**
 * @brief HTTP status response codes.
 */
typedef enum
{
    // Successful status codes.
    HTTP_OK = 200,
    HTTP_CREATED = 201,

    // Failure status codes.
    HTTP_BAD_REQUEST = 400,
    HTTP_UNAUTHORIZED = 401,
    HTTP_FORBIDDEN = 403,
    HTTP_NOT_FOUND = 404
} HTTP_status_t;

/**
 * @brief Configuration settings and data for rest_api_config().
 * 
 * @note Setting HTTP_method member to HTTP_METHOD_GET will call parse_json().
 *       Setting HTTP_method member to HTTP_METHOD_POST will send SMS message.
 */
typedef struct
{
    esp_http_client_method_t HTTP_method;                      // HTTP method
    char endpoint[ENDPOINT_SIZE];                              // Endpoint.
    void (*parse_json)(const char *json_string, char *output); // Application-specific JSON string parser.

    /* Payload string. 
     * For GET method, payload string is parsed from received JSON string.
     * For POST method, payload string is SMS message to send via Twilio REST API. */
    char payload_str[PAYLOAD_SIZE]; 
} rest_config_t;

/**
 * @brief Execute REST transaction.
 * 
 * @note GET transaction assumes parsing of JSON.
 *       POST transaction assumes sending SMS via Twilio REST API.
 * 
 * @param[in,out] rest_config_t  Input: rest_config holds HTTP configuration settings.
 *                               Output: rest_config will hold parsed string if HTTP GET request.
 * 
 * @return HTTP response status code.
 */
int rest_execute(rest_config_t *rest_config);

#endif