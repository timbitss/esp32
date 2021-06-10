#ifndef _REST_H_
#define _REST_H_

#define PAYLOAD_SIZE 300  // Size of payload string
#define ENDPOINT_SIZE 100 // Size of endpoint (URL)

/**
 * @brief JSON parse data.
 */
typedef struct
{
    char endpoint[ENDPOINT_SIZE];                              // URL to get JSON string from.
    void (*parse_json)(const char *json_string, char *output); // Application-specific JSON string parser.
    char payload_str[PAYLOAD_SIZE];                            // Payload string parsed from JSON string.
} json_parse_t;

/**
 * @brief GET resource located at URL using REST/HTTP API
 * 
 * @param url URL of resource
 */
void rest_get(json_parse_t *json_parse_ptr);

#endif