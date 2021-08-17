#include <stdio.h>
#include "esp_log.h"
#include "MinIMU9.hpp"

#define TAG "main" // tag for logging

extern "C" void app_main(void);

void app_main(void)
{
    ESP_LOGI(TAG, "Hello World");
}
