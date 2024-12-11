#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "RMC_parse.h"

QueueHandle_t gpsQueue;
QueueHandle_t barometerQueue;
QueueHandle_t accelerometerQueue;


void app_main(void)
{
    xQueueCreate(sizeof(RMC_Struct) * 10, barometerQueue);
    xQueueCreate(sizeof(RMC_Struct) * 10, accelerometerQueue);
    xQueueCreate(sizeof(RMC_Struct) * 10, gpsQueue);

    barometerStart((void *)barometerQueue);
    accelerometerStart((void *)accelerometerQueue);
    gpsStart((void *)gpsQueue);
}
