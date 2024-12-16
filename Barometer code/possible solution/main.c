#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <inttypes.h>  // For PRIu32 macro

#include "bmp180.h"

static const char *TAG = "BMP180 I2C Read";

#define REFERENCE_PRESSURE 101325l  // Standard atmospheric pressure in Pa
#define I2C_PIN_SDA 21
#define I2C_PIN_SCL 22

void bmp180_task(void *pvParameter)
{
    while(1) {
        esp_err_t err;
        uint32_t pressure;
        float altitude;
        float temperature;

        // Read pressure
        err = bmp180_read_pressure(&pressure);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Reading of pressure from BMP180 failed, err = %d", err);
        }

        // Read altitude
        err = bmp180_read_altitude(REFERENCE_PRESSURE, &altitude);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Reading of altitude from BMP180 failed, err = %d", err);
        }

        // Read temperature
        err = bmp180_read_temperature(&temperature);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Reading of temperature from BMP180 failed, err = %d", err);
        }

        // Log results with correct format specifier
        ESP_LOGI(TAG, "Pressure %" PRIu32 " Pa, Altitude %.1f m, Temperature: %.1f degC", pressure, altitude, temperature);

        // Delay before next readings (1 second)
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    esp_err_t err;

    // Initialize BMP180 sensor with I2C pins
    err = bmp180_init(I2C_PIN_SDA, I2C_PIN_SCL);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "BMP180 initialized successfully.");
        // Create the FreeRTOS task to read data from BMP180
        xTaskCreate(&bmp180_task, "bmp180_task", 1024 * 4, NULL, 5, NULL);
    } else {
        ESP_LOGE(TAG, "BMP180 init failed with error = %d", err);
    }
}
