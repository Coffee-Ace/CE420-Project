#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "bmp180.h"

// I2C configuration
#define I2C_MASTER_SCL_IO           22    // GPIO for SCL
#define I2C_MASTER_SDA_IO           21    // GPIO for SDA
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

static const char *TAG = "BMP180_APP";

// Sea level pressure in hPa (adjust according to your location)
#define SEA_LEVEL_PRESSURE          1013.25

// Function to initialize the I2C master
void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                                       I2C_MASTER_RX_BUF_DISABLE,
                                       I2C_MASTER_TX_BUF_DISABLE, 0));
}

// FreeRTOS task to read temperature, pressure, and altitude
void bmp180_task(void *pvParameters) {
    bmp180_dev_t bmp180 = { 0 };
    ESP_ERROR_CHECK(bmp180_init_desc(&bmp180, I2C_MASTER_NUM, BMP180_I2C_ADDRESS));
    ESP_ERROR_CHECK(bmp180_init(&bmp180));

    float temperature, pressure, altitude;

    while (1) {
        if (bmp180_measure(&bmp180, &temperature, &pressure) == ESP_OK) {
            altitude = bmp180_calculate_altitude(pressure, SEA_LEVEL_PRESSURE);
            ESP_LOGI(TAG, "Temperature: %.2f °C, Pressure: %.2f hPa, Altitude: %.2f m", temperature, pressure, altitude);
        } else {
            ESP_LOGE(TAG, "Failed to read BMP180 sensor.");
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Initializing I2C...");
    i2c_master_init();

    ESP_LOGI(TAG, "Creating BMP180 task...");
    xTaskCreate(bmp180_task, "bmp180_task", 2048, NULL, 5, NULL);
}
