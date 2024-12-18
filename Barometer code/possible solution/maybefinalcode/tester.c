//last changed by Anthony Malone 12/18/2024 1:18pm
// tester.c
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "esp_log.h"
#include "bmp180.h"

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

static const char* TAG = "Tester";

// Create a queue to send BMP180 data
typedef struct {
    float temperature;
    uint32_t pressure;
} bmp180_data_t;

QueueHandle_t bmp180_queue;

// Task to read BMP180 data and send it to the queue
void bmp180_task(void* param) {
    bmp180_data_t data;
    esp_err_t ret;

    while (1) {
        ret = bmp180_read_temperature(&data.temperature);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read temperature");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ret = bmp180_read_pressure(&data.pressure);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read pressure");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Send data to the queue
        if (xQueueSend(bmp180_queue, &data, pdMS_TO_TICKS(1000)) != pdPASS) {
            ESP_LOGE(TAG, "Failed to send data to queue");
        }

        vTaskDelay(pdMS_TO_TICKS(5000));  // Delay 5 seconds
    }
}

// Task to receive data from the queue and print it
void print_task(void* param) {
    bmp180_data_t data;

    while (1) {
        if (xQueueReceive(bmp180_queue, &data, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Temperature: %.2f °C, Pressure: %u Pa", data.temperature, data.pressure);
        }
    }
}

void app_main(void) {
    // Initialize BMP180
    if (bmp180_init(I2C_NUM_0, SDA_PIN, SCL_PIN) != ESP_OK) {
        ESP_LOGE(TAG, "BMP180 initialization failed");
        return;
    }

    ESP_LOGI(TAG, "BMP180 initialized successfully");

    // Create a queue to hold BMP180 data
    bmp180_queue = xQueueCreate(5, sizeof(bmp180_data_t));
    if (bmp180_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue");
        return;
    }

    // Create tasks
    xTaskCreate(bmp180_task, "bmp180_task", 2048, NULL, 2, NULL);
    xTaskCreate(print_task, "print_task", 2048, NULL, 1, NULL);
}
