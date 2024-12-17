// derived from the esp32 tutorial bmp180 file but changed and updated to work with newer version of freertos
//esp32 tutorial link https://github.com/ESP32Tutorials/BMP180-ESP32-ESP-IDF/blob/main/components/bmp180.c
//last changed by Anthony Malone 12/16/2024 2:56pm

#ifndef BMP180_H
#define BMP180_H

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// Define a custom error code for BMP180 not detected
#define ESP_ERR_BMP180_NOT_DETECTED 0x1234

// BMP180 Functions Declarations
esp_err_t bmp180_init(i2c_port_t i2c_num, gpio_num_t sda_pin, gpio_num_t scl_pin);
esp_err_t bmp180_read_temperature(float* temperature);
esp_err_t bmp180_read_pressure(uint32_t* pressure);
esp_err_t bmp180_read_altitude(uint32_t reference_pressure, float* altitude);

#ifdef __cplusplus
}
#endif

#endif  // BMP180_H
