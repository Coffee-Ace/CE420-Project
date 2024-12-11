#ifndef BMP180_H
#define BMP180_H

#include "esp_err.h"

#define BMP180_I2C_ADDRESS 0x77

typedef struct {
    int i2c_port;
    uint8_t i2c_address;
} bmp180_dev_t;

esp_err_t bmp180_init_desc(bmp180_dev_t *dev, int i2c_port, uint8_t i2c_address);
esp_err_t bmp180_init(bmp180_dev_t *dev);
esp_err_t bmp180_measure_temperature(bmp180_dev_t *dev, float *temperature);
esp_err_t bmp180_measure_pressure(bmp180_dev_t *dev, float *pressure);
float bmp180_calculate_altitude(float pressure, float sea_level_pressure);

#endif
