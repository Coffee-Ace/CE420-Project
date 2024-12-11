#include "bmp180.h"
#include "driver/i2c.h"
#include <math.h>

// BMP180 I2C Commands
#define BMP180_READ_TEMP_CMD 0x2E  // Command to read temperature (MSB)
#define BMP180_READ_PRESSURE_CMD 0x34  // Command to read pressure (mode 3)

// Calibration constants for the BMP180 sensor
#define AC1  408
#define AC2 -72
#define AC3 -14383
#define AC4  32741
#define AC5  32757
#define AC6  23153
#define B1  6190
#define B2  4
#define MB -32768
#define MC -8711
#define MD  2868

// Function to read 8-bit data from BMP180
static esp_err_t bmp180_read8(bmp180_dev_t *dev, uint8_t reg, uint8_t *value) {
    return i2c_master_write_read_device(dev->i2c_port, dev->i2c_address, &reg, 1, value, 1, 1000 / portTICK_PERIOD_MS);
}

// Function to read raw temperature data
esp_err_t bmp180_read_raw_temperature(bmp180_dev_t *dev, int16_t *raw_temp) {
    uint8_t temp_cmd = BMP180_READ_TEMP_CMD;
    if (bmp180_read8(dev, temp_cmd, (uint8_t *)raw_temp) != ESP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

// Function to read raw pressure data
esp_err_t bmp180_read_raw_pressure(bmp180_dev_t *dev, int32_t *raw_press) {
    uint8_t press_cmd = BMP180_READ_PRESSURE_CMD;
    uint8_t buffer[3];
    if (bmp180_read8(dev, press_cmd, buffer) != ESP_OK) {
        return ESP_FAIL;
    }
    *raw_press = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
    return ESP_OK;
}

// Function to calculate temperature in Celsius and return B5 value
float bmp180_calculate_temperature(int16_t raw_temp, int32_t *B5) {
    int32_t X1 = (raw_temp - AC6) * AC5 / (1 << 15);
    int32_t X2 = MC * (1 << 11) / (X1 + MD);
    *B5 = X1 + X2;  // Store B5 value
    return (*B5 + 8) / 16.0;  // Convert to temperature in Celsius
}

// Function to calculate pressure in Pascals
float bmp180_calculate_pressure(int32_t raw_press, int32_t B5) {
    int32_t B6 = B5 - 4000;
    int32_t X1 = (B2 * (B6 * B6) >> 12) >> 11;
    int32_t X2 = (AC2 * B6) >> 11;
    int32_t X3 = X1 + X2;
    int32_t B3 = (((AC1 * 4 + X3) << 1) + 2) >> 2;
    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = X1 + X2;
    uint32_t B4 = (AC4 * (uint32_t)(X3 + 32768)) >> 15;
    uint32_t B7 = ((uint32_t)raw_press - B3) * (50000);
    if (B7 < 0x80000000) {
        raw_press = (B7 * 2) / B4;
    } else {
        raw_press = (B7 / B4) * 2;
    }
    X1 = (raw_press >> 8) * (raw_press >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * raw_press) >> 16;
    return (raw_press + (X1 + X2 + 3791)) >> 4;
}

// Function to initiate BMP180 measurement
esp_err_t bmp180_measure(bmp180_dev_t *dev, float *temperature, float *pressure) {
    int16_t raw_temp = 0;
    int32_t raw_press = 0;
    int32_t B5 = 0;  // Declare B5 and initialize it

    // Read raw temperature and pressure
    if (bmp180_read_raw_temperature(dev, &raw_temp) != ESP_OK || bmp180_read_raw_pressure(dev, &raw_press) != ESP_OK) {
        return ESP_FAIL;
    }

    // Calculate temperature (and get B5 value)
    *temperature = bmp180_calculate_temperature(raw_temp, &B5);

    // Calculate pressure using the B5 value
    *pressure = bmp180_calculate_pressure(raw_press, B5);

    return ESP_OK;
}

// Function to calculate altitude in meters based on pressure
float bmp180_calculate_altitude(float pressure, float sea_level_pressure) {
    return 44330.0 * (1.0 - pow(pressure / sea_level_pressure, 0.1903));
}
