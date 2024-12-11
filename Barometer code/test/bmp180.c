#include "bmp180.h"
#include "driver/i2c.h"
#include <math.h>

static esp_err_t bmp180_read8(bmp180_dev_t *dev, uint8_t reg, uint8_t *value) {
    return i2c_master_write_read_device(dev->i2c_port, dev->i2c_address, &reg, 1, value, 1, 1000 / portTICK_PERIOD_MS);
}

static esp_err_t bmp180_read16(bmp180_dev_t *dev, uint8_t reg, uint16_t *value) {
    uint8_t data[2];
    esp_err_t ret = i2c_master_write_read_device(dev->i2c_port, dev->i2c_address, &reg, 1, data, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;
    *value = (data[0] << 8) | data[1];  // Combine the two bytes
    return ESP_OK;
}

esp_err_t bmp180_init_desc(bmp180_dev_t *dev, int i2c_port, uint8_t i2c_address) {
    dev->i2c_port = i2c_port;
    dev->i2c_address = i2c_address;
    return ESP_OK;
}

esp_err_t bmp180_init(bmp180_dev_t *dev) {
    uint8_t chip_id;
    if (bmp180_read8(dev, 0xD0, &chip_id) != ESP_OK || chip_id != 0x55) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t bmp180_measure_temperature(bmp180_dev_t *dev, float *temperature) {
    uint16_t raw_temp;
    if (bmp180_read16(dev, 0xF6, &raw_temp) != ESP_OK) {
        return ESP_FAIL;
    }
    *temperature = (float)raw_temp / 10.0; // Assuming temperature data is in 0.1°C units
    return ESP_OK;
}

esp_err_t bmp180_measure_pressure(bmp180_dev_t *dev, float *pressure) {
    uint16_t raw_press;
    if (bmp180_read16(dev, 0xF6, &raw_press) != ESP_OK) {
        return ESP_FAIL;
    }
    *pressure = (float)raw_press; // Assuming pressure data is in hPa units (simplified)
    return ESP_OK;
}

// Function to calculate altitude in meters based on pressure
float bmp180_calculate_altitude(float pressure, float sea_level_pressure) {
    return 44330.0 * (1.0 - pow(pressure / sea_level_pressure, 0.1903));
}
