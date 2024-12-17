#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "bmp180.h"

static const char* TAG = "BMP180 I2C Driver";

// I2C-related macros
#define ACK_CHECK_EN    0x1     
#define ACK_CHECK_DIS   0x0     
#define ACK_VAL         0x0     
#define NACK_VAL        0x1     
#define BMP180_ADDRESS 0x77     

// BMP180 resolution settings
#define BMP180_ULTRA_LOW_POWER  0
#define BMP180_STANDARD         1
#define BMP180_HIGH_RES         2
#define BMP180_ULTRA_HIGH_RES   3

// Calibration parameters registers
#define BMP180_CAL_AC1          0xAA  
#define BMP180_CAL_AC2          0xAC  
#define BMP180_CAL_AC3          0xAE  
#define BMP180_CAL_AC4          0xB0  
#define BMP180_CAL_AC5          0xB2  
#define BMP180_CAL_AC6          0xB4  
#define BMP180_CAL_B1           0xB6  
#define BMP180_CAL_B2           0xB8  
#define BMP180_CAL_MB           0xBA  
#define BMP180_CAL_MC           0xBC  
#define BMP180_CAL_MD           0xBE  

// BMP180 control and data registers
#define BMP180_CONTROL             0xF4  
#define BMP180_DATA_TO_READ        0xF6  
#define BMP180_READ_TEMP_CMD       0x2E  
#define BMP180_READ_PRESSURE_CMD   0x34  

// Calibration parameters (global)
static int16_t ac1, ac2, ac3;
static uint16_t ac4, ac5, ac6;
static int16_t b1, b2, mb, mc, md;
static uint8_t oversampling = BMP180_ULTRA_HIGH_RES;

// Function to write data to I2C slave
static esp_err_t bmp180_master_write_slave(i2c_port_t i2c_num, uint8_t* data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP180_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);  
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Function to write a register value to BMP180
static esp_err_t bmp180_write_reg(i2c_port_t i2c_num, uint8_t reg, uint8_t cmd)
{
    uint8_t data_wr[] = {reg, cmd};
    esp_err_t err = bmp180_master_write_slave(i2c_num, data_wr, 2);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Write [0x%02x] = 0x%02x failed, err = %d", reg, cmd, err);
    }
    return err;
}

// Function to read data from I2C slave
static esp_err_t bmp180_master_read_slave(i2c_port_t i2c_num, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP180_ADDRESS << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);  
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Function to read a 16-bit value from a register
static esp_err_t bmp180_read_int16(i2c_port_t i2c_num, uint8_t reg, int16_t* value)
{
    esp_err_t err = bmp180_master_write_slave(i2c_num, &reg, 1);
    if (err == ESP_OK) {
        uint8_t data_rd[2] = {0};
        err = bmp180_master_read_slave(i2c_num, data_rd, 2);
        if (err == ESP_OK) {
            *value = (int16_t)((data_rd[0] << 8) | data_rd[1]);
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read [0x%02x] int16 failed, err = %d", reg, err);
    }
    return err;
}

// Function to read an unsigned 16-bit value
static esp_err_t bmp180_read_uint16(i2c_port_t i2c_num, uint8_t reg, uint16_t* value)
{
    esp_err_t err = bmp180_master_write_slave(i2c_num, &reg, 1);
    if (err == ESP_OK) {
        uint8_t data_rd[2] = {0};
        err = bmp180_master_read_slave(i2c_num, data_rd, 2);
        if (err == ESP_OK) {
            *value = (uint16_t)((data_rd[0] << 8) | data_rd[1]);
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read [0x%02x] uint16 failed, err = %d", reg, err);
    }
    return err;
}

// Function to initialize the BMP180 sensor
esp_err_t bmp180_init(i2c_port_t i2c_num, gpio_num_t sda_pin, gpio_num_t scl_pin)
{
    esp_err_t err = ESP_OK;

    // Read all calibration parameters
    err = bmp180_read_int16(i2c_num, BMP180_CAL_AC1, &ac1);
    err |= bmp180_read_int16(i2c_num, BMP180_CAL_AC2, &ac2);
    err |= bmp180_read_int16(i2c_num, BMP180_CAL_AC3, &ac3);
    err |= bmp180_read_uint16(i2c_num, BMP180_CAL_AC4, &ac4);
    err |= bmp180_read_uint16(i2c_num, BMP180_CAL_AC5, &ac5);
    err |= bmp180_read_uint16(i2c_num, BMP180_CAL_AC6, &ac6);
    err |= bmp180_read_int16(i2c_num, BMP180_CAL_B1, &b1);
    err |= bmp180_read_int16(i2c_num, BMP180_CAL_B2, &b2);
    err |= bmp180_read_int16(i2c_num, BMP180_CAL_MB, &mb);
    err |= bmp180_read_int16(i2c_num, BMP180_CAL_MC, &mc);
    err |= bmp180_read_int16(i2c_num, BMP180_CAL_MD, &md);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data, err = %d", err);
        return ESP_ERR_BMP180_NOT_DETECTED;
    }

    return ESP_OK;
}

// Function to read the temperature from BMP180 sensor
esp_err_t bmp180_read_temperature(float* temperature)
{
    esp_err_t err = ESP_OK;
    uint8_t data[2];
    
    // Write to BMP180 control register to start temperature measurement
    err = bmp180_write_reg(I2C_NUM_0, BMP180_CONTROL, BMP180_READ_TEMP_CMD);
    if (err != ESP_OK) return err;
    
    // Wait for the measurement to complete (4.5ms max)
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // Read the temperature data from the sensor
    err = bmp180_master_read_slave(I2C_NUM_0, data, 2);
    if (err != ESP_OK) return err;

    // Calculate the temperature (using calibration data)
    int16_t raw_temp = (data[0] << 8) | data[1];
    int32_t X1 = ((raw_temp - ac6) * ac5) >> 15;
    int32_t X2 = (mc << 11) / (X1 + md);
    int32_t B5 = X1 + X2;
    *temperature = (B5 + 8) >> 4;  // Convert to temperature in Celsius
    
    return ESP_OK;
}

// Function to read the pressure from BMP180 sensor
esp_err_t bmp180_read_pressure(uint32_t* pressure)
{
    esp_err_t err = ESP_OK;
    uint8_t data[3];
    
    // Write to BMP180 control register to start pressure measurement
    err = bmp180_write_reg(I2C_NUM_0, BMP180_CONTROL, BMP180_READ_PRESSURE_CMD);
    if (err != ESP_OK) return err;
    
    // Wait for the measurement to complete (26.5ms max for ultra high res)
    vTaskDelay(30 / portTICK_PERIOD_MS);

    // Read the pressure data from the sensor
    err = bmp180_master_read_slave(I2C_NUM_0, data, 3);
    if (err != ESP_OK) return err;

    // Combine the pressure data
    *pressure = ((data[0] << 16) | (data[1] << 8) | data[2]) >> (8 - oversampling);

    return ESP_OK;
}

// Function to calculate altitude based on pressure
esp_err_t bmp180_read_altitude(uint32_t reference_pressure, float* altitude)
{
    float temp = 0;
    uint32_t pressure = 0;
    esp_err_t err = bmp180_read_temperature(&temp);
    if (err != ESP_OK) return err;

    // Read the current pressure
    err = bmp180_read_pressure(&pressure);
    if (err != ESP_OK) return err;

    // Calculate the altitude using the barometric formula
    *altitude = 44330.0 * (1.0 - pow(((float)reference_pressure / (float)pressure), 0.1903));

    return ESP_OK;
}
