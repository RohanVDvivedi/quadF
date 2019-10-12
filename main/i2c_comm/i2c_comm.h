#ifndef I2C_COMM_H
#define I2C_COMM_H

// for i2c for sensors
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO   22
#define I2C_MASTER_SDA_IO   21

// basic i2c functionality, could be similar for most sensors, being user for mpu6050 and will be used for ms5611
void i2c_init();
esp_err_t i2c_read(uint8_t device_address, uint8_t reg_address, void* buffer, unsigned int bytes_to_read);
esp_err_t i2c_write(uint8_t device_address, uint8_t reg_address, void* buffer, unsigned int bytes_to_write);
esp_err_t i2c_read_raw(uint8_t device_address, void* buffer, unsigned int bytes_to_read);
esp_err_t i2c_write_raw(uint8_t device_address, void* buffer, unsigned int bytes_to_write);
void i2c_destroy();

#endif