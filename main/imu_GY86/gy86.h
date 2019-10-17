#ifndef GY86_H
#define GY86_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include<i2c_comm.h>
#include<geometry.h>

// i2c addresses, for connected sensors namesly MPU6050 : accl/gyro, HMC5883L : magnetometer and MS5611 : barometer
#define MPU6050_ADDRESS     0x68
#define HMC5883_ADDRESS     0x1e
#define MS5611_ADDRESS      0x77

typedef struct MPUdatascaled MPUdatascaled;
struct MPUdatascaled
{
    vector accl;
    double temp;
    vector gyro;
};

typedef struct HMCdatascaled HMCdatascaled;
struct HMCdatascaled
{
    vector magn;
};

typedef struct Barodatascaled Barodatascaled;
struct Barodatascaled
{
    double temperature;
    double abspressure;
    double altitude;
};

typedef enum MS5611state MS5611state;
enum MS5611state
{
    INIT,
    REQUESTED_TEMPERATURE,
    READ_TEMPERATURE,
    REQUESTED_PRESSURE,
    READ_PRESSURE
};

// MPU6050 acclerometer and gyroscope data, returns initial offset values averages over 500 cycles
const MPUdatascaled* mpu_init();
esp_err_t get_scaled_MPUdata(MPUdatascaled* result);

void get_raw_quaternion_change_from_gyroscope(quat_raw* change, quaternion* previous_quaternion, vector* gyroscope, double time_in_seconds_since_last_reading);
void fuse_raw_quaternion_change_with_accelerometer(quat_raw* change, quaternion* previous_quaternion, vector* accelerometer);

// magnetometer data, returns initial offset values averages over 500 cycles
const HMCdatascaled* hmc_init();
esp_err_t get_scaled_HMCdata(HMCdatascaled* result);

// barometer data
MS5611state get_current_ms5611_state();
void baro_init();
esp_err_t request_Barodata_temperature();
esp_err_t get_raw_Barodata_temperature();
esp_err_t request_Barodata_abspressure();
esp_err_t get_raw_Barodata_abspressure();
void scale_and_compensate_Barodata(Barodatascaled* result);


#endif