#ifndef GY86_H
#define GY86_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include<i2c_comm.h>

// i2c addresses, for connected sensors namesly MPU6050 : accl/gyro, HMC5883L : magnetometer and MS5611 : barometer
#define MPU6050_ADDRESS     0x68
#define HMC5883_ADDRESS     0x1e
#define MS5611_ADDRESS      0x77

typedef struct IMUdata IMUdata;
struct IMUdata
{
    int16_t acclx;
    int16_t accly;
    int16_t acclz;
    int16_t temp;
    int16_t gyrox;
    int16_t gyroy;
    int16_t gyroz;
    int16_t magnx;
    int16_t magny;
    int16_t magnz;
};

typedef struct IMUdatascaled IMUdatascaled;
struct IMUdatascaled
{
    double acclx;
    double accly;
    double acclz;
    double temp;
    double gyrox;
    double gyroy;
    double gyroz;
    double magnx;
    double magny;
    double magnz;
};

typedef struct Barodata Barodata;
struct Barodata
{
    uint16_t C1_SENS_T1;
    uint16_t C2_OFF_T1;
    uint16_t C3_TCS;
    uint16_t C4_TCO;
    uint16_t C5_TREF;
    uint16_t C6_TEMPSENS;

    uint32_t D1;
    uint32_t D2;

    int32_t dT;
    int32_t TEMP;

    int64_t OFF;
    int64_t SENS;
    int64_t P;

    double temperature;
    double abspressure;
    double altitude;
};

// imu data
void imu_init();
esp_err_t get_raw_IMUdata(IMUdata* data, uint8_t mpu_data, uint8_t hmc_data);
void scale_IMUdata(IMUdatascaled* result, IMUdata* data, uint8_t mpu_data, uint8_t hmc_data);

// barometer data
void baro_init(Barodata* data);
esp_err_t request_Barodata_temperature();
esp_err_t request_Barodata_abspressure();
esp_err_t get_raw_Barodata_temperature(Barodata* data);
esp_err_t get_raw_Barodata_abspressure(Barodata* data);
void scale_and_compensate_Barodata(Barodata* data);


#endif