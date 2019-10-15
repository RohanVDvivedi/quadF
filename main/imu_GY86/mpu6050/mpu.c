#include<gy86.h>

typedef struct MPUdata MPUdata;
struct MPUdata
{
    int16_t acclx;
    int16_t accly;
    int16_t acclz;
    int16_t temp;
    int16_t gyrox;
    int16_t gyroy;
    int16_t gyroz;
};

// gyroscope steady state initial values may not be 0
// while initial accelerometer values will help us get final rotation
static MPUdatascaled offsets = {.accl = {.xi = 0.0, .yj = 0.0, .zk = 0.0}, .temp = 0.0, .gyro = {.xi = 0.0, .yj = 0.0, .zk = 0.0}};

const MPUdatascaled* mpu_init()
{
    uint8_t data;

    // write 0 to pwr_mgmt_1 register to wake it up
    data = 0x00;
    i2c_write(MPU6050_ADDRESS, 0x6b, &data, 1);

    // below wries will enable us to talk to HMC5883 sensor
        // turn on master of i2c, and reset accl and gyro data path and its registers
        data = 0x01;
        i2c_write(MPU6050_ADDRESS, 0x6a, &data, 1);

        // turnon i2c bypass mode
        i2c_write(MPU6050_ADDRESS, 0x37, &data, 1);
        data = data | (1 << 1);
        i2c_write(MPU6050_ADDRESS, 0x37, &data, 1);
    // yay we can now talk to magnetometer, or anyother sensor attached on the auxilary bus of mou6050

    // a small delay before we start reading the sensors
    vTaskDelay(20 / portTICK_PERIOD_MS);

    for(uint16_t i = 0; i < 500; i++)
    {
        MPUdatascaled datasc;
        get_scaled_MPUdata(&datasc);
        offsets.accl.xi += (datasc.accl.xi/500);
        offsets.accl.yj += (datasc.accl.yj/500);
        offsets.accl.zk += (datasc.accl.zk/500);
        offsets.gyro.xi += (datasc.gyro.xi/500);
        offsets.gyro.yj += (datasc.gyro.yj/500);
        offsets.gyro.zk += (datasc.gyro.zk/500);
        vTaskDelay(14 / portTICK_PERIOD_MS);
    }

    return &offsets;
}

esp_err_t get_scaled_MPUdata(MPUdatascaled* result)
{
    MPUdata data;
    esp_err_t err = ESP_OK;

    err = i2c_read(MPU6050_ADDRESS, 0x3b, &data, 14);
    if(err != ESP_OK)
    {
        return err;
    }

    // we have to change the data from network ordr to host order integers
    data.acclx = (data.acclx << 8) | ((data.acclx >> 8) & 0x00ff);
    data.accly = (data.accly << 8) | ((data.accly >> 8) & 0x00ff);
    data.acclz = (data.acclz << 8) | ((data.acclz >> 8) & 0x00ff);
    data.temp = (data.temp << 8) | ((data.temp >> 8) & 0x00ff);
    data.gyrox = (data.gyrox << 8) | ((data.gyrox >> 8) & 0x00ff);
    data.gyroy = (data.gyroy << 8) | ((data.gyroy >> 8) & 0x00ff);
    data.gyroz = (data.gyroz << 8) | ((data.gyroz >> 8) & 0x00ff);

    // in m/s2, meter per second square => sensitivity = +/-2g = +/-19.6
    result->accl.xi = ((((double)(data.acclx)) * 19.6) / 32768.0);
    result->accl.yj = ((((double)(data.accly)) * 19.6) / 32768.0);
    result->accl.zk = ((((double)(data.acclz)) * 19.6) / 32768.0);

    // temperature is in degree celcius
    result->temp  = (((double)(data.temp)) / 340) + 36.53;

    // in dps, degrees per second => sensitivity = +/-250
    result->gyro.xi = ((((double)(data.gyrox)) * 250.0) / 32768.0) - offsets.gyro.xi;
    result->gyro.yj = ((((double)(data.gyroy)) * 250.0) / 32768.0) - offsets.gyro.yj;
    result->gyro.zk = ((((double)(data.gyroz)) * 250.0) / 32768.0) - offsets.gyro.zk;

    return err;
}