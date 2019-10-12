#include<gy86.h>

static IMUdatascaled offsets = {.acclx = 0, .accly = 0, .acclz = 0, .gyrox = 0, .gyroy = 0, .gyroz = 0, .temp = 0, .magnx = 0, .magny = 0, .magnz = 0};

void imu_init()
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
    // yay we can now talk to magnetometer

    // acess HMC5883 magnetometer here, and configure them
    data = 0x78;
    i2c_write(HMC5883_ADDRESS, 0x01, &data, 1);

    data = 0x20;
    i2c_write(HMC5883_ADDRESS, 0x02, &data, 1);

    data = 0x00;
    i2c_write(HMC5883_ADDRESS, 0x03, &data, 1);
    // magnetometer settings done


// by uncommenting the below lines we can start reading manetometer data
/*
    // write slave address o with magnetometer's address
    data = HMC5883_ADDRESS | (1 << 7);
    i2c_write(MPU6050_ADDRESS, 0x25, &data, 1);

    // write read register number of slave 0
    data = 0x03;
    i2c_write(MPU6050_ADDRESS, 0x26, &data, 1);

    // other configurations for slave 0
    data = 0xa6;
    i2c_write(MPU6050_ADDRESS, 0x27, &data, 1);

    // turn on master of i2c, and reset accl and gyro data path and its registers
    data = 0x20;
    i2c_write(MPU6050_ADDRESS, 0x6a, &data, 1);

    // turnon i2c bypass mode
    i2c_write(MPU6050_ADDRESS, 0x37, &data, 1);
    data = data & (~(1 << 1));
    i2c_write(MPU6050_ADDRESS, 0x37, &data, 1);
*/

    for(uint16_t i = 0; i < 500; i++)
    {
        IMUdata data;
        get_raw_IMUdata(&data, 1, 1);
        offsets.acclx += (((double)(data.acclx))/500);
        offsets.accly += (((double)(data.accly))/500);
        offsets.acclz += ((((double)(data.acclz))-16384)/500);
        offsets.gyrox += (((double)(data.gyrox))/500);
        offsets.gyroy += (((double)(data.gyroy))/500);
        offsets.gyroz += (((double)(data.gyroz))/500);
        offsets.magnx += (((double)(data.magnx))/500);
        offsets.magny += (((double)(data.magny))/500);
        offsets.magnz += (((double)(data.magnz))/500);
        vTaskDelay(14 / portTICK_PERIOD_MS);
    }
}

esp_err_t get_raw_IMUdata(IMUdata* data, uint8_t mpu_data, uint8_t hmc_data)
{
    esp_err_t err = ESP_OK;

    if(mpu_data)
    {
        err = i2c_read(MPU6050_ADDRESS, 0x3b, data, 14);
        if(err != ESP_OK)
        {
            return err;
        }

        // we have to change the data from network ordr to host order integers
        data->acclx = (data->acclx << 8) | ((data->acclx >> 8) & 0x00ff);
        data->accly = (data->accly << 8) | ((data->accly >> 8) & 0x00ff);
        data->acclz = (data->acclz << 8) | ((data->acclz >> 8) & 0x00ff);
        data->temp = (data->temp << 8) | ((data->temp >> 8) & 0x00ff);
        data->gyrox = (data->gyrox << 8) | ((data->gyrox >> 8) & 0x00ff);
        data->gyroy = (data->gyroy << 8) | ((data->gyroy >> 8) & 0x00ff);
        data->gyroz = (data->gyroz << 8) | ((data->gyroz >> 8) & 0x00ff);
    }

    if(hmc_data)
    {
        err = i2c_read(HMC5883_ADDRESS, 0x03, ((void*)data) + 14, 6);
        if(err != ESP_OK)
        {
            return err;
        }

        // we have to change the data from network ordr to host order integers
        data->magnx = (data->magnx << 8) | ((data->magnx >> 8) & 0x00ff);
        data->magny = (data->magny << 8) | ((data->magny >> 8) & 0x00ff);
        data->magnz = (data->magnz << 8) | ((data->magnz >> 8) & 0x00ff);
    }

    return err;
}

void scale_IMUdata(IMUdatascaled* result, IMUdata* data, uint8_t mpu_data, uint8_t hmc_data)
{
    if(mpu_data)
    {
        // in m/s2, meter per second square
        result->acclx = ((((double)(data->acclx)) - offsets.acclx) * 19.6) / 32768.0;
        result->accly = ((((double)(data->accly)) - offsets.accly) * 19.6) / 32768.0;
        result->acclz = ((((double)(data->acclz)) - offsets.acclz) * 19.6) / 32768.0;

        // temperature is in degree celcius
        result->temp  = (((double)(data->temp)) / 340) + 36.53;

        // in dps, degrees per second
        result->gyrox = ((((double)(data->gyrox)) - offsets.gyrox) * 250.0) / 32768.0;
        result->gyroy = ((((double)(data->gyroy)) - offsets.gyroy) * 250.0) / 32768.0;
        result->gyroz = ((((double)(data->gyroz)) - offsets.gyroz) * 250.0) / 32768.0;
    }

    // in mG, milli Gauss
    if(hmc_data)
    {
        result->magnx = ((((double)(data->magnx)) - offsets.magnx) * 0.92);
        result->magny = ((((double)(data->magny)) - offsets.magny) * 0.92);
        result->magnz = ((((double)(data->magnz)) - offsets.magnz) * 0.92);
    }
}

