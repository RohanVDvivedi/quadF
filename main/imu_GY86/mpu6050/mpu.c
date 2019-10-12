#include<gy86.h>

static MPUdatascaled offsets = {.acclx = 0, .accly = 0, .acclz = 0, .temp = 0, .gyrox = 0, .gyroy = 0, .gyroz = 0};

void mpu_init()
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

    for(uint16_t i = 0; i < 500; i++)
    {
        MPUdata data;
        get_raw_MPUdata(&data);
        offsets.acclx += (((double)(data.acclx))/500);
        offsets.accly += (((double)(data.accly))/500);
        offsets.acclz += ((((double)(data.acclz))-16384)/500);
        offsets.gyrox += (((double)(data.gyrox))/500);
        offsets.gyroy += (((double)(data.gyroy))/500);
        offsets.gyroz += (((double)(data.gyroz))/500);
        vTaskDelay(14 / portTICK_PERIOD_MS);
    }
}

esp_err_t get_raw_MPUdata(MPUdata* data)
{
    esp_err_t err = ESP_OK;

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

    return err;
}

void scale_MPUdata(MPUdatascaled* result, MPUdata* data)
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

