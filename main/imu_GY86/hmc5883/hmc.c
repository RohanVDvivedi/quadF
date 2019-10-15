#include<gy86.h>

typedef struct HMCdata HMCdata;
struct HMCdata
{
    int16_t magnx;
    int16_t magny;
    int16_t magnz;
};

// while initial accelerometer values will help us get final rotation
static HMCdatascaled offsets = {.magn = {.xi = 0.0, .yj = 0.0, .zk = 0.0}};

const HMCdatascaled* hmc_init()
{
    uint8_t data;

    // acess HMC5883 magnetometer here, and configure them
    data = 0x78;
    i2c_write(HMC5883_ADDRESS, 0x01, &data, 1);

    data = 0x20;
    i2c_write(HMC5883_ADDRESS, 0x02, &data, 1);

    data = 0x00;
    i2c_write(HMC5883_ADDRESS, 0x03, &data, 1);
    // magnetometer settings done

    // a small delay before we start reading the sensors
    vTaskDelay(20 / portTICK_PERIOD_MS);

    for(uint16_t i = 0; i < 500; i++)
    {
        HMCdatascaled datasc;
        get_scaled_HMCdata(&datasc);
        offsets.magn.xi += (datasc.magn.xi/500);
        offsets.magn.yj += (datasc.magn.yj/500);
        offsets.magn.zk += (datasc.magn.zk/500);
        vTaskDelay(14 / portTICK_PERIOD_MS);
    }

    return &offsets;
}

esp_err_t get_scaled_HMCdata(HMCdatascaled* result)
{
    HMCdata data;
    esp_err_t err = ESP_OK;

    err = i2c_read(HMC5883_ADDRESS, 0x03, &data, 6);
    if(err != ESP_OK)
    {
        return err;
    }

    // we have to change the data from network ordr to host order integers
    data.magnx = (data.magnx << 8) | ((data.magnx >> 8) & 0x00ff);
    data.magny = (data.magny << 8) | ((data.magny >> 8) & 0x00ff);
    data.magnz = (data.magnz << 8) | ((data.magnz >> 8) & 0x00ff);

    // in mG, milli Gauss
    result->magn.xi = (((double)(data.magnx)) * 0.92);
    result->magn.yj = (((double)(data.magny)) * 0.92);
    result->magn.zk = (((double)(data.magnz)) * 0.92);

    return err;
}