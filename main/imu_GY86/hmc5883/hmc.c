#include<gy86.h>

typedef struct HMCdata HMCdata;
struct HMCdata
{
    int16_t magnx;
    int16_t magnz;
    int16_t magny;
};

// while offsets magnetometer values will help us get final rotation
static HMCdatascaled offsets = {.magn = {.xi = 82.84, .yj = -135.7, .zk = -19.78}};

// while initial magnetometer values will help us get final rotation
static HMCdatascaled initial = {.magn = {.xi = 0.0, .yj = 0.0, .zk = 0.0}};

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
    vTaskDelay(200 / portTICK_PERIOD_MS);

    for(uint16_t i = 0; i < 200; i++)
    {
        HMCdatascaled datasc;
        get_scaled_HMCdata(&datasc);
        initial.magn.xi += (datasc.magn.xi/200);
        initial.magn.yj += (datasc.magn.yj/200);
        initial.magn.zk += (datasc.magn.zk/200);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    return &initial;
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
    result->magn.xi = ((((double)(data.magnx)) * 0.92) - offsets.magn.xi) * 20/747.1;
    result->magn.yj = ((((double)(data.magny)) * 0.92) - offsets.magn.yj) * 20/764.52;
    result->magn.zk = ((((double)(data.magnz)) * 0.92) - offsets.magn.zk) * 20/713;

    return err;
}