#include<gy86.h>

static HMCdatascaled offsets = {.magnx = 0, .magny = 0, .magnz = 0};

void hmc_init()
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

    for(uint16_t i = 0; i < 500; i++)
    {
        HMCdata data;
        get_raw_HMCdata(&data);
        offsets.magnx += (((double)(data.magnx))/500);
        offsets.magny += (((double)(data.magny))/500);
        offsets.magnz += (((double)(data.magnz))/500);
        vTaskDelay(14 / portTICK_PERIOD_MS);
    }
}

esp_err_t get_raw_HMCdata(HMCdata* data)
{
    esp_err_t err = ESP_OK;

    err = i2c_read(HMC5883_ADDRESS, 0x03, data, 6);
    if(err != ESP_OK)
    {
        return err;
    }

    // we have to change the data from network ordr to host order integers
    data->magnx = (data->magnx << 8) | ((data->magnx >> 8) & 0x00ff);
    data->magny = (data->magny << 8) | ((data->magny >> 8) & 0x00ff);
    data->magnz = (data->magnz << 8) | ((data->magnz >> 8) & 0x00ff);

    return err;
}

void scale_HMCdata(HMCdatascaled* result, HMCdata* data)
{
    // in mG, milli Gauss
    result->magnx = ((((double)(data->magnx)) - offsets.magnx) * 0.92);
    result->magny = ((((double)(data->magny)) - offsets.magny) * 0.92);
    result->magnz = ((((double)(data->magnz)) - offsets.magnz) * 0.92);
}

