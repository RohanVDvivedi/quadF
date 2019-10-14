#include<gy86.h>

typedef struct HMCdata HMCdata;
struct HMCdata
{
    int16_t magnx;
    int16_t magny;
    int16_t magnz;
};

// while initial accelerometer values will help us get final rotation
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

    // a small delay before we start reading the sensors
    vTaskDelay(20 / portTICK_PERIOD_MS);

    for(uint16_t i = 0; i < 500; i++)
    {
        HMCdatascaled datasc;
        get_scaled_HMCdata(&datasc);
        offsets.magnx += (datasc.magnx/500);
        offsets.magny += (datasc.magny/500);
        offsets.magnz += (datasc.magnz/500);
        vTaskDelay(14 / portTICK_PERIOD_MS);
    }
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
    result->magnx = (((double)(data.magnx)) * 0.92);
    result->magny = (((double)(data.magny)) * 0.92);
    result->magnz = (((double)(data.magnz)) * 0.92);

    return err;
}

void get_quaternion_from_initial_state_based_on_magn(quaternion* actual, HMCdatascaled* data)
{
    vector magn_now;
    magn_now.xi = data->magnx;
    magn_now.yj = data->magny;
    magn_now.zk = data->magnz;

    vector magn_init;
    magn_init.xi = offsets.magnx;
    magn_init.yj = offsets.magny;
    magn_init.zk = offsets.magnz;

    quat_raw raw_quat;

    cross(&(raw_quat.vectr), &magn_init, &magn_now);

    raw_quat.theta = (acos(dot(&magn_init, &magn_now)/(magnitude_vector(&magn_init)*magnitude_vector(&magn_now))) * 180) / M_PI;

    to_quaternion(actual, &raw_quat);
}