#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define BLINK_GPIO 2

void app_main(void)
{
    i2c_init();
    imu_init();
    Barodata bdata;baro_init(&bdata);
    //all_bldc_init();
    //channels_init();

    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    do
    {
        IMUdata data;
        IMUdatascaled datas;
        get_raw_IMUdata(&data, 1, 1);
        scale_IMUdata(&datas, &data, 1, 1);

        // small report in form of command, I am requiring delat between mpu6050 call and ms5611 sensor calls do not know why

        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        request_Barodata_abspressure();
        vTaskDelay(10 / portTICK_PERIOD_MS);
        get_raw_Barodata_abspressure(&bdata);
        request_Barodata_temperature();
        vTaskDelay(10 / portTICK_PERIOD_MS);
        get_raw_Barodata_temperature(&bdata);
        scale_and_compensate_Barodata(&bdata);

        printf("accl : \t%lf \t%lf \t%lf\n", datas.acclx, datas.accly, datas.acclz);
        printf("gyro : \t%lf \t%lf \t%lf\n", datas.gyrox, datas.gyroy, datas.gyroz);
        printf("magn : \t%lf \t%lf \t%lf\n", datas.magnx, datas.magny, datas.magnz);
        printf("temp : \t%lf\n\n", datas.temp);
        printf("altitude : \t%lf\n", bdata.altitude);
        printf("abspressure : \t%lf\n", bdata.abspressure);
        printf("temperature : \t%lf\n\n", bdata.temperature);
    }
    while(1);

    i2c_destroy();
    //channels_destroy();
}