#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

// the i2c comm has to be started hence i2c_comm
#include<i2c_comm.h>

// this is where we get our scaled sensor readings from
#include<gy86.h>

// this is where we get out rc receivers first 4 channels input data from
#include<rc_receiver.h>

// this is where we finally write out calculated motor speed values
#include<bldc.h>

#define BLINK_GPIO 2

void app_main(void)
{
    i2c_init();

    mpu_init();

    hmc_init();

    Barodata bdata;baro_init(&bdata);

    all_bldc_init();
    
    channels_init();

    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    do
    {
        MPUdata mpudata;
        MPUdatascaled mpudatasc;
        get_raw_MPUdata(&mpudata);
        scale_MPUdata(&mpudatasc, &mpudata);

        HMCdata hmcdata;
        HMCdatascaled hmcdatasc;
        get_raw_HMCdata(&hmcdata);
        scale_HMCdata(&hmcdatasc, &hmcdata);

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

        Barodatascaled bdatasc;

        scale_and_compensate_Barodata(&bdatasc, &bdata);

        printf("accl : \t%lf \t%lf \t%lf\n", mpudatasc.acclx, mpudatasc.accly, mpudatasc.acclz);
        printf("temp : \t%lf\n", mpudatasc.temp);
        printf("gyro : \t%lf \t%lf \t%lf\n\n", mpudatasc.gyrox, mpudatasc.gyroy, mpudatasc.gyroz);

        printf("magn : \t%lf \t%lf \t%lf\n\n", hmcdatasc.magnx, hmcdatasc.magny, hmcdatasc.magnz);
        
        printf("altitude : \t%lf\n", bdatasc.altitude);
        printf("abspressure : \t%lf\n", bdatasc.abspressure);
        printf("temperature : \t%lf\n\n", bdatasc.temperature);
    }
    while(1);

    channels_destroy();

    i2c_destroy();
}