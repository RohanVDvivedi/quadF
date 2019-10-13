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

// this timer is needed to understand whwn to do the calculation
#include<millitimer.h>

#define BLINK_GPIO 2

MPUdatascaled mpudatasc;
HMCdatascaled hmcdatasc;
Barodatascaled bdatasc;

void sensor_loop(void* not_required);

void app_main(void)
{
    //all_bldc_init();
    
    //channels_init();

    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    TaskHandle_t sensorLoopHandle = NULL;
    xTaskCreate(sensor_loop, "SENOR_LOOP", 2048, NULL, configMAX_PRIORITIES - 1, sensorLoopHandle);

    do
    {
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        printf("accl : \t%lf \t%lf \t%lf\n", mpudatasc.acclx, mpudatasc.accly, mpudatasc.acclz);
        printf("temp : \t%lf\n", mpudatasc.temp);
        printf("gyro : \t%lf \t%lf \t%lf\n\n", mpudatasc.gyrox, mpudatasc.gyroy, mpudatasc.gyroz);

        printf("magn : \t%lf \t%lf \t%lf\n\n", hmcdatasc.magnx, hmcdatasc.magny, hmcdatasc.magnz);
        
        printf("altitude : \t%lf\n", bdatasc.altitude);
        printf("abspressure : \t%lf\n", bdatasc.abspressure);
        printf("temperature : \t%lf\n\n", bdatasc.temperature);
    }
    while(1);

    if( sensorLoopHandle != NULL )
    {
        vTaskDelete(sensorLoopHandle);
    }

    //channels_destroy();
}

void sensor_loop(void* not_required)
{
    milli_timer_init();
    int64_t now_time = get_milli_timer_ticks_count();

    i2c_init();

    uint64_t last_mpu_read_time = now_time;
    mpu_init();

    uint64_t last_hmc_read_time = now_time;
    hmc_init();

    uint64_t last_ms5_read_time = now_time;
    baro_init();

    while(1)
    {
        now_time = get_milli_timer_ticks_count();

        // read mpu every millisecond
        if(now_time - last_mpu_read_time >= 1000)
        {
            get_scaled_MPUdata(&mpudatasc);
            now_time = get_milli_timer_ticks_count();
            last_mpu_read_time = now_time;
        }

        // read hmc every 10 milliseconds
        if(now_time - last_hmc_read_time >= 10000)
        {
            get_scaled_HMCdata(&hmcdatasc);
            now_time = get_milli_timer_ticks_count();
            last_hmc_read_time = now_time;
        }

        // check on ms5611 every 12 milliseconds
        if(now_time - last_ms5_read_time >= 12000)
        {
            if(get_current_ms5611_state() == REQUESTED_TEMPERATURE)
            {
                get_raw_Barodata_temperature();
                request_Barodata_abspressure();
            }
            else if(get_current_ms5611_state() == REQUESTED_PRESSURE)
            {
                get_raw_Barodata_abspressure();

                // once we have got both the raw digital temperature and pressure values we can scale our data
                scale_and_compensate_Barodata(&bdatasc);

                request_Barodata_temperature();
            }
            else
            {
                request_Barodata_temperature();
            }
            now_time = get_milli_timer_ticks_count();
            last_ms5_read_time = now_time;
        }
    }

    i2c_destroy();
}