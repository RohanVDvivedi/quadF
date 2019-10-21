#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"



// this is where we get out rc receivers first 4 channels input data from
#include<rc_receiver.h>

// this is where we finally write out calculated motor speed values
#include<bldc.h>

// this timer is needed to understand whwn to do the calculation
#include<millitimer.h>

#include<state.h>

#define BLINK_GPIO 2

void sensor_loop(void* not_required);

void app_main(void)
{
    //all_bldc_init();
    
    //channels_init();

    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    TaskHandle_t sensorLoopHandle = NULL;
    xTaskCreate(sensor_loop, "SENOR_LOOP", 4096, NULL, configMAX_PRIORITIES - 1, sensorLoopHandle);

    do
    {
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        //quat_raw quat_r;
        //get_unit_rotation_axis(&(quat_r.vectr), &(State.orientation));
        //quat_r.theta = 2 * acos(State.orientation.sc) * 180 / M_PI;
        //printf("%lf \t %lf \t %lf \t\t %lf\n\n", quat_r.vectr.xi, quat_r.vectr.yj, quat_r.vectr.zk, quat_r.theta);

        vector angles;
        get_absolute_rotation_angles_about_local_axis(&angles);
        //vector xl;  get_current_local_X_axis(&xl);
        //vector yl;  get_current_local_Y_axis(&yl);
        //vector zl;  get_current_local_Z_axis(&zl);
        //printf("xl : %lf \t %lf \t %lf\n", xl.xi, xl.yj, xl.zk);
        //printf("yl : %lf \t %lf \t %lf\n", yl.xi, yl.yj, yl.zk);
        //printf("zl : %lf \t %lf \t %lf\n", zl.xi, zl.yj, zl.zk);
        printf("%lf \t %lf \t %lf\n\n", angles.xi, angles.yj, angles.zk);
    }
    while(1);

    if( sensorLoopHandle != NULL )
    {
        vTaskDelete(sensorLoopHandle);
    }

    //channels_destroy();
}