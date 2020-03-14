#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

// this provides you the mail sensor loop of the flight controller
#include<sensor_loop.h>

// this is where we finally write out our correction calculations to motor speeds
#include<motor_manager.h>

// this is what calculates corrections of the drone from the channel state and the current state
#include<pid_manager.h>

#include<state.h>

#define BLINK_GPIO 2

state curr_state;

channel_state chn_state;

void app_main(void)
{
    // this will turn on all the bldc motors and set their min and max speed setting (this setting can be controller from bldc.h)
    all_bldc_init();

    //vTaskDelay(10000 / portTICK_PERIOD_MS);

    // stay away from the remote and dron give no control inputs while the gpio is on
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BLINK_GPIO, 1);

    TaskHandle_t sensorLoopHandle = NULL;
    xTaskCreate(sensor_loop, "SENOR_LOOP", 4096, &curr_state, configMAX_PRIORITIES - 1, sensorLoopHandle);

    // this will turn on all the bldc motors and set their min and max speed setting (this setting can be controller from bldc.h)
    //all_bldc_init();
    
    channels_init();

    state curr_state_t = curr_state;

    while(curr_state_t.init == 0)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        curr_state_t = curr_state;printf("not ready\n");
    }

    gpio_set_level(BLINK_GPIO, 0);
    // gpio off so now give controls

    vector min = {0, 0, 0};
    vector max = {0, 0, 0};
    do
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);

        // read current sensor states
        state curr_state_t = curr_state;

        // read current inputs from the user
        update_channel_state(&chn_state);

        corrections corr;

        get_corrections(&corr, &curr_state_t, &chn_state);

        write_corrections_to_motors(&corr);

        /*min.xi = MIN(curr_state_t.magn_data.xi, min.xi);
        min.yj = MIN(curr_state_t.magn_data.yj, min.yj);
        min.zk = MIN(curr_state_t.magn_data.zk, min.zk);
        max.xi = MAX(curr_state_t.magn_data.xi, max.xi);
        max.yj = MAX(curr_state_t.magn_data.yj, max.yj);
        max.zk = MAX(curr_state_t.magn_data.zk, max.zk);
        static int i = 0;
        if(i == 30){printf("min: %lf, %lf, %lf\n", min.xi, min.yj, min.zk);}
        if(i == 30){printf("max: %lf, %lf, %lf\n", max.xi, max.yj, max.zk); i = 0;}
        i++;*/

        //printf("A: %lf, %lf, %lf \n\n", curr_state_t.accl_data.xi, curr_state_t.accl_data.yj, curr_state_t.accl_data.zk);
        //printf("M: %lf, %lf, %lf \n\n", curr_state_t.magn_data.xi, curr_state_t.magn_data.yj, curr_state_t.magn_data.zk);
        //printf("G: %lf, %lf, %lf \n\n", curr_state_t.gyro_data.xi, curr_state_t.gyro_data.yj, curr_state_t.gyro_data.zk);
        //printf("R: %lf \t \t P: %lf \n\n", curr_state_t.abs_roll, curr_state_t.abs_pitch);
        //printf("Alt: %lf \n\n", curr_state_t.altitude);
    }
    while(1);

    if( sensorLoopHandle != NULL )
    {
        vTaskDelete(sensorLoopHandle);
    }

    channels_destroy();
}