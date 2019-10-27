#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

// this provides you the mail sensor loop of the flight controller
#include<sensor_loop.h>

// this is where we finally write out our correction calculations to motor speeds
#include<motor_manager.h>

// this is what calculates corrections of the drone from the channel state and the current state
#include<pid_manager.h>

#include<state.h>

#define BLINK_GPIO 2

volatile state curr_state = {
    .orientation = {.sc = 1.0, .xi = 0.0, .yj = 0.0, .zk = 0.0},
    .angular_velocity_local = {.xi = 0.0, .yj = 0.0, .zk = 0.0},
    .acceleration_local = {.xi = 0.0, .yj = 0.0, .zk = 0.0},
    .altitude = NAN,
    .altitude_rate = 0.0,
};

channel_state chn_state = {
    .throttle = 0.0,
    .yaw = 0.0,
    .pitch = 0.0,
    .roll = 0.0,
    .swit = 0.0,
    .knob = 0.0
};

void app_main(void)
{
    TaskHandle_t sensorLoopHandle = NULL;
    xTaskCreate(sensor_loop, "SENOR_LOOP", 4096, &curr_state, configMAX_PRIORITIES - 1, sensorLoopHandle);

    // this will turn on all the bldc motors and set their min and max speed setting
    all_bldc_init();
    
    channels_init();

    state curr_state_t = curr_state;

    while(curr_state_t.init == 0)
    {
        curr_state_t = curr_state;
    }

    do
    {
        vTaskDelay(4 / portTICK_PERIOD_MS);

        // read current sensor states
        state curr_state_t = curr_state;

        // read current inputs from the user
        update_channel_state(&chn_state);

        corrections corr;

        get_corrections(&corr, &curr_state_t, &chn_state);

        //printf("%lf \t %lf \t %lf \t %lf\n", corr.altitude_corr, corr.pitch_corr, corr.roll_corr, corr.yaw_corr);

        write_corrections_to_motors(&corr);

        //printf("yaw = %lf \t pitch = %lf \t roll = %lf \t throttle %lf \t swit = %d \t knob = %lf \n\n", chn_state.yaw, chn_state.pitch, chn_state.roll, chn_state.throttle, chn_state.swit, chn_state.knob);

        //quat_raw quat_r;
        //get_unit_rotation_axis(&(quat_r.vectr), &(curr_state_t.orientation));
        //quat_r.theta = 2 * acos(curr_state_t.orientation.sc) * 180 / M_PI;
        //printf("%lf \t %lf \t %lf \t\t %lf\n", quat_r.vectr.xi, quat_r.vectr.yj, quat_r.vectr.zk, quat_r.theta);
    }
    while(1);

    if( sensorLoopHandle != NULL )
    {
        vTaskDelete(sensorLoopHandle);
    }

    channels_destroy();
}