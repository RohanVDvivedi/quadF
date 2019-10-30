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

state curr_state = {
    .orientation = {.sc = 1.0, .xi = 0.0, .yj = 0.0, .zk = 0.0},
    .angular_velocity_local = {.xi = 0.0, .yj = 0.0, .zk = 0.0},
    .acceleration_local = {.xi = 0.0, .yj = 0.0, .zk = 0.0},
    .magnetic_heading_local = {.xi = 0.0, .yj = 0.0, .zk = 0.0},
    .abs_roll = {0.0, 0.0},
    .abs_pitch = {0.0, 0.0},
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

    do
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);

        // read current sensor states
        state curr_state_t = curr_state;

        // read current inputs from the user
        update_channel_state(&chn_state);

        // find absolute roll and pitch
        get_absolute_rotation_angles_about_local_axis(&(curr_state_t));

        corrections corr;

        get_corrections(&corr, &curr_state_t, &chn_state);

        write_corrections_to_motors(&corr);

        //quat_raw quat_r;
        //get_unit_rotation_axis(&(quat_r.vectr), &(curr_state_t.orientation));
        //quat_r.theta = 2 * acos(curr_state_t.orientation.sc) * 180 / M_PI;
        //printf("A: %lf, %lf, %lf\n", curr_state_t.acceleration_local.xi, curr_state_t.acceleration_local.yj, curr_state_t.acceleration_local.zk);
        //printf("M: %lf, %lf, %lf\n\n", curr_state_t.magnetic_heading_local.xi, curr_state_t.magnetic_heading_local.yj, curr_state_t.magnetic_heading_local.zk);
        //printf("G: %lf, %lf, %lf\n", curr_state_t.angular_velocity_local.xi, curr_state_t.angular_velocity_local.yj, curr_state_t.angular_velocity_local.zk);
        //printf("%lf \t %lf \t %lf \t\t %lf\n\n", quat_r.vectr.xi, quat_r.vectr.yj, quat_r.vectr.zk, quat_r.theta);
        //printf("R: %lf %lf \t \t P: %lf %lf\n", curr_state_t.abs_roll[0], curr_state_t.abs_roll[1], curr_state_t.abs_pitch[0], curr_state_t.abs_pitch[1]);
    }
    while(1);

    if( sensorLoopHandle != NULL )
    {
        vTaskDelete(sensorLoopHandle);
    }

    channels_destroy();
}