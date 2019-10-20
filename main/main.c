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

#include<state.h>

#define BLINK_GPIO 2

MPUdatascaled mpudatasc;
HMCdatascaled hmcdatasc;
Barodatascaled bdatasc;

state State = {
    .orientation = {.sc = 1.0, .xi = 0.0, .yj = 0.0, .zk = 0.0},
    .angular_velocity_local = {.xi = 0.0, .yj = 0.0, .zk = 0.0},
    .acceleration_local = {.xi = 0.0, .yj = 0.0, .zk = 0.0},
    .altitude = -1,
    .altitude_rate = 0.0
};

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

        quat_raw quat_r;
        get_unit_rotation_axis(&(quat_r.vectr), &(State.orientation));
        quat_r.theta = 2 * acos(State.orientation.sc) * 180 / M_PI;
        printf("%lf \t %lf \t %lf \t\t %lf\n\n", quat_r.vectr.xi, quat_r.vectr.yj, quat_r.vectr.zk, quat_r.theta);

        //printf("accl : \t%lf \t%lf \t%lf\n", mpudatasc.accl.xi, mpudatasc.accl.yj, mpudatasc.accl.zk);
        //printf("temp : \t%lf\n", mpudatasc.temp);
        //printf("gyro : \t%lf \t%lf \t%lf\n\n", mpudatasc.gyro.xi, mpudatasc.gyro.yj, mpudatasc.gyro.zk);

        //printf("magn : \t%lf \t%lf \t%lf\n\n", hmcdatasc.magn.xi, hmcdatasc.magn.yj, hmcdatasc.magn.zk);

        //printf("quat : \t%lf \t%lf \t%lf \t%lf\n"  , quat.sc, quat.xi, quat.yj, quat.zk);
        
        //printf("altitude : \t%lf\n", bdatasc.altitude);
        //if(alt <= 0)
        //{
        //    alt = bdatasc.altitude;
        //}
        //else
        //{
        //    alt = (alt * 0.995) + (bdatasc.altitude * 0.005);
        //}
        //printf("filtered altitude : \t%lf\n", alt);
        //printf("abspressure : \t%lf\n", bdatasc.abspressure);
        //printf("temperature : \t%lf\n\n", bdatasc.temperature);
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
    const MPUdatascaled* mpuInit = mpu_init();
    mpudatasc = (*mpuInit);

    State.acceleration_local = mpuInit->accl;
    State.angular_velocity_local = mpuInit->gyro;

    uint64_t last_hmc_read_time = now_time;
    const HMCdatascaled* hmcInit = hmc_init();
    hmcdatasc = (*hmcInit);

    uint64_t last_ms5_read_time = now_time;
    baro_init();

    // initialize quaternions to 0 rotation 1 0 0 0
    quaternion oreo = {.sc = 1.0, .xi = 0.0, .yj = 0.0, .zk = 0.0};

    while(1)
    {
        now_time = get_milli_timer_ticks_count();

        // read mpu every millisecond
        if(now_time - last_mpu_read_time >= 1000)
        {
            get_scaled_MPUdata(&mpudatasc);

            // gyroscope integration logic
            now_time = get_milli_timer_ticks_count();
            quat_raw quat_raw_change;
            get_raw_quaternion_change_from_gyroscope(&quat_raw_change, &oreo, &(mpudatasc.gyro), ((double)(now_time - last_mpu_read_time))/1000000);
            quaternion quat_change;
            to_quaternion(&quat_change, &quat_raw_change);
            quaternion final_quat_gyro;
            hamilton_product(&final_quat_gyro, &quat_change, &oreo);

            // accelerometer magnetometer logic
            quaternion final_quat_accl_magn;
            get_quaternion_from_vectors_changes(&final_quat_accl_magn, &(mpudatasc.accl), &(mpuInit->accl), &(hmcdatasc.magn), &(hmcInit->magn));
            conjugate(&final_quat_accl_magn, &final_quat_accl_magn);
            if( isnan(final_quat_accl_magn.xi) || isnan(final_quat_accl_magn.yj) || isnan(final_quat_accl_magn.zk) || isnan(final_quat_accl_magn.sc) )
            {
                final_quat_accl_magn = oreo;
            }

            if( isnan(final_quat_gyro.xi) || isnan(final_quat_gyro.yj) || isnan(final_quat_gyro.zk) || isnan(final_quat_gyro.sc) )
            {
                final_quat_gyro = final_quat_accl_magn;
            }

            // actual fusion logic called
            slerp_quaternion(&oreo, &final_quat_gyro, 0.98, &final_quat_accl_magn);

            // update the global state vector
            State.orientation = oreo;
            update_vector(&(State.angular_velocity_local), &(mpudatasc.gyro), 1.0);
            update_vector(&(State.acceleration_local), &(mpudatasc.accl), 0.08);
            
            now_time = get_milli_timer_ticks_count();
            last_mpu_read_time = now_time;
        }

        // read hmc every 11 milliseconds
        if(now_time - last_hmc_read_time >= 11000)
        {
            get_scaled_HMCdata(&hmcdatasc);

            // accelerometer magnetometer logic
            quaternion final_quat_accl_magn;
            get_quaternion_from_vectors_changes(&final_quat_accl_magn, &(mpudatasc.accl), &(mpuInit->accl), &(hmcdatasc.magn), &(hmcInit->magn));
            conjugate(&final_quat_accl_magn, &final_quat_accl_magn);
            if( isnan(final_quat_accl_magn.xi) || isnan(final_quat_accl_magn.yj) || isnan(final_quat_accl_magn.zk) || isnan(final_quat_accl_magn.sc) )
            {
                final_quat_accl_magn = oreo;
            }

            // actual fusion
            quaternion old_oreo = oreo;
            slerp_quaternion(&oreo, &final_quat_accl_magn, 0.02, &old_oreo);

            // update the global state vector
            State.orientation = oreo;

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
                if(State.altitude == -1)
                {
                    State.altitude = bdatasc.altitude;
                }
                State.altitude = State.altitude * 0.9 + bdatasc.altitude * 0.1;

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