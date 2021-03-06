#include<sensor_loop.h>

#if   defined(ORIENTATION_ONLY_GYRO)
    #define GYRO_FUSION_FACTOR 1.00
#elif defined(ORIENTATION_ONLY_ACCL_MAGN)
    #define GYRO_FUSION_FACTOR 0.00
#else
    #define GYRO_FUSION_FACTOR 0.98
#endif

void sensor_loop(void* state_pointer)
{
    state* state_p = ((state*)(state_pointer));

    micro_timer_init();
    uint64_t now_time = get_micro_timer_ticks_count();

    i2c_init();

    MPUdatascaled mpudatasc;
    const MPUdatascaled* mpuInit = mpu_init();
    mpudatasc = (*mpuInit);
    now_time = get_micro_timer_ticks_count();
    uint64_t last_mpu_read_time = now_time;

	HMCdatascaled hmcdatasc;
    const HMCdatascaled* hmcInit = hmc_init();
    hmcdatasc = (*hmcInit);
    now_time = get_micro_timer_ticks_count();
    uint64_t last_hmc_read_time = now_time;

    Barodatascaled bdatasc;
    uint64_t last_ms5_read_time = now_time;
    baro_init();

    // initialize quaternions to 0 rotation 1 0 0 0
    quaternion oreo = {.sc = 1.0, .xi = 0.0, .yj = 0.0, .zk = 0.0};

    state_p->acceleration_local = mpuInit->accl;
    state_p->angular_velocity_local = mpuInit->gyro;
    state_p->magnetic_heading_local = hmcdatasc.magn;
    state_p->orientation = oreo;

    state_p->init = 1;

    while(1)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);

        now_time = get_micro_timer_ticks_count();

        // read mpu every millisecond
        if(now_time - last_mpu_read_time >= 2500)
        {
            // read mpu6050 data, and low pass accl
            get_scaled_MPUdata(&mpudatasc);

            // after reading mpu data, calculate time delta and update the last read time
            now_time = get_micro_timer_ticks_count();
            double time_delta_in_seconds = ((double)(now_time - last_mpu_read_time))/1000000;
            last_mpu_read_time = now_time;

            state_p->angular_velocity_local = mpudatasc.gyro;
            update_vector(&(state_p->acceleration_local), &(mpudatasc.accl), 1.0);

            // simply use gyro and raw accel to find absolute pitch and roll
            state_p->abs_roll[1]
                = (state_p->abs_roll[1]  + state_p->angular_velocity_local.xi * time_delta_in_seconds) * 0.98
                + ((atan( mpudatasc.accl.yj/mpudatasc.accl.zk) - atan( mpuInit->accl.yj/mpuInit->accl.zk)) * 180 / M_PI) * 0.02;
            state_p->abs_pitch[1]
                = (state_p->abs_pitch[1] + state_p->angular_velocity_local.yj * time_delta_in_seconds) * 0.98
                + ((atan(-mpudatasc.accl.xi/mpudatasc.accl.zk) - atan(-mpuInit->accl.xi/mpuInit->accl.zk)) * 180 / M_PI) * 0.02;

            // gyroscope integration logic
            now_time = get_micro_timer_ticks_count();
            quat_raw quat_raw_change;
            get_raw_quaternion_change_from_gyroscope(&quat_raw_change, &oreo, &(mpudatasc.gyro), time_delta_in_seconds);
            quaternion quat_change;
            to_quaternion(&quat_change, &quat_raw_change);
            quaternion final_quat_gyro = oreo;
            hamilton_product(&final_quat_gyro, &quat_change, &oreo);

            // accelerometer magnetometer logic
            quaternion final_quat_accl_magn = oreo;
            get_quaternion_from_vectors_changes(&final_quat_accl_magn, &(state_p->acceleration_local), &(mpuInit->accl), &(state_p->magnetic_heading_local), &(hmcInit->magn));
            conjugate(&final_quat_accl_magn, &final_quat_accl_magn);

            // actual fusion logic called
            slerp_quaternion(&oreo, &final_quat_gyro, GYRO_FUSION_FACTOR, &final_quat_accl_magn);

            // update the global state vector
            state_p->orientation = oreo;
        }

        // read hmc every 13.3 milliseconds
        if(now_time - last_hmc_read_time >= 13340)
        {
            // read hmc5883l data, and low pass magnetometer
            get_scaled_HMCdata(&hmcdatasc);

            // update last read time
            now_time = get_micro_timer_ticks_count();
            last_hmc_read_time = now_time;

            // update the global state vector
            update_vector(&(state_p->magnetic_heading_local), &(hmcdatasc.magn), 1.0);
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
                if(isnan(state_p->altitude) || state_p->altitude == 0.0)
                {
                    state_p->altitude = bdatasc.altitude;
                }
                state_p->altitude = state_p->altitude * 0.9 + bdatasc.altitude * 0.1;

                request_Barodata_temperature();
            }
            else
            {
                request_Barodata_temperature();
            }
            now_time = get_micro_timer_ticks_count();
            last_ms5_read_time = now_time;
        }
    }

    i2c_destroy();
}