#include<sensor_loop.h>

void sensor_loop(void* not_required)
{
    milli_timer_init();
    int64_t now_time = get_milli_timer_ticks_count();

    i2c_init();

    MPUdatascaled mpudatasc;
    uint64_t last_mpu_read_time = now_time;
    const MPUdatascaled* mpuInit = mpu_init();
    mpudatasc = (*mpuInit);

    State.acceleration_local = mpuInit->accl;
    State.angular_velocity_local = mpuInit->gyro;

	HMCdatascaled hmcdatasc;
    uint64_t last_hmc_read_time = now_time;
    const HMCdatascaled* hmcInit = hmc_init();
    hmcdatasc = (*hmcInit);

    Barodatascaled bdatasc;
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
            // read mpu6050 data, but also low pass the accelerometer data
            vector accl_old = mpudatasc.accl;
            get_scaled_MPUdata(&mpudatasc);

            // after reading mpu data, calculate time and update the last read time
            now_time = get_milli_timer_ticks_count();
            double time_delta_in_seconds = ((double)(now_time - last_mpu_read_time))/1000000;
            last_mpu_read_time = now_time;

            update_vector(&accl_old, &(mpudatasc.accl), 0.02);
            mpudatasc.accl = accl_old;

            // gyroscope integration logic
            now_time = get_milli_timer_ticks_count();
            State.temp = State.temp + (mpudatasc.gyro.zk * time_delta_in_seconds);
            quat_raw quat_raw_change;
            get_raw_quaternion_change_from_gyroscope(&quat_raw_change, &oreo, &(mpudatasc.gyro), time_delta_in_seconds);
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
            update_vector(&(State.acceleration_local), &(mpudatasc.accl), 1.0);
        }

        // read hmc every 11 milliseconds
        if(now_time - last_hmc_read_time >= 11000)
        {
            // read hmc5883l data
            get_scaled_HMCdata(&hmcdatasc);

            // update last read time
            now_time = get_milli_timer_ticks_count();
            last_hmc_read_time = now_time;

            // update the global state vector
            update_vector(&(State.magnetic_heading_local), &(hmcdatasc.magn), 1.0);
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