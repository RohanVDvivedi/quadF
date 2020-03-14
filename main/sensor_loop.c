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

    state_p->accl_data = mpuInit->accl;
    state_p->gyro_data = mpuInit->gyro;
    state_p->magn_data = hmcInit->magn;

    state_p->init = 1;

    while(1)
    {
        now_time = get_micro_timer_ticks_count();

        // read mpu every millisecond
        if(now_time - last_mpu_read_time >= 2500)   // execution time = 780-850 microseconds
        {
            // read mpu6050 data, and low pass accl
            get_scaled_MPUdata(&mpudatasc);

            // after reading mpu data, calculate time delta and update the last read time
            now_time = get_micro_timer_ticks_count();
            double time_delta_in_seconds = ((double)(now_time - last_mpu_read_time))/1000000;
            last_mpu_read_time = now_time;

            state_p->gyro_data = mpudatasc.gyro;
            state_p->accl_data = mpudatasc.accl;

            // simply use gyro and raw accel to find absolute pitch and roll
            state_p->abs_roll
                = (state_p->abs_roll  + mpudatasc.gyro.xi * time_delta_in_seconds) * GYRO_FUSION_FACTOR
                + ((atan( mpudatasc.accl.yj/mpudatasc.accl.zk) - atan( mpuInit->accl.yj/mpuInit->accl.zk)) * 180 / M_PI) * (1.0 - GYRO_FUSION_FACTOR);
            state_p->abs_pitch
                = (state_p->abs_pitch + mpudatasc.gyro.yj * time_delta_in_seconds) * GYRO_FUSION_FACTOR
                + ((atan(-mpudatasc.accl.xi/mpudatasc.accl.zk) - atan(-mpuInit->accl.xi/mpuInit->accl.zk)) * 180 / M_PI) * (1.0 - GYRO_FUSION_FACTOR);
        }

        // read hmc every 13.3 milliseconds
        if(now_time - last_hmc_read_time >= 13340)  // execution time = 470 microseconds
        {
            // read hmc5883l data, and low pass magnetometer
            get_scaled_HMCdata(&hmcdatasc);

            // update last read time
            now_time = get_micro_timer_ticks_count();
            last_hmc_read_time = now_time;

            // update the global state vector
            state_p->magn_data = hmcdatasc.magn;
        }

        // check on ms5611 every 12 milliseconds
        if(now_time - last_ms5_read_time >= 12000)   // execution time = 600-900 microseconds
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
                if(isnan(state_p->altitude))
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