#include<sensor_loop.h>

#if   defined(ORIENTATION_ONLY_GYRO)
    #define GYRO_FUSION_FACTOR 1.00
#elif defined(ORIENTATION_ONLY_ACCL_MAGN)
    #define GYRO_FUSION_FACTOR 0.00
#else
    #define GYRO_FUSION_FACTOR 0.98
#endif

void sensor_event_loop(void* state_pointer)
{
    state* state_p = ((state*)(state_pointer));

    i2c_init();

    MPUdatascaled mpudatasc;
    const MPUdatascaled* mpuInit = mpu_init();
    mpudatasc = (*mpuInit);
    uint64_t last_mpu_read_time = get_micro_timer_ticks_count();
    double time_delta_in_seconds = 0;

	HMCdatascaled hmcdatasc;
    const HMCdatascaled* hmcInit = hmc_init();
    hmcdatasc = (*hmcInit);
    uint64_t last_hmc_read_time = get_micro_timer_ticks_count();

    Barodatascaled bdatasc;
    baro_init();
    uint64_t last_ms5_read_time = get_micro_timer_ticks_count();

    state_p->accl_data = mpuInit->accl;
    state_p->gyro_data = mpuInit->gyro;
    state_p->magn_data = hmcInit->magn;

    state_p->init = 1;

    micro_timer_init();
    micro_timer_start();

    timer_event tim_evnt = 0;
    QueueHandle_t eventQueue = xQueueCreate(8, sizeof(uint8_t));

    // reading MPU6050 every 2500 microseconds
    register_microtimer_event(MPU_READ, 2500, eventQueue);
    // reading HMC5883l every 13340 microseconds
    register_microtimer_event(HMC_READ, 13340, eventQueue);
    // reading MS5611 every 12000 microseconds
    register_microtimer_event(MS5_READ, 12000, eventQueue);

    register_microtimer_event(TEST_SENSOR, 3000000, eventQueue);

    while(xQueueReceive(eventQueue, &tim_evnt, portMAX_DELAY) == pdPASS)
    {
        switch(tim_evnt)
        {
            // read mpu every 2.5 milliseconds
            case MPU_READ :   // execution time = 780-850 microseconds
            {
                // read mpu6050 data, and low pass accl
                get_scaled_MPUdata(&mpudatasc);

                // after reading mpu data, calculate time delta and update the last read time
                time_delta_in_seconds = ((double)(get_micro_timer_ticks_count() - last_mpu_read_time))/1000000.0;
                last_mpu_read_time = get_micro_timer_ticks_count();

                state_p->gyro_data = mpudatasc.gyro;
                state_p->accl_data = mpudatasc.accl;

                // simply use gyro and raw accel to find absolute pitch and roll
                state_p->abs_roll
                    = (state_p->abs_roll  + mpudatasc.gyro.xi * time_delta_in_seconds) * GYRO_FUSION_FACTOR
                    + ((atan( mpudatasc.accl.yj/mpudatasc.accl.zk) - atan( mpuInit->accl.yj/mpuInit->accl.zk)) * 180 / M_PI) * (1.0 - GYRO_FUSION_FACTOR);
                state_p->abs_pitch
                    = (state_p->abs_pitch + mpudatasc.gyro.yj * time_delta_in_seconds) * GYRO_FUSION_FACTOR
                    + ((atan(-mpudatasc.accl.xi/mpudatasc.accl.zk) - atan(-mpuInit->accl.xi/mpuInit->accl.zk)) * 180 / M_PI) * (1.0 - GYRO_FUSION_FACTOR);

                // THIS SHIT BELOW MUST NOT BE DONE
                // BUT I NEED THIS WORKING BADLY, SO DID IT ANYWAY
                    // feed watchdog timer 0
                    TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
                    TIMERG0.wdt_feed=1;
                    TIMERG0.wdt_wprotect=0;
                    // feed watchdog timer 1
                    TIMERG1.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
                    TIMERG1.wdt_feed=1;
                    TIMERG1.wdt_wprotect=0;

                break;
            }
            // read hmc every 13.34 milliseconds
            case HMC_READ :  // execution time = 470 microseconds
            {
                // read hmc5883l data, and low pass magnetometer
                get_scaled_HMCdata(&hmcdatasc);

                // update last read time
                last_hmc_read_time = get_micro_timer_ticks_count();

                // update the global state vector
                state_p->magn_data = hmcdatasc.magn;
                break;
            }
            // check on ms5611 every 12 milliseconds
            case MS5_READ :   // execution time = 600-900 microseconds
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
                last_ms5_read_time = get_micro_timer_ticks_count();
                break;
            }
            case TEST_SENSOR :
            {
                if(time_delta_in_seconds < 2500.0/1000000.0)
                {
                    printf("Test Sensor mpu = %lf\n", time_delta_in_seconds * 1000000.0);
                    peek_microtimer_event(MPU_READ);
                    /*printf("A: %lf, %lf, %lf \n\n", state_p->accl_data.xi, state_p->accl_data.yj, state_p->accl_data.zk);
                    printf("M: %lf, %lf, %lf \n\n", state_p->magn_data.xi, state_p->magn_data.yj, state_p->magn_data.zk);
                    printf("G: %lf, %lf, %lf \n\n", state_p->gyro_data.xi, state_p->gyro_data.yj, state_p->gyro_data.zk);
                    printf("R: %lf \t \t P: %lf \n\n", state_p->abs_roll, state_p->abs_pitch);
                    printf("Alt: %lf \n\n", state_p->altitude);
                    printf("-------------------------\n\n\n");*/
                }
                break;
            }
            default :
            {
                printf("unrecognized event in sensor event loop %u\n", tim_evnt);
                break;
            }
        }
    }
    vQueueDelete(eventQueue);

    i2c_destroy();
}