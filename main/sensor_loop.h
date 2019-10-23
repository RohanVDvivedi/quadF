#ifndef SENSOR_LOOP_H
#define SENSOR_LOOP_H

// the i2c comm has to be started hence i2c_comm
#include<i2c_comm.h>

// this is where we get our scaled sensor readings from
#include<gy86.h>

// this timer is needed to understand whwn to do the calculation
#include<millitimer.h>

// this is wehre we get a global state variable, which gets updated by the sensor logic
#include<state.h>

//#define ORIENTATION_ONLY_ACCL_MAGN
//#define ORIENTATION_ONLY_GYRO

void sensor_loop(void* not_required);

#endif