#include<state.h>

state State = {
    .orientation = {.sc = 1.0, .xi = 0.0, .yj = 0.0, .zk = 0.0},
    .angular_velocity_local = {.xi = 0.0, .yj = 0.0, .zk = 0.0},
    .acceleration_local = {.xi = 0.0, .yj = 0.0, .zk = 0.0},
    .altitude = -1,
    .altitude_rate = 0.0
};

void get_current_local_X_axis(vector* xl, quaternion oreo)
{

}

void get_current_local_Y_axis(vector* yl, quaternion oreo)
{
	
}

void get_current_local_Z_axis(vector* zl, quaternion oreo)
{
	
}