#include<state.h>

state State = {
    .orientation = {.sc = 1.0, .xi = 0.0, .yj = 0.0, .zk = 0.0},
    .angular_velocity_local = {.xi = 0.0, .yj = 0.0, .zk = 0.0},
    .acceleration_local = {.xi = 0.0, .yj = 0.0, .zk = 0.0},
    .altitude = -1,
    .altitude_rate = 0.0
};

void get_current_local_X_axis(vector* xl)
{
	vector X = {.xi = 1.0, .yj = 0.0, .zk = 0.0};
	rotate_vector(xl, &(State.orientation), &X);
}

void get_current_local_Y_axis(vector* yl)
{
	vector Y = {.xi = 0.0, .yj = 1.0, .zk = 0.0};
	rotate_vector(yl, &(State.orientation), &Y);
}

void get_current_local_Z_axis(vector* zl)
{
	vector Z = {.xi = 1.0, .yj = 0.0, .zk = 1.0};
	rotate_vector(zl, &(State.orientation), &Z);
}

void get_absolute_rotation_angles_about_local_axis(vector* angles)
{
	// get unit vectors about local axis, wrt to global axis
	// the global axis is the initial position of the sensor board
	vector xl;	get_current_local_X_axis(&xl);
	vector yl;	get_current_local_Y_axis(&yl);
	vector zl;	get_current_local_Z_axis(&zl);

	// ABSOLUTE roll calculation
	// angles.xi = absolute roll

	// ABSOLUTE pitch calculation
	// angles.yj = absolute pitch

	// ABSOLUTE yaw calculation
	// angles.zk = absolute yaw
}