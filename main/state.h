#ifndef STATE_H
#define STATE_H

#include<geometry.h>

typedef struct state state;
struct state
{
	// the quaternion rotation from the initial state
	quaternion orientation;

	// angular velocity about local sensor axis
	vector angular_velocity_local;

	// acceleration about local sensor axis
	vector acceleration_local;

	// altitude from the ground
	double altitude;

	// the rate at which moving away from ground
	double altitude_rate;
};

extern state State;

void get_current_local_X_axis(vector* xl, quaternion oreo);

void get_current_local_Y_axis(vector* yl, quaternion oreo);

void get_current_local_Z_axis(vector* zl, quaternion oreo);

#endif