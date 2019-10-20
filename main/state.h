#ifndef STATE_H
#define STATE_H

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

#endif