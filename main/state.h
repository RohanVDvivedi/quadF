#ifndef STATE_H
#define STATE_H

#include<rc_receiver.h>
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

	// magnetic heading of earth wrt local axis
	vector magnetic_heading_local;

	// altitude from the ground
	double altitude;

	// the rate at which moving away from ground
	double altitude_rate;
};

typedef struct channel_state channel_state;
struct channel_state
{
	double throttle;
	double yaw;
	double pitch;
	double roll;

	uint8_t swit;
	double knob;
};

extern state State;
extern channel_state cstate;

void get_current_local_X_axis(vector* xl);

void get_current_local_Y_axis(vector* yl);

void get_current_local_Z_axis(vector* zl);

void get_absolute_rotation_angles_about_local_axis(vector* angles);

void update_channel_state();

#endif