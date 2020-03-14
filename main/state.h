#ifndef STATE_H
#define STATE_H

#include<rc_receiver.h>
#include<geometry.h>

typedef struct state state;
struct state
{
	// sensor module will set this as 1, when we can start reading
	uint8_t init;

	// angular velocity about local sensor axis
	vector gyro_data;

	// acceleration about local sensor axis
	vector accl_data;

	// magnetic heading of earth wrt local axis
	vector magn_data;

	// angle of required to rotate about X axis, for Y axis to be parallel to horizon
	double abs_roll;

	// angle of required to rotate about Y axis, for X axis to be parallel to horizon
	double abs_pitch;

	// altitude from the ground
	double altitude;
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

typedef struct corrections corrections;
struct corrections
{
	double yaw_corr;
	double pitch_corr;
	double roll_corr;
	double altitude_corr;
};

void update_channel_state(channel_state* cstate);

#endif