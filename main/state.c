#include<state.h>

void update_channel_state(channel_state* cstate)
{
	double channels_d[6];
	get_channel_values_scaled(channels_d);
	cstate->roll 		= channels_d[0];
	cstate->pitch 		= channels_d[1];
	cstate->throttle 	= channels_d[2];
	cstate->yaw	 		= -channels_d[3];
	cstate->swit 		= (uint8_t)channels_d[4];
	cstate->knob 		= channels_d[5];
}