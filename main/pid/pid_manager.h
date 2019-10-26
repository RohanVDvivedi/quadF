#ifndef PID_MANAGER_H
#define PID_MANAGER_H

#include<pid.h>
#include<state.h>

void get_corrections(corrections* corr, state* state_p, channel_state* cstate_p);

#endif