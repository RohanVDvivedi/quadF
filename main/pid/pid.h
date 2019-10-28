#ifndef PID_H
#define PID_H

#include<math.h>

typedef struct pid_const pid_const;
struct pid_const
{
	double Kp;
	double Ki;
	double Kd;
	double range;
};

typedef struct pid_state pid_state;
struct pid_state
{
	pid_const constants;
	double previous_value;
	double accumulated_error;
};

void pid_init(pid_state* pid);

// it is your responsibility to call pid_update at a regular interval in your program
double pid_update(pid_state* pid, double current_value, double set_point);

#endif