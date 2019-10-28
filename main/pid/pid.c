#include<pid.h>

void pid_init(pid_state* pid)
{
	pid->previous_value    = NAN;
	pid->accumulated_error = 0.0;
}

double pid_update(pid_state* pid, double current_value, double set_point)
{
	// this condition is when the pid_update is forst called
	// we set it to current value so that the derivative component is 0
	if(isnan(pid->previous_value))
	{
		pid->previous_value = current_value;
	}

	// calculate error
	double error = set_point - current_value;

	// propotional component
	double propotional = pid->constants.Kp * error;

	// integral component
	pid->accumulated_error += error;
	double integral = pid->constants.Ki * pid->accumulated_error;

	// do not consider setpoint in error derivative, since this can create un wanted spikes
	double derivative  = pid->constants.Kd * ( - current_value + pid->previous_value);
	pid->previous_value = current_value;

	double result = propotional + integral + derivative;

	// bound result if it is out of bounds
	if(result > pid->constants.range)
	{
		result = pid->constants.range;
	}
	else if(result < -pid->constants.range)
	{
		result = -pid->constants.range;
	}

	return result;
}