#include<pid_manager.h>

pid_state pitch_rate_pid = {
	.constants = {
		.Kp = 0.0,
		.Ki = 0.0,
		.Kd = 0.0,
		.Irange = 200.0
	}
};

pid_state roll_rate_pid = {
	.constants = {
		.Kp = 0.0,
		.Ki = 0.0,
		.Kd = 0.0,
		.Irange = 200.0
	}
};

#define TUNE pitch_rate_pid

void get_corrections(corrections* corr, state* state_p, channel_state* cstate_p)
{
	#if defined(TUNE)
		static uint8_t old_state = 1;
		static double old_value = 0.0;
		static double* var_to_update = NULL;
		// pick something up to update if 1->2
		if(old_state == 1 && cstate_p->swit == 2)
		{
			if(var_to_update == NULL || var_to_update == &(TUNE.constants.Kd))
			{
				var_to_update = &(TUNE.constants.Kp);
			}
			else if(var_to_update == &(TUNE.constants.Kp))
			{
				var_to_update = &(TUNE.constants.Ki);
			}
			else if(var_to_update == &(TUNE.constants.Ki))
			{
				var_to_update = &(TUNE.constants.Kd);
			}
		}
		// save the old state so we can try and start updating
		else if(old_state == 2 && cstate_p->swit == 3)
		{
			old_value = (*var_to_update);
		}
		// update the value in the eeprom
		else if(old_state == 3 && cstate_p->swit == 2)
		{
			// update the corresponding value in the eeprom
		}

		if(cstate_p->swit == 3)
		{
			(*var_to_update) = old_value + (cstate_p->knob/100);
		}
		old_state = cstate_p->swit;
	#endif

	vector rate_required;
	rate_required.xi = cstate_p->roll;
	rate_required.yj = cstate_p->pitch;

	vector angular_rates = state_p->angular_velocity_local;
	angular_rates.xi = fabs(angular_rates.xi) < 1.5 ? 0.0 : angular_rates.xi;
	angular_rates.yj = fabs(angular_rates.yj) < 1.5 ? 0.0 : angular_rates.yj;
	angular_rates.zk = fabs(angular_rates.zk) < 1.5 ? 0.0 : angular_rates.zk;

	if(cstate_p->throttle < 100)
	{
		corr->roll_corr  = 0.0;
		corr->pitch_corr = 0.0;
		corr->yaw_corr   = 0.0;

		if(cstate_p->throttle < 50)
		{
			pitch_rate_pid.accumulated_error = 0.0;
			roll_rate_pid.accumulated_error = 0.0;
		}
	}
	else
	{
		corr->roll_corr = pid_update(&roll_rate_pid, rate_required.xi, angular_rates.xi);
		corr->pitch_corr  = pid_update(&pitch_rate_pid, rate_required.yj, angular_rates.yj);
		corr->yaw_corr = 0.0;
	}
	corr->altitude_corr = cstate_p->throttle;
}