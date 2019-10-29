#include<pid_manager.h>

pid_state pitch_rate_pid = {
	.constants = {
		.Kp = 1.7,
		.Ki = 0.0,
		.Kd = 0.9,
		.range = 300.0
	}
};

pid_state roll_rate_pid = {
	.constants = {
		.Kp = 1.7,
		.Ki = 0.0,
		.Kd = 0.9,
		.range = 300.0
	}
};

pid_state yaw_rate_pid = {
	.constants = {
		.Kp = 2.0,
		.Ki = 0.01,
		.Kd = 0.53,
		.range = 300.0
	}
};

//#define ERASE_CONSTANTS
//#define TUNE roll_rate_pid

void get_or_map_pid_constants();
void update_pid_constants(pid_state* pid);
void close_persistent_mem();
void erase_all_pid_constants();

void get_corrections(corrections* corr, state* state_p, channel_state* cstate_p)
{
	static uint8_t pid_constants_uninitialized = 1;
	if(pid_constants_uninitialized)
	{
		#if defined(ERASE_CONSTANTS)
			printf("Erase and Rewrite constants from code\n");
			//erase_all_pid_constants();
		#endif
		//get_or_map_pid_constants();
		pid_init(&pitch_rate_pid);
		pid_init(&roll_rate_pid);
		pid_init(&yaw_rate_pid);
		printf("Rr : %lf %lf %lf\n", roll_rate_pid.constants.Kp, roll_rate_pid.constants.Ki, roll_rate_pid.constants.Kd);
		printf("Pr : %lf %lf %lf\n", pitch_rate_pid.constants.Kp, pitch_rate_pid.constants.Ki, pitch_rate_pid.constants.Kd);
		printf("Yr : %lf %lf %lf\n", yaw_rate_pid.constants.Kp, yaw_rate_pid.constants.Ki, yaw_rate_pid.constants.Kd);
		pid_constants_uninitialized = 0;
	}


	#if defined(TUNE)
		static uint8_t old_state = 1;
		static double old_value = 0.0;
		static double initial_value = 0.0;
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
			initial_value = (cstate_p->knob/100);
		}
		// update the value in the eeprom
		else if(old_state == 3 && cstate_p->swit == 2)
		{
			// update the corresponding value in the eeprom
			update_pid_constants(&TUNE);
		}

		if(cstate_p->swit == 3 && var_to_update != NULL)
		{
			(*var_to_update) = old_value + ((cstate_p->knob/100) - initial_value);
			if( (*var_to_update) < 0.0 )
			{
				(*var_to_update) = 0.0;
			}
		}
		old_state = cstate_p->swit;
	#else
		//close_persistent_mem();
	#endif

	vector rate_required;
	rate_required.xi = 3.0 * (cstate_p->roll  - state_p->abs_roll[1]);
	rate_required.yj = 3.0 * (cstate_p->pitch - state_p->abs_pitch[1]);

	vector angular_rates = state_p->angular_velocity_local;

	if(cstate_p->throttle <= 100)
	{
		corr->roll_corr  = 0.0;
		corr->pitch_corr = 0.0;
		corr->yaw_corr   = 0.0;

		pid_init(&pitch_rate_pid);
		pid_init(&roll_rate_pid);
		pid_init(&yaw_rate_pid);
	}
	else
	{
		corr->roll_corr = pid_update(&roll_rate_pid, rate_required.xi, angular_rates.xi);
		corr->pitch_corr  = pid_update(&pitch_rate_pid, rate_required.yj, angular_rates.yj);
		corr->yaw_corr = pid_update(&yaw_rate_pid, 0.0, angular_rates.zk);
	}
	corr->altitude_corr = cstate_p->throttle;

	/*static int it = 0;
	if(it == 10)
	{
		printf("R: %lf %lf \t\t P: %lf %lf\n", cstate_p->roll, state_p->abs_roll[1], cstate_p->pitch, state_p->abs_pitch[1]);
		printf("Rates required : %lf %lf\n", rate_required.xi, rate_required.yj);
		printf("Rates current  : %lf %lf\n", angular_rates.xi, angular_rates.yj);
		printf("corr %lf R : %lf \n P : %lf\n", corr->altitude_corr, corr->roll_corr, corr->pitch_corr);
		it = 0;
    }
	it++;*/
}

#include<nvs_flash.h>
#include<nvs.h>

nvs_handle_t nvs_h;

#define KEY_NAMESPACE "PID_CONSTANTS"

#define ROLL_RATE_CONSTANTS  "rolrat_const"
#define PITCH_RATE_CONSTANTS "pitrat_const"
#define YAW_RATE_CONSTANTS   "yawrat_const"

void init_persist_mem_if_not()
{
	static uint8_t init = 0;
	if(init == 0)
	{
		nvs_flash_init();
		nvs_open(KEY_NAMESPACE, NVS_READWRITE, &nvs_h);
		init = 1;
	}
}

// get_pid_consts_entry_if_present_else_create_empty_one_from_given_data
uint8_t get_pid_consts_entry(char* key, pid_const* data_out)
{
	size_t len = sizeof(pid_const);
	esp_err_t err = nvs_get_blob(nvs_h, key, data_out, &len);
	if(err != ESP_OK)
	{
		nvs_set_blob(nvs_h, key, data_out, sizeof(pid_const));
		return 1;
	}
	return 0;
}

void get_or_map_pid_constants()
{
	init_persist_mem_if_not();
	int to_commit = 0;
	to_commit |= get_pid_consts_entry(ROLL_RATE_CONSTANTS, &(roll_rate_pid.constants));
	to_commit |= get_pid_consts_entry(PITCH_RATE_CONSTANTS, &(pitch_rate_pid.constants));
	to_commit |= get_pid_consts_entry(YAW_RATE_CONSTANTS, &(yaw_rate_pid.constants));
	if(to_commit == 1)
	{
		nvs_commit(nvs_h);
	}
}

void update_pid_consts_entry(char* key, pid_const* data_out)
{
	nvs_set_blob(nvs_h, key, data_out, sizeof(pid_const));
	nvs_commit(nvs_h);
}

void update_pid_constants(pid_state* pid)
{
	init_persist_mem_if_not();
	if(pid == &(pitch_rate_pid))
	{
		update_pid_consts_entry(PITCH_RATE_CONSTANTS, &(pitch_rate_pid.constants));
	}
	else if(pid == &(roll_rate_pid))
	{
		update_pid_consts_entry(ROLL_RATE_CONSTANTS, &(roll_rate_pid.constants));
	}
	else if(pid == &(yaw_rate_pid))
	{
		update_pid_consts_entry(YAW_RATE_CONSTANTS, &(yaw_rate_pid.constants));
	}
}

void close_persistent_mem()
{
	nvs_close(nvs_h);
}

void erase_all_pid_constants()
{
	init_persist_mem_if_not();
	nvs_erase_all(nvs_h);
}