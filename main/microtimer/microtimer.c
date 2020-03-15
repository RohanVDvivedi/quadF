#include<microtimer.h>

static volatile uint8_t microtimer_is_already_setup = 0;

void micro_timer_init()
{
	if( microtimer_is_already_setup == 0)
	{
		microtimer_is_already_setup = 1;

		// setup a timer
    	timer_config_t conf;
    	conf.counter_en = true;
    	conf.counter_dir = TIMER_COUNT_UP;
    	conf.divider = 80;
    	timer_init(TIMER_GROUP_0, 0, &conf);
    }
}

static volatile uint8_t microtimer_is_already_running = 0;

void micro_timer_start()
{
    if( microtimer_is_already_running == 0)
    {
        microtimer_is_already_running = 1;

        // start the timer
        timer_start(TIMER_GROUP_0, 0);
    }
}

uint64_t get_micro_timer_ticks_count()
{
	uint64_t now_ticks;
    timer_get_counter_value(TIMER_GROUP_0, 0, &now_ticks);
    return now_ticks;
}