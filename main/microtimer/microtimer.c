#include<microtimer.h>

static volatile uint8_t microtimer_is_already_setup = 0;

void micro_timer_init()
{
	if( microtimer_is_already_setup == 0)
	{
		microtimer_is_already_setup = 1;

		// setup  and start a timer, so the channels can themselves monitor their ppm signals
    	timer_config_t conf;
    	conf.counter_en = true;
    	conf.counter_dir = TIMER_COUNT_UP;
    	conf.divider = 80;
    	timer_init(TIMER_GROUP_0, TIMER_0, &conf);
    	timer_start(TIMER_GROUP_0, TIMER_0);
    }
}

uint64_t get_micro_timer_ticks_count()
{
	uint64_t now_ticks;
    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &now_ticks);
    return now_ticks;
}