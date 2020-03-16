#include<microtimer.h>

typedef struct timer_event_info timer_event_info;
struct timer_event_info
{
    uint8_t enabled;
    uint64_t every_x_ticks;
    uint64_t last_occurence;
    uint64_t next_occurence;    // we will always find out the event with the smallest next occurence
    QueueHandle_t queue_to_inform_event;
};

// you may not have any more than MAX_TIMER_EVENTS timer events, in your application
// increase this value by changing this number :p
timer_event_info timer_events_informations[MAX_TIMER_EVENTS];

#include"driver/gpio.h"
#define BLINK_GPIO 2
void timer_event_isr(void* param)
{
    int level = gpio_get_level(BLINK_GPIO);
    gpio_set_level(BLINK_GPIO, 1 - level);
    uint8_t event_test = 0;
    xQueueSendFromISR(timer_events_informations[event_test].queue_to_inform_event, &event_test, NULL);
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[0].update = 1;
    if((intr_status & BIT(0)))
    {
        uint64_t now_ticks_count;
        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &now_ticks_count);
        uint64_t minimum_next_occurence_value = (uint64_t)((int64_t)(-1));
        for(uint8_t i = 0; i < MAX_TIMER_EVENTS; i++)
        {
            if(timer_events_informations[i].enabled)
            {
                timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &now_ticks_count);
                if(timer_events_informations[i].next_occurence <= now_ticks_count)
                {
                    xQueueSendFromISR(timer_events_informations[i].queue_to_inform_event, &i, NULL);
                    timer_events_informations[i].last_occurence = now_ticks_count;
                    timer_events_informations[i].next_occurence = timer_events_informations[i].last_occurence + timer_events_informations[i].every_x_ticks;
                }
                if(timer_events_informations[i].next_occurence < minimum_next_occurence_value)
                {
                    minimum_next_occurence_value = timer_events_informations[i].next_occurence;
                }
            }
        }
        TIMERG0.int_clr_timers.t0 = 1;
        timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, minimum_next_occurence_value);
        timer_set_alarm(TIMER_GROUP_0, TIMER_0, TIMER_ALARM_EN);
    }
}

//
// The externally accessible microtimer functions are given below
//

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
    	timer_init(TIMER_GROUP_0, TIMER_0, &conf);
        timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
    }
}

static volatile uint8_t microtimer_is_already_running = 0;

void micro_timer_start()
{
    if( microtimer_is_already_running == 0)
    {
        microtimer_is_already_running = 1;

        // set interrupt and start the timer, the interrupt goes off, if and only if alarm is set
        timer_enable_intr(TIMER_GROUP_0, TIMER_0);
        timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_event_isr, NULL, 0, NULL);
        timer_start(TIMER_GROUP_0, TIMER_0);
    }
}

void register_microtimer_event(uint8_t timer_event_no, uint64_t every_x_ticks, QueueHandle_t queue_to_inform_event)
{
    uint64_t now_ticks_count = get_micro_timer_ticks_count();

    timer_events_informations[timer_event_no].queue_to_inform_event = queue_to_inform_event;
    timer_events_informations[timer_event_no].every_x_ticks = every_x_ticks;
    timer_events_informations[timer_event_no].last_occurence = now_ticks_count;
    timer_events_informations[timer_event_no].next_occurence = now_ticks_count + every_x_ticks;
    timer_events_informations[timer_event_no].enabled = 1;

    uint64_t alarm_value;
    timer_get_alarm_value(TIMER_GROUP_0, TIMER_0, &alarm_value);
    if(alarm_value > timer_events_informations[timer_event_no].next_occurence || alarm_value <= now_ticks_count)
    {
        alarm_value = timer_events_informations[timer_event_no].next_occurence;
    }

    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, alarm_value);
    timer_set_alarm(TIMER_GROUP_0, TIMER_0, TIMER_ALARM_EN);
    xQueueSend(timer_events_informations[timer_event_no].queue_to_inform_event, &timer_event_no, 0);
}

uint64_t get_micro_timer_ticks_count()
{
	uint64_t now_ticks;
    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &now_ticks);
    return now_ticks;
}