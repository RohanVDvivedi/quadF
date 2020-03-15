#ifndef MICRO_TIMER_H
#define MICRO_TIMER_H

#include"freertos/FreeRTOS.h"
#include"freertos/queue.h"
#include"driver/timer.h"

// this is the timer, we are going to use
// each tick of this counter is a microsecond apart

#define MAX_TIMER_EVENTS 8

void micro_timer_init();
void micro_timer_start();
void register_microtimer_event(uint8_t timer_event_no, uint64_t every_x_ticks, QueueHandle_t queue_to_inform_event);
uint64_t get_micro_timer_ticks_count();

#endif