#ifndef MICRO_TIMER_H
#define MICRO_TIMER_H

#include "driver/timer.h"

// this is the timer, we are going to use
// each tick of this counter is a microsecond apart

void micro_timer_init();
uint64_t get_micro_timer_ticks_count();

#endif