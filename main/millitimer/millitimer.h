#ifndef MILLI_TIMER_H
#define MILLI_TIMER_H

#include "driver/timer.h"

// this is the timer, we are going to use
// each tick of this counter is a millisecond apart

void milli_timer_init();
uint64_t get_milli_timer_ticks_count();

#endif