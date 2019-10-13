#ifndef RC_RECEIVER_H
#define RC_RECEIVER_H

// for channel inputs
#include "driver/gpio.h"
#include "driver/timer.h"

#include<millitimer.h>

// the pins that are taking input from the channels, in CHANNEL_PINS_ARRAY[0] = 54 means channel 0 is connected to controller pin 54
#define CHANNEL_COUNT 4
#define CHANNEL_PINS_ARRAY {34,35,4,5}

// input channels
void channels_init();
esp_err_t get_channel_values(uint16_t* channel_values);
void channels_destroy();

#endif