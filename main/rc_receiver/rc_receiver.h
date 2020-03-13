#ifndef RC_RECEIVER_H
#define RC_RECEIVER_H

#include "driver/gpio.h"

#include<microtimer.h>

#define MAX_ANGLE 25

typedef enum channel_purpose channel_purpose;
enum channel_purpose
{
	KNOB,		// throttle, VarA, VarB etc like knobs value = [0, 1000]
	ANGLE,		// the angles that we get for yaw pitch and roll = []
	SWITCH		// switch can be in 3 positions, 1, 2 and 3
};

// the pins that are taking input from the channels, in CHANNEL_PINS_ARRAY[0] = 54 means channel 0 is connected to controller pin 54
#define CHANNEL_COUNT            6
#define CHANNEL_PINS_ARRAY       {34,35,4,5,23,27}
#define CHANNEL_PURPOSE_ARRAY    {ANGLE,ANGLE,KNOB,ANGLE,SWITCH,KNOB}

// input channels
void channels_init();
esp_err_t get_channel_values(uint16_t* channel_values);
esp_err_t get_channel_values_scaled(double* channel_values_d);
void channels_destroy();

#endif