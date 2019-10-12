#ifndef BLDC_H
#define BLDC_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// for pwm for bldcs
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

// motor pins
#define LEFT_FRONT_MOTOR    33
#define RIGHT_FRONT_MOTOR   32
#define LEFT_BACK_MOTOR     25
#define RIGHT_BACK_MOTOR    26

// function to write values to bldc motors
void all_bldc_init();
void write_values_bldc(unsigned int left_front, unsigned int right_front, unsigned int left_back, unsigned int right_back);

#endif