#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#define BLINK_GPIO 2

void write_values_bldc(unsigned int left_front, unsigned int right_front, unsigned int left_back, unsigned int right_back);

void app_main(void)
{
    // motor startup and max min configuring 
    // this is the first call hence the motors will be initalized also to their max values
    write_values_bldc(1000, 1000, 1000, 1000);
    vTaskDelay(4000 / portTICK_PERIOD_MS);
    write_values_bldc(0, 0, 0, 0);
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while(1)
    {
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 0);

        write_values_bldc(1000, 0, 0, 0);

        vTaskDelay(10000 / portTICK_PERIOD_MS);

        write_values_bldc(0, 1000, 0, 0);

        vTaskDelay(10000 / portTICK_PERIOD_MS);

        write_values_bldc(0, 0, 1000, 0);

        vTaskDelay(10000 / portTICK_PERIOD_MS);

        write_values_bldc(0, 0, 0, 1000);

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void write_values_bldc(unsigned int left_front, unsigned int right_front, unsigned int left_back, unsigned int right_back)
{

    #define LEFT_FRONT_MOTOR    33
    #define RIGHT_FRONT_MOTOR   32
    #define LEFT_BACK_MOTOR     25
    #define RIGHT_BACK_MOTOR    26

    static unsigned char setup_done = 0;

    if(setup_done == 0)
    {
        // the gpio are initialized and connected to corresponding output of MCPWM hardware
        mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, LEFT_FRONT_MOTOR);
        mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, RIGHT_FRONT_MOTOR);
        mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, LEFT_BACK_MOTOR);
        mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, RIGHT_BACK_MOTOR);

        // counter initial config we are going to use
        mcpwm_config_t pwm_config;
        pwm_config.frequency = 50;
        pwm_config.cmpr_a = 0;
        pwm_config.cmpr_b = 0;
        pwm_config.counter_mode = MCPWM_UP_COUNTER;
        pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

        // setup both the counters
        mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
        mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);

        // mark the setup as done, so we do not execute it again
        setup_done = 1;
    }



    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (left_front  > 1000 ? 1000 : left_front ) + 1000);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, (right_front > 1000 ? 1000 : right_front) + 1000);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, (left_back   > 1000 ? 1000 : left_back  ) + 1000);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, (right_back  > 1000 ? 1000 : right_back ) + 1000);
}