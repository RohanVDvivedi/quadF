#include<bldc.h>

void all_bldc_init()
{
    #if defined(MOTOR_RANGE_SETUP)
        write_values_bldc(1000, 1000, 1000, 1000);
        vTaskDelay(4000 / portTICK_PERIOD_MS);
        write_values_bldc(0, 0, 0, 0);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    #else
        write_values_bldc(0, 0, 0, 0);
    #endif
}

void write_values_bldc(unsigned int left_front, unsigned int right_front, unsigned int left_back, unsigned int right_back)
{
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
        pwm_config.frequency = 400;
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

    // set motor speed values
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (left_front  > 1000 ? 1000 : left_front ) + 1000);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, (right_front > 1000 ? 1000 : right_front) + 1000);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, (left_back   > 1000 ? 1000 : left_back  ) + 1000);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, (right_back  > 1000 ? 1000 : right_back ) + 1000);
}