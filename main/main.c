#include<stdio.h>
#include"freertos/FreeRTOS.h"
#include"freertos/task.h"
#include"freertos/queue.h"
#include"sdkconfig.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

// this provides you the mail sensor loop of the flight controller
#include<sensor_loop.h>

// this is where we finally write out our correction calculations to motor speeds
#include<motor_manager.h>

// this is what calculates corrections of the drone from the channel state and the current state
#include<pid_manager.h>

#include<state.h>

#define BLINK_GPIO 2

state curr_state;

channel_state chn_state;

corrections corr;

void app_main(void)
{
    // stay away from the remote and drone give no control inputs while the led on the ESP32 is lit
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BLINK_GPIO, 1);

    // this will initialize the input from the rc receiver
    channels_init();

    // this will turn on all the bldc motors and set their min and max speed setting (this setting can be controller from bldc.h)
    all_bldc_init();

    TaskHandle_t sensorEventLoopHandle = NULL;
    xTaskCreate(sensor_event_loop, "SENSOR_EVENT_LOOP", 4096, &curr_state, configMAX_PRIORITIES - 1, sensorEventLoopHandle);

    state curr_state_t = curr_state;
    while(curr_state_t.init == 0)
    {
        printf("not ready\n");
        vTaskDelay(500 / portTICK_PERIOD_MS);
        curr_state_t = curr_state;
    }

    gpio_set_level(BLINK_GPIO, 0);
    // gpio off so now give controls

    micro_timer_init();
    micro_timer_start();

    timer_event tim_evnt = 0;
    QueueHandle_t eventQueue = xQueueCreate(8, sizeof(tim_evnt));

    //register_microtimer_event(PID_UPDATE, 2500, eventQueue);
    register_microtimer_event(TEST, 1000000, eventQueue);

    while(xQueueReceive(eventQueue, &tim_evnt, portMAX_DELAY) == pdPASS)
    {
        if(tim_evnt == PID_UPDATE)
        {
            state curr_state_t = curr_state;
            update_channel_state(&chn_state);
            get_corrections(&corr, &curr_state_t, &chn_state);
            write_corrections_to_motors(&corr);
        }
        else if(tim_evnt == TEST)
        {
            printf("Test\n");
        }
    }
    vQueueDelete(eventQueue);

    if(sensorEventLoopHandle != NULL)
    {
        vTaskDelete(sensorEventLoopHandle);
    }

    channels_destroy();
}