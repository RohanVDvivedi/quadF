#include<rc_receiver.h>

static const uint16_t          channel_arr             [CHANNEL_COUNT] = CHANNEL_PINS_ARRAY;
static uint8_t                 channel_nos             [CHANNEL_COUNT];
static volatile uint64_t       channel_values_up_new   [CHANNEL_COUNT];
static volatile uint64_t       channel_values_raw      [CHANNEL_COUNT];

static void on_channel_edge(void* which_channel)
{
    uint64_t now_time;
    timer_get_counter_value(TIMER_GROUP_0, 0, &now_time);

    uint8_t channel_no = *((uint8_t*)(which_channel));
    uint8_t pin_no = channel_arr[channel_no];

    // if high, this is a positive edge
    if(gpio_get_level(pin_no))
    {
        channel_values_up_new[channel_no] = now_time;
    }
    // negative edge
    else
    {
        channel_values_raw[channel_no] = now_time - channel_values_up_new[channel_no];
    }
}

void channels_init()
{
    // setup  and start a timer, so the channels can themselves monitor their ppm signals
    timer_config_t conf;
    conf.counter_en = true;
    conf.counter_dir = TIMER_COUNT_UP;
    conf.divider = 80;
    timer_init(TIMER_GROUP_0, 0, &conf);
    timer_start(TIMER_GROUP_0, 0);

    // set the channel pins direction to input, and to interrupt on any edge
    for(uint8_t i = 0; i < CHANNEL_COUNT; i++)
    {
        channel_nos[i] = i;
        channel_values_up_new[i] = 0;
        channel_values_raw[i] = 0xffffffffffffffff;
        gpio_set_direction(channel_arr[i], GPIO_MODE_INPUT);
        gpio_set_intr_type(channel_arr[i], GPIO_PIN_INTR_ANYEDGE);
    }

    gpio_install_isr_service(0);

    for(uint8_t i = 0; i < CHANNEL_COUNT; i++)
    {
        gpio_isr_handler_add(channel_arr[i], on_channel_edge, &(channel_nos[i]));
        gpio_intr_enable(channel_arr[i]);
    }
}

esp_err_t get_channel_values(uint16_t* channel_values)
{
    esp_err_t err = ESP_OK;
    for(uint8_t i = 0; i < CHANNEL_COUNT; i++)
    {
        channel_values[i] = channel_values_raw[i];
        channel_values[i] = ((channel_values[i] < 999) ? 0 : (channel_values[i] - 1000));
        if(err != ESP_FAIL && (channel_values[i] > 3000 || channel_values[i] < 500))
        {
            err = ESP_FAIL;
        }
    }
    return err;
}

void channels_destroy()
{
    for(uint8_t i = 0; i < CHANNEL_COUNT; i++)
    {
        gpio_isr_handler_remove(channel_arr[i]);
    }
    gpio_uninstall_isr_service();
}