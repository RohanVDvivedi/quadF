#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

// for pwm for bldcs
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

// for i2c for sensors
#include "driver/i2c.h"

#define BLINK_GPIO 2

#define I2C_MASTER_SCL_IO   22
#define I2C_MASTER_SDA_IO   21

// i2c addresses, for connected sensors namesly MPU6050 : accl/gyro, HMC5883L : magnetometer and MS5611 : barometer
#define MPU6050_ADDRESS     0x68
#define MS5611_ADDRESS      0x77

// motor pins
#define LEFT_FRONT_MOTOR    33
#define RIGHT_FRONT_MOTOR   32
#define LEFT_BACK_MOTOR     25
#define RIGHT_BACK_MOTOR    26

// the pins that are taking input from the channels, in CHANNEL_PINS_ARRAY[0] = 54 means channel 0 is connected to controller pin 54
#define CHANNEL_COUNT 1
#define CHANNEL_PINS_ARRAY {54}

typedef struct IMUdata IMUdata;
struct IMUdata
{
    int16_t accx;
    int16_t accy;
    int16_t accz;
    int16_t temp;
    int16_t gyrox;
    int16_t gyroy;
    int16_t gyroz;
    int16_t magx;
    int16_t magy;
    int16_t magz;
};

typedef struct IMUdatascaled IMUdatascaled;
struct IMUdatascaled
{
    double accx;
    double accy;
    double accz;
    double temp;
    double gyrox;
    double gyroy;
    double gyroz;
    double magx;
    double magy;
    double magz;
};

// input channels
void channels_init();
esp_err_t get_channel_values(uint16_t* channel_values);
void channels_destroy();

// mpu data
void imu_init();
esp_err_t get_raw_IMUdata(IMUdata* data);


// basic i2c functionality, could be similar for most sensors, being user for mpu6050 and will be used for ms5611
void i2c_init();
esp_err_t i2c_read(uint8_t device_address, uint8_t reg_address, void* buffer, unsigned int bytes_to_read);
esp_err_t i2c_write(uint8_t device_address, uint8_t reg_address, void* buffer, unsigned int bytes_to_write);
void i2c_destroy();

// function to write values to bldc motors
void all_bldc_init();
void write_values_bldc(unsigned int left_front, unsigned int right_front, unsigned int left_back, unsigned int right_back);

void app_main(void)
{
    //i2c_init();
    //imu_init();
    //all_bldc_init();
    channels_init();

    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    uint16_t channel_values[CHANNEL_COUNT];

    do
    {
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);


        esp_err_t erro = get_channel_values(channel_values);

        printf("error = %x\n", erro);
        for(uint8_t i = 0; i < CHANNEL_COUNT; i++)
        {
            printf("%u\t", channel_values[i]);
        }
        printf("\n");
    }
    while(1);

    //i2c_destroy();
    channels_destroy();
}

const uint16_t          channel_arr             [CHANNEL_COUNT] = CHANNEL_PINS_ARRAY;
uint8_t                 channel_nos             [CHANNEL_COUNT];
volatile uint16_t       channel_values_up_new   [CHANNEL_COUNT];
volatile uint16_t       channel_values_raw      [CHANNEL_COUNT];

static void on_channel_edge(void* which_channel)
{
    uint8_t channel_no = *((uint8_t*)(which_channel));
    uint8_t pin_no = channel_arr[channel_no];

    // if high, this is a positive edge
    if(gpio_get_level(pin_no))
    {
        channel_values_up_new[channel_no] = now();
    }
    // negative edge
    else
    {
        channel_values_raw[channel_no] = now() - channel_values_up_new[channel_no];
    }
}

void channels_init()
{
    // set the channel pins direction to input, and to interrupt on any edge
    for(uint8_t i = 0; i < CHANNEL_COUNT; i++)
    {
        channel_nos[i] = i;
        gpio_set_direction(channel_arr[i], GPIO_MODE_INPUT);
        gpio_set_intr_type(channel_arr[i], GPIO_PIN_INTR_ANYEDGE);
    }

    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);

    for(uint8_t i = 0; i < CHANNEL_COUNT; i++)
    {
        gpio_isr_handler_add(channel_arr[i], on_channel_edge, &(channel_nos[i]));
    }
}

esp_err_t get_channel_values(uint16_t* channel_values)
{
    esp_err_t err = ESP_OK;
    for(uint8_t i = 0; i < CHANNEL_COUNT; i++)
    {
        channel_values[i] = channel_values_raw[i] - 1000;
        if(err != ESP_FAIL && (channel_values[i] > 2500 || channel_values[i] < 1500))
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

void imu_init()
{
    uint8_t data;

    // write 0 to pwr_mgmt_1 register to wake it up
    data = 0x00;
    i2c_write(MPU6050_ADDRESS, 0x6b, &data, 1);
}

esp_err_t get_raw_IMUdata(IMUdata* data)
{
    esp_err_t err = i2c_read(MPU6050_ADDRESS, 0x3b, data, sizeof(IMUdata));

    if(err != ESP_OK)
    {
        return err;
    }

    // we have to change the data from network ordr to host order integers
    data->accx = (data->accx << 8) | ((data->accx >> 8) & 0x00ff);
    data->accy = (data->accy << 8) | ((data->accy >> 8) & 0x00ff);
    data->accz = (data->accz << 8) | ((data->accz >> 8) & 0x00ff);
    data->temp = (data->temp << 8) | ((data->temp >> 8) & 0x00ff);
    data->gyrox = (data->gyrox << 8) | ((data->gyrox >> 8) & 0x00ff);
    data->gyroy = (data->gyroy << 8) | ((data->gyroy >> 8) & 0x00ff);
    data->gyroz = (data->gyroz << 8) | ((data->gyroz >> 8) & 0x00ff);
    data->magx = (data->magx << 8) | ((data->magx >> 8) & 0x00ff);
    data->magy = (data->magy << 8) | ((data->magy >> 8) & 0x00ff);
    data->magz = (data->magz << 8) | ((data->magz >> 8) & 0x00ff);

    return err;
}

void i2c_init()
{
    i2c_port_t port = I2C_NUM_1;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(port, &conf);
    i2c_driver_install(port, conf.mode, 0, 0, 0);
}

esp_err_t i2c_read(uint8_t device_address, uint8_t reg_address, void* buffer, unsigned int bytes_to_read)
{
    // get a handle on which you want to queue the i2c communiction commands of write and reads
    i2c_cmd_handle_t handle = i2c_cmd_link_create();

        // send start bit from master
        i2c_master_start(handle);

            // write address of the device on the bus, last param true signifies we are checking for slave to ack on receive
            i2c_master_write_byte(handle, device_address << 1, true);

            if(bytes_to_read > 0)
            {
                // write address from where you want to start reading, last param true signifies we are checking for slave to ack on receive
                i2c_master_write_byte(handle, reg_address, true);

                // send start bit from master
                i2c_master_start(handle);

                // write address of the device on the bus, last param true signifies we are checking for slave to ack on receive
                i2c_master_write_byte(handle, (device_address << 1) | 1, true);

                // read data from there, but send NACK at the last byte read, instead of ACK
                i2c_master_read(handle, buffer, bytes_to_read, I2C_MASTER_LAST_NACK);
            }

        // stop bit from master
        i2c_master_stop(handle);

        // execute the commands
        esp_err_t error = i2c_master_cmd_begin(I2C_NUM_1, handle, 1000 / portTICK_PERIOD_MS);

    // delete the handle once done
    i2c_cmd_link_delete(handle);

    return error;
}

esp_err_t i2c_write(uint8_t device_address, uint8_t reg_address, void* buffer, unsigned int bytes_to_write)
{
    // get a handle on which you want to queue the i2c communiction commands of write and reads 
    i2c_cmd_handle_t handle = i2c_cmd_link_create();

        // send start bit from master
        i2c_master_start(handle);

            // write device address on the bus
            i2c_master_write_byte(handle, device_address << 1, true);

            if(bytes_to_write > 0)
            {
                // write the address you want to write to,  last param true signifies we are checking for slave to ack on receive
                i2c_master_write_byte(handle, reg_address, true);

                // write the data,  last param true signifies we are checking for slave to ack on receive
                i2c_master_write(handle, buffer, bytes_to_write, true);
            }

        // stop bit from master
        i2c_master_stop(handle);

        // execute the commands
        esp_err_t error = i2c_master_cmd_begin(I2C_NUM_1, handle, 1000 / portTICK_PERIOD_MS);

    // delete the handle once done
    i2c_cmd_link_delete(handle);

    return error;
}

void i2c_destroy()
{
    i2c_port_t port = I2C_NUM_1;
    i2c_driver_delete(port);
}

void all_bldc_init()
{
    write_values_bldc(1000, 1000, 1000, 1000);

    vTaskDelay(4000 / portTICK_PERIOD_MS);

    write_values_bldc(0, 0, 0, 0);

    vTaskDelay(3000 / portTICK_PERIOD_MS);
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

    // set motor speed values
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (left_front  > 1000 ? 1000 : left_front ) + 1000);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, (right_front > 1000 ? 1000 : right_front) + 1000);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, (left_back   > 1000 ? 1000 : left_back  ) + 1000);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, (right_back  > 1000 ? 1000 : right_back ) + 1000);
}