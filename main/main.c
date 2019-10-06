#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#define BLINK_GPIO 2

#define I2C_MASTER_SCL_IO   22
#define I2C_MASTER_SDA_IO   21

// i2c addresses, for connected sensors namesly MPU6050 : accl/gyro, HMC5883L : magnetometer and MS5611 : barometer
#define MPU6050_ADDRESS     0x68
#define HMC5883L_ADDRESS    0x1e
#define MS5611_ADDRESS      0x76

// motor pins
#define LEFT_FRONT_MOTOR    33
#define RIGHT_FRONT_MOTOR   32
#define LEFT_BACK_MOTOR     25
#define RIGHT_BACK_MOTOR    26

void i2c_init();
void i2c_read(uint8_t device_address, uint8_t reg_address, void* buffer, unsigned int bytes_to_read);
void i2c_write(uint8_t device_address, uint8_t reg_address, void* buffer, unsigned int bytes_to_write);
void i2c_destroy();
void write_values_bldc(unsigned int left_front, unsigned int right_front, unsigned int left_back, unsigned int right_back);

void app_main(void)
{
    /*
    // motor startup and max min configuring 
    // this is the first call hence the motors will be initalized also to their max values
    write_values_bldc(1000, 1000, 1000, 1000);
    vTaskDelay(4000 / portTICK_PERIOD_MS);
    write_values_bldc(0, 0, 0, 0);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    */

    i2c_init();

    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while(1)
    {
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        int16_t accz;
        i2c_read(MPU6050_ADDRESS, 0x3f, &accz, 2);

        printf("accz = %d\n", accz);

    }

    i2c_destroy();
}

void i2c_init()
{
    i2c_port_t port = I2C_NUM_0;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 1000000;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(port, conf.mode, 0, 0, 0);
}

void i2c_read(uint8_t device_address, uint8_t reg_address, void* buffer, unsigned int bytes_to_read)
{
    // get a handle on which you want to queue the i2c communiction commands of write and reads
    i2c_cmd_handle_t handle i2c_cmd_link_create();

        // send start bit from master
        i2c_master_start(handle);

            // write address of the device on the bus, last param true signifies we are checking for slave to ack on receive
            i2c_master_write_byte(cmd, (device_address << 1) | 1, true);

            // write address from where you want to start reading, last param true signifies we are checking for slave to ack on receive
            i2c_master_write_byte(cmd, reg_address,true);

            // read data from there, but send NACK at the last byte read, instead of ACK
            i2c_master_read(cmd, buffer, bytes_to_read, I2C_MASTER_LAST_NACK);

        // stop bit from master
        i2c_master_stop(handle);

        // execute the commands
        i2c_master_cmd_begin(I2C_NUM_0, handle, );

    // delete the handle once done
    i2c_cmd_link_delete(handle);
}

void i2c_write(uint8_t device_address, uint8_t reg_address, void* buffer, unsigned int bytes_to_write)
{
    // get a handle on which you want to queue the i2c communiction commands of write and reads 
    i2c_cmd_handle_t handle i2c_cmd_link_create();

        // send start bit from master
        i2c_master_start(handle);

            // write device address on the bus
            i2c_master_write_byte(cmd, device_address << 1, ACK);

            // write the address you want to write to,  last param true signifies we are checking for slave to ack on receive
            i2c_master_write_byte(cmd, reg_address, true);

            // write the data,  last param true signifies we are checking for slave to ack on receive
            i2c_master_write(handle, buffer, bytes_to_write, true);

        // stop bit from master
        i2c_master_stop(handle);

        // execute the commands
        i2c_master_cmd_begin(I2C_NUM_0, handle, );

    // delete the handle once done
    i2c_cmd_link_delete(handle);
}

void i2c_destroy()
{
    i2c_port_t port = I2C_NUM_0;
    i2c_driver_destroy(port, I2C_MODE_MASTER, 0, 0, 0);
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



    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (left_front  > 1000 ? 1000 : left_front ) + 1000);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, (right_front > 1000 ? 1000 : right_front) + 1000);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, (left_back   > 1000 ? 1000 : left_back  ) + 1000);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, (right_back  > 1000 ? 1000 : right_back ) + 1000);
}