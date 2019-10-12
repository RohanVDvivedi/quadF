#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

// for pwm for bldcs
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

// for channel inputs
#include "driver/timer.h"

#define BLINK_GPIO 2

#define I2C_MASTER_SCL_IO   22
#define I2C_MASTER_SDA_IO   21

// i2c addresses, for connected sensors namesly MPU6050 : accl/gyro, HMC5883L : magnetometer and MS5611 : barometer
#define MPU6050_ADDRESS     0x68
#define HMC5883_ADDRESS     0x1e
#define MS5611_ADDRESS      0x77

// motor pins
#define LEFT_FRONT_MOTOR    33
#define RIGHT_FRONT_MOTOR   32
#define LEFT_BACK_MOTOR     25
#define RIGHT_BACK_MOTOR    26

// the pins that are taking input from the channels, in CHANNEL_PINS_ARRAY[0] = 54 means channel 0 is connected to controller pin 54
#define CHANNEL_COUNT 4
#define CHANNEL_PINS_ARRAY {34,35,4,5}

typedef struct IMUdata IMUdata;
struct IMUdata
{
    int16_t acclx;
    int16_t accly;
    int16_t acclz;
    int16_t temp;
    int16_t gyrox;
    int16_t gyroy;
    int16_t gyroz;
    int16_t magnx;
    int16_t magny;
    int16_t magnz;
};

typedef struct IMUdatascaled IMUdatascaled;
struct IMUdatascaled
{
    double acclx;
    double accly;
    double acclz;
    double temp;
    double gyrox;
    double gyroy;
    double gyroz;
    double magnx;
    double magny;
    double magnz;
};

typedef struct Barodata Barodata;
struct Barodata
{
    uint16_t C1_SENS_T1;
    uint16_t C2_OFF_T1;
    uint16_t C3_TCS;
    uint16_t C4_TCO;
    uint16_t C5_TREF;
    uint16_t C6_TEMPSENS;

    uint32_t D1;
    uint32_t D2;

    int32_t dT;
    int32_t TEMP;

    int64_t OFF;
    int64_t SENS;
    int64_t P;

    double temperature;
    double abspressure;
    double altitude;
};

// input channels
void channels_init();
esp_err_t get_channel_values(uint16_t* channel_values);
void channels_destroy();

// imu data
void imu_init();
esp_err_t get_raw_IMUdata(IMUdata* data, uint8_t mpu_data, uint8_t hmc_data);
void scale_IMUdata(IMUdatascaled* result, IMUdata* data, uint8_t mpu_data, uint8_t hmc_data);

// barometer data
void baro_init(Barodata* data);
esp_err_t request_Barodata_temperature();
esp_err_t request_Barodata_abspressure();
esp_err_t get_raw_Barodata_temperature(Barodata* data);
esp_err_t get_raw_Barodata_abspressure(Barodata* data);
void scale_and_compensate_Barodata(Barodata* data);

// function to write values to bldc motors
void all_bldc_init();
void write_values_bldc(unsigned int left_front, unsigned int right_front, unsigned int left_back, unsigned int right_back);

void app_main(void)
{
    i2c_init();
    imu_init();
    Barodata bdata;baro_init(&bdata);
    //all_bldc_init();
    //channels_init();

    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    do
    {
        IMUdata data;
        IMUdatascaled datas;
        get_raw_IMUdata(&data, 1, 1);
        scale_IMUdata(&datas, &data, 1, 1);

        // small report in form of command, I am requiring delat between mpu6050 call and ms5611 sensor calls do not know why

        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        request_Barodata_abspressure();
        vTaskDelay(10 / portTICK_PERIOD_MS);
        get_raw_Barodata_abspressure(&bdata);
        request_Barodata_temperature();
        vTaskDelay(10 / portTICK_PERIOD_MS);
        get_raw_Barodata_temperature(&bdata);
        scale_and_compensate_Barodata(&bdata);

        printf("accl : \t%lf \t%lf \t%lf\n", datas.acclx, datas.accly, datas.acclz);
        printf("gyro : \t%lf \t%lf \t%lf\n", datas.gyrox, datas.gyroy, datas.gyroz);
        printf("magn : \t%lf \t%lf \t%lf\n", datas.magnx, datas.magny, datas.magnz);
        printf("temp : \t%lf\n\n", datas.temp);
        printf("altitude : \t%lf\n", bdata.altitude);
        printf("abspressure : \t%lf\n", bdata.abspressure);
        printf("temperature : \t%lf\n\n", bdata.temperature);
    }
    while(1);

    i2c_destroy();
    //channels_destroy();
}

const uint16_t          channel_arr             [CHANNEL_COUNT] = CHANNEL_PINS_ARRAY;
uint8_t                 channel_nos             [CHANNEL_COUNT];
volatile uint64_t       channel_values_up_new   [CHANNEL_COUNT];
volatile uint64_t       channel_values_raw      [CHANNEL_COUNT];

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

static IMUdatascaled offsets = {.acclx = 0, .accly = 0, .acclz = 0, .gyrox = 0, .gyroy = 0, .gyroz = 0, .temp = 0, .magnx = 0, .magny = 0, .magnz = 0};

void imu_init()
{
    uint8_t data;

    // write 0 to pwr_mgmt_1 register to wake it up
    data = 0x00;
    i2c_write(MPU6050_ADDRESS, 0x6b, &data, 1);

    // below wries will enable us to talk to HMC5883 sensor
        // turn on master of i2c, and reset accl and gyro data path and its registers
        data = 0x01;
        i2c_write(MPU6050_ADDRESS, 0x6a, &data, 1);

        // turnon i2c bypass mode
        i2c_write(MPU6050_ADDRESS, 0x37, &data, 1);
        data = data | (1 << 1);
        i2c_write(MPU6050_ADDRESS, 0x37, &data, 1);
    // yay we can now talk to magnetometer

    // acess HMC5883 magnetometer here, and configure them
    data = 0x78;
    i2c_write(HMC5883_ADDRESS, 0x01, &data, 1);

    data = 0x20;
    i2c_write(HMC5883_ADDRESS, 0x02, &data, 1);

    data = 0x00;
    i2c_write(HMC5883_ADDRESS, 0x03, &data, 1);
    // magnetometer settings done


// by uncommenting the below lines we can start reading manetometer data
/*
    // write slave address o with magnetometer's address
    data = HMC5883_ADDRESS | (1 << 7);
    i2c_write(MPU6050_ADDRESS, 0x25, &data, 1);

    // write read register number of slave 0
    data = 0x03;
    i2c_write(MPU6050_ADDRESS, 0x26, &data, 1);

    // other configurations for slave 0
    data = 0xa6;
    i2c_write(MPU6050_ADDRESS, 0x27, &data, 1);

    // turn on master of i2c, and reset accl and gyro data path and its registers
    data = 0x20;
    i2c_write(MPU6050_ADDRESS, 0x6a, &data, 1);

    // turnon i2c bypass mode
    i2c_write(MPU6050_ADDRESS, 0x37, &data, 1);
    data = data & (~(1 << 1));
    i2c_write(MPU6050_ADDRESS, 0x37, &data, 1);
*/

    for(uint16_t i = 0; i < 500; i++)
    {
        IMUdata data;
        get_raw_IMUdata(&data, 1, 1);
        offsets.acclx += (((double)(data.acclx))/500);
        offsets.accly += (((double)(data.accly))/500);
        offsets.acclz += ((((double)(data.acclz))-16384)/500);
        offsets.gyrox += (((double)(data.gyrox))/500);
        offsets.gyroy += (((double)(data.gyroy))/500);
        offsets.gyroz += (((double)(data.gyroz))/500);
        offsets.magnx += (((double)(data.magnx))/500);
        offsets.magny += (((double)(data.magny))/500);
        offsets.magnz += (((double)(data.magnz))/500);
        vTaskDelay(14 / portTICK_PERIOD_MS);
    }
}

esp_err_t get_raw_IMUdata(IMUdata* data, uint8_t mpu_data, uint8_t hmc_data)
{
    esp_err_t err = ESP_OK;

    if(mpu_data)
    {
        err = i2c_read(MPU6050_ADDRESS, 0x3b, data, 14);
        if(err != ESP_OK)
        {
            return err;
        }

        // we have to change the data from network ordr to host order integers
        data->acclx = (data->acclx << 8) | ((data->acclx >> 8) & 0x00ff);
        data->accly = (data->accly << 8) | ((data->accly >> 8) & 0x00ff);
        data->acclz = (data->acclz << 8) | ((data->acclz >> 8) & 0x00ff);
        data->temp = (data->temp << 8) | ((data->temp >> 8) & 0x00ff);
        data->gyrox = (data->gyrox << 8) | ((data->gyrox >> 8) & 0x00ff);
        data->gyroy = (data->gyroy << 8) | ((data->gyroy >> 8) & 0x00ff);
        data->gyroz = (data->gyroz << 8) | ((data->gyroz >> 8) & 0x00ff);
    }

    if(hmc_data)
    {
        err = i2c_read(HMC5883_ADDRESS, 0x03, ((void*)data) + 14, 6);
        if(err != ESP_OK)
        {
            return err;
        }

        // we have to change the data from network ordr to host order integers
        data->magnx = (data->magnx << 8) | ((data->magnx >> 8) & 0x00ff);
        data->magny = (data->magny << 8) | ((data->magny >> 8) & 0x00ff);
        data->magnz = (data->magnz << 8) | ((data->magnz >> 8) & 0x00ff);
    }

    return err;
}

void scale_IMUdata(IMUdatascaled* result, IMUdata* data, uint8_t mpu_data, uint8_t hmc_data)
{
    if(mpu_data)
    {
        // in m/s2, meter per second square
        result->acclx = ((((double)(data->acclx)) - offsets.acclx) * 19.6) / 32768.0;
        result->accly = ((((double)(data->accly)) - offsets.accly) * 19.6) / 32768.0;
        result->acclz = ((((double)(data->acclz)) - offsets.acclz) * 19.6) / 32768.0;

        // temperature is in degree celcius
        result->temp  = (((double)(data->temp)) / 340) + 36.53;

        // in dps, degrees per second
        result->gyrox = ((((double)(data->gyrox)) - offsets.gyrox) * 250.0) / 32768.0;
        result->gyroy = ((((double)(data->gyroy)) - offsets.gyroy) * 250.0) / 32768.0;
        result->gyroz = ((((double)(data->gyroz)) - offsets.gyroz) * 250.0) / 32768.0;
    }

    // in mG, milli Gauss
    if(hmc_data)
    {
        result->magnx = ((((double)(data->magnx)) - offsets.magnx) * 0.92);
        result->magny = ((((double)(data->magny)) - offsets.magny) * 0.92);
        result->magnz = ((((double)(data->magnz)) - offsets.magnz) * 0.92);
    }
}

void baro_init(Barodata* data)
{
    uint8_t command;

    // sending reset sequence for MS5611 barometer
    command = 0x1e;
    i2c_write_raw(MS5611_ADDRESS, &command, 1);

    // ther is delay required after reset to read prom
    vTaskDelay(15 / portTICK_PERIOD_MS);

    // prom read sequence
    command = 0xa0 | (0x01 << 1);
    i2c_write_raw(MS5611_ADDRESS, &command, 1);
    i2c_read_raw(MS5611_ADDRESS, &(data->C1_SENS_T1), 2);
    data->C1_SENS_T1 = (data->C1_SENS_T1 << 8) | ((data->C1_SENS_T1 >> 8) & 0x00ff);

    // prom read sequence
    command = 0xa0 | (0x02 << 1);
    i2c_write_raw(MS5611_ADDRESS, &command, 1);
    i2c_read_raw(MS5611_ADDRESS, &(data->C2_OFF_T1), 2);
    data->C2_OFF_T1 = (data->C2_OFF_T1 << 8) | ((data->C2_OFF_T1 >> 8) & 0x00ff);

    // prom read sequence
    command = 0xa0 | (0x03 << 1);
    i2c_write_raw(MS5611_ADDRESS, &command, 1);
    i2c_read_raw(MS5611_ADDRESS, &(data->C3_TCS), 2);
    data->C3_TCS = (data->C3_TCS << 8) | ((data->C3_TCS >> 8) & 0x00ff);

    // prom read sequence
    command = 0xa0 | (0x04 << 1);
    i2c_write_raw(MS5611_ADDRESS, &command, 1);
    i2c_read_raw(MS5611_ADDRESS, &(data->C4_TCO), 2);
    data->C4_TCO= (data->C4_TCO << 8) | ((data->C4_TCO >> 8) & 0x00ff);

    // prom read sequence
    command = 0xa0 | (0x05 << 1);
    i2c_write_raw(MS5611_ADDRESS, &command, 1);
    i2c_read_raw(MS5611_ADDRESS, &(data->C5_TREF), 2);
    data->C5_TREF = (data->C5_TREF << 8) | ((data->C5_TREF >> 8) & 0x00ff);

    // prom read sequence
    command = 0xa0 | (0x06 << 1);
    i2c_write_raw(MS5611_ADDRESS, &command, 1);
    i2c_read_raw(MS5611_ADDRESS, &(data->C6_TEMPSENS), 2);
    data->C6_TEMPSENS = (data->C6_TEMPSENS << 8) | ((data->C6_TEMPSENS >> 8) & 0x00ff);

    // ther is delay required after reading prom
    vTaskDelay(15 / portTICK_PERIOD_MS);
}

esp_err_t request_Barodata_abspressure()
{
    // Pressure conversion start
    uint8_t command = 0x48;
    esp_err_t err = i2c_write_raw(MS5611_ADDRESS, &(command), 1);
    return err;
}

esp_err_t request_Barodata_temperature()
{
    // Temperature conversion start
    uint8_t command = 0x58;
    esp_err_t err = i2c_write_raw(MS5611_ADDRESS, &(command), 1);
    return err;
}

esp_err_t get_raw_Barodata_abspressure(Barodata* data)
{
    // sensor data read command
    uint8_t command = 0x00;
    esp_err_t err;
    err = i2c_write_raw(MS5611_ADDRESS, &(command), 1);

    if(err == ESP_OK)
    {
        // read and place it in pressure D1
        uint8_t data_read[3];
        err = i2c_read_raw(MS5611_ADDRESS, data_read, 3);
        data->D1 = (data_read[0] << 16) | (data_read[1] << 8) | data_read[0];
    }
    return err;
}

esp_err_t get_raw_Barodata_temperature(Barodata* data)
{
    // sensor data read command
    uint8_t command = 0x00;
    esp_err_t err;
    err = i2c_write_raw(MS5611_ADDRESS, &(command), 1);

    if(err == ESP_OK)
    {
        // read and place it in temperature D2
        uint8_t data_read[3];
        err = i2c_read_raw(MS5611_ADDRESS, data_read, 3);
        data->D2 = (data_read[0] << 16) | (data_read[1] << 8) | data_read[0];
    }
    return err;
}

void scale_and_compensate_Barodata(Barodata* data)
{   
    data->dT = ((int64_t)(data->D2)) - ( ((int64_t)(data->C5_TREF)) * (((int64_t)1) << 8) );

    data->TEMP = 2000 + ( ((int64_t)(data->dT)) * ((int64_t)(data->C6_TEMPSENS)) / (((int64_t)1) << 23));

    data->OFF = ( ((int64_t)(data->C2_OFF_T1)) * (((int64_t)1) << 16) ) + (( ((int64_t)(data->C4_TCO)) * ((int64_t)(data->dT)) ) / (((int64_t)1) << 7));

    data->SENS = ( ((int64_t)(data->C1_SENS_T1)) * (((int64_t)1) << 15) ) + ( ( ((int64_t)(data->C3_TCS)) * ((int64_t)(data->dT)) ) / (((int64_t)1) << 8) );

    data->P = ((( ((int64_t)(data->D1)) * ((int64_t)(data->SENS)) )/(((int64_t)1) << 21)) - ((int64_t)(data->OFF)) ) / ( ((int64_t)1) << 15 );

    // in degree celcius
    data->temperature = ((double)(data->TEMP))/100;

    // in mbar
    data->abspressure = ((double)(data->P))/100;

    // in meters above sea level
    data->altitude = ((pow( 10.0, log(data->abspressure / 1013.25) / 5.2558797 ) - 1) * 1000000 * 0.3048)/(-6.8755856);
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