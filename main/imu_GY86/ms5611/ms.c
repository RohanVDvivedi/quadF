#include<gy86.h>
#include<math.h>

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
};

// this is where we store the configuration data of MS5611 and the digital raw values of MS5611
static volatile Barodata data;

// the current state the sensor is in
static volatile MS5611state current_state;

MS5611state get_current_ms5611_state()
{
    return current_state;
}

void baro_init()
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
    i2c_read_raw(MS5611_ADDRESS, &(data.C1_SENS_T1), 2);
    data.C1_SENS_T1 = (data.C1_SENS_T1 << 8) | ((data.C1_SENS_T1 >> 8) & 0x00ff);

    // prom read sequence
    command = 0xa0 | (0x02 << 1);
    i2c_write_raw(MS5611_ADDRESS, &command, 1);
    i2c_read_raw(MS5611_ADDRESS, &(data.C2_OFF_T1), 2);
    data.C2_OFF_T1 = (data.C2_OFF_T1 << 8) | ((data.C2_OFF_T1 >> 8) & 0x00ff);

    // prom read sequence
    command = 0xa0 | (0x03 << 1);
    i2c_write_raw(MS5611_ADDRESS, &command, 1);
    i2c_read_raw(MS5611_ADDRESS, &(data.C3_TCS), 2);
    data.C3_TCS = (data.C3_TCS << 8) | ((data.C3_TCS >> 8) & 0x00ff);

    // prom read sequence
    command = 0xa0 | (0x04 << 1);
    i2c_write_raw(MS5611_ADDRESS, &command, 1);
    i2c_read_raw(MS5611_ADDRESS, &(data.C4_TCO), 2);
    data.C4_TCO= (data.C4_TCO << 8) | ((data.C4_TCO >> 8) & 0x00ff);

    // prom read sequence
    command = 0xa0 | (0x05 << 1);
    i2c_write_raw(MS5611_ADDRESS, &command, 1);
    i2c_read_raw(MS5611_ADDRESS, &(data.C5_TREF), 2);
    data.C5_TREF = (data.C5_TREF << 8) | ((data.C5_TREF >> 8) & 0x00ff);

    // prom read sequence
    command = 0xa0 | (0x06 << 1);
    i2c_write_raw(MS5611_ADDRESS, &command, 1);
    i2c_read_raw(MS5611_ADDRESS, &(data.C6_TEMPSENS), 2);
    data.C6_TEMPSENS = (data.C6_TEMPSENS << 8) | ((data.C6_TEMPSENS >> 8) & 0x00ff);

    // ther is delay required after reading prom
    vTaskDelay(15 / portTICK_PERIOD_MS);

    current_state = INIT;
}

esp_err_t request_Barodata_abspressure()
{
    // Pressure conversion start
    uint8_t command = 0x48;
    esp_err_t err = i2c_write_raw(MS5611_ADDRESS, &(command), 1);
    current_state = REQUESTED_PRESSURE;
    return err;
}

esp_err_t request_Barodata_temperature()
{
    // Temperature conversion start
    uint8_t command = 0x58;
    esp_err_t err = i2c_write_raw(MS5611_ADDRESS, &(command), 1);
    current_state = REQUESTED_TEMPERATURE;
    return err;
}

esp_err_t get_raw_Barodata_abspressure()
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
        data.D1 = (data_read[0] << 16) | (data_read[1] << 8) | data_read[2];
    }

    current_state = READ_PRESSURE;
    return err;
}

esp_err_t get_raw_Barodata_temperature()
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
        data.D2 = (data_read[0] << 16) | (data_read[1] << 8) | data_read[2];
    }

    current_state = READ_TEMPERATURE;
    return err;
}

void scale_and_compensate_Barodata(Barodatascaled* result)
{   
    int64_t dT = ((int64_t)(data.D2)) - ( ((int64_t)(data.C5_TREF)) * (((int64_t)1) << 8) );

    int64_t TEMP = 2000 + ( dT * ((int64_t)(data.C6_TEMPSENS)) / (((int64_t)1) << 23));

    int64_t OFF = ( ((int64_t)(data.C2_OFF_T1)) * (((int64_t)1) << 16) ) + (( ((int64_t)(data.C4_TCO)) * dT ) / (((int64_t)1) << 7));

    int64_t SENS = ( ((int64_t)(data.C1_SENS_T1)) * (((int64_t)1) << 15) ) + ( ( ((int64_t)(data.C3_TCS)) * dT ) / (((int64_t)1) << 8) );

    int64_t P = ((( ((int64_t)(data.D1)) * SENS )/(((int64_t)1) << 21)) - OFF ) / ( ((int64_t)1) << 15 );

    // in degree celcius
    result->temperature = ((double)TEMP)/100;

    // in mbar
    result->abspressure = ((double)P)/100;

    // in meters above sea level
    result->altitude = ((pow( 10.0, log10(result->abspressure / 1013.25) / 5.2558797 ) - 1) * 1000000 * 0.3048)/(-6.8755856);
}

