#include<i2c_comm.h>

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

esp_err_t i2c_read_raw(uint8_t device_address, void* buffer, unsigned int bytes_to_read)
{
    // get a handle on which you want to queue the i2c communiction commands of write and reads
    i2c_cmd_handle_t handle = i2c_cmd_link_create();

        // send start bit from master
        i2c_master_start(handle);

            // write address of the device on the bus, last param true signifies we are checking for slave to ack on receive
            i2c_master_write_byte(handle, (device_address << 1) | 1, true);

            if(bytes_to_read > 0)
            {
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

esp_err_t i2c_write_raw(uint8_t device_address, void* buffer, unsigned int bytes_to_write)
{
    // get a handle on which you want to queue the i2c communiction commands of write and reads 
    i2c_cmd_handle_t handle = i2c_cmd_link_create();

        // send start bit from master
        i2c_master_start(handle);

            // write device address on the bus
            i2c_master_write_byte(handle, device_address << 1, true);

            if(bytes_to_write > 0)
            {
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

