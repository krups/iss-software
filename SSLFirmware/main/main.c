//SSL Firmware
//Firmware for all current SSL missions and devices
//Developed for RockSat-X GHOST
//Based on Matt Ruffner's KREPE 2 firmware
//Developed by Hersch Nathan, Alex Barrera


#include <config.h> 
#include "freertos/task.h" //backbone of this firmware

//if I2C is defined, include the I2C library
#ifdef I2C1 || I2C2

#endif





void app_main(void)
{

//if I2C is defined, configure the I2C pins
#ifdef I2C1
i2c_master_bus_config_t i2c1_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT, //local source
    .i2c_port = I2C_NUM_1,
    .scl_io_num = SCL1_PIN,
    .sda_io_num = SDA1_PIN,
    .glitch_ignore_cnt = 7 //defaul value
    //I2C pullup is done with hardware

};
i2c_master_bus_handle_t i2c1_handle;
ESP_ERROR_CHECK(i2c_new_master_bus(&i2c1_mst_config, &i2c1_handle));
#endif
}

