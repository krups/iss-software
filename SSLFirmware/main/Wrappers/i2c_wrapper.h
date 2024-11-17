//i2c_wrapper.h
//wraps i2c for use with freeRTOS
//Developed for RockSat-X GHOST
//Based off of UncleRus ESP32 I2C library
//Developed by Hersch Nathan


#ifndef I2C_WRAPPER_H
#define I2C_WRAPPER_H

#include <stdint.h>
#include <stddef.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"

//defines, constants
#define CONFIG_I2CDEV_TIMEOUT 1000
#define I2C_DEV_MAX_STRETCH_TIME 0x00ffffff 

// I2C device descriptor
typedef struct {
    i2c_port_t port;         // I2C port number
    i2c_config_t cfg;        // I2C driver configuration
    uint8_t addr;            // Unshifted address
    SemaphoreHandle_t mutex; // Device mutex
    uint32_t timeout_ticks;  /*!< HW I2C bus timeout (stretch time), in ticks. When this value is 0, I2CDEV_MAX_STRETCH_TIME will be used */
} i2c_dev_t;

// I2C transaction type
typedef enum {
    I2C_DEV_WRITE = 0, // Write operation, default
    I2C_DEV_READ       // Read operation
} i2c_dev_type_t;

// I2C port state
typedef struct {
    SemaphoreHandle_t lock;
    i2c_config_t config;
    bool installed;
} i2c_port_state_t;

// I2C device configuration
esp_err_t i2c_dev_init();
esp_err_t i2c_dev_done();

// Mutex functions
esp_err_t i2c_dev_create_mutex(i2c_dev_t *dev);
esp_err_t i2c_dev_delete_mutex(i2c_dev_t *dev);
esp_err_t i2c_dev_take_mutex(i2c_dev_t *dev);
esp_err_t i2c_dev_give_mutex(i2c_dev_t *dev);

// I2C functions
esp_err_t i2c_dev_probe(i2c_dev_t *dev, i2c_dev_type_t operation_type);
esp_err_t i2c_dev_read(const i2c_dev_t *dev, const void *out_data, size_t out_size, void *in_data, size_t in_size);
esp_err_t i2c_dev_write(const i2c_dev_t *dev, const void *out_data, size_t out_size);
esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev, uint8_t reg, void *in_data, size_t in_size);
esp_err_t i2c_dev_write_reg(const i2c_dev_t *dev, uint8_t reg, const void *out_data, size_t out_size);

#endif // I2C_WRAPPER_H