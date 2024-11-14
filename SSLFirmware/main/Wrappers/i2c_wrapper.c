//i2c_wrapper.c
//wraps i2c for use with freeRTOS
//Developed for RockSat-X GHOST
//Based off of UncleRus ESP32 I2C library
//Developed by Hersch Nathan


#include "i2c_wrapper.h"


//array of struct holding configuration for each i2c port
static i2c_port_state_t i2c_port_state[I2C_NUM_MAX] = {0};

static const char *TAG = "i2c_wrapper";

esp_err_t _SemaphoreTake(int port)
{
    if(!xSemaphoreTake(i2c_port_state[port].lock, pdMS_TO_TICKS(CONFIG_I2C_TIMEOUT_MS)))
    {
        ESP_LOGE(TAG, "Failed to take I2C port %d mutex", port);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t i2c_dev_init()
{
    memset(i2c_port_state, 0, sizeof(i2c_port_state));
    
    for (int i = 0; i < I2C_NUM_MAX; i++)
    {
        i2c_port_state[i].lock = xSemaphoreCreateMutex();
        if (!i2c_port_state[i].lock)
        {
            ESP_LOGE(TAG, "Failed to create I2C port %d mutex", i);
            return ESP_FAIL;
        }
    }
    
    return ESP_OK;
}

esp_err_t i2c_dev_done()
{
    for (int i = 0; i < I2C_NUM_MAX; i++)
    {
        if (!i2c_port_state[i].lock) 
        {
            continue;
        }
        if (i2c_port_state[i].installed)
        {
            ESP_LOGE(TAG, "I2C port %d is still installed", i);
            return ESP_FAIL;
        }
    }
    
    for (int i = 0; i < I2C_NUM_MAX; i++)
    {
        vSemaphoreDelete(i2c_port_state[i].lock);
    }
    
    return ESP_OK;
}
