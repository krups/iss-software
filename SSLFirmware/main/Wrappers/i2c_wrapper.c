//i2c_wrapper.c
//wraps i2c for use with freeRTOS
//Developed for RockSat-X GHOST
//Based off of UncleRus ESP32 I2C library
//Developed by Hersch Nathan


#include "i2c_wrapper.h"


//array of struct holding configuration for each i2c port
static i2c_port_state_t i2c_port_state[I2C_NUM_MAX] = {0};

static const char *TAG = "i2c_wrapper";

//private functions
esp_err_t _SemaphoreTake_i2c_dev(int port)
{
    if(!xSemaphoreTake(i2c_port_state[port].lock, pdMS_TO_TICKS(CONFIG_I2C_TIMEOUT_MS)))
    {
        ESP_LOGE(TAG, "Failed to take I2C port %d mutex", port);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t _SemaphoreGive_i2c_dev(int port)
{
    if(!xSemaphoreGive(i2c_port_state[port].lock))
    {
        ESP_LOGE(TAG, "Failed to give I2C port %d mutex", port);
        return ESP_FAIL;
    }
    return ESP_OK;
}

//I2C device configuration
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
            _SemaphoreTake_i2c_dev(i)
            i2c_driver_delete(i);
            i2c_port_state[i].installed = false;
            _SemaphoreGive_i2c_dev(i)
        }

        vSemaphoreDelete(i2c_port_state[i].lock);
    }
    
   
    
    return ESP_OK;
}

//Mutex functions

esp_err_t i2c_dev_create_mutex(i2c_dev_t *dev)
{
    if (!dev)
    {
        ESP_LOGE(TAG, "I2C device descriptor is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (dev->mutex)
    {
        ESP_LOGE(TAG, "I2C device mutex already created");
        return ESP_FAIL;
    }

    ESP_LOGV(TAG, "[0x%02x at %d] Creating I2C device mutex", dev->addr, dev->port);

    dev->mutex = xSemaphoreCreateMutex();
    if (!dev->mutex)
    {
        ESP_LOGE(TAG, "Failed to create I2C device mutex");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t i2c_dev_delete_mutex(i2c_dev_t *dev)
{
    if (!dev)
    {
        ESP_LOGE(TAG, "I2C device descriptor is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->mutex)
    {
        ESP_LOGE(TAG, "I2C device mutex is not created");
        return ESP_FAIL;
    }

    ESP_LOGV(TAG, "[0x%02x at %d] Deleting I2C device mutex", dev->addr, dev->port);

    vSemaphoreDelete(dev->mutex);
    dev->mutex = NULL;
    
    return ESP_OK;

    esp_err_t i2c_dev_take_mutex(i2c_dev_t *dev)
    {
        if (!dev)
        {
            ESP_LOGE(TAG, "I2C device descriptor is NULL");
            return ESP_ERR_INVALID_ARG;
        }

        if (!dev->mutex)
        {
            ESP_LOGE(TAG, "I2C device mutex is not created");
            return ESP_FAIL;
        }

        ESP_LOGV(TAG, "[0x%02x at %d] taking mutex", dev->addr, dev->port);

        if (!xSemaphoreTake(dev->mutex, pdMS_TO_TICKS(CONFIG_I2C_TIMEOUT_MS)))
        {
            ESP_LOGE(TAG, "[0x%02x at %d] Failed to take I2C device mutex", dev->addr, dev->port);
            return ESP_FAIL;
        }
        
        return ESP_OK;
    }
}

esp_err_t i2c_dev_give_mutex(i2c_dev_t *dev)
{
    if (!dev)
    {
        ESP_LOGE(TAG, "I2C device descriptor is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->mutex)
    {
        ESP_LOGE(TAG, "I2C device mutex is not created");
        return ESP_FAIL;
    }

    ESP_LOGV(TAG, "[0x%02x at %d] giving mutex", dev->addr, dev->port);


    if (!xSemaphoreGive(dev->mutex))
    {
        ESP_LOGE(TAG, "[0x%02x at %d] Failed to give I2C device mutex", dev->addr, dev->port);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

//I2C functions

