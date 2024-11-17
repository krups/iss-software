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
intline static bool  _i2c_cfg_equal (const i2c_config_t *a, const i2c_config_t *b)
{
    return a->mode == b->mode 
        && a->sda_io_num == b->sda_io_num 
        && a->scl_io_num == b->scl_io_num 
        && a->sda_pullup_en == b->sda_pullup_en
        && a->scl_pullup_en == b->scl_pullup_en 
        && a->master.clk_speed == b->master.clk_speed;
}
static esp_err_t _i2c_setup_port(const i2c_dev_t *dev)
{
    if (dev->port >= I2C_NUM_MAX)
    {
        ESP_LOGE(TAG, "Invalid I2C port number");
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t res;
    if(!_i2c_cfg_equal(&dev->cfg, &i2c_port_state[dev->port].config) || !i2c_port_state[dev->port].installed)
    {
        ESP_LOGE(TAG, "Reconfiguring I2C driver on port %d", dev->port);
        i2c_config_t temp;
        memcpy(&temp, &dev->cfg, sizeof(i2c_config_t));
        temp.mode = I2C_MODE_MASTER;
        

        if(i2c_port_state[dev->port].installed)
        {
            i2c_driver_delete(dev->port);
            i2c_port_state[dev->port].installed = false;
        }
        if((res = i2c_param_config(dev->port, &temp)) != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to configure I2C driver on port %d", dev->port);
            return res;
        }
        if((res = i2c_param_config(dev->port, &temp)) != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to configure I2C driver on port %d", dev->port);
            return res;
        }
        i2c_port_state[dev->port].installed = true;
        memcpy(&i2c_port_state[dev->port].config, &temp, sizeof(i2c_config_t));
        ESP_LOGD(TAG, "I2C driver on port %d successfully reconfigured ", dev->port);

        int t 
        if ((res = i2c_get_timeout(dev->port, &t)) != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to get I2C timeout on port %d", dev->port);
            return res;
        }
        uint32_t ticks = dev->timeout_ticks ? dev->timeout_ticks : I2C_DEV_MAX_STRETCH_TIME;
        if ((ticks != t) && (res = i2c_set_timeout(dev->port, ticks)) != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to set I2C timeout on port %d", dev->port);
            return res;
        }
        ESP_LOGD(TAG, "Timeout: ticks = %" PRIu32 " (%" PRIu32 " usec) on port %d", dev->timeout_ticks, dev->timeout_ticks / 80, dev->port);

        return ESP_OK;

}

esp_err_t i2c_dev_probe(i2c_dev_t *dev, i2c_dev_type_t operation_type)
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
    
    esp_err_t res; = _i2c_setup_port(dev);
    if (res == ESP_OK)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev->addr << 1) | (operation_type == I2C_DEV_READ ? 1 : 0));
        i2c_master_stop(cmd);

        res = i2x_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(CONFIG_I2C_TIMEOUT_MS));

        i2c_cmd_link_delete(cmd);
    }

    _SemaphoreGive_i2c_dev(dev->port);

    return res;
}

esp_err_t i2c_dev_read(const i2c_dev_t *dev, const void *out_data , size_t out_size, void *in_data, size_t in_size)
{
    if(!dev || !out_data || !in_data)
    {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    _SemaphoreTake_i2c_dev(dev->port);

    esp_err_t res = _i2c_setup_port(dev);
    if(res == ESP_OK)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        if (out_data && out_size)
        {
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, dev->addr << 1, true);
            i2c_master_write(cmd, (void *)out_data, out_size, true);
        }

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev->addr << 1) | 1, true);
        i2c_master_read(cmd, in_data, in_size, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);

        res = i2c_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(CONFIG_I2C_TIMEOUT_MS));
        if(res != ESP_OK)
        {
            ESP_LOGE(TAG, "Could not read from device [0x%02x at %d]: %d (%s)", dev->addr, dev->port, res, esp_err_to_name(res));
        }
        i2c_cmd_link_delete(cmd);
    }

    _SemaphoreGive_i2c_dev(dev->port);
    return res;
}

esp_err_t i2c_dev_write(const i2c_dev_t *dev, const void *out_reg, size_t out_reg_size, const void *out_data, size_t out_size)
{
    if(!dev || !out_data || !out_ize)
    {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    _SemaphoreTake_i2c_dev(dev->port);

    esp_err_t res = _i2c_setup_port(dev);
    if (res == ESP_OK)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, dev->addr << 1, true);
        if(out_reg && out_reg_size)
        {
            i2c_master_write(cmd, (void *)out_reg, out_reg_size, true);
        }
        i2c_master_write(cmd, (void *)out_data, out_size, true);
        i2c_master_stop(cmd);
        res = i2c_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(CONFIG_I2C_TIMEOUT_MS));
        if(res != ESP_OK)
        {
            ESP_LOGE(TAG, "Could not write to device [0x%02x at %d]: %d (%s)", dev->addr, dev->port, res, esp_err_to_name(res));
        }

    }
    _SemaphoreGive_i2c_dev(dev->port);
    return res;
}

esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev, uint8_t reg, void *in_data, size_t in_size)
{
    return i2c_dev_read(dev, &reg, 1, in_data, in_size);
}

esp_err_t i2c_dev_write_reg(const i2c_dev_t *dev, uint8_t reg, const void *out_data, size_t out_size)
{
    return i2c_dev_write(dev, &reg, 1, out_data, out_size);
}