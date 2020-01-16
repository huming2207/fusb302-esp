#include <hal/i2c_types.h>
#include <hal/gpio_types.h>
#include <driver/i2c.h>
#include "fusb302.h"

#define I2C_WRITE_BIT 0
#define I2C_READ_BIT 1

static void i2c_read_burst(uint8_t addr, uint8_t reg, uint8_t *rx_buf, size_t rx_len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (addr << 1U) | I2C_WRITE_BIT, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, true));

    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (addr << 1U) | I2C_READ_BIT, true));
    ESP_ERROR_CHECK(i2c_master_read(cmd, rx_buf, rx_len, I2C_MASTER_LAST_NACK));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));

    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000)));
    i2c_cmd_link_delete(cmd);
}

static void i2c_write_burst(uint8_t addr, uint8_t reg, uint8_t *tx_buf, size_t tx_len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (addr << 1U) | I2C_WRITE_BIT, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, true));
    ESP_ERROR_CHECK(i2c_master_write(cmd, tx_buf, tx_len, true));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));

    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000)));
    i2c_cmd_link_delete(cmd);
}

static inline void i2c_write(uint8_t addr, uint8_t reg, uint8_t param)
{
    i2c_write_burst(addr, reg, &param, 1);
}

static inline uint8_t i2c_read(uint8_t addr, uint8_t reg)
{
    uint8_t result = 0;
    i2c_read_burst(addr, reg, &result, 1);
    return result;
}

esp_err_t fusb302_init(int sda, int scl, int intr)
{
    i2c_config_t fusb_i2c_config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = (gpio_num_t)sda,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_io_num = (gpio_num_t)scl,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 100000,
    };

    i2c_param_config(I2C_NUM_0, &fusb_i2c_config);
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, fusb_i2c_config.mode, 0, 0, 0));

    return ESP_OK;
}


