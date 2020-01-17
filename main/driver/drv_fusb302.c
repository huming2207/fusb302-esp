#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <hal/i2c_types.h>
#include <hal/gpio_types.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include "drv_fusb302.h"

#define I2C_WRITE_BIT 0
#define I2C_READ_BIT 1

#define FUSB302_ADDR 0x22

#define TAG "fusb302_drv"

static xQueueHandle intr_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(intr_evt_queue, &gpio_num, NULL);
}

static void fusb302_intr_task(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(intr_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

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

static void i2c_write(uint8_t addr, uint8_t reg, uint8_t param)
{
    i2c_write_burst(addr, reg, &param, 1);
}

static uint8_t i2c_read(uint8_t addr, uint8_t reg)
{
    uint8_t result = 0;
    i2c_read_burst(addr, reg, &result, 1);
    return result;
}

esp_err_t fusb302_init(int sda, int scl, int intr)
{
    // Setup I2C
    i2c_config_t fusb_i2c_config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = (gpio_num_t)sda,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_io_num = (gpio_num_t)scl,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 100000,
    };

    esp_err_t err = i2c_param_config(I2C_NUM_0, &fusb_i2c_config);
    err = err ?: i2c_driver_install(I2C_NUM_0, fusb_i2c_config.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set I2C");
        return err;
    }

    // Setup INTR pin
    gpio_config_t intr_gpio_conf = {
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_POSEDGE,
            .pull_down_en = 0,
            .pull_up_en = 0,
            .pin_bit_mask = (1U << (uint32_t)intr)
    };

    gpio_config(&intr_gpio_conf);

    // Reset FUSB302 and PD
    fusb302_reset(true, true);

    // Create GPIO Interrupt queue
    intr_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    if (intr_evt_queue == NULL) {
        ESP_LOGE(TAG, "Cannot create interrupt queue");
        return ESP_ERR_NO_MEM;
    }

    // Start GPIO Interrupt task
    xTaskCreate(fusb302_intr_task, "fusb302_intr", 4096, NULL, 10, NULL);

    // Setup interrupt service
    err = gpio_install_isr_service(0);
    err = err ?: gpio_isr_handler_add(intr, gpio_isr_handler, (void*)intr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot setup GPIO interrupt services");
        return err;
    }

    // Read Device ID
    uint8_t dev_id = fusb302_get_dev_id();
    ESP_LOGI(TAG, "Init sequence started, DevID: 0x%02x", dev_id);

    // Retry 3 times, auto retry enable
    uint8_t reg_ctrl_3 = fusb302_get_ctrl_3();
    reg_ctrl_3 |= FUSB302_REG_CONTROL3_AUTO_RETRY;
    reg_ctrl_3 |= PD_RETRY_COUNT << FUSB302_REG_CONTROL3_N_RETRIES_POS;
    fusb302_set_ctrl_3(reg_ctrl_3);

    // Switches0: Enable pull-downs, monitor CC1
    fusb302_set_switch_0(FUSB302_REG_SWITCHES0_CC1_PD_EN |
                                FUSB302_REG_SWITCHES0_CC2_PD_EN |
                                FUSB302_REG_SWITCHES0_MEAS_CC1);

    // Mask interrupts
    fusb302_set_mask(0xEF); // All except CRC_CHK
    fusb302_set_mask_a(0xFF);
    fusb302_set_mask_b(FUSB302_REG_MASK_B_GCRCSENT);

    // Switches1: CC1 Tx Enable, PD 2.0, Auto CRC Enable
    fusb302_set_switch_1(FUSB302_REG_SWITCHES1_TXCC1_EN |
                            FUSB302_REG_SWITCHES1_AUTO_GCRC |
                            FUSB302_REG_SWITCHES1_SPECREV1);

    // Enable all power
    fusb302_set_power(FUSB302_REG_POWER_PWR_ALL);
    return ESP_OK;
}

uint8_t fusb302_get_dev_id()
{
    return i2c_read(FUSB302_ADDR, FUSB302_REG_DEVICE_ID);
}

void fusb302_set_switch_0(uint8_t val)
{
    i2c_write(FUSB302_ADDR, FUSB302_REG_SWITCHES0, val);
}

uint8_t fusb302_get_switch_0()
{
    return i2c_read(FUSB302_ADDR, FUSB302_REG_SWITCHES0);
}

void fusb302_set_switch_1(uint8_t val)
{
    i2c_write(FUSB302_ADDR, FUSB302_REG_SWITCHES1, val);
}

uint8_t fusb302_get_switch_1()
{
    return i2c_read(FUSB302_ADDR, FUSB302_REG_SWITCHES1);
}

void fusb302_set_measure(uint8_t val)
{
    i2c_write(FUSB302_ADDR, FUSB302_REG_MEASURE, val);
}

uint8_t fusb302_get_measure()
{
    return i2c_read(FUSB302_ADDR, FUSB302_REG_MEASURE);
}

void fusb302_set_slice(uint8_t val)
{
    i2c_write(FUSB302_ADDR, FUSB302_REG_SLICE, val);
}

uint8_t fusb302_get_slice()
{
    return i2c_read(FUSB302_ADDR, FUSB302_REG_SLICE);
}

void fusb302_set_ctrl_0(uint8_t val)
{
    i2c_write(FUSB302_ADDR, FUSB302_REG_CONTROL0, val);
}

uint8_t fusb302_get_ctrl_0()
{
    return i2c_read(FUSB302_ADDR, FUSB302_REG_CONTROL0);
}

void fusb302_set_ctrl_1(uint8_t val)
{
    i2c_write(FUSB302_ADDR, FUSB302_REG_CONTROL1, val);
}

uint8_t fusb302_get_ctrl_1()
{
    return i2c_read(FUSB302_ADDR, FUSB302_REG_CONTROL1);
}

void fusb302_set_ctrl_2(uint8_t val)
{
    i2c_write(FUSB302_ADDR, FUSB302_REG_CONTROL2, val);
}

uint8_t fusb302_get_ctrl_2()
{
    return i2c_read(FUSB302_ADDR, FUSB302_REG_CONTROL2);
}

void fusb302_set_ctrl_3(uint8_t val)
{
    i2c_write(FUSB302_ADDR, FUSB302_REG_CONTROL3, val);
}

uint8_t fusb302_get_ctrl_3()
{
    return i2c_read(FUSB302_ADDR, FUSB302_REG_CONTROL3);
}

void fusb302_set_ctrl_4(uint8_t val)
{
    i2c_write(FUSB302_ADDR, FUSB302_REG_CONTROL4, val);
}

uint8_t fusb302_get_ctrl_4()
{
    return i2c_read(FUSB302_ADDR, FUSB302_REG_CONTROL4);
}

void fusb302_set_mask(uint8_t val)
{
    i2c_write(FUSB302_ADDR, FUSB302_REG_MASK, val);
}

uint8_t fusb302_get_mask()
{
    return i2c_read(FUSB302_ADDR, FUSB302_REG_MASK);
}

void fusb302_set_power(uint8_t val)
{
    i2c_write(FUSB302_ADDR, FUSB302_REG_POWER, val);
}

uint8_t fusb302_get_power()
{
    return i2c_read(FUSB302_ADDR, FUSB302_REG_POWER);
}

void fusb302_reset(bool pd_rst, bool sw_rst)
{
    i2c_write(FUSB302_ADDR, FUSB302_REG_RESET,
                (pd_rst ? FUSB302_REG_RESET_PD_RESET : 0U) |
                (sw_rst ? FUSB302_REG_RESET_SW_RESET : 0U));
}

uint8_t fusb302_get_ocp()
{
    return i2c_read(FUSB302_ADDR, FUSB302_REG_OCP);
}

void fusb302_set_ocp(uint8_t val)
{
    i2c_write(FUSB302_ADDR, FUSB302_REG_OCP, val);
}

void fusb302_set_mask_a(uint8_t val)
{
    i2c_write(FUSB302_ADDR, FUSB302_REG_MASK_A, val);
}

uint8_t fusb302_get_mask_a()
{
    return i2c_read(FUSB302_ADDR, FUSB302_REG_MASK_A);
}

void fusb302_set_mask_b(uint8_t val)
{
    i2c_write(FUSB302_ADDR, FUSB302_REG_MASK_B, val);
}

uint8_t fusb302_get_mask_b()
{
    return i2c_read(FUSB302_ADDR, FUSB302_REG_MASK_B);
}

uint8_t fusb302_get_status_0a()
{
    return i2c_read(FUSB302_ADDR, FUSB302_REG_STATUS0A);
}

uint8_t fusb302_get_status_1a()
{
    return i2c_read(FUSB302_ADDR, FUSB302_REG_STATUS1A);
}

uint8_t fusb302_get_status_0()
{
    return i2c_read(FUSB302_ADDR, FUSB302_REG_STATUS0);
}

uint8_t fusb302_get_status_1()
{
    return i2c_read(FUSB302_ADDR, FUSB302_REG_STATUS1);
}

void fusb302_clear_interrupt_a(uint8_t val)
{
    i2c_write(FUSB302_ADDR, FUSB302_REG_INTERRUPT_A, val);
}

uint8_t fusb302_get_interrupt_a()
{
    return i2c_read(FUSB302_ADDR, FUSB302_REG_INTERRUPT_A);
}

void fusb302_clear_interrupt_b(uint8_t val)
{
    i2c_write(FUSB302_ADDR, FUSB302_REG_INTERRUPT_B, val);
}

uint8_t fusb302_get_interrupt_b()
{
    return i2c_read(FUSB302_ADDR, FUSB302_REG_INTERRUPT_B);
}

void fusb302_clear_interrupt(uint8_t val)
{
    i2c_write(FUSB302_ADDR, FUSB302_REG_INTERRUPT, val);
}

uint8_t fusb302_get_interrupt()
{
    return i2c_read(FUSB302_ADDR, FUSB302_REG_INTERRUPT);
}

void fusb302_write_fifo(uint8_t *buf, size_t len)
{
    i2c_write_burst(FUSB302_ADDR, FUSB302_REG_FIFOS, buf, len);
}

void fusb302_read_fifo(uint8_t *buf, size_t len)
{
    i2c_read_burst(FUSB302_ADDR, FUSB302_REG_FIFOS, buf, len);
}


