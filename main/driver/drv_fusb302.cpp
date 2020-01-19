#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <hal/i2c_types.h>
#include <hal/gpio_types.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include "drv_fusb302.hpp"
#include "tcpc_drv.hpp"

#define I2C_WRITE_BIT 0U
#define I2C_READ_BIT 1U

#define FUSB302_ADDR 0x22U

#define TAG "fusb302_drv"

using namespace drv;

QueueHandle_t fusb302::intr_evt_queue = nullptr;

void IRAM_ATTR fusb302::gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = *(uint32_t *)arg;
    xQueueSendFromISR(intr_evt_queue, &gpio_num, nullptr);
}

void fusb302::write_reg(uint8_t reg, uint8_t param)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (uint8_t)(FUSB302_ADDR << 1U) | I2C_WRITE_BIT, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, param, true));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));

    ESP_ERROR_CHECK(i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000)));
    i2c_cmd_link_delete(cmd);
}

uint8_t fusb302::read_reg(uint8_t reg)
{
    uint8_t result = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (uint8_t)(FUSB302_ADDR << 1U) | I2C_WRITE_BIT, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, true));

    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (uint8_t)(FUSB302_ADDR << 1U) | I2C_READ_BIT, true));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &result, I2C_MASTER_LAST_NACK));

    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000)));
    i2c_cmd_link_delete(cmd);
    return result;
}

esp_err_t fusb302::transmit_pkt(uint16_t header, const uint32_t *data_objs, size_t obj_cnt)
{
    if (data_objs == nullptr || obj_cnt < 0) return ESP_ERR_INVALID_ARG;

    // Step 1. Flush Tx FIFO
    set_ctrl_0(FUSB302_REG_CONTROL0_TX_FLUSH);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (uint8_t)(FUSB302_ADDR << 1U) | I2C_WRITE_BIT, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, FUSB302_REG_FIFOS, true));

    // Step 2. Send SOP tokens
    uint8_t sop_token[4] = { fusb302_token::SYNC1, fusb302_token::SYNC1, fusb302_token::SYNC1, fusb302_token::SYNC2 };
    ESP_ERROR_CHECK(i2c_master_write(cmd, sop_token, sizeof(sop_token), true));

    // Step 3. Send packet header
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (uint8_t)(header & 0xffU), true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (uint8_t)(header >> 8U), true));

    // Step 4. Send data objects
    for (size_t idx = 0; idx < obj_cnt; idx ++) {
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (uint8_t)(data_objs[idx] & 0xffU), true));
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (uint8_t)(data_objs[idx] >> 8U), true));
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (uint8_t)(data_objs[idx] >> 16U), true));
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (uint8_t)(data_objs[idx] >> 24U), true));
    }

    // Step 5. Send CRC, End of Packet and TX_OFF tokens, with TX_ON token appended
    uint8_t post_token[4] = { fusb302_token::JAMCRC, fusb302_token::EOP, fusb302_token::TXOFF, fusb302_token::TXON };
    ESP_ERROR_CHECK(i2c_master_write(cmd, post_token, sizeof(post_token), true));

    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000)));
    i2c_cmd_link_delete(cmd);
    return ESP_OK;
}


/**
 * Read FIFO
 * @param drv_handle[in]
 * @param header[out] PD Header (2 bytes, 16 bits)
 * @param data_objs[out] Data objects (4 bytes, 32 bits for each)
 * @param max_cnt Maxmium data objects can be received
 * @param actual_cnt Actual received data objects
 * @return
 *
 */
esp_err_t fusb302::receive_pkt(uint16_t *header, uint32_t *data_objs, size_t max_cnt, size_t *actual_cnt)
{
    if (header == nullptr || data_objs == nullptr || actual_cnt == nullptr) return ESP_ERR_INVALID_ARG;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (FUSB302_ADDR << 1U) | I2C_WRITE_BIT, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, FUSB302_REG_FIFOS, true));

    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (FUSB302_ADDR << 1U) | I2C_READ_BIT, true));

    // Step 1. Read first token byte
    uint8_t token_byte = 0;
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &token_byte, I2C_MASTER_ACK));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000)));

    // Step 2. Read header, 2 bytes
    uint8_t header_bytes[2] = { 0 };
    ESP_ERROR_CHECK(i2c_master_read(cmd, header_bytes, 2, I2C_MASTER_ACK));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000)));
    *header = (uint16_t)(header_bytes[1] << 8U) | header_bytes[0];

    // Step 3. Calculate packet length
    uint8_t data_obj_cnt = TCPC_PD_HEADER_DATA_OBJ_CNT(*header);
    *actual_cnt = data_obj_cnt;

    // Step 4. Read data object (if it is not a control message)
    if (data_obj_cnt > 0) {
        if (data_obj_cnt > max_cnt) {
            ESP_LOGE(TAG, "Buffer is too small: got %u DO's, but max space is %u", data_obj_cnt, max_cnt);
            return ESP_ERR_NO_MEM;
        }

        uint8_t data_obj_bytes[28] = { 0 }; // Maximum data objects is 7, so 7 * 4 = 28 bytes
        ESP_ERROR_CHECK(i2c_master_read(cmd, data_obj_bytes, sizeof(data_obj_bytes), I2C_MASTER_ACK));
        ESP_ERROR_CHECK(i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000)));

        // Copy to output buffer
        for (size_t idx = 0; idx < data_obj_cnt * 4; idx += 4) {
            data_objs[idx] = (uint32_t)(data_obj_bytes[idx + 3] << 24U) |
                            (uint32_t)(data_obj_bytes[idx + 2] << 16U) |
                            (uint32_t)(data_obj_bytes[idx + 1] << 8U)  | data_obj_bytes[idx];
        }
    }

    // Step 5. Read CRC-32
    uint8_t checksum_bytes[4] = { 0 };
    ESP_ERROR_CHECK(i2c_master_read(cmd, checksum_bytes, sizeof(checksum_bytes), I2C_MASTER_LAST_NACK));

    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000)));
    i2c_cmd_link_delete(cmd);

    // Step 6: Flush FIFO
    set_ctrl_1(FUSB302_REG_CONTROL1_RX_FLUSH);



    return ESP_OK;
}

void fusb302::intr_task(void* arg)
{
    gpio_num_t io_num;
    auto *ctx = static_cast<fusb302 *>(arg);
    for(;;) {
        if(xQueueReceive(intr_evt_queue, &io_num, portMAX_DELAY)) {

        }
    }
}

fusb302::fusb302(int sda, int scl, int intr, i2c_port_t port)
{
    // Setup I2C
    i2c_port = port;
    i2c_config_t fusb_i2c_config = {};
    fusb_i2c_config.mode = I2C_MODE_MASTER;
    fusb_i2c_config.sda_io_num = (gpio_num_t)sda;
    fusb_i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    fusb_i2c_config.scl_io_num = (gpio_num_t)scl;
    fusb_i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    fusb_i2c_config.master.clk_speed = 100000;

    esp_err_t err = i2c_param_config(i2c_port, &fusb_i2c_config);
    err = err ?: i2c_driver_install(i2c_port, fusb_i2c_config.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set I2C");
    }

    // Setup INTR pin
    gpio_config_t intr_gpio_conf = {};

    intr_gpio_conf.mode = GPIO_MODE_INPUT;
    intr_gpio_conf.intr_type = GPIO_INTR_POSEDGE;
    intr_gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    intr_gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    intr_gpio_conf.pin_bit_mask = (1U << (uint32_t)intr);

    gpio_config(&intr_gpio_conf);

    // Reset FUSB302 and PD
    reset(true, true);

    // Create GPIO Interrupt queue
    intr_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    if (intr_evt_queue == nullptr) {
        ESP_LOGE(TAG, "Cannot create interrupt queue");
    }

    // Start GPIO Interrupt task
    xTaskCreate(intr_task, "fusb302_intr", 4096, this, 10, nullptr);

    // Setup interrupt service
    err = gpio_install_isr_service(0);
    err = err ?: gpio_isr_handler_add((gpio_num_t)intr, gpio_isr_handler, (void*)intr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot setup GPIO interrupt services");
    }

    // Read Device ID
    uint8_t dev_id = get_dev_id();
    ESP_LOGI(TAG, "Init sequence started, DevID: 0x%02x", dev_id);

    // Retry 3 times, auto retry enable
    uint8_t reg_ctrl_3 = get_ctrl_3();
    reg_ctrl_3 |= FUSB302_REG_CONTROL3_AUTO_RETRY;
    reg_ctrl_3 |= PD_RETRY_COUNT << FUSB302_REG_CONTROL3_N_RETRIES_POS;
    set_ctrl_3(reg_ctrl_3);

    // Switches0: Enable pull-downs, monitor CC1
    set_switch_0(FUSB302_REG_SWITCHES0_CC1_PD_EN |
                                FUSB302_REG_SWITCHES0_CC2_PD_EN |
                                FUSB302_REG_SWITCHES0_MEAS_CC1);

    // Mask interrupts
    set_mask(0xEF); // All except CRC_CHK
    set_mask_a(0xFF);
    set_mask_b(FUSB302_REG_MASK_B_GCRCSENT);

    // Switches1: CC1 Tx Enable, PD 2.0, Auto CRC Enable
    set_switch_1(FUSB302_REG_SWITCHES1_TXCC1_EN |
                            FUSB302_REG_SWITCHES1_AUTO_GCRC |
                            FUSB302_REG_SWITCHES1_SPECREV1);

    // Enable all power
    set_power(FUSB302_REG_POWER_PWR_ALL);
}

void fusb302::on_pkt_received(const tcpc_def::rx_cb_t &func)
{
    rx_cb = func;
}

bool fusb302::detect_vbus()
{
    return false;
}

esp_err_t fusb302::set_rp(tcpc_def::rp_t rp)
{
    return 0;
}

esp_err_t fusb302::set_cc(tcpc_def::cc_pull_t pull)
{
    return 0;
}

esp_err_t fusb302::get_cc(tcpc_def::cc_status_t *status_cc1, tcpc_def::cc_status_t *status_cc2)
{
    return 0;
}

uint8_t fusb302::get_dev_id()
{
    return read_reg(FUSB302_REG_DEVICE_ID);
}

void fusb302::set_switch_0(uint8_t val)
{
    write_reg(FUSB302_REG_SWITCHES0, val);
}

uint8_t fusb302::get_switch_0()
{
    return read_reg(FUSB302_REG_SWITCHES0);
}

void fusb302::set_switch_1(uint8_t val)
{
    write_reg(FUSB302_REG_SWITCHES1, val);
}

uint8_t fusb302::get_switch_1()
{
    return read_reg(FUSB302_REG_SWITCHES1);
}

void fusb302::set_measure(uint8_t val)
{
    write_reg(FUSB302_REG_MEASURE, val);
}

uint8_t fusb302::get_measure()
{
    return read_reg(FUSB302_REG_MEASURE);
}

void fusb302::set_slice(uint8_t val)
{
    write_reg(FUSB302_REG_SLICE, val);
}

uint8_t fusb302::get_slice()
{
    return read_reg(FUSB302_REG_SLICE);
}

void fusb302::set_ctrl_0(uint8_t val)
{
    write_reg(FUSB302_REG_CONTROL0, val);
}

uint8_t fusb302::get_ctrl_0()
{
    return read_reg(FUSB302_REG_CONTROL0);
}

void fusb302::set_ctrl_1(uint8_t val)
{
    write_reg(FUSB302_REG_CONTROL1, val);
}

uint8_t fusb302::get_ctrl_1()
{
    return read_reg(FUSB302_REG_CONTROL1);
}

void fusb302::set_ctrl_2(uint8_t val)
{
    write_reg(FUSB302_REG_CONTROL2, val);
}

uint8_t fusb302::get_ctrl_2()
{
    return read_reg(FUSB302_REG_CONTROL2);
}

void fusb302::set_ctrl_3(uint8_t val)
{
    write_reg(FUSB302_REG_CONTROL3, val);
}

uint8_t fusb302::get_ctrl_3()
{
    return read_reg(FUSB302_REG_CONTROL3);
}

void fusb302::set_ctrl_4(uint8_t val)
{
    write_reg(FUSB302_REG_CONTROL4, val);
}

uint8_t fusb302::get_ctrl_4()
{
    return read_reg(FUSB302_REG_CONTROL4);
}

void fusb302::set_mask(uint8_t val)
{
    write_reg(FUSB302_REG_MASK, val);
}

uint8_t fusb302::get_mask()
{
    return read_reg(FUSB302_REG_MASK);
}

void fusb302::set_power(uint8_t val)
{
    write_reg(FUSB302_REG_POWER, val);
}

uint8_t fusb302::get_power()
{
    return read_reg(FUSB302_REG_POWER);
}

void fusb302::reset(bool pd_rst, bool sw_rst)
{
    write_reg(FUSB302_REG_RESET,
                      (pd_rst ? FUSB302_REG_RESET_PD_RESET : 0U) |
                      (sw_rst ? FUSB302_REG_RESET_SW_RESET : 0U));
}

uint8_t fusb302::get_ocp()
{
    return read_reg(FUSB302_REG_OCP);
}

void fusb302::set_ocp(uint8_t val)
{
    write_reg(FUSB302_REG_OCP, val);
}

void fusb302::set_mask_a(uint8_t val)
{
    write_reg(FUSB302_REG_MASK_A, val);
}

uint8_t fusb302::get_mask_a()
{
    return read_reg(FUSB302_REG_MASK_A);
}

void fusb302::set_mask_b(uint8_t val)
{
    write_reg(FUSB302_REG_MASK_B, val);
}

uint8_t fusb302::get_mask_b()
{
    return read_reg(FUSB302_REG_MASK_B);
}

uint8_t fusb302::get_status_0a()
{
    return read_reg(FUSB302_REG_STATUS0A);
}

uint8_t fusb302::get_status_1a()
{
    return read_reg(FUSB302_REG_STATUS1A);
}

uint8_t fusb302::get_status_0()
{
    return read_reg(FUSB302_REG_STATUS0);
}

uint8_t fusb302::get_status_1()
{
    return read_reg(FUSB302_REG_STATUS1);
}

void fusb302::clear_interrupt_a(uint8_t val)
{
    write_reg(FUSB302_REG_INTERRUPT_A, val);
}

uint8_t fusb302::get_interrupt_a()
{
    return read_reg(FUSB302_REG_INTERRUPT_A);
}

void fusb302::clear_interrupt_b(uint8_t val)
{
    write_reg(FUSB302_REG_INTERRUPT_B, val);
}

uint8_t fusb302::get_interrupt_b()
{
    return read_reg(FUSB302_REG_INTERRUPT_B);
}

void fusb302::clear_interrupt(uint8_t val)
{
    write_reg(FUSB302_REG_INTERRUPT, val);
}

uint8_t fusb302::get_interrupt()
{
    return read_reg(FUSB302_REG_INTERRUPT);
}


