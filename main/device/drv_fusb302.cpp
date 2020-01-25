#include <hal/i2c_types.h>
#include <hal/gpio_types.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include "drv_fusb302.hpp"
#include "tcpc_drv.hpp"

#define I2C_WRITE_BIT 0U
#define I2C_READ_BIT 1U

#define FUSB302_ADDR 0x22U
#define FUSB302_INTR_GENERAL_BIT  (1U << 0U)
#define FUSB302_INTR_TX_DONE_BIT  (1U << 1U)
#define FUSB302_INTR_TX_FAIL_BIT  (1U << 2U)

#define TAG "fusb302_drv"

using namespace device;

EventGroupHandle_t fusb302::intr_evt_group = nullptr;

void IRAM_ATTR fusb302::gpio_isr_handler(void* arg)
{
    static BaseType_t taskStatus = pdFALSE;
    xEventGroupSetBitsFromISR(intr_evt_group, FUSB302_INTR_GENERAL_BIT, &taskStatus);
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
    if (data_objs == nullptr && obj_cnt > 0) return ESP_ERR_INVALID_ARG;

    // Step 1. Flush Tx FIFO
    set_ctrl_0(FUSB302_REG_CONTROL0_TX_FLUSH);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (uint8_t)(FUSB302_ADDR << 1U) | I2C_WRITE_BIT, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, FUSB302_REG_FIFOS, true));

    // Step 2. Send SOP tokens
    uint8_t sop_token[4] = {fusb302_token::SYNC1, fusb302_token::SYNC1, fusb302_token::SYNC1, fusb302_token::SYNC2 };
    ESP_ERROR_CHECK(i2c_master_write(cmd, sop_token, sizeof(sop_token), true));

    // Step 3. Send packet header
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (uint8_t)(header & 0xffU), true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (uint8_t)(header >> 8U), true));

    // Step 4. Send data objects (if there is any)
    if (data_objs != nullptr && obj_cnt > 0) {
        for (size_t idx = 0; idx < obj_cnt; idx ++) {
            ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (uint8_t)(data_objs[idx] & 0xffU), true));
            ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (uint8_t)(data_objs[idx] >> 8U), true));
            ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (uint8_t)(data_objs[idx] >> 16U), true));
            ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (uint8_t)(data_objs[idx] >> 24U), true));
        }
    }

    // Step 5. Send CRC, End of Packet and TX_OFF tokens, with TX_ON token appended
    uint8_t post_token[4] = {fusb302_token::JAMCRC, fusb302_token::EOP, fusb302_token::TXOFF, fusb302_token::TXON };
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
    if (header == nullptr || ((data_objs == nullptr || actual_cnt == nullptr) && max_cnt > 0 ))
        return ESP_ERR_INVALID_ARG;

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
    i2c_cmd_link_delete(cmd);

    // Step 2. Read header, 2 bytes
    cmd = i2c_cmd_link_create();
    uint8_t header_bytes[2] = { 0 };
    ESP_ERROR_CHECK(i2c_master_read(cmd, header_bytes, 2, I2C_MASTER_ACK));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000)));
    *header = (uint16_t)(header_bytes[1] << 8U) | header_bytes[0];
    i2c_cmd_link_delete(cmd);

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
        cmd = i2c_cmd_link_create();
        ESP_ERROR_CHECK(i2c_master_read(cmd, data_obj_bytes, sizeof(data_obj_bytes), I2C_MASTER_ACK));
        ESP_ERROR_CHECK(i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000)));
        i2c_cmd_link_delete(cmd);

        // Copy to output buffer
        for (size_t idx = 0; idx < data_obj_cnt * 4; idx += 4) {
            data_objs[idx] = (uint32_t)(data_obj_bytes[idx + 3] << 24U) |
                            (uint32_t)(data_obj_bytes[idx + 2] << 16U) |
                            (uint32_t)(data_obj_bytes[idx + 1] << 8U)  | data_obj_bytes[idx];
        }
    }

    // Step 5. Read CRC-32
    uint8_t checksum_bytes[4] = { 0 };
    cmd = i2c_cmd_link_create();
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
    ESP_LOGD(TAG, "Interrupt task started");
    auto *ctx = static_cast<fusb302 *>(arg);
    while(true) {
        // Clear on exit, wait only one bit
        xEventGroupWaitBits(intr_evt_group, FUSB302_INTR_GENERAL_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

        if (ctx == nullptr) {
            ESP_LOGE(TAG, "Interrupt triggered but ctx is invalid");
            return;
        }

        ESP_LOGD(TAG, "Interrupt triggered, talking on I2C num: ");
        ctx->clear_irq();
        if ((ctx->interrupt_reg & FUSB302_REG_INTERRUPT_VBUSOK) > 0) {
            ESP_LOGI(TAG, "Cable connected!");
            ctx->auto_config_polarity();
        } else if ((ctx->interrupt_reg & FUSB302_REG_INTERRUPT_CRC_CHK) > 0) {
            ESP_LOGI(TAG, "Packet received!");
            if (ctx->rx_cb) ctx->rx_cb();
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
    intr_gpio_conf.intr_type = GPIO_INTR_NEGEDGE;
    intr_gpio_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    intr_gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    intr_gpio_conf.pin_bit_mask = (1U << (uint32_t)intr);

    gpio_config(&intr_gpio_conf);

    // Reset FUSB302 and PD
    reset(true, true);

    // Create GPIO Interrupt event group
    intr_evt_group = xEventGroupCreate();
    if (intr_evt_group == nullptr) {
        ESP_LOGE(TAG, "Cannot create interrupt event group");
    }

    // Setup interrupt service
    err = gpio_install_isr_service(0);
    err = err ?: gpio_isr_handler_add((gpio_num_t)intr, gpio_isr_handler, nullptr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot setup GPIO interrupt services");
    }

    // Start GPIO Interrupt task
    xTaskCreate(fusb302::intr_task, "fusb_intr", 4096, this, 3, nullptr);

    // Read Device ID
    uint8_t dev_id = get_dev_id();
    ESP_LOGI(TAG, "Init sequence started, DevID: 0x%02x", dev_id);

    // Retry 3 times, auto retry enable
    uint8_t reg_ctrl_3 = get_ctrl_3();
    reg_ctrl_3 |= FUSB302_REG_CONTROL3_AUTO_RETRY;
    reg_ctrl_3 |= PD_RETRY_COUNT << FUSB302_REG_CONTROL3_N_RETRIES_POS;
    set_ctrl_3(reg_ctrl_3);

    // Mask interrupts
    // Enable CRC_CHK and VBUSOK (set to 0), disable others
    uint8_t mask_bits = 0xff;
    mask_bits &= ~FUSB302_REG_MASK_CRC_CHK; // CRC_CHK interrupt for packet receive callback
    mask_bits &= ~FUSB302_REG_MASK_VBUSOK; // VBUSOK interrupt for charger connection detection
    set_mask(mask_bits);
    set_mask_a(0xff);
    set_mask_b(FUSB302_REG_MASK_B_GCRCSENT);

    // Enable global interrupt
    uint8_t ctrl0_reg = get_ctrl_0();
    ctrl0_reg &= ~FUSB302_REG_CONTROL0_INT_MASK;
    set_ctrl_0(ctrl0_reg);

    // Clear IRQ
    clear_irq();

    // Switches1: CC1 Tx Enable, PD 2.0, Auto GoodCRC Enable
    set_switch_1(FUSB302_REG_SWITCHES1_TXCC1_EN |
                            FUSB302_REG_SWITCHES1_AUTO_GCRC |
                            FUSB302_REG_SWITCHES1_SPECREV0);

    // Enable all power
    set_power(FUSB302_REG_POWER_PWR_ALL);

    // Clear IRQ
    clear_irq();
}

void fusb302::on_pkt_received(const tcpc_def::rx_ready_cb &func)
{
    rx_cb = func;
}

bool fusb302::detect_vbus()
{
    return false;
}

esp_err_t fusb302::set_rp(tcpc_def::rp_mode rp)
{
    return 0;
}

esp_err_t fusb302::set_cc(tcpc_def::cc_pull pull)
{
    return 0;
}

esp_err_t fusb302::get_cc(tcpc_def::cc_status *status_cc1, tcpc_def::cc_status *status_cc2)
{
    ESP_LOGD(TAG, "Getting CC status");
    if(status_cc1 == nullptr || status_cc2 == nullptr) return ESP_ERR_INVALID_ARG;

    // TODO: implement source mode
    if (is_pull_up) return ESP_ERR_NOT_SUPPORTED;
    detect_cc_pin_sink(status_cc1, status_cc2);
    return ESP_OK;
}

esp_err_t fusb302::set_polarity(bool is_flipped)
{
    // Step 1: Configure Switch0
    uint8_t sw0_reg = get_switch_0();

    // Step 1.1: Re-configure VCONN
    sw0_reg &= ~FUSB302_REG_SWITCHES0_VCONN_CC1;
    sw0_reg &= ~FUSB302_REG_SWITCHES0_VCONN_CC2;
    if (vconn_enabled) {
        sw0_reg |= is_flipped ? FUSB302_REG_SWITCHES0_VCONN_CC2 : FUSB302_REG_SWITCHES0_VCONN_CC1;
    }

    // Step 1.2: Re-configure MEAS
    sw0_reg &= ~FUSB302_REG_SWITCHES0_MEAS_CC1;
    sw0_reg &= ~FUSB302_REG_SWITCHES0_MEAS_CC2;
    sw0_reg |= is_flipped ? FUSB302_REG_SWITCHES0_MEAS_CC2 : FUSB302_REG_SWITCHES0_MEAS_CC1;

    // Step 1.3: Write back to Switch0
    set_switch_0(sw0_reg);

    // Step 2: Configure Switch1
    uint8_t sw1_reg = get_switch_1();
    sw1_reg &= ~FUSB302_REG_SWITCHES1_TXCC1_EN;
    sw1_reg &= ~FUSB302_REG_SWITCHES1_TXCC2_EN;
    sw1_reg |= is_flipped ? FUSB302_REG_SWITCHES1_TXCC2_EN : FUSB302_REG_SWITCHES1_TXCC1_EN;
    sw1_reg |= FUSB302_REG_SWITCHES1_AUTO_GCRC; // Enable Auto GoodCRC again?

    // Step 2.1: Write back to Switch1
    set_switch_1(sw1_reg);

    return ESP_OK;
}


esp_err_t fusb302::auto_config_polarity()
{
    ESP_LOGD(TAG, "Polarity auto config started");
    tcpc_def::cc_status status_cc1, status_cc2;
    auto ret = get_cc(&status_cc1, &status_cc2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get CC status");
        return ESP_FAIL;
    }

    if (status_cc1 == status_cc2) {
        ESP_LOGE(TAG, "Both CC lines are active or disconnected!");
        return ESP_ERR_INVALID_STATE;
    }

    bool is_flipped = status_cc1 < status_cc2;
    ESP_LOGI(TAG, "Cable flipped: %s", is_flipped ? "Yes" : "No");
    return set_polarity(is_flipped);
}



esp_err_t fusb302::set_vconn(bool enable)
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

void fusb302::detect_cc_pin_sink(tcpc_def::cc_status *cc1, tcpc_def::cc_status *cc2)
{
    // Step 1. Read Switch0 register
    uint8_t sw0_reg = get_switch_0();

    // Step 2. Backup original state
    uint8_t meas_cc1_orig = ((sw0_reg & FUSB302_REG_SWITCHES0_MEAS_CC1) == 0) ? 0 : 1;
    uint8_t meas_cc2_orig = ((sw0_reg & FUSB302_REG_SWITCHES0_MEAS_CC2) == 0) ? 0 : 1;

    // Step 3. Disable CC2 and enable CC1 measurement
    sw0_reg &= ~FUSB302_REG_SWITCHES0_MEAS_CC2;
    sw0_reg |= FUSB302_REG_SWITCHES0_MEAS_CC1;
    set_switch_0(sw0_reg);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Step 4. Get CC1 measurement result
    uint8_t bc_lvl_cc1 = get_status_0();
    bc_lvl_cc1 &= (FUSB302_REG_STATUS0_BC_LVL0 | FUSB302_REG_STATUS0_BC_LVL1);

    // Step 5. Disable CC1 and enable CC2 measurement
    sw0_reg &= ~FUSB302_REG_SWITCHES0_MEAS_CC1;
    sw0_reg |= FUSB302_REG_SWITCHES0_MEAS_CC2;
    set_switch_0(sw0_reg);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Step 6. Get CC1 measurement result
    uint8_t bc_lvl_cc2 = get_status_0();
    bc_lvl_cc2 &= (FUSB302_REG_STATUS0_BC_LVL0 | FUSB302_REG_STATUS0_BC_LVL1);

    // Step 7. Convert detection output
    *cc1 = convert_bc_lvl(bc_lvl_cc1);
    *cc2 = convert_bc_lvl(bc_lvl_cc2);

    // Step 8. Restore Switch0 register
    sw0_reg = get_switch_0();
    if (meas_cc1_orig > 0) {
        sw0_reg |= FUSB302_REG_SWITCHES0_MEAS_CC1;
    } else {
        sw0_reg &= ~FUSB302_REG_SWITCHES0_MEAS_CC1;
    }

    if (meas_cc2_orig > 0) {
        sw0_reg |= FUSB302_REG_SWITCHES0_MEAS_CC2;
    } else {
        sw0_reg &= ~FUSB302_REG_SWITCHES0_MEAS_CC2;
    }
    set_switch_0(sw0_reg);
}

tcpc_def::cc_status fusb302::convert_bc_lvl(uint8_t bc_lvl)
{
    auto ret = tcpc_def::TCPC_VOLT_OPEN;

    if (is_pull_up) {
        if (bc_lvl == 0x00)
            ret = tcpc_def::TCPC_VOLT_RA;
        else if (bc_lvl < 0x3)
            ret = tcpc_def::TCPC_VOLT_RD;
    } else {
        if (bc_lvl == 0x1)
            ret = tcpc_def::TCPC_VOLT_SNK_DEF;
        else if (bc_lvl == 0x2)
            ret = tcpc_def::TCPC_VOLT_SNK_1_5;
        else if (bc_lvl == 0x3)
            ret = tcpc_def::TCPC_VOLT_SNK_3_0;
    }

    return ret;
}

void fusb302::clear_irq()
{
    interrupt_reg = get_interrupt();
    interrupt_a_reg = get_interrupt_a();
    interrupt_b_reg = get_interrupt_b();

    ESP_LOGD(TAG, "IRQ: 0x%02x, a: 0x%02x, b: 0x%02x", interrupt_reg, interrupt_a_reg, interrupt_b_reg);
}

