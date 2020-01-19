#pragma once

/* Copyright 2015 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 *
 * Author: Gabe Noblesmith
 */

/* Chip Device ID - 302A or 302B */
#include <esp_err.h>
#include <functional>
#include "tcpc_drv.hpp"

#define FUSB302_DEVID_302A 0x08
#define FUSB302_DEVID_302B 0x09

/* Default retry count for transmitting */
#define PD_RETRY_COUNT		3U

#define FUSB302_REG_DEVICE_ID	0x01

#define FUSB302_REG_SWITCHES0	0x02
#define FUSB302_REG_SWITCHES0_CC2_PU_EN	(1U<<7U)
#define FUSB302_REG_SWITCHES0_CC1_PU_EN	(1U<<6U)
#define FUSB302_REG_SWITCHES0_VCONN_CC2	(1U<<5U)
#define FUSB302_REG_SWITCHES0_VCONN_CC1	(1U<<4U)
#define FUSB302_REG_SWITCHES0_MEAS_CC2	(1U<<3U)
#define FUSB302_REG_SWITCHES0_MEAS_CC1	(1U<<2U)
#define FUSB302_REG_SWITCHES0_CC2_PD_EN	(1U<<1U)
#define FUSB302_REG_SWITCHES0_CC1_PD_EN	(1U<<0U)

#define FUSB302_REG_SWITCHES1	0x03
#define FUSB302_REG_SWITCHES1_POWERROLE	(1U<<7U)
#define FUSB302_REG_SWITCHES1_SPECREV1	(1U<<6U)
#define FUSB302_REG_SWITCHES1_SPECREV0	(1U<<5U)
#define FUSB302_REG_SWITCHES1_DATAROLE	(1U<<4U)
#define FUSB302_REG_SWITCHES1_AUTO_GCRC	(1U<<2U)
#define FUSB302_REG_SWITCHES1_TXCC2_EN	(1U<<1U)
#define FUSB302_REG_SWITCHES1_TXCC1_EN	(1U<<0U)

#define FUSB302_REG_MEASURE	0x04
#define FUSB302_REG_MEASURE_MDAC_MASK	0x3F
#define FUSB302_REG_MEASURE_VBUS		(1U<<6U)

#define FUSB302_REG_SLICE   0x05

/*
 * MDAC reference voltage step size is 42 mV. Round our thresholds to reduce
 * maximum error, which also matches suggested thresholds in datasheet
 * (Table 3. Host Interrupt Summary).
 */
#define FUSB302_REG_MEASURE_MDAC_MV(mv)	(DIV_ROUND_NEAREST((mv), 42) & 0x3f)

#define FUSB302_REG_CONTROL0	0x06
#define FUSB302_REG_CONTROL0_TX_FLUSH	(1U<<6U)
#define FUSB302_REG_CONTROL0_INT_MASK	(1U<<5U)
#define FUSB302_REG_CONTROL0_HOST_CUR_MASK (3U<<2U)
#define FUSB302_REG_CONTROL0_HOST_CUR_3A0  (3U<<2U)
#define FUSB302_REG_CONTROL0_HOST_CUR_1A5  (2U<<2U)
#define FUSB302_REG_CONTROL0_HOST_CUR_USB  (1U<<2U)
#define FUSB302_REG_CONTROL0_TX_START	(1U<<0U)

#define FUSB302_REG_CONTROL1	0x07
#define FUSB302_REG_CONTROL1_ENSOP2DB	(1U<<6U)
#define FUSB302_REG_CONTROL1_ENSOP1DB	(1U<<5U)
#define FUSB302_REG_CONTROL1_BIST_MODE2	(1U<<4U)
#define FUSB302_REG_CONTROL1_RX_FLUSH	(1U<<2U)
#define FUSB302_REG_CONTROL1_ENSOP2	(1U<<1U)
#define FUSB302_REG_CONTROL1_ENSOP1	(1U<<0U)

#define FUSB302_REG_CONTROL2	0x08
/* two-bit field, valid values below */
#define FUSB302_REG_CONTROL2_MODE_MASK	(0x3U<<FUSB302_REG_CONTROL2_MODE_POS)
#define FUSB302_REG_CONTROL2_MODE_DFP	(0x3U)
#define FUSB302_REG_CONTROL2_MODE_UFP	(0x2U)
#define FUSB302_REG_CONTROL2_MODE_DRP	(0x1U)
#define FUSB302_REG_CONTROL2_MODE_POS	(1U)
#define FUSB302_REG_CONTROL2_TOGGLE	(1U<<0U)

#define FUSB302_REG_CONTROL3	0x09
#define FUSB302_REG_CONTROL3_SEND_HARDRESET	(1U<<6U)
#define FUSB302_REG_CONTROL3_BIST_TMODE		(1U<<5U) /* 302B Only */
#define FUSB302_REG_CONTROL3_AUTO_HARDRESET	(1U<<4U)
#define FUSB302_REG_CONTROL3_AUTO_SOFTRESET	(1U<<3U)
/* two-bit field */
#define FUSB302_REG_CONTROL3_N_RETRIES		(1U<<1U)
#define FUSB302_REG_CONTROL3_N_RETRIES_POS		(1U)
#define FUSB302_REG_CONTROL3_N_RETRIES_SIZE	(2U)
#define FUSB302_REG_CONTROL3_AUTO_RETRY		(1U<<0U)

#define FUSB302_REG_MASK		0x0A
#define FUSB302_REG_MASK_VBUSOK		(1U<<7U)
#define FUSB302_REG_MASK_ACTIVITY		(1U<<6U)
#define FUSB302_REG_MASK_COMP_CHNG		(1U<<5U)
#define FUSB302_REG_MASK_CRC_CHK		(1U<<4U)
#define FUSB302_REG_MASK_ALERT		(1U<<3U)
#define FUSB302_REG_MASK_WAKE		(1U<<2U)
#define FUSB302_REG_MASK_COLLISION		(1U<<1U)
#define FUSB302_REG_MASK_BC_LVL		(1U<<0U)

#define FUSB302_REG_POWER		0x0B
#define FUSB302_REG_POWER_PWR		(1U<<0U)	/* four-bit field */
#define FUSB302_REG_POWER_PWR_LOW		0x1 /* Bandgap + Wake circuitry */
#define FUSB302_REG_POWER_PWR_MEDIUM	0x3 /* LOW + Receiver + Current refs */
#define FUSB302_REG_POWER_PWR_HIGH		0x7 /* MEDIUM + Measure block */
#define FUSB302_REG_POWER_PWR_ALL		0xF /* HIGH + Internal Oscillator */

#define FUSB302_REG_RESET		0x0C
#define FUSB302_REG_RESET_PD_RESET		(1U<<1U)
#define FUSB302_REG_RESET_SW_RESET		(1U<<0U)

#define FUSB302_REG_OCP         0x0D

#define FUSB302_REG_MASK_A		0x0E
#define FUSB302_REG_MASK_A_OCP_TEMP		(1U<<7U)
#define FUSB302_REG_MASK_A_TOGDONE		(1U<<6U)
#define FUSB302_REG_MASK_A_SOFTFAIL		(1U<<5U)
#define FUSB302_REG_MASK_A_RETRYFAIL	(1U<<4U)
#define FUSB302_REG_MASK_A_HARDSENT		(1U<<3U)
#define FUSB302_REG_MASK_A_TX_SUCCESS	(1U<<2U)
#define FUSB302_REG_MASK_A_SOFTRESET	(1U<<1U)
#define FUSB302_REG_MASK_A_HARDRESET	(1U<<0U)

#define FUSB302_REG_MASK_B		0x0F
#define FUSB302_REG_MASK_B_GCRCSENT		(1U<<0U)

#define FUSB302_REG_STATUS0A	0x3C
#define FUSB302_REG_STATUS0A_SOFTFAIL	(1U<<5U)
#define FUSB302_REG_STATUS0A_RETRYFAIL	(1U<<4U)
#define FUSB302_REG_STATUS0A_POWER		(1U<<2U) /* two-bit field */
#define FUSB302_REG_STATUS0A_RX_SOFT_RESET	(1U<<1U)
#define FUSB302_REG_STATUS0A_RX_HARD_RESEt	(1U<<0U)

#define FUSB302_REG_CONTROL4    0x10

#define FUSB302_REG_STATUS1A	0x3D
/* three-bit field, valid values below */
#define FUSB302_REG_STATUS1A_TOGSS		(1U<<3U)
#define FUSB302_REG_STATUS1A_TOGSS_RUNNING		0x0U
#define FUSB302_REG_STATUS1A_TOGSS_SRC1		0x1U
#define FUSB302_REG_STATUS1A_TOGSS_SRC2		0x2U
#define FUSB302_REG_STATUS1A_TOGSS_SNK1		0x5U
#define FUSB302_REG_STATUS1A_TOGSS_SNK2		0x6U
#define FUSB302_REG_STATUS1A_TOGSS_AA		0x7U
#define FUSB302_REG_STATUS1A_TOGSS_POS		(3U)
#define FUSB302_REG_STATUS1A_TOGSS_MASK		(0x7U)

#define FUSB302_REG_STATUS1A_RXSOP2DB	(1U<<2U)
#define FUSB302_REG_STATUS1A_RXSOP1DB	(1U<<1U)
#define FUSB302_REG_STATUS1A_RXSOP		(1U<<0U)

#define FUSB302_REG_INTERRUPT_A	0x3E
#define FUSB302_REG_INTERRUPT_A_OCP_TEMP	(1U<<7U)
#define FUSB302_REG_INTERRUPT_A_TOGDONE	(1U<<6U)
#define FUSB302_REG_INTERRUPT_A_SOFTFAIL	(1U<<5U)
#define FUSB302_REG_INTERRUPT_A_RETRYFAIL	(1U<<4U)
#define FUSB302_REG_INTERRUPT_A_HARDSENT	(1U<<3U)
#define FUSB302_REG_INTERRUPT_A_TX_SUCCESS	(1U<<2U)
#define FUSB302_REG_INTERRUPT_A_SOFTRESET	(1U<<1U)
#define FUSB302_REG_INTERRUPT_A_HARDRESET	(1U<<0U)

#define FUSB302_REG_INTERRUPT_B	0x3F
#define FUSB302_REG_INTERRUPT_B_GCRCSENT		(1U<<0U)

#define FUSB302_REG_STATUS0	0x40
#define FUSB302_REG_STATUS0_VBUSOK		(1U<<7U)
#define FUSB302_REG_STATUS0_ACTIVITY	(1U<<6U)
#define FUSB302_REG_STATUS0_COMP		(1U<<5U)
#define FUSB302_REG_STATUS0_CRC_CHK	(1U<<4U)
#define FUSB302_REG_STATUS0_ALERT		(1U<<3U)
#define FUSB302_REG_STATUS0_WAKE		(1U<<2U)
#define FUSB302_REG_STATUS0_BC_LVL1	(1U<<1U) /* two-bit field */
#define FUSB302_REG_STATUS0_BC_LVL0	(1U<<0U) /* two-bit field */

#define FUSB302_REG_STATUS1	0x41
#define FUSB302_REG_STATUS1_RXSOP2		(1U<<7U)
#define FUSB302_REG_STATUS1_RXSOP1		(1U<<6U)
#define FUSB302_REG_STATUS1_RX_EMPTY	(1U<<5U)
#define FUSB302_REG_STATUS1_RX_FULL	(1U<<4U)
#define FUSB302_REG_STATUS1_TX_EMPTY	(1U<<3U)
#define FUSB302_REG_STATUS1_TX_FULL	(1U<<2U)

#define FUSB302_REG_INTERRUPT	0x42
#define FUSB302_REG_INTERRUPT_VBUSOK	(1U<<7U)
#define FUSB302_REG_INTERRUPT_ACTIVITY	(1U<<6U)
#define FUSB302_REG_INTERRUPT_COMP_CHNG	(1U<<5U)
#define FUSB302_REG_INTERRUPT_CRC_CHK	(1U<<4U)
#define FUSB302_REG_INTERRUPT_ALERT	(1U<<3U)
#define FUSB302_REG_INTERRUPT_WAKE		(1U<<2U)
#define FUSB302_REG_INTERRUPT_COLLISION	(1U<<1U)
#define FUSB302_REG_INTERRUPT_BC_LVL	(1U<<0U)

#define FUSB302_REG_FIFOS		0x43

namespace fusb302_token
{
    /* Tokens defined for the FUSB302 TX FIFO */
    typedef enum  {
        TXON = 0xA1,
        SYNC1 = 0x12,
        SYNC2 = 0x13,
        SYNC3 = 0x1B,
        RST1 = 0x15,
        RST2 = 0x16,
        PACKSYM = 0x80,
        JAMCRC = 0xFF,
        EOP = 0x14,
        TXOFF = 0xFE,
    } tx_token_t;

    typedef enum {
        SOP     = 0b11100000,
        SOP1    = 0b11000000,
        SOP2    = 0b10100000,
        SOP1DB  = 0b10000000,
        SOP2DB  = 0b01100000
    } fusb302_rx_token_t;
}

namespace drv
{
    class fusb302 : public tcpc
    {
    public:
        fusb302(int sda, int scl, int intr, i2c_port_t port = I2C_NUM_0);
        esp_err_t receive_pkt(uint16_t *header, uint32_t *data_objs, size_t max_cnt, size_t *actual_cnt) override;
        esp_err_t transmit_pkt(uint16_t header, const uint32_t *data_objs, size_t obj_cnt) override;
        void on_pkt_received(const tcpc_def::rx_cb_t &func) override;
        bool detect_vbus() override;
        esp_err_t set_rp(tcpc_def::rp_t rp) override;
        esp_err_t set_cc(tcpc_def::cc_pull_t pull) override;
        esp_err_t get_cc(tcpc_def::cc_status_t *status_cc1, tcpc_def::cc_status_t *status_cc2) override;

    private:
        uint8_t read_reg(uint8_t reg);
        void write_reg(uint8_t reg, uint8_t param);
        uint8_t get_dev_id();
        void set_switch_0(uint8_t val);
        uint8_t get_switch_0();
        void set_switch_1(uint8_t val);
        uint8_t get_switch_1();
        void set_measure(uint8_t val);
        uint8_t get_measure();
        void set_slice(uint8_t val);
        uint8_t get_slice();
        void set_ctrl_0(uint8_t val);
        uint8_t get_ctrl_0();
        void set_ctrl_1(uint8_t val);
        uint8_t get_ctrl_1();
        void set_ctrl_2(uint8_t val);
        uint8_t get_ctrl_2();
        void set_ctrl_3(uint8_t val);
        uint8_t get_ctrl_3();
        void set_ctrl_4(uint8_t val);
        uint8_t get_ctrl_4();
        void set_mask(uint8_t val);
        uint8_t get_mask();
        void set_power(uint8_t val);
        uint8_t get_power();
        void reset(bool pd_rst, bool sw_rst);
        uint8_t get_ocp();
        void set_ocp(uint8_t val);
        void set_mask_a(uint8_t val);
        uint8_t get_mask_a();
        void set_mask_b(uint8_t val);
        uint8_t get_mask_b();
        uint8_t get_status_0a();
        uint8_t get_status_1a();
        uint8_t get_status_0();
        uint8_t get_status_1();
        void clear_interrupt_a(uint8_t val);
        uint8_t get_interrupt_a();
        void clear_interrupt_b(uint8_t val);
        uint8_t get_interrupt_b();
        void clear_interrupt(uint8_t val);
        uint8_t get_interrupt();
        static void intr_task(void *arg);
        static void gpio_isr_handler(void* arg);

    private:
        static xQueueHandle intr_evt_queue;
        i2c_port_t i2c_port = I2C_NUM_0;
        tcpc_def::rx_cb_t rx_cb = {};
    };
}

