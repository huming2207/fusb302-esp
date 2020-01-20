#pragma once

#include <esp_err.h>
#include <esp_http_server.h>
#include <functional>
#include <hal/i2c_types.h>

#define TCPC_PD_HEADER_DATA_OBJ_CNT(header)   ((uint8_t)((header) >> 12U) & 7U)

#define TCPC_PD_HEADER_CNT(header)  (((header) >> 12) & 7)
#define TCPC_PD_HEADER_TYPE(header) ((header) & 0xF)
#define TCPC_PD_HEADER_ID(header)   (((header) >> 9) & 7)

/* Minimum PD supply current  (mA) */
#define TCPC_PD_MIN_MA   500

/* Minimum PD voltage (mV) */
#define TCPC_PD_MIN_MV   5000

/* No connect voltage threshold for sources based on Rp */
#define TCPC_PD_SRC_DEF_VNC_MV        1600
#define TCPC_PD_SRC_1_5_VNC_MV        1600
#define TCPC_PD_SRC_3_0_VNC_MV        2600

/* Rd voltage threshold for sources based on Rp */
#define TCPC_PD_SRC_DEF_RD_THRESH_MV  200
#define TCPC_PD_SRC_1_5_RD_THRESH_MV  400
#define TCPC_PD_SRC_3_0_RD_THRESH_MV  800

/* Voltage threshold to detect connection when presenting Rd */
#define TCPC_PD_SNK_VA_MV             250

namespace tcpc_def
{
    enum rp_mode {
        TCPC_RP_USB = 0,
        TCPC_RP_1A5 = 1,
        TCPC_RP_3A0 = 2
    };

    enum cc_status {
        TCPC_VOLT_OPEN = 0,
        TCPC_VOLT_RA = 1,	  /* Port partner is applying Ra */
        TCPC_VOLT_RD = 2,	  /* Port partner is applying Rd */
        TCPC_VOLT_SNK_DEF = 5, /* Port partner is applying Rp (0.5A) */
        TCPC_VOLT_SNK_1_5 = 6, /* Port partner is applying Rp (1.5A) */
        TCPC_VOLT_SNK_3_0 = 7, /* Port partner is applying Rp (3.0A) */
    } ;

    enum tx_mode {
        TCPC_TX_SOP = 0,
        TCPC_TX_SOP_PRIME = 1,
        TCPC_TX_SOP_PRIME_PRIME = 2,
        TCPC_TX_SOP_DEBUG_PRIME = 3,
        TCPC_TX_SOP_DEBUG_PRIME_PRIME = 4,
        TCPC_TX_HARD_RESET = 5,
        TCPC_TX_CABLE_RESET = 6,
        TCPC_TX_BIST_MODE_2 = 7
    };

    enum cc_pull {
        TCPC_CC_RA = 0,
        TCPC_CC_RP = 1,
        TCPC_CC_RD = 2,
        TCPC_CC_OPEN = 3,
        TCPC_CC_RA_RD = 4, /* Powered cable with Sink */
    } ;

    enum cc_polarity {
        TCPC_CC_POLARITY_CC1 = 0,
        TCPC_CC_POLARITY_CC2 = 1
    };

    typedef std::function<int()> rx_ready_cb;
    typedef std::function<int()> cc_status_change_cb;
}

namespace device
{
    /**
     * Type-C Port Controller Interface (virtual class)
     */
    class tcpc
    {
    public:
        virtual esp_err_t transmit_pkt(uint16_t header, const uint32_t *data_objs, size_t obj_cnt) = 0;
        virtual esp_err_t receive_pkt(uint16_t *header, uint32_t *data_objs, size_t max_cnt, size_t *actual_cnt) = 0;
        virtual void on_pkt_received(const tcpc_def::rx_ready_cb &rx_cb) = 0;
        virtual bool detect_vbus() = 0;
        virtual esp_err_t set_rp(tcpc_def::rp_mode rp) = 0;
        virtual esp_err_t set_cc(tcpc_def::cc_pull pull) = 0;

        /**
         * Get CC status
         *
         * @param status_cc1[out] CC1's status
         * @param status_cc2[out] CC2's status
         * @return
         */
        virtual esp_err_t get_cc(tcpc_def::cc_status *status_cc1, tcpc_def::cc_status *status_cc2) = 0;

        /**
         * Set polarity of a cable
         *
         * @param is_flipped True for flipped (i.e. CC2 is being used)
         * @return ESP_OK if successful
         */
        virtual esp_err_t set_polarity(bool is_flipped) = 0;
        virtual esp_err_t set_vconn(bool enable) = 0;

        /**
         * Automatically detect and configure polarity of a cable connection
         *
         * This is a combination of get_cc() and set_polarity()
         *
         * @return
         *      - ESP_OK if successful
         *      - ESP_ERR_INVALID_STATE if two CC wires are being pulled or disconnected
         *      - ESP_FAIL if CC status detection failed
         *
         */
        virtual esp_err_t auto_config_polarity() = 0;

    protected:
        i2c_port_t i2c_port = 0;
        tcpc_def::rx_ready_cb rx_cb = {};
        tcpc_def::cc_status_change_cb cc_change_cb = {};
    };
}



