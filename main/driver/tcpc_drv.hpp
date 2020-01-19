#pragma once

#include <esp_err.h>
#include <esp_http_server.h>
#include <functional>
#include "../tcpm/tcpm.hpp"

#define TCPC_PD_HEADER_DATA_OBJ_CNT(header)   ((uint8_t)((header) >> 12U) & 7U)

namespace tcpc_def
{
    typedef enum {
        TCPC_RP_USB = 0,
        TCPC_RP_1A5 = 1,
        TCPC_RP_3A0 = 2
    } rp_t;

    typedef enum  {
        TCPC_VOLT_OPEN = 0,
        TCPC_VOLT_RA = 1,	  /* Port partner is applying Ra */
        TCPC_VOLT_RD = 2,	  /* Port partner is applying Rd */
        TCPC_VOLT_RP_DEF = 5, /* Port partner is applying Rp (0.5A) */
        TCPC_VOLT_RP_1_5 = 6, /* Port partner is applying Rp (1.5A) */
        TCPC_VOLT_RP_3_0 = 7, /* Port partner is applying Rp (3.0A) */
    } cc_status_t;

    typedef enum {
        TCPC_TX_SOP = 0,
        TCPC_TX_SOP_PRIME = 1,
        TCPC_TX_SOP_PRIME_PRIME = 2,
        TCPC_TX_SOP_DEBUG_PRIME = 3,
        TCPC_TX_SOP_DEBUG_PRIME_PRIME = 4,
        TCPC_TX_HARD_RESET = 5,
        TCPC_TX_CABLE_RESET = 6,
        TCPC_TX_BIST_MODE_2 = 7
    } tx_t;

    typedef enum {
        TCPC_CC_RA = 0,
        TCPC_CC_RP = 1,
        TCPC_CC_RD = 2,
        TCPC_CC_OPEN = 3,
        TCPC_CC_RA_RD = 4, /* Powered cable with Sink */
    } cc_pull_t;

    typedef std::function<esp_err_t()> rx_cb_t;
}

namespace drv
{
    class tcpc
    {
    public:
        virtual esp_err_t transmit_pkt(uint16_t header, const uint32_t *data_objs, size_t obj_cnt) = 0;
        virtual esp_err_t receive_pkt(uint16_t *header, uint32_t *data_objs, size_t max_cnt, size_t *actual_cnt) = 0;
        virtual void on_pkt_received(const tcpc_def::rx_cb_t &rx_cb) = 0;
        virtual bool detect_vbus() = 0;
        virtual esp_err_t set_rp(tcpc_def::rp_t rp) = 0;
        virtual esp_err_t set_cc(tcpc_def::cc_pull_t pull) = 0;
        virtual esp_err_t get_cc(tcpc_def::cc_status_t *status_cc1, tcpc_def::cc_status_t *status_cc2) = 0;
    };
}



