#pragma once

#include <esp_err.h>
#include <tcpc_drv.hpp>

namespace proto_def
{
    enum pdo_type {
        FIXED_PDO           = 0b00,
        BATTERY_PDO         = 0b01,
        VARIABLE_PDO        = 0b10,
        PROGRAMMABLE_PDO    = 0b11,
    };

    enum ctrl_pkt_type {
        GOOD_CRC            = 0b0001,
        GOTO_MIN            = 0b0010,
        ACCEPT              = 0b0011,
        REJECT              = 0b0100,
        PING                = 0b0101,
        PS_READY            = 0b0110,
        GET_SOURCE_CAP      = 0b0111,
        GET_SINK_CAP        = 0b1000,
        DR_SWAP             = 0b1001,
        PR_SWAP             = 0b1010,
        VCONN_SWAP          = 0b1011,
        WAIT                = 0b1100,
        SOFT_RESET          = 0b1101,
        NOT_SUPPORTED       = 0b10000,
        GET_SOURCE_CAP_EXTENDED = 0b10001,
        GET_STATUS              = 0b10010,
        FR_SWAP                 = 0b10011,
        GET_PPS_STATUS          = 0b10100,
        GET_COUNTRY_CODES       = 0b10101,
        GET_SINK_CAP_EXTENDED   = 0b10110
    };

    struct pdo {
        bool suspend                = false;
        bool unconstrained_power    = false;
        bool usb_comm               = false;
        bool dual_role_data         = false;
        bool dual_role_power        = false;
        bool unchunked_msg_support  = false;
        pdo_type type               = FIXED_PDO;
        uint32_t voltage_min        = 0;
        uint32_t voltage_max        = 0;
        uint32_t current_min        = 0;
        uint32_t power_min          = 0;
        uint32_t power_max          = 0;
    };
}

namespace protocol
{
    class tcpm
    {
    public:
        tcpm(device::tcpc& _device);
        int on_pkt_rx();

    private:
        device::tcpc& port_dev;
        std::array<proto_def::pdo, 7> rx_pdo_list;
    };
}