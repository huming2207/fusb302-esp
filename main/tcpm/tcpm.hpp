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

    enum spec_revision {
        REV_1_0 = 0b00,
        REV_2_0 = 0b01,
        REV_3_0 = 0b10
    };

    enum pkt_type {
        // Control packets
        GOOD_CRC                = 0b00001,
        GOTO_MIN                = 0b00010,
        ACCEPT                  = 0b00011,
        REJECT                  = 0b00100,
        PING                    = 0b00101,
        PS_READY                = 0b00110,
        GET_SOURCE_CAP          = 0b00111,
        GET_SINK_CAP            = 0b01000,
        DR_SWAP                 = 0b01001,
        PR_SWAP                 = 0b01010,
        VCONN_SWAP              = 0b01011,
        WAIT                    = 0b01100,
        SOFT_RESET              = 0b01101,
        NOT_SUPPORTED           = 0b10000,
        GET_SOURCE_CAP_EXTENDED = 0b10001,
        GET_STATUS              = 0b10010,
        FR_SWAP                 = 0b10011,
        GET_PPS_STATUS          = 0b10100,
        GET_COUNTRY_CODES       = 0b10101,
        GET_SINK_CAP_EXTENDED   = 0b10110,

        // Data packets
        SOURCE_CAPABILITIES     = -0b00001,
        REQUEST                 = -0b00010,
        BIST                    = -0b00011,
        SINK_CAPABILITIES       = -0b00100,
        BATTERY_STATUS          = -0b00101,
        ALERT                   = -0b00110,
        GET_COUNTRY_INFO        = -0b00111,
        ENTER_USB               = -0b01000,
        VENDOR_DEFINED          = -0b01111
    };

    enum port_power_role {
        SINK = 0,
        SOURCE = 1,
    };

    enum port_data_role {
        UFP = 0,
        DFP = 1
    };

    struct header {
        uint8_t num_obj = 0;
        uint8_t msg_id = 0;
        spec_revision revision = REV_2_0;
        port_data_role data_role = DFP;
        port_power_role power_role = SINK;
        pkt_type type = GOOD_CRC;
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
        uint32_t current_max        = 0;
        uint32_t power_min          = 0;
        uint32_t power_max          = 0;
    };

    typedef std::array<pdo, 7> pdos;
}

namespace protocol
{
    class tcpm
    {
    public:
        explicit tcpm(device::tcpc& _device);
        esp_err_t on_pkt_rx();

    private:



    private:
        device::tcpc& port_dev;
        std::array<proto_def::pdo, 7> rx_pdo_list;
    };
}