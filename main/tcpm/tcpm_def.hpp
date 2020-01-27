#pragma once

namespace protocol::def
{
    enum pdo_type {
        FIXED_PDO           = 0b00,
        BATTERY_PDO         = 0b01,
        VARIABLE_PDO        = 0b10,
        AUGMENTED_PDO       = 0b11,
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
        DATA_PACKET_BASE        = 0b1000000,
        SOURCE_CAPABILITIES     = 0b1000001,
        REQUEST                 = 0b1000010,
        BIST                    = 0b1000011,
        SINK_CAPABILITIES       = 0b1000100,
        BATTERY_STATUS          = 0b1000101,
        ALERT                   = 0b1000110,
        GET_COUNTRY_INFO        = 0b1000111,
        ENTER_USB               = 0b1001000,
        VENDOR_DEFINED          = 0b1001111,
    };

    enum port_power_role {
        SINK = 0,
        SOURCE = 1,
    };

    enum port_data_role {
        UFP = 0,
        DFP = 1
    };

    enum overload_cap {
        OVERLOAD_NONE       = 0b00,
        OVERLOAD_LEVEL_1    = 0b01,
        OVERLOAD_LEVEL_2    = 0b10,
        OVERLOAD_LEVEL_3    = 0b11
    };
}