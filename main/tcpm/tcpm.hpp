#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <tcpc_drv.hpp>

namespace proto_def
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

        // FSM related
        FSM_START               = 0b10000000,
        FSM_STOP                = 0b11111111,
        FSM_ERROR               = 0b10101010,
        UNIMPLEMENTED           = INT32_MAX,
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

    struct header {
        uint8_t num_obj = 0;
        uint8_t msg_id = 0;
        spec_revision revision = REV_2_0;
        port_data_role data_role = DFP;
        port_power_role power_role = SINK;
        pkt_type type = GOOD_CRC;
    };

    struct pdo {
        bool suspend_support        = false;
        bool unconstrained_power    = false;
        bool usb_comm               = false;
        bool dual_role_data         = false;
        bool dual_role_power        = false;
        bool unchunked_msg_support  = false;
        pdo_type type               = FIXED_PDO;
        overload_cap overload       = OVERLOAD_NONE;
        uint32_t voltage_min        = 0;
        uint32_t voltage_max        = 0;
        uint32_t current            = 0;
        uint32_t power              = 0;
    };

    enum fsm_direction {
        TRANSIT_SINK_TO_SOURCE = 0,
        TRANSIT_SOURCE_TO_SINK = 1,
        TRANSIT_NO_DIRECTION = 2,
    };
}

namespace protocol
{
    struct tcpm_state {
        proto_def::pkt_type curr;
        proto_def::pkt_type next;
        proto_def::fsm_direction direction;
        TickType_t timeout_ticks;
        std::function<esp_err_t()> cb;
    };

    class tcpm
    {
    public:
        explicit tcpm(device::tcpc& _device);
        esp_err_t on_pkt_rx();
        esp_err_t request_fixed_power(uint32_t voltage_mv, uint32_t current_ma);

    private:
        esp_err_t on_src_cap_received();
        esp_err_t on_request_sent();
        esp_err_t on_ps_ready_received();

    private:
        esp_err_t add_pdo(uint32_t data_obj);
        esp_err_t add_pdo(const uint32_t *data_objs, uint8_t len);

    private:
        const tcpm_state sink_fsm[6] = {
            {
                    proto_def::FSM_START,
                    proto_def::GET_SOURCE_CAP,
                    proto_def::TRANSIT_NO_DIRECTION,
                    portMAX_DELAY,
                    [&]() { return ESP_OK; }
            },{
                    proto_def::GET_SOURCE_CAP,
                    proto_def::SOURCE_CAPABILITIES,
                    proto_def::TRANSIT_SINK_TO_SOURCE,
                    pdMS_TO_TICKS(1000),
                    [&]() { return ESP_OK; }
            },{
                    proto_def::SOURCE_CAPABILITIES,
                    proto_def::REQUEST,
                    proto_def::TRANSIT_SOURCE_TO_SINK,
                    portMAX_DELAY,
                    [&]() { return on_src_cap_received(); }
            }, {
                    proto_def::REQUEST,
                    proto_def::ACCEPT,
                    proto_def::TRANSIT_SINK_TO_SOURCE,
                    pdMS_TO_TICKS(1000),
                    [&]() { return on_request_sent(); }
            }, {
                    proto_def::ACCEPT,
                    proto_def::PS_READY,
                    proto_def::TRANSIT_SINK_TO_SOURCE,
                    pdMS_TO_TICKS(1000),
                    [&]() { return ESP_OK; }
            }, {
                    proto_def::PS_READY,
                    proto_def::FSM_STOP,
                    proto_def::TRANSIT_NO_DIRECTION,
                    portMAX_DELAY,
                    [&]() { return on_ps_ready_received(); }
            }
        };


    private:
        device::tcpc& port_dev;

        proto_def::header pkt_header = {};
        std::vector<proto_def::pdo> pdo_list;
        uint8_t curr_sink_state = 0;
        
        
    };
}