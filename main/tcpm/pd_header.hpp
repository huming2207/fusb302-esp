#pragma once

#include <cstdint>
#include <esp_err.h>

#include "tcpm_def.hpp"

namespace protocol
{
    class pd_header
    {
    public:
        uint16_t encode();
        esp_err_t decode(uint16_t header);

    public:
        bool extended = false;
        uint8_t num_obj = 0;
        uint8_t msg_id = 0;
        def::spec_revision revision = def::REV_2_0;
        def::port_data_role data_role = def::DFP;
        def::port_power_role power_role = def::SINK;
        def::message_type msg_type = def::GOOD_CRC;
    };
}
