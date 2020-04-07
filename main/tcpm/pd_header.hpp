#pragma once

#include <cstdint>
#include <esp_err.h>

#include "tcpm_def.hpp"

namespace protocol
{
    class pd_header
    {
    public:
        pd_header() = default;
        explicit pd_header(uint16_t header);
        uint16_t encode();
        esp_err_t decode(uint16_t header);
        bool is_ctrl_msg();

    public:
        bool extended = false;
        uint8_t num_obj = 0;
        uint8_t msg_id = 0;
        def::spec_revision revision = def::REV_2_0;
        def::port_data_role data_role = def::DFP;
        def::port_power_role power_role = def::SINK;
        def::message_type msg_type = def::GOOD_CRC;

    private:
        constexpr static bool get_extended(uint16_t header)
        {
            return ((uint8_t)((uint16_t)((header)) >> 15U) & 0b1U) == 0;
        };

        constexpr static uint8_t get_number_of_data_obj(uint16_t header)
        {
            return (((uint8_t)((uint16_t)((header)) >> 12U) & 0b111U));
        };

        constexpr static uint8_t get_msg_id(uint16_t header)
        {
            return (((uint8_t)((uint16_t)((header)) >> 9U) & 0b111U));
        };

        constexpr static def::port_power_role get_port_power_role(uint16_t header)
        {
            return ((protocol::def::port_power_role)((uint8_t)((uint16_t)((header)) >> 8U) & 0b1U));
        };

        constexpr static def::spec_revision get_spec_rev(uint16_t header)
        {
            return ((protocol::def::spec_revision)((uint8_t)((uint16_t)((header)) >> 6U) & 0b11U));
        };

        constexpr static def::port_data_role get_port_data_role(uint16_t header)
        {
            return ((protocol::def::port_data_role)((uint8_t)((uint16_t)((header)) >> 5U) & 0b1U));
        };

        constexpr static def::message_type get_data_msg_type(uint16_t header)
        {
            return ((protocol::def::message_type)(((uint16_t)((header)) & 0b11111U) + def::DATA_PACKET_BASE));
        };

        constexpr static def::message_type get_ctrl_msg_type(uint16_t header)
        {
            return ((protocol::def::message_type)((uint16_t)((header)) & 0b11111U));
        };

        constexpr static uint16_t set_extended(uint8_t bit)
        {
            return (uint16_t)((((bit) ? 1U : 0U) & 0b1U) << 15U);
        };

        constexpr static uint16_t set_number_of_data_obj(uint8_t bit)
        {
            return (((uint16_t)((bit)) & 0b111U) << 12U);
        };

        constexpr static uint16_t set_msg_id(uint8_t id)
        {
            return (((uint16_t)((id)) & 0b111U) << 9U);
        };

        constexpr static uint16_t set_port_power_role(uint8_t bit)
        {
            return (((uint16_t)((bit)) & 0b1U) << 8U);
        };

        constexpr static uint16_t set_spec_rev(uint8_t bit)
        {
            return (((uint16_t)((bit)) & 0b11U) << 6U);
        };

        constexpr static uint16_t set_port_data_role(uint8_t bit)
        {
            return (((uint16_t)((bit)) & 0b1U) << 5U);
        };

        constexpr static uint16_t set_msg_type(uint8_t bit)
        {
            return (((uint16_t)((bit)) & 0b11111U));
        };
    };
}
