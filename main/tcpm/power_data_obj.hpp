#pragma once

#include <cstdint>
#include <esp_err.h>
#include <esp_bit_defs.h>


#include "tcpm_def.hpp"

namespace protocol
{
    class power_data_obj
    {
    public:
        power_data_obj() = default;
        power_data_obj(power_data_obj const &) = default;
        explicit power_data_obj(uint32_t pdo_word);
        uint32_t encode_fixed_pdo_request(int obj_pos = 0, int op_current = -1);
        uint32_t encode_augmented_pdo(uint8_t obj_pos = 0);
        esp_err_t decode_pdo(uint32_t pdo_word);
        bool operator==(power_data_obj &other);
        bool operator>(power_data_obj &other);
        bool operator<(power_data_obj &other);

    public:
        bool suspend_support = false;
        bool unconstrained_power = false;
        bool usb_comm = false;
        bool dual_role_data = false;
        bool dual_role_power = false;
        bool unchunked_msg_support = false;
        def::pdo_type pdo_type = def::FIXED_PDO;
        def::peak_current peak_curr_lvl = def::PEAK_CURRENT_NONE;
        uint32_t voltage_min = 0;
        uint32_t voltage_max = 0;
        uint32_t current = 0;
        uint32_t power = 0;

    private:
        constexpr static uint32_t set_fixed_current(uint32_t curr_ma)
        {
            return ((uint32_t) (((uint16_t) ((curr_ma) / 10U) & 0x3ffU) << 0U));
        };

        constexpr static uint32_t set_fixed_voltage(uint32_t volt_mv)
        {
            return ((uint32_t) (((uint16_t) ((volt_mv) / 10U) & 0x3ffU) << 10U));
        };

        constexpr static uint32_t set_unchunked_msg(uint8_t bit)
        {
            return ((uint32_t) (((uint8_t) (bit)) << 23U));
        };

        constexpr static uint32_t set_no_usb_suspend(uint8_t bit)
        {
            return ((uint32_t) (((uint8_t) (bit)) << 24U));
        };

        constexpr static uint32_t set_usb_comm_cap(uint8_t bit)
        {
            return ((uint32_t) (((uint8_t) (bit)) << 25U));
        };

        constexpr static uint32_t set_cap_mismatch(uint8_t bit)
        {
            return ((uint32_t) (((uint8_t) (bit)) << 26U));
        };

        constexpr static uint32_t set_giveback(uint8_t bit)
        {
            return ((uint32_t) (((uint8_t) (bit)) << 27U));
        };

        constexpr static uint32_t set_obj_position(uint8_t pos)
        {
            return ((uint32_t) (((uint8_t) (pos) & 0b111U) << 28U));
        };

        constexpr static uint32_t get_dual_role_power(uint32_t pdo_word)
        {
            return ((((pdo_word) & BIT(29U)) != 0));
        };

        constexpr static uint32_t get_usb_suspend_support(uint32_t pdo_word)
        {
            return ((((pdo_word) & BIT(28U)) != 0));
        };

        constexpr static uint32_t get_unconstrained_power(uint32_t pdo_word)
        {
            return ((((pdo_word) & BIT(27U)) != 0));
        };

        constexpr static uint32_t get_usb_comm_cap(uint32_t pdo_word)
        {
            return ((((pdo_word) & BIT(26U)) != 0));
        };

        constexpr static uint32_t get_dual_role_data(uint32_t pdo_word)
        {
            return ((((pdo_word) & BIT(25U)) != 0));
        };

        constexpr static uint32_t get_unchunked_msg_support(uint32_t pdo_word)
        {
            return ((((pdo_word) & BIT(24U)) != 0));
        };

        constexpr static def::peak_current get_peak_current(uint32_t pdo_word)
        {
            return ((def::peak_current) (((pdo_word) >> 20U) & 0x3U));
        };

        constexpr static uint32_t get_fixed_current(uint32_t pdo_word)
        {
            return ((((pdo_word) >> 10U) & 0x3ffU) * 10U);
        };

        constexpr static uint32_t get_fixed_voltage(uint32_t pdo_word)
        {
            return ((((pdo_word)) & 0x3ffU) * 50U);
        };

        constexpr static uint32_t get_var_max_voltage(uint32_t pdo_word)
        {
            return ((((pdo_word) >> 20U) & 0x3ffU) * 50U);
        };

        constexpr static uint32_t get_var_min_voltage(uint32_t pdo_word)
        {
            return ((((pdo_word) >> 10U) & 0x3ffU) * 50);
        };

        constexpr static uint32_t get_var_max_current(uint32_t pdo_word)
        {
            return ((((pdo_word) & 0x3ffU) * 10));
        };

        constexpr static uint32_t get_batt_max_voltage(uint32_t pdo_word)
        {
            return ((((pdo_word) >> 20U) & 0x3ffU) * 50U);
        };

        constexpr static uint32_t get_batt_min_voltage(uint32_t pdo_word)
        {
            return ((((pdo_word) >> 10U) & 0x3ffU) * 50U);
        };

        constexpr static uint32_t get_batt_max_power(uint32_t pdo_word)
        {
            return ((((pdo_word) & 0x3ffU) * 10U));
        };

        constexpr static uint32_t get_apdo_max_current(uint32_t pdo_word)
        {
            return ((((pdo_word) & 0x7fU) * 50U));
        };

        constexpr static uint32_t get_apdo_min_voltage(uint32_t pdo_word)
        {
            return ((((pdo_word) >> 8U) & 0x7fU) * 100U);
        };

        constexpr static uint32_t get_apdo_max_voltage(uint32_t pdo_word)
        {
            return ((((pdo_word) >> 16U) & 0x7fU) * 100U);
        };
    };
}
