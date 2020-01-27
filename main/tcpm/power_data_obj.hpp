#pragma once

#include <cstdint>
#include <esp_err.h>

#include "tcpm_def.hpp"

namespace protocol
{
    class power_data_obj
    {
    private:
        power_data_obj() = default;
        power_data_obj(power_data_obj const &) = default;
        power_data_obj(uint32_t pdo_word);

        uint32_t encode_fixed_pdo();
        uint32_t encode_augmented_pdo();

        esp_err_t decode_pdo(uint32_t pdo_word);

    private:
        bool suspend_support        = false;
        bool unconstrained_power    = false;
        bool usb_comm               = false;
        bool dual_role_data         = false;
        bool dual_role_power        = false;
        bool unchunked_msg_support  = false;
        def::pdo_type pdo_type      = def::FIXED_PDO;
        def::overload_cap overload  = def::OVERLOAD_NONE;
        uint32_t voltage_min        = 0;
        uint32_t voltage_max        = 0;
        uint32_t current            = 0;
        uint32_t power              = 0;
    };
}
