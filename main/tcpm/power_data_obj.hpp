#pragma once

#include <cstdint>
#include <esp_err.h>

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

        bool operator== (power_data_obj &other);
        bool operator> (power_data_obj &other);
        bool operator< (power_data_obj &other);

    public:
        bool suspend_support        = false;
        bool unconstrained_power    = false;
        bool usb_comm               = false;
        bool dual_role_data         = false;
        bool dual_role_power        = false;
        bool unchunked_msg_support  = false;
        def::pdo_type pdo_type      = def::FIXED_PDO;
        def::peak_current peak_curr_lvl  = def::PEAK_CURRENT_NONE;
        uint32_t voltage_min        = 0;
        uint32_t voltage_max        = 0;
        uint32_t current            = 0;
        uint32_t power              = 0;
    };
}
