#include <esp_log.h>
#include <esp_bit_defs.h>

#include "power_data_obj.hpp"

using namespace protocol;

#define TAG "tcpm_pdo"

power_data_obj::power_data_obj(uint32_t pdo_word)
{
    decode_pdo(pdo_word);
}

uint32_t power_data_obj::encode_fixed_pdo()
{
    return 0;
}

uint32_t power_data_obj::encode_augmented_pdo()
{
    return 0;
}

esp_err_t power_data_obj::decode_pdo(uint32_t pdo_word)
{
    // Step 1: Parse PDO type
    uint32_t type = (pdo_word >> 30U) & 0x3U;
    if(type >= def::AUGMENTED_PDO) {
        ESP_LOGW(TAG, "Unsupported PDO detected: %u", type);
        return ESP_ERR_NOT_SUPPORTED;
    } else {
        pdo_type = (def::pdo_type)type;
    }

    // Step 2: Parse according to different types
    switch(pdo_type) {
        case def::FIXED_PDO: {
            dual_role_power         = ((pdo_word & BIT(29U)) != 0);
            suspend_support         = ((pdo_word & BIT(28U)) != 0);
            unconstrained_power     = ((pdo_word & BIT(27U)) != 0);
            usb_comm                = ((pdo_word & BIT(26U)) != 0);
            dual_role_data          = ((pdo_word & BIT(25U)) != 0);
            unchunked_msg_support   = ((pdo_word & BIT(24U)) != 0);
            overload                = (def::overload_cap)((pdo_word >> 20U) & 0x3U);
            current                 = ((pdo_word >> 10U) & 0x3ffU) * 10; // 10mA step
            voltage_max             = ((pdo_word) & 0x3ffU) * 50; // 50mV step
            voltage_min             = voltage_max;
            break;
        }

        case def::BATTERY_PDO: {
            voltage_max             = ((pdo_word >> 20U) & 0x3ffU) * 50;
            voltage_min             = ((pdo_word >> 10U) & 0x3ffU) * 50;
            power                   = ((pdo_word & 0x3ffU) * 250);
            break;
        }

        case def::VARIABLE_PDO: {
            voltage_max             = ((pdo_word >> 20U) & 0x3ffU) * 50;
            voltage_min             = ((pdo_word >> 10U) & 0x3ffU) * 50;
            current                 = ((pdo_word & 0x3ffU) * 10);
            break;
        }

        case def::AUGMENTED_PDO: {
            current                 = ((pdo_word & 0x7fU) * 50);
            voltage_min             = ((pdo_word >> 8U) & 0x7fU) * 100;
            voltage_max             = ((pdo_word >> 16U) & 0x7fU) * 100;
            break;
        }
    }

    return ESP_OK;
}

