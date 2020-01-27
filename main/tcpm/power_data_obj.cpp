#include <cmath>
#include <esp_log.h>
#include <esp_bit_defs.h>

#include "power_data_obj.hpp"

using namespace protocol;

#define TAG "tcpm_pdo"

#define SET_MAXIMUM_CURRENT(val)      ((uint32_t)(((uint16_t)val & 0x3ffU) << 0U))
#define SET_OPERATING_CURRENT(val)    ((uint32_t)(((uint16_t)val & 0x3ffU) << 10U))
#define SET_UNCHUNKED_MSG(bit)        ((uint32_t)(((uint8_t)bit) << 23U))
#define SET_NO_USB_SUSPEND(bit)       ((uint32_t)(((uint8_t)bit) << 24U))
#define SET_USB_COMM_CAP(bit)         ((uint32_t)(((uint8_t)bit) << 25U))
#define SET_CAP_MISMATCH(bit)         ((uint32_t)(((uint8_t)bit) << 26U))
#define SET_GIVEBACK(bit)             ((uint32_t)(((uint8_t)bit) << 27U))
#define SET_OBJ_POSITION(pos)         ((uint32_t)(((uint8_t)pos & 0b111U) << 28U))

power_data_obj::power_data_obj(uint32_t pdo_word)
{
    decode_pdo(pdo_word);
}

uint32_t power_data_obj::encode_fixed_pdo_request(int obj_pos, int op_current)
{
    if (op_current <= 0 || op_current > current) {
        op_current = current;
    }

    if (obj_pos <= 0 || obj_pos > 7) {
        return 0; // Not possible
    }

    uint32_t pdo_word = 0;

    pdo_word |= SET_OBJ_POSITION(obj_pos);
    pdo_word |= SET_GIVEBACK(1);
    pdo_word |= SET_CAP_MISMATCH(0);
    pdo_word |= SET_USB_COMM_CAP(0);
    pdo_word |= SET_NO_USB_SUSPEND(0);
    pdo_word |= SET_UNCHUNKED_MSG(0);
    pdo_word |= SET_MAXIMUM_CURRENT(std::floor(current / 10));
    pdo_word |= SET_OPERATING_CURRENT(std::floor(op_current / 10));

    return pdo_word;
}

uint32_t power_data_obj::encode_augmented_pdo(uint8_t obj_pos)
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

bool power_data_obj::operator==(power_data_obj &other)
{
    return suspend_support          == other.suspend_support &&
            unconstrained_power     == other.unconstrained_power &&
            usb_comm                == other.usb_comm &&
            dual_role_data          == other.dual_role_data &&
            dual_role_power         == other.dual_role_power &&
            unchunked_msg_support   == other.unchunked_msg_support &&
            pdo_type                == other.pdo_type &&
            overload                == other.overload &&
            voltage_min             == other.voltage_min &&
            voltage_max             == other.voltage_max &&
            current                 == other.current &&
            power                   == other.power;
}

bool power_data_obj::operator>(power_data_obj &other)
{
    return voltage_min > other.voltage_min && voltage_max > other.voltage_max;
}

bool power_data_obj::operator<(power_data_obj &other)
{
    return voltage_min < other.voltage_min && voltage_max < other.voltage_max;
}

