#include <cmath>
#include <esp_log.h>
#include <esp_bit_defs.h>

#include "power_data_obj.hpp"

using namespace protocol;

#define TAG "tcpm_pdo"

#define SET_MAXIMUM_CURRENT(val)      ((uint32_t)(((uint16_t)(val / 10U) & 0x3ffU) << 0U))
#define SET_OPERATING_CURRENT(val)    ((uint32_t)(((uint16_t)(val / 10U) & 0x3ffU) << 10U))
#define SET_UNCHUNKED_MSG(bit)        ((uint32_t)(((uint8_t)bit) << 23U))
#define SET_NO_USB_SUSPEND(bit)       ((uint32_t)(((uint8_t)bit) << 24U))
#define SET_USB_COMM_CAP(bit)         ((uint32_t)(((uint8_t)bit) << 25U))
#define SET_CAP_MISMATCH(bit)         ((uint32_t)(((uint8_t)bit) << 26U))
#define SET_GIVEBACK(bit)             ((uint32_t)(((uint8_t)bit) << 27U))
#define SET_OBJ_POSITION(pos)         ((uint32_t)(((uint8_t)pos & 0b111U) << 28U))

#define GET_DUAL_ROLE_POWER(pdo_word)           ((((pdo_word) & BIT(29U)) != 0))
#define GET_USB_SUSPEND_SUPPORT(pdo_word)       ((((pdo_word) & BIT(28U)) != 0))
#define GET_UNCONSTRAINED_POWER(pdo_word)       ((((pdo_word) & BIT(27U)) != 0))
#define GET_USB_COMM_CAP(pdo_word)              ((((pdo_word) & BIT(26U)) != 0))
#define GET_DUAL_ROLE_DATA(pdo_word)            ((((pdo_word) & BIT(25U)) != 0))
#define GET_UNCHUNKED_MSG_SUPPORT(pdo_word)     ((((pdo_word) & BIT(24U)) != 0))
#define GET_PEAK_CURRENT(pdo_word)              ((def::overload_cap)(((pdo_word) >> 20U) & 0x3U))
#define GET_FIXED_CURRENT(pdo_word)             ((((pdo_word) >> 10U) & 0x3ffU) * 10U)
#define GET_FIXED_VOLTAGE(pdo_word)             ((((pdo_word)) & 0x3ffU) * 50U)
#define GET_VAR_MAX_VOLTAGE(pdo_word)           (((pdo_word >> 20U) & 0x3ffU) * 50U)
#define GET_VAR_MIN_VOLTAGE(pdo_word)           (((pdo_word >> 10U) & 0x3ffU) * 50)
#define GET_VAR_MAX_CURRENT(pdo_word)           (((pdo_word & 0x3ffU) * 10))
#define GET_BATT_MAX_VOLTAGE(pdo_word)          (((pdo_word >> 20U) & 0x3ffU) * 50U)
#define GET_BATT_MIN_VOLTAGE(pdo_word)          (((pdo_word >> 10U) & 0x3ffU) * 50U)
#define GET_BATT_MAX_POWER(pdo_word)            (((pdo_word & 0x3ffU) * 10U))
#define GET_APDO_MAX_CURRENT(pdo_word)          (((pdo_word & 0x7fU) * 50U))
#define GET_APDO_MIN_VOLTAGE(pdo_word)          (((pdo_word >> 8U) & 0x7fU) * 100U)
#define GET_APDO_MAX_VOLTAGE(pdo_word)          (((pdo_word >> 16U) & 0x7fU) * 100U)

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
    pdo_word |= SET_MAXIMUM_CURRENT(current);
    pdo_word |= SET_OPERATING_CURRENT(op_current);

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
            dual_role_power         = GET_DUAL_ROLE_POWER(pdo_word);
            suspend_support         = GET_USB_SUSPEND_SUPPORT(pdo_word);
            unconstrained_power     = GET_UNCONSTRAINED_POWER(pdo_word);
            usb_comm                = GET_USB_COMM_CAP(pdo_word);
            dual_role_data          = GET_DUAL_ROLE_DATA(pdo_word);
            unchunked_msg_support   = GET_UNCHUNKED_MSG_SUPPORT(pdo_word);
            peak_current_lvl        = GET_PEAK_CURRENT(pdo_word);
            current                 = GET_FIXED_CURRENT(pdo_word); // 10mA step
            voltage_max             = GET_FIXED_VOLTAGE(pdo_word); // 50mV step
            voltage_min             = voltage_max;
            break;
        }

        case def::BATTERY_PDO: {
            voltage_max             = GET_BATT_MAX_VOLTAGE(pdo_word);
            voltage_min             = GET_BATT_MIN_VOLTAGE(pdo_word);
            power                   = GET_BATT_MAX_POWER(pdo_word);
            break;
        }

        case def::VARIABLE_PDO: {
            voltage_max             = GET_VAR_MAX_VOLTAGE(pdo_word);
            voltage_min             = GET_VAR_MIN_VOLTAGE(pdo_word);
            current                 = GET_VAR_MAX_CURRENT(pdo_word);
            break;
        }

        case def::AUGMENTED_PDO: {
            current                 = GET_APDO_MAX_CURRENT(pdo_word);
            voltage_min             = GET_APDO_MIN_VOLTAGE(pdo_word);
            voltage_max             = GET_APDO_MAX_VOLTAGE(pdo_word);
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
           peak_current_lvl == other.peak_current_lvl &&
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

