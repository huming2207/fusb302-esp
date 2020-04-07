#include <cmath>
#include <esp_log.h>
#include "power_data_obj.hpp"

using namespace protocol;

#define TAG "tcpm_pdo"


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

    pdo_word |= set_obj_position(obj_pos);
    pdo_word |= set_giveback(1);
    pdo_word |= set_cap_mismatch(0);
    pdo_word |= set_usb_comm_cap(0);
    pdo_word |= set_no_usb_suspend(0);
    pdo_word |= set_unchunked_msg(0);
    pdo_word |= set_fixed_current(current);
    pdo_word |= set_fixed_voltage(voltage_max);

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
            dual_role_power         = get_dual_role_power(pdo_word);
            suspend_support         = get_usb_suspend_support(pdo_word);
            unconstrained_power     = get_unconstrained_power(pdo_word);
            usb_comm                = get_usb_comm_cap(pdo_word);
            dual_role_data          = get_dual_role_data(pdo_word);
            unchunked_msg_support   = get_unchunked_msg_support(pdo_word);
            peak_curr_lvl           = get_peak_current(pdo_word);
            current                 = get_fixed_current(pdo_word); // 10mA step
            voltage_max             = get_fixed_voltage(pdo_word); // 50mV step
            voltage_min             = voltage_max;
            break;
        }

        case def::BATTERY_PDO: {
            voltage_max             = get_batt_max_voltage(pdo_word);
            voltage_min             = get_batt_min_voltage(pdo_word);
            power                   = get_batt_max_power(pdo_word);
            break;
        }

        case def::VARIABLE_PDO: {
            voltage_max             = get_var_max_voltage(pdo_word);
            voltage_min             = get_var_min_voltage(pdo_word);
            current                 = get_var_max_current(pdo_word);
            break;
        }

        case def::AUGMENTED_PDO: {
            current                 = get_apdo_max_current(pdo_word);
            voltage_min             = get_apdo_min_voltage(pdo_word);
            voltage_max             = get_apdo_max_voltage(pdo_word);
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
            peak_curr_lvl           == other.peak_curr_lvl &&
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

