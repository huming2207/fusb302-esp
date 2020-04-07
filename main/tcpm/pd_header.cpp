#include <esp_bit_defs.h>

#include "pd_header.hpp"
#include "tcpm_def.hpp"

using namespace protocol;

esp_err_t pd_header::decode(uint16_t header)
{
    if (header == 0 || header == UINT16_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    extended = get_extended(header);
    num_obj = get_number_of_data_obj(header);
    msg_id = get_msg_id(header);
    power_role = get_port_power_role(header);
    revision = get_spec_rev(header);
    data_role = get_port_data_role(header);
    if (num_obj > 0)
        msg_type = get_data_msg_type(header);
    else
        msg_type = get_ctrl_msg_type(header);

    return ESP_OK;
}

uint16_t pd_header::encode()
{
    uint16_t header = 0;
    header |= set_extended(extended);
    header |= set_msg_id(msg_id);
    header |= set_msg_type(msg_type);
    header |= set_port_data_role(data_role);
    header |= set_port_power_role(power_role);
    header |= set_spec_rev(revision);
    header |= set_number_of_data_obj(num_obj);

    return header;
}

pd_header::pd_header(uint16_t header)
{
    decode(header);
}

bool pd_header::is_ctrl_msg()
{
    return num_obj == 0;
}
