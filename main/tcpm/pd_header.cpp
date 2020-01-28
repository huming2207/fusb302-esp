#include <esp_bit_defs.h>

#include "pd_header.hpp"
#include "tcpm_def.hpp"

using namespace protocol;

// Ref: USB PD 3.0 Spec, v2, pp. 99
#define GET_EXTENDED(header)                    (((uint8_t)((uint16_t)((header)) >> 15U) & 0b1U) == 0)
#define GET_NUMBER_OF_DATA_OBJ(header)          (((uint8_t)((uint16_t)((header)) >> 12U) & 0b111U))
#define GET_MESSAGE_ID(header)                  (((uint8_t)((uint16_t)((header)) >> 9U) & 0b111U))
#define GET_PORT_POWER_ROLE(header)             ((protocol::def::port_power_role)((uint8_t)((uint16_t)((header)) >> 8U) & 0b1U))
#define GET_SPEC_REVISION(header)               ((protocol::def::spec_revision)((uint8_t)((uint16_t)((header)) >> 6U) & 0b11U))
#define GET_PORT_DATA_ROLE(header)              ((protocol::def::port_data_role)((uint8_t)((uint16_t)((header)) >> 5U) & 0b1U))
#define GET_DATA_MESSAGE_TYPE(header)           ((protocol::def::message_type)(((uint16_t)((header)) & 0b11111U) + protocol::def::DATA_PACKET_BASE))
#define GET_CTRL_MESSAGE_TYPE(header)           ((protocol::def::message_type)((uint16_t)((header)) & 0b11111U))

#define SET_EXTENDED(bit)                       ((((bit) ? 1U : 0U) & 0b1U) << 15U)
#define SET_NUMBER_OF_DATA_OBJ(bit)             (((uint16_t)((bit)) & 0b111U) << 12U)
#define SET_MESSAGE_ID(id)                      (((uint16_t)((id)) & 0b111U) << 9U)
#define SET_PORT_POWER_ROLE(bit)                (((uint16_t)((bit)) & 0b1U) << 8U)
#define SET_SPEC_REVISION(bit)                  (((uint16_t)((bit)) & 0b11U) << 6U)
#define SET_PORT_DATA_ROLE(bit)                 (((uint16_t)((bit)) & 0b1U) << 5U)
#define SET_MESSAGE_TYPE(bit)                   (((uint16_t)((bit)) & 0b11111U))

esp_err_t pd_header::decode(uint16_t header)
{
    if (header == 0 || header == UINT16_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    extended = GET_EXTENDED(header);
    num_obj = GET_NUMBER_OF_DATA_OBJ(header);
    msg_id = GET_MESSAGE_ID(header);
    power_role = GET_PORT_POWER_ROLE(header);
    revision = GET_SPEC_REVISION(header);
    data_role = GET_PORT_DATA_ROLE(header);
    if (num_obj > 0)
        msg_type = GET_DATA_MESSAGE_TYPE(header);
    else
        msg_type = GET_CTRL_MESSAGE_TYPE(header);

    return ESP_OK;
}

uint16_t pd_header::encode()
{
    uint16_t header = 0;
    header |= SET_EXTENDED(extended);
    header |= SET_MESSAGE_ID(msg_id);
    header |= SET_MESSAGE_TYPE(msg_type);
    header |= SET_PORT_DATA_ROLE(data_role);
    header |= SET_PORT_POWER_ROLE(power_role);
    header |= SET_SPEC_REVISION(revision);
    header |= SET_NUMBER_OF_DATA_OBJ(num_obj);

    return header;
}

pd_header::pd_header(uint16_t header)
{
    decode(header);
}
