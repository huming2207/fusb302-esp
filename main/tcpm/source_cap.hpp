#pragma once

#include <esp_log.h>
#include "pkt_parser.hpp"

#define TAG "src_cap"

namespace protocol::parser
{
    class source_cap : public pkt_parser
    {
    public:
        esp_err_t decode(uint16_t header, uint32_t *data_objs, size_t len, const tcpc_def::tx_func &tx_cb) override
        {
            if(!detect_pkt_type(proto_def::SOURCE_CAPABILITIES, header, len)) return ESP_ERR_NOT_SUPPORTED;
        };

        esp_err_t encode(proto_def::header &header, proto_def::pdo *pdos, size_t pdo_len,
                         uint16_t *header_out, uint32_t *data_objs, size_t data_obj_len, size_t *len_out) override
        {

        }
    };
}
