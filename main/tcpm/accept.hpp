#pragma once

#include "pkt_parser.hpp"

namespace protocol::parser
{
    class accept : public pkt_parser
    {
    public:
        esp_err_t decode(uint16_t header, uint32_t *data_objs, size_t len, const tcpc_def::tx_func &tx_cb) override
        {
            if(!detect_pkt_type(proto_def::ACCEPT, header, len)) return ESP_ERR_NOT_SUPPORTED;
        };

        esp_err_t encode(proto_def::header &_header, proto_def::pdo *_pdos, size_t pdo_len,
                        uint16_t *header_out, uint32_t *data_objs, size_t data_obj_len, size_t *len_out) override
        {

        }
    };
}

