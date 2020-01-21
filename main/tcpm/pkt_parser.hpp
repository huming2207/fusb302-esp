#pragma once

#include <cstdint>
#include <cstddef>
#include <esp_err.h>
#include <esp_log.h>
#include <functional>
#include "tcpm.hpp"

#define PKT_HEADER_MSG_TYPE_MASK        (0x1FU)
#define PKT_HEADER_PORT_DATA_ROLE_MASK  (0x20U)
#define PKT_HEADER_SPEC_REV_MASK        (0xC0U)

namespace protocol::parser
{
    class pkt_parser
    {
    public:
        virtual esp_err_t decode(uint16_t header, uint32_t *data_objs,
                                size_t len, const tcpc_def::tx_func &tx_cb) = 0;
        virtual esp_err_t encode(proto_def::header &header, proto_def::pdo *pdos, size_t pdo_len,
                                uint16_t *header_out, uint32_t *data_objs, size_t data_obj_len, size_t *len_out) = 0;

    protected:
        static bool detect_pkt_type(proto_def::pkt_type expected, uint16_t header, size_t len)
        {
            // Detect control packet validness
            // Data frames is negative, message length should be more than 1;
            // Control frame is positive, message length should be 0.
            if((expected >= proto_def::GOOD_CRC && len != 0) ||
                (expected <= proto_def::SOURCE_CAPABILITIES && len == 0)) return false;

            // Match packet type
            return ((header & PKT_HEADER_MSG_TYPE_MASK) == ((len == 0) ? expected : -expected));
        };
    };
}



