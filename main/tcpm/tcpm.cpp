#include <tcpc_drv.hpp>
#include <esp_err.h>
#include <esp_log.h>
#include "tcpm.hpp"

#define TAG "tcpm"

#define PKT_HEADER_MSG_TYPE_MASK        (0x1FU)
#define PKT_HEADER_PORT_DATA_ROLE_MASK  (0x20U)
#define PKT_HEADER_SPEC_REV_MASK        (0xC0U)

using namespace protocol;

tcpm::tcpm(device::tcpc &_device) : port_dev(_device)
{
    port_dev.on_pkt_received([&]() -> esp_err_t {
        return on_pkt_rx();
    });
}

esp_err_t tcpm::on_pkt_rx()
{
    // Read packet from FIFO
    uint16_t header = 0;
    uint32_t data_obj[7] = { 0 };
    size_t data_obj_cnt = 0;
    auto ret = port_dev.receive_pkt(&header, data_obj, sizeof(data_obj), &data_obj_cnt);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read Rx FIFO");
        return ret;
    }

    // Parse packet
    for (auto &parser : parser_lut) {
        if ((parser.type == (header & PKT_HEADER_MSG_TYPE_MASK)) && (parser.ctrl_packet == (data_obj_cnt == 0))) {

        }
    }

    return ret;
}

esp_err_t tcpm::decode_get_src_cap(uint16_t header, uint32_t *data_objs, size_t len)
{
    return 0;
}

esp_err_t tcpm::encode_get_src_cap(proto_def::header &header, proto_def::pdo *pdos, size_t len)
{
    return 0;
}

esp_err_t tcpm::decode_source_cap(uint16_t header, uint32_t *data_objs, size_t len)
{
    return 0;
}

esp_err_t tcpm::encode_source_cap(proto_def::header &header, proto_def::pdo *pdos, size_t len)
{
    return 0;
}

esp_err_t tcpm::decode_accept(uint16_t header, uint32_t *data_objs, size_t len)
{
    return 0;
}

esp_err_t tcpm::encode_accept(proto_def::header &header, proto_def::pdo *pdos, size_t len)
{
    return 0;
}

esp_err_t tcpm::decode_request(uint16_t header, uint32_t *data_objs, size_t len)
{
    return 0;
}

esp_err_t tcpm::encode_request(proto_def::header &header, proto_def::pdo *pdos, size_t len)
{
    return 0;
}

esp_err_t tcpm::decode_ps_ready(uint16_t header, uint32_t *data_objs, size_t len)
{
    return 0;
}

esp_err_t tcpm::encode_ps_ready(proto_def::header &header, proto_def::pdo *pdos, size_t len)
{
    return 0;
}

esp_err_t tcpm::decode_not_supported(uint16_t header, uint32_t *data_objs, size_t len)
{
    return 0;
}

esp_err_t tcpm::encode_not_supported(proto_def::header &header, proto_def::pdo *pdos, size_t len)
{
    return 0;
}

esp_err_t tcpm::decode_unimplemented(uint16_t header, uint32_t *data_objs, size_t len)
{
    return 0;
}

esp_err_t tcpm::encode_unimplemented(proto_def::header &header, proto_def::pdo *pdos, size_t len)
{
    return 0;
}

