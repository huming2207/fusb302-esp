#include <tcpc_drv.hpp>
#include <esp_err.h>
#include <esp_log.h>
#include "tcpm.hpp"

#define TAG "tcpm"

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


    return ret;
}

