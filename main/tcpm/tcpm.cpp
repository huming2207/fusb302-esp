#include <tcpc_drv.hpp>
#include "tcpm.hpp"

using namespace protocol;

tcpm::tcpm(device::tcpc &_device) : port_dev(_device)
{
    port_dev.on_pkt_received([&]() -> int {
        on_pkt_rx();
        return ESP_OK;
    });
}

int tcpm::on_pkt_rx()
{
    // Read packet from FIFO
    uint16_t header = 0;
    uint32_t data_obj[7] = { 0 };
    size_t data_obj_cnt = 0;
    auto ret = port_dev.receive_pkt(&header, data_obj, sizeof(data_obj), &data_obj_cnt);

    // Parse packet


    return ret;
}

