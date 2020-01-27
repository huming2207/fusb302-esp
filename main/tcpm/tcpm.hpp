#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <tcpc_drv.hpp>

#include "tcpm_def.hpp"
#include "power_data_obj.hpp"

namespace protocol::def
{
    struct header {
        uint8_t num_obj = 0;
        uint8_t msg_id = 0;
        spec_revision revision = REV_2_0;
        port_data_role data_role = DFP;
        port_power_role power_role = SINK;
        pkt_type type = GOOD_CRC;
    };
}

namespace protocol
{
    class tcpm
    {
    public:
        explicit tcpm(device::tcpc& _device);
        esp_err_t on_pkt_rx();
        esp_err_t perform_sink(uint32_t timeout_ms = 1000);
        esp_err_t send_request_fixed(uint32_t voltage_mv, uint32_t current_ma);

    private:
        esp_err_t wait_src_cap(uint32_t timeout_ms = 1000);
        esp_err_t send_get_src_cap();
        esp_err_t send_request();

    private:
        esp_err_t add_pdo(uint32_t data_obj);
        esp_err_t add_pdo(const uint32_t *data_objs, uint8_t len);

    private:
        device::tcpc& port_dev;

        def::header pkt_header = {};
        std::vector<power_data_obj> pdo_list;

        QueueHandle_t msg_queue = nullptr;
        uint8_t msg_id = 0;
        def::spec_revision pd_rev = def::REV_3_0;
    };
}