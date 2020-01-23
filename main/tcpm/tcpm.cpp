#include <tcpc_drv.hpp>
#include <esp_err.h>
#include <esp_log.h>
#include "tcpm.hpp"

#define TAG "tcpm"

#define PKT_HEADER_MSG_TYPE_MASK        (0x1FU)
#define PKT_HEADER_PORT_DATA_ROLE_MASK  (0x20U)
#define PKT_HEADER_SPEC_REV_MASK        (0xC0U)
#define PKT_HEADER_NUM_DATA_OBJ_POS     (12U)
#define PKT_HEADER_NUM_DATA_OBJ_MASK    (0x7000U)

using namespace protocol;

tcpm::tcpm(device::tcpc &_device) : port_dev(_device)
{
    pdo_list.reserve(7);
    port_dev.on_pkt_received([&]() -> esp_err_t {
        return on_pkt_rx();
    });
}

esp_err_t tcpm::request_fixed_power(uint32_t voltage_mv, uint32_t current_ma)
{
    return 0;
}

esp_err_t tcpm::on_pkt_rx()
{
    // Read packet from FIFO
    uint16_t header = 0;
    uint32_t data_objs[7] = {0 };
    size_t data_obj_cnt = 0;
    auto ret = port_dev.receive_pkt(&header, data_objs, sizeof(data_objs), &data_obj_cnt);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read Rx FIFO");
        return ret;
    }

    // Parse packet header
    pkt_header.num_obj = ((header & PKT_HEADER_NUM_DATA_OBJ_MASK) >> PKT_HEADER_NUM_DATA_OBJ_POS);
    pkt_header.type         = (proto_def::pkt_type)(header & 0xfU);
    pkt_header.data_role    = (proto_def::port_data_role)((uint8_t)(header >> 5U) & 0b1U);
    pkt_header.msg_id       = ((uint8_t)(header >> 9U) & 0b111U);
    pkt_header.revision     = (proto_def::spec_revision)((uint8_t)(header >> 6U) & 0b11U);
    pkt_header.power_role   = (proto_def::port_power_role)((uint8_t)(header >> 8U) & 0b1U);

    return add_pdo(data_objs, pkt_header.num_obj);;
}

esp_err_t tcpm::add_pdo(uint32_t data_obj)
{
    proto_def::pdo new_pdo = {};

    // Step 1: Parse PDO type
    uint32_t type = (data_obj >> 30U) & 0x3U;
    if(type >= proto_def::AUGMENTED_PDO) {
        ESP_LOGW(TAG, "Unsupported PDO detected");
        return ESP_ERR_NOT_SUPPORTED;
    } else {
        new_pdo.type = (proto_def::pdo_type)type;
    }

    // Step 2: Parse according to different types
    switch(new_pdo.type) {
        case proto_def::FIXED_PDO: {
            new_pdo.dual_role_power         = ((data_obj & BIT(29U)) != 0);
            new_pdo.suspend_support         = ((data_obj & BIT(28U)) != 0);
            new_pdo.unconstrained_power     = ((data_obj & BIT(27U)) != 0);
            new_pdo.usb_comm                = ((data_obj & BIT(26U)) != 0);
            new_pdo.dual_role_data          = ((data_obj & BIT(25U)) != 0);
            new_pdo.unchunked_msg_support   = ((data_obj & BIT(24U)) != 0);
            new_pdo.overload                = (proto_def::overload_cap)((data_obj >> 20U) & 0x3U);
            new_pdo.current                 = ((data_obj >> 10U) & 0x3ffU) * 10; // 10mA step
            new_pdo.voltage_max             = ((data_obj) & 0x3ffU) * 50; // 50mV step
            new_pdo.voltage_min             = new_pdo.voltage_max;
            break;
        }

        case proto_def::BATTERY_PDO: {
            new_pdo.voltage_max             = ((data_obj >> 20U) & 0x3ffU) * 50;
            new_pdo.voltage_min             = ((data_obj >> 10U) & 0x3ffU) * 50;
            new_pdo.power                   = ((data_obj & 0x3ffU) * 250);
            break;
        }

        case proto_def::VARIABLE_PDO: {
            new_pdo.voltage_max             = ((data_obj >> 20U) & 0x3ffU) * 50;
            new_pdo.voltage_min             = ((data_obj >> 10U) & 0x3ffU) * 50;
            new_pdo.current                 = ((data_obj & 0x3ffU) * 10);
            break;
        }

        case proto_def::AUGMENTED_PDO: {
            new_pdo.current                 = ((data_obj & 0x7fU) * 50);
            new_pdo.voltage_min             = ((data_obj >> 8U) & 0x7fU) * 100;
            new_pdo.voltage_max             = ((data_obj >> 16U) & 0x7fU) * 100;
            break;
        }
    }

    // Step 3: Append to PDO List
    pdo_list.push_back(new_pdo);
    return ESP_OK;
}

esp_err_t tcpm::add_pdo(const uint32_t *data_objs, uint8_t len)
{
    auto ret = ESP_OK;
    for(uint8_t idx = 0; idx < len; idx++) {
        ret = ret ?: add_pdo(data_objs[idx]);
    }

    return ret;
}

esp_err_t tcpm::on_src_cap_received()
{
    return 0;
}

esp_err_t tcpm::on_request_sent()
{
    return 0;
}

esp_err_t tcpm::on_ps_ready_received()
{
    return 0;
}

void tcpm::sink_fsm_task(void *arg)
{
    auto *ctx = static_cast<tcpm *>(arg);

    // Setup timeout timer
    esp_timer_handle_t timer_handle = nullptr;
    esp_timer_create_args_t timer_config = {};
    timer_config.callback = &tcpm::on_sink_fsm_timeout;
    timer_config.arg = arg;
    timer_config.name = "sink_fsm_tm";
    timer_config.dispatch_method = ESP_TIMER_TASK;

    while (true) {
        if(ctx == nullptr) break;
        auto &curr_state = ctx->sink_fsm[ctx->curr_sink_state];

        // Enable timer
        if (curr_state.timeout_ms > 0) {
            ctx->sink_fsm_ret = esp_timer_start_once(timer_handle, curr_state.timeout_ms);
            if (ctx->sink_fsm_ret != ESP_OK) {
                ESP_LOGE(TAG, "Cannot enable SINK FSM's timeout timer, ret: 0x%x", ctx->sink_fsm_ret);
                ctx->set_sink_state(proto_def::FSM_ERROR);
                return;
            }
        }

        // Run callback
        ctx->sink_fsm_ret = curr_state.cb();
        if (ctx->sink_fsm_ret != ESP_OK) {
            ESP_LOGE(TAG, "SINK State at 0x%x returns: 0x%x", curr_state.curr, ctx->sink_fsm_ret);
            ctx->set_sink_state(proto_def::FSM_ERROR);
            return;
        }

        // Stop timer
        ctx->sink_fsm_ret = esp_timer_stop(timer_handle);
        if (ctx->sink_fsm_ret != ESP_OK) {
            ESP_LOGE(TAG, "Cannot disable SINK FSM's timeout timer, ret: 0x%x", ctx->sink_fsm_ret);
            ctx->set_sink_state(proto_def::FSM_ERROR);
            return;
        }

        // Set next state
        ctx->set_sink_state(curr_state.next);
    }

    vTaskDelete(nullptr);
}

void tcpm::on_sink_fsm_timeout(void *arg)
{

}

void tcpm::set_sink_state(proto_def::pkt_type type)
{
    for(size_t idx = 0; idx < sizeof(sink_fsm); idx++) {
        if (type == sink_fsm[idx].curr) {
            curr_sink_state = idx;
        }
    }
}

