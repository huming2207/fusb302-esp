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

    // Initialise message queue
    msg_queue = xQueueCreate(10, sizeof(proto_def::pkt_type));
}

esp_err_t tcpm::request_fixed_power(uint32_t voltage_mv, uint32_t current_ma)
{
    return 0;
}

esp_err_t tcpm::on_pkt_rx()
{
    // Read packet from FIFO
    uint16_t header = 0;
    uint32_t data_objs[7] = { 0 };
    size_t data_obj_cnt = 0;
    auto ret = port_dev.receive_pkt(&header, data_objs, sizeof(data_objs), &data_obj_cnt);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read Rx FIFO");
        return ret;
    }

    // Parse packet header
    pkt_header.num_obj = ((header & PKT_HEADER_NUM_DATA_OBJ_MASK) >> PKT_HEADER_NUM_DATA_OBJ_POS);
    if(pkt_header.num_obj > 0) pkt_header.type = (proto_def::pkt_type)((header & 0x1fU) + proto_def::DATA_PACKET_BASE);
    else pkt_header.type = (proto_def::pkt_type)(header & 0x1fU);
    pkt_header.data_role    = (proto_def::port_data_role)((uint8_t)(header >> 5U) & 0b1U);
    pkt_header.msg_id       = ((uint8_t)(header >> 9U) & 0b111U);
    pkt_header.revision     = (proto_def::spec_revision)((uint8_t)(header >> 6U) & 0b11U);
    pkt_header.power_role   = (proto_def::port_power_role)((uint8_t)(header >> 8U) & 0b1U);

    if(xQueueSend(msg_queue, (const void *)&pkt_header.type, pdMS_TO_TICKS(5000)) != pdPASS) {
        return ESP_ERR_TIMEOUT; // Shouldn't reach here though...
    }

    // Refresh PDO list when Src Cap packet is received
    if(pkt_header.type == proto_def::SOURCE_CAPABILITIES) {
        pdo_list.clear();
        ret = add_pdo(data_objs, pkt_header.num_obj);
    }

    return ret;
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
    if(pdo_list.size() < 7) {
        pdo_list.push_back(new_pdo);
        return ESP_OK;
    } else {
        return ESP_ERR_NO_MEM;
    }
}

esp_err_t tcpm::add_pdo(const uint32_t *data_objs, uint8_t len)
{
    auto ret = ESP_OK;
    for(uint8_t idx = 0; idx < len; idx++) {
        ret = ret ?: add_pdo(data_objs[idx]);
    }

    return ret;
}

esp_err_t tcpm::perform_sink(uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    proto_def::pkt_type rx_type;

    // Step 1. Wait for Source Capabilities.
    ret = wait_src_cap(timeout_ms);
    if (ret != ESP_OK) {

        // If no luck, ask for it.
        ret = send_get_src_cap();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send Src_Cap");
            return ret;
        }

        // Try again?
        ret = wait_src_cap(timeout_ms);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Still no Src_Cap received after Get_Src_Cap sent");
            return ret;
        }
    }

    // Step 2: Send REQUEST
    ret = send_request();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot send REQUEST packet");
        return ret;
    }

    // Step 3: Wait for ACCEPT
    if (xQueueReceive(msg_queue, &rx_type, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGE(TAG, "ACCEPT packet receive timeout");
        return ESP_ERR_TIMEOUT;
    }

    if (rx_type != proto_def::ACCEPT) {
        ESP_LOGE(TAG, "Received packet type is not ACCEPT: 0x%x", rx_type);
        return ESP_ERR_INVALID_STATE;
    }

    // Step 4: Wait for PS_READY
    if (xQueueReceive(msg_queue, &rx_type, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGE(TAG, "ACCEPT packet receive timeout");
        return ESP_ERR_TIMEOUT;
    }

    if (rx_type != proto_def::PS_READY) {
        ESP_LOGE(TAG, "Received packet type is not PS_READY: 0x%x", rx_type);
        return ESP_ERR_INVALID_STATE;
    }

    return ret;
}

esp_err_t tcpm::send_get_src_cap()
{
    return 0;
}

esp_err_t tcpm::send_request()
{
    return 0;
}

esp_err_t tcpm::wait_src_cap(uint32_t timeout_ms)
{
    proto_def::pkt_type rx_type;

    if (xQueueReceive(msg_queue, &rx_type, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "No message received in %u ms", timeout_ms);
        return ESP_ERR_TIMEOUT;
    }

    if (rx_type != proto_def::SOURCE_CAPABILITIES) {
        ESP_LOGE(TAG, "Message received but it's not Src_Cap: 0x%x", rx_type);
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

