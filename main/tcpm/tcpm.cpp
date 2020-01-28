#include <tcpc_drv.hpp>
#include <esp_err.h>
#include <esp_log.h>
#include "tcpm.hpp"
#include "pd_header.hpp"

#define TAG "tcpm"

using namespace protocol;

tcpm::tcpm(device::tcpc &_device) : port_dev(_device)
{
    pdo_list.reserve(7);
    port_dev.on_pkt_received([&]() -> esp_err_t {
        return on_pkt_rx();
    });

    // Initialise message queue
    msg_queue = xQueueCreate(10, sizeof(def::message_type));
}


esp_err_t tcpm::on_pkt_rx()
{
    // Read packet from FIFO
    pd_header pkt_header;
    uint16_t header_raw = 0;
    uint32_t data_objs[7] = { 0 };
    size_t data_obj_cnt = 0;
    auto ret = port_dev.receive_pkt(&header_raw, data_objs, sizeof(data_objs), &data_obj_cnt);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read Rx FIFO");
        return ret;
    }

    // Parse packet header
    ret = pkt_header.decode(header_raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to decode header: 0x%x", header_raw);
        return ret;
    }

    if(xQueueSend(msg_queue, (const void *)&pkt_header.msg_type, pdMS_TO_TICKS(5000)) != pdPASS) {
        return ESP_ERR_TIMEOUT; // Shouldn't reach here though...
    }

    // Refresh PDO list when Src Cap packet is received
    if(pkt_header.msg_type == def::SOURCE_CAPABILITIES) {
        pdo_list.clear();
        ret = add_pdo(data_objs, pkt_header.num_obj);
    }

    return ret;
}

esp_err_t tcpm::add_pdo(uint32_t data_obj)
{
    // Step 1: Parse PDO type
    power_data_obj new_pdo;

    // Step 2: Parse PDO word
    auto ret = new_pdo.decode_pdo(data_obj);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot decode PDO: 0x%x", data_obj);
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
    def::message_type rx_type;

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

    if (rx_type != def::ACCEPT) {
        ESP_LOGE(TAG, "Received packet type is not ACCEPT: 0x%x", rx_type);
        return ESP_ERR_INVALID_STATE;
    }

    // Step 4: Wait for PS_READY
    if (xQueueReceive(msg_queue, &rx_type, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGE(TAG, "ACCEPT packet receive timeout");
        return ESP_ERR_TIMEOUT;
    }

    if (rx_type != def::PS_READY) {
        ESP_LOGE(TAG, "Received packet type is not PS_READY: 0x%x", rx_type);
        return ESP_ERR_INVALID_STATE;
    }

    return ret;
}

esp_err_t tcpm::send_request_fixed(uint32_t voltage_mv, uint32_t current_ma)
{
    // Step 1: Find object position
    int idx = -1;
    for (auto it = pdo_list.begin(); it != pdo_list.end(); ++it) {
        if (it->voltage_max == voltage_mv && it->current >= current_ma &&
            it->pdo_type == def::FIXED_PDO) {
            idx = (it - pdo_list.begin()) + 1; // Object position in PD Spec starts from 1, not 0!
            break;
        }
    }


    if (idx < 0) {
        ESP_LOGE(TAG, "PDO Not found with %u mV / %u mA", voltage_mv, current_ma);
        return ESP_ERR_NOT_FOUND;
    }

    // Step 2: Encode and assign PDO index
    uint32_t pdo_word = pdo_list[idx].encode_fixed_pdo_request(idx, current_ma);
    if (pdo_word == 0) {
        ESP_LOGE(TAG, "Invalid PDO encoded: 0x%x", pdo_word);
        return ESP_ERR_INVALID_STATE;
    }

    // Step 3: Send out PDO word


    return ESP_OK;
}

esp_err_t tcpm::send_get_src_cap()
{
    uint16_t header = 0;
    header |= (msg_id & 0b111U) << 9U;                      // Set MessageID
    header |= (pd_rev & 0b11U) << 6U;                       // Set PD Revision
    header |= (def::GET_SOURCE_CAP & 0b11111U) << 0U; // Set Message Type

    return port_dev.transmit_pkt(header, nullptr, 0);
}

esp_err_t tcpm::send_request()
{
    return 0;
}

esp_err_t tcpm::wait_src_cap(uint32_t timeout_ms)
{
    def::message_type rx_type;

    if (xQueueReceive(msg_queue, &rx_type, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "No message received in %u ms", timeout_ms);
        return ESP_ERR_TIMEOUT;
    }

    if (rx_type != def::SOURCE_CAPABILITIES) {
        ESP_LOGE(TAG, "Message received but it's not Src_Cap: 0x%x", rx_type);
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

