#include "tcpc_drv.h"

esp_err_t tcpc_tx(tcpc_drv_t *drv, uint16_t header, const uint32_t *data_obj, size_t len)
{
    if(drv == NULL) return ESP_ERR_INVALID_ARG;
    return drv->tx_func(header, data_obj, len);
}

esp_err_t tcpc_on_rx(tcpc_drv_t *drv, uint16_t *header, uint32_t *data_obj, size_t *data_obj_len)
{
    if(drv == NULL) return ESP_ERR_INVALID_ARG;
    return ESP_OK;
}

bool tcpc_detect_vbus(tcpc_drv_t *drv)
{
    if(drv == NULL) return false;
    return drv->vbus_detect();
}

esp_err_t tcpc_set_rp(tcpc_drv_t *drv, tcpc_rp_t rp)
{
    if(drv == NULL) return ESP_ERR_INVALID_ARG;
    return drv->set_rp(rp);
}

esp_err_t tcpc_set_cc(tcpc_drv_t *drv, tcpc_cc_pull_t pull)
{
    if(drv == NULL) return ESP_ERR_INVALID_ARG;
    return drv->set_cc(pull);
}

esp_err_t tcpc_get_cc(tcpc_drv_t *drv, tcpc_cc_status_t *status_cc1, tcpc_cc_status_t *status_cc2)
{
    if(drv == NULL) return ESP_ERR_INVALID_ARG;
    return drv->get_cc(status_cc1, status_cc2);
}