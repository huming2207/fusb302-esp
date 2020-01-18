#pragma once

#include <esp_err.h>
#include <esp_http_server.h>
#include "../tcpm/tcpm.h"

typedef enum {
    TCPC_RP_USB = 0,
    TCPC_RP_1A5 = 1,
    TCPC_RP_3A0 = 2
} tcpc_rp_t;

typedef enum  {
    TCPC_VOLT_OPEN = 0,
    TCPC_VOLT_RA = 1,	  /* Port partner is applying Ra */
    TCPC_VOLT_RD = 2,	  /* Port partner is applying Rd */
    TCPC_VOLT_RP_DEF = 5, /* Port partner is applying Rp (0.5A) */
    TCPC_VOLT_RP_1_5 = 6, /* Port partner is applying Rp (1.5A) */
    TCPC_VOLT_RP_3_0 = 7, /* Port partner is applying Rp (3.0A) */
} tcpc_cc_status_t;

typedef enum {
    TCPC_TX_SOP = 0,
    TCPC_TX_SOP_PRIME = 1,
    TCPC_TX_SOP_PRIME_PRIME = 2,
    TCPC_TX_SOP_DEBUG_PRIME = 3,
    TCPC_TX_SOP_DEBUG_PRIME_PRIME = 4,
    TCPC_TX_HARD_RESET = 5,
    TCPC_TX_CABLE_RESET = 6,
    TCPC_TX_BIST_MODE_2 = 7
} tcpc_tx_t;

typedef enum {
    TCPC_CC_RA = 0,
    TCPC_CC_RP = 1,
    TCPC_CC_RD = 2,
    TCPC_CC_OPEN = 3,
    TCPC_CC_RA_RD = 4, /* Powered cable with Sink */
} tcpc_cc_pull_t;

typedef esp_err_t (*tcpc_tx_func_t)(const uint16_t header, const uint32_t *data_obj, size_t len);
typedef esp_err_t (*tcpc_rx_cb_t)(uint16_t *header, uint32_t *data_obj, size_t *data_obj_len);
typedef bool (*tcpc_detect_vbus_func_t)();
typedef esp_err_t (*tcpc_set_rp_func_t)(tcpc_rp_t rp);
typedef esp_err_t (*tcpc_set_cc_func_t)(tcpc_cc_pull_t pull);
typedef esp_err_t (*tcpc_get_cc_func_t)(tcpc_cc_status_t *status_cc1, tcpc_cc_status_t *status_cc2);

typedef struct {
    tcpc_tx_func_t tx_func;
    tcpc_rx_cb_t rx_cb;
    tcpc_detect_vbus_func_t vbus_detect;
    tcpc_set_rp_func_t set_rp;
    tcpc_get_cc_func_t get_cc;
    tcpc_set_cc_func_t set_cc;
} tcpc_drv_t;

esp_err_t tcpc_tx(tcpc_drv_t *drv, uint16_t header, const uint32_t *data_obj, size_t len);
esp_err_t tcpc_on_rx(tcpc_drv_t *drv, uint16_t *header, uint32_t *data_obj, size_t *data_obj_len);
bool tcpc_detect_vbus(tcpc_drv_t *drv);
esp_err_t tcpc_set_rp(tcpc_drv_t *drv, tcpc_rp_t rp);
esp_err_t tcpc_set_cc(tcpc_drv_t *drv, tcpc_cc_pull_t pull);
esp_err_t tcpc_get_cc(tcpc_drv_t *drv, tcpc_cc_status_t *status_cc1, tcpc_cc_status_t *status_cc2);
