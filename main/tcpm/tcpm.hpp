#pragma once

#include <esp_err.h>

typedef enum {
    TCPM_SOP_ONLY = 0,
    TCPM_SOP_PRIME = 1,
    TCPM_SOP_PRIME_PRIME = 2,
} tcpm_sop_t;


esp_err_t tcpm_init();