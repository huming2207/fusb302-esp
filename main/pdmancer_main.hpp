#pragma once

#include <esp_err.h>

class pd_main
{
public:
    void start();
    int on_rx();

private:
    uint32_t data_objs[7] = { 0 };
    uint16_t header = 0;
};