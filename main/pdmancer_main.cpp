#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <drv_fusb302.hpp>
#include "pdmancer_main.hpp"


#define TAG "fusb302-main"

extern "C" void app_main(void)
{
    pd_main main;
    main.start();
    vTaskDelay(portMAX_DELAY);
}


void pd_main::start()
{
    auto fusb302 = device::fusb302(21, 22, 4);
    fusb302.on_pkt_received([&]() -> int {
        fusb302.receive_pkt(&header, data_objs, sizeof(data_objs));
        ESP_LOGI(TAG, "Header: 0x%x, data: 0x%x", header, data_objs[0]);
        return ESP_OK;
    });
    fusb302.auto_config_polarity();
    vTaskDelay(portMAX_DELAY);
}

int pd_main::on_rx()
{
    return 0;
}
