#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <drv_fusb302.hpp>

#define TAG "fusb302-main"

extern "C" void app_main(void)
{
    auto fusb302 = device::fusb302(21, 22, 4);
    fusb302.auto_config_polarity();
}
