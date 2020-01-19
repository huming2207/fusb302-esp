#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <drv_fusb302.hpp>

extern "C" void app_main(void)
{
    drv::fusb302(21, 22, 4);
}
