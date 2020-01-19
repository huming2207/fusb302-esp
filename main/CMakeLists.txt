idf_component_register(SRCS "pdmancer_main.cpp"
                    "driver/tcpc_drv.cpp" "driver/tcpc_drv.hpp"
                    "driver/drv_fusb302.cpp" "driver/drv_fusb302.hpp"
                    "middleware/mid_fusb302.c" "middleware/mid_fusb302.h"
                    "tcpm/tcpm.cpp" "tcpm/tcpm.hpp"
                    INCLUDE_DIRS "" "driver")