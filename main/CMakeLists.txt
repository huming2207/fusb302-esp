idf_component_register(SRCS "pdmancer_main.cpp" "pdmancer_main.hpp"
                    "device/tcpc_drv.cpp" "device/tcpc_drv.hpp"
                    "device/drv_fusb302.cpp" "device/drv_fusb302.hpp"
                    "middleware/mid_fusb302.c" "middleware/mid_fusb302.h"
                    "tcpm/tcpm.cpp" "tcpm/tcpm.hpp" "tcpm/tcpm_def.hpp"
                    "tcpm/power_data_obj.cpp" "tcpm/power_data_obj.hpp"
                    "tcpm/pd_header.cpp" "tcpm/pd_header.hpp"
                    INCLUDE_DIRS "" "device")