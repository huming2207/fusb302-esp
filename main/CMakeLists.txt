idf_component_register(SRCS "pdmancer_main.cpp" "pdmancer_main.hpp"
                    "device/tcpc_drv.cpp" "device/tcpc_drv.hpp"
                    "device/drv_fusb302.cpp" "device/drv_fusb302.hpp"
                    "middleware/mid_fusb302.c" "middleware/mid_fusb302.h"
                    "tcpm/tcpm.cpp" "tcpm/tcpm.hpp"
                    "tcpm/pkt_parser.hpp" "tcpm/accept.hpp" "tcpm/get_src_cap.hpp" "tcpm/source_cap.hpp"
                    INCLUDE_DIRS "" "device")