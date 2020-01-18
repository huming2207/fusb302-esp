idf_component_register(SRCS "pdmancer_main.c"
                    "driver/tcpc_drv.c" "driver/tcpc_drv.h"
                    "driver/drv_fusb302.c" "driver/drv_fusb302.h"
                    "middleware/mid_fusb302.c" "middleware/mid_fusb302.h"
                    "tcpm/tcpm.c" "tcpm/tcpm.h"
                    INCLUDE_DIRS "" "driver")