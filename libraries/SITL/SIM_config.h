#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_SIM_ADSB_ENABLED
#define HAL_SIM_ADSB_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL) && !defined(HAL_BUILD_AP_PERIPH)
#endif

#ifndef HAL_SIM_SERIALPROXIMITYSENSOR_ENABLED
#define HAL_SIM_SERIALPROXIMITYSENSOR_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef HAL_SIM_PS_RPLIDARA1_ENABLED
#define HAL_SIM_PS_RPLIDARA1_ENABLED HAL_SIM_SERIALPROXIMITYSENSOR_ENABLED
#endif

#ifndef HAL_SIM_PS_RPLIDARA2_ENABLED
#define HAL_SIM_PS_RPLIDARA2_ENABLED HAL_SIM_SERIALPROXIMITYSENSOR_ENABLED
#endif

#ifndef AP_SIM_IS31FL3195_ENABLED
#define AP_SIM_IS31FL3195_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef AP_SIM_SHIP_ENABLED
#define AP_SIM_SHIP_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef AP_SIM_TSYS03_ENABLED
#define AP_SIM_TSYS03_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef AP_SIM_ADSB_SAGETECH_MXS_ENABLED
#define AP_SIM_ADSB_SAGETECH_MXS_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif
