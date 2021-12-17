#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_EFI_ENABLED
#define HAL_EFI_ENABLED !HAL_MINIMIZE_FEATURES && BOARD_FLASH_SIZE > 1024
#endif

#ifndef AP_EFI_SCRIPTING_ENABLED
#define AP_EFI_SCRIPTING_ENABLED (HAL_EFI_ENABLED && AP_SCRIPTING_ENABLED)
#endif

#ifndef AP_EFI_LOWEHEISER_ENABLED
#define AP_EFI_LOWEHEISER_ENABLED 0
#endif
