#pragma once
// Pre-included by the explicit PlatformIO environment, including framework C.
#ifdef __cplusplus
#include <sdkconfig.h>
#if defined(CONFIG_H)
#error "Pico public defaults must be loaded before any other configuration"
#endif
#if defined(WBB_TOP_BOARD_R3)
#error "Pico and R3 profiles are mutually exclusive"
#endif
#include "config.example.h"
#include "board_profile_pico.h"
#endif
