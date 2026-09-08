#pragma once
// PlatformIO -include runs before every translation unit, including framework C.
#ifdef __cplusplus
#include <sdkconfig.h>
#if defined(CONFIG_H)
#error "R3 public defaults must be loaded before any local config.h"
#endif
#include "config.example.h"
#include "board_profile_r3.h"
#if !defined(WBB_TOP_BOARD_V2) && !defined(WBB_R3_DIAGNOSTIC)
#error "R3 must use the gated integrated runtime or the read-only diagnostic"
#endif
#ifdef MOTOR_TEST_MODE
#error "R3 is unqualified: the motor test entry point is forbidden"
#endif
#endif
