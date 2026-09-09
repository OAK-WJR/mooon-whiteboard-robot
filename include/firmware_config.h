#pragma once
// Board selection precedes any legacy/private configuration. Pico builds never
// include config.h, even when it exists with pins from another board.
#if defined(WBB_TOP_BOARD_PICO) && defined(WBB_TOP_BOARD_R3)
#error "Pico and R3 profiles are mutually exclusive"
#endif
#if defined(WBB_TOP_BOARD_KISS_C6)
#include "config.example.h"
#include "board_profile_kiss_c6.h"
#elif defined(WBB_TOP_BOARD_PICO)
#include "pico_build_config.h"
#elif defined(WBB_TOP_BOARD_R3)
#include "config.example.h"
#include "board_profile_r3.h"
#elif defined(CONFIG_IDF_TARGET_ESP32S3) && defined(WBB_TOP_BOARD_V2)
#error "The S3 integrated runtime requires explicit WBB_TOP_BOARD_PICO selection"
#else
#include "config.h"
#endif
