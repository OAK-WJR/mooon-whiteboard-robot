#pragma once
#ifdef __cplusplus
#include <sdkconfig.h>
#if defined(CONFIG_H)
#error "KISS C6 public defaults must precede any other configuration"
#endif
#include "config.example.h"
#include "board_profile_kiss_c6.h"
#endif
