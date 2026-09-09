#pragma once
#include "board_profile_kiss_c6.h"
#include "c6_safe_low.h"
namespace kiss_c6 {
using MotorArmSafeLow = c6_gpio::SafeLowOutputs<1ULL << motorArmRequest>;
}
