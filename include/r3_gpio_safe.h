#pragma once
#include "board_profile_r3.h"
#include "c6_safe_low.h"

namespace r3 {
template<uint64_t PinMask> using SafeLowOutputs = c6_gpio::SafeLowOutputs<PinMask>;
using MotorArmSafeLow = SafeLowOutputs<1ULL << motorArmRequest>;
constexpr uint64_t diagnosticMotorMask() {
    uint64_t mask = 0;
    for (const auto& p : headers) mask |= (1ULL << p.fi) | (1ULL << p.bi);
    return mask;
}
using DiagnosticMotorSafeLow = SafeLowOutputs<diagnosticMotorMask()>;
} // namespace r3
