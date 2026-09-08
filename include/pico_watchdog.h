#pragma once
#include "pico_gpio_safe.h"

namespace pico {
// TPS3851H33E: CWD=10k to VDD, SET1=VDD, falling-edge WDI, tWD=170..230ms.
// Its outputs clear the safety latch, NOT MCU RUN. Startup/ROM need no pulses.
// Only the control task calls this service, after completed work. No timer/ISR.
class ExternalWatchdog {
public:
    static constexpr uint32_t startupLimitMs = 5000;
    static constexpr uint32_t pulsePeriodMs = 40;
    static constexpr uint32_t progressLimitMs = 100;
    static constexpr uint32_t pulseLimitMs = 120; // 50ms before minimum tWD.
    static constexpr uint32_t acquisitionMs = 500; // Exceeds tRST(max)+next pulse.
    enum class State { Startup, Acquiring, Active, Stale, InitFailed, GpioFault };

    explicit ExternalWatchdog(SafeDefaults& pins) : pins_(pins) {}
    void begin(uint32_t now) {
        if (begun_) return;
        begun_ = true;
        startedAt_ = now;
        if (!pins_.ready()) stop(State::GpioFault);
    }
    void finishStartup(uint32_t now, bool healthy) {
        if (!begun_ || state_ != State::Startup) return;
        if (!healthy || uint32_t(now - startedAt_) >= startupLimitMs) {
            stop(State::InitFailed);
            return;
        }
        state_ = State::Acquiring;
        lastProgressAt_ = now;
    }
    bool startupExpired(uint32_t now) const {
        return begun_ && state_ == State::Startup && uint32_t(now - startedAt_) >= startupLimitMs;
    }
    void check(uint32_t now) {
        if (!pins_.ready()) { stop(State::GpioFault); return; }
        if ((state_ == State::Acquiring || state_ == State::Active) && !fresh(now))
            stop(State::Stale);
    }
    void completedControl(uint32_t now) {
        // Test the OLD progress timestamp before accepting this completion.
        // A late update must not revive a stale service or emit a catch-up edge.
        check(now);
        if (state_ != State::Acquiring && state_ != State::Active) return;
        lastProgressAt_ = now;
        if (pulsed_ && uint32_t(now - lastPulseAt_) < pulsePeriodMs) return;
        if (!pins_.pulseHeartbeat()) { stop(State::GpioFault); return; }
        if (!pulsed_) firstPulseAt_ = now;
        pulsed_ = true;
        lastPulseAt_ = now;
        if (uint32_t(now - firstPulseAt_) >= acquisitionMs) state_ = State::Active;
    }
    void stop(State failure) {
        // GPIO faults supersede timing/init faults, retaining SDK errors in pins_.
        if (failed() && failure != State::GpioFault) return;
        state_ = failure;
        if (pins_.ready() && !pins_.releaseHeartbeat()) state_ = State::GpioFault;
    }
    bool failed() const {
        return state_ == State::Stale || state_ == State::InitFailed || state_ == State::GpioFault;
    }
    bool ready(uint32_t now) const {
        return state_ == State::Active && pins_.ready() && fresh(now);
    }
    const char* stateName(uint32_t now) const {
        if (!pins_.ready()) return "gpio_fault";
        if ((state_ == State::Acquiring || state_ == State::Active) && !fresh(now)) return "stale";
        switch (state_) {
        case State::Startup: return "startup";
        case State::Acquiring: return "acquiring";
        case State::Active: return "active";
        case State::Stale: return "stale";
        case State::InitFailed: return "init_failed";
        case State::GpioFault: return "gpio_fault";
        }
        return "init_failed";
    }
    const char* failureReason() const {
        if (!pins_.ready() || state_ == State::GpioFault) return "PICO_SAFE_DEFAULTS_FAILED";
        return state_ == State::Stale ? "EXTERNAL_WATCHDOG_STALE" : "EXTERNAL_WATCHDOG_INIT_FAILED";
    }
private:
    SafeDefaults& pins_;
    State state_ = State::Startup;
    bool begun_ = false, pulsed_ = false;
    uint32_t startedAt_ = 0, lastProgressAt_ = 0, firstPulseAt_ = 0, lastPulseAt_ = 0;
    bool fresh(uint32_t now) const {
        return uint32_t(now - lastProgressAt_) <= progressLimitMs &&
               (!pulsed_ || uint32_t(now - lastPulseAt_) <= pulseLimitMs);
    }
};
} // namespace pico
