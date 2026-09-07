#pragma once
#include <stdint.h>

namespace diag {
// Values are test limits, not the PCB's electrical ratings.
struct Safety {
    static constexpr uint32_t heartbeatMs = 350;
    static constexpr uint32_t maxPulseMs = 1000;
    bool armed = false;
    bool running = false;
    bool bench = false;
    uint32_t lastPing = 0, start = 0, duration = 0, stoppedAt = 0;
    uint16_t nominalMv = 0;
    uint8_t maxDuty = 0;
    const char* reason = "POWER_ON";

    void stop(uint32_t now, const char* why) {
        armed = running = false;
        duration = 0;
        stoppedAt = now;
        reason = why;
    }
    static bool voltageMatches(int nominal, int measured) {
        return (nominal == 6000 || nominal == 12000) &&
               measured >= nominal * 95 / 100 && measured <= nominal * 105 / 100;
    }
    const char* armBench(uint32_t now, int nominal, int measured, int duty, bool pwmReady) {
        if (running) return "BUSY";
        if (!pwmReady) return "PWM_INIT";
        if (!voltageMatches(nominal, measured)) return "VOLTAGE_MISMATCH";
        if (duty < 1 || duty > 255) return "DUTY_RANGE";
        nominalMv = nominal;
        maxDuty = duty;
        bench = armed = true;
        lastPing = now;
        reason = "BENCH_ARMED";
        return nullptr;
    }
    const char* pulse(uint32_t now, int motor, int direction, int duty, int ms) {
        if (!armed) return "NOT_ARMED";
        if (running) return "BUSY";
        if (uint32_t(now-lastPing) >= heartbeatMs) { stop(now,"HEARTBEAT_TIMEOUT"); return reason; }
        if (uint32_t(now-stoppedAt) < 500) return "COAST_WAIT";
        if (motor < 0 || motor > 3 || (direction != 1 && direction != -1)) return "MOTOR_RANGE";
        if (duty < 1 || duty > maxDuty) return "DUTY_RANGE";
        if (ms < 1 || ms > int(maxPulseMs)) return "DURATION_RANGE";
        start = now; duration = ms; running = true; reason = "PULSE";
        return nullptr;
    }
    // Must be called before serial parsing and after every accepted command.
    bool tick(uint32_t now, bool connected) {
        if (armed && !connected) { stop(now,"USB_LOST"); return true; }
        if (armed && uint32_t(now-lastPing) >= heartbeatMs) { stop(now,"HEARTBEAT_TIMEOUT"); return true; }
        if (running && uint32_t(now-start) >= duration) { stop(now,"PULSE_DONE"); return true; }
        return false;
    }
};
// A valid AB transition changes exactly one bit. Both phases are counted.
__attribute__((always_inline)) inline int quadrature(uint8_t before, uint8_t after) {
    const uint8_t changed = (before ^ after) & 3;
    if(changed != 1 && changed != 2) return 0;
    return ((before & 1) ^ ((after >> 1) & 1)) ? 1 : -1;
}
} // namespace diag
