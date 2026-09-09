#pragma once
#include "board_profile_pico.h"
#include <Arduino.h>
#include <driver/gpio.h>

namespace pico {
// IDF owns these pins. No Arduino write-before-pinMode ownership assumption.
// Owns checked safe defaults and OD writes; scheduling belongs to pico_watchdog.h.
class SafeDefaults {
public:
    bool begin() {
        if (initialized_) return ready_; // A fault cannot be cleared by retry.
        initialized_ = true;
        if (!writeLow()) return releaseAfterFailure();
        if (!check(configure((1ULL << motorArmRequest) | (1ULL << logicCycleRequest) |
                             (1ULL << chargeRequest), GPIO_MODE_OUTPUT)))
            return releaseAfterFailure();
        // Release output enable before preloading HIGH: never briefly drive WDI
        // push-pull HIGH while converting a retained GPIO state to open drain.
        if (!check(gpio_set_direction(gpio_num_t(watchdogHeartbeat), GPIO_MODE_INPUT)) ||
            !check(gpio_set_level(gpio_num_t(watchdogHeartbeat), 1)) ||
            !check(configure(1ULL << watchdogHeartbeat, GPIO_MODE_OUTPUT_OD)) ||
            // R101 pulls wake inactive HIGH. Disable any retained output first;
            // preload release before OD enable, with no LOW pulse or push-pull HIGH.
            !check(gpio_set_direction(gpio_num_t(wakeRequestN), GPIO_MODE_INPUT)) ||
            !check(gpio_set_level(gpio_num_t(wakeRequestN), 1)) ||
            !check(configure(1ULL << wakeRequestN, GPIO_MODE_OUTPUT_OD)) ||
            !check(configure((1ULL << logicFaultN) | (1ULL << logicPgood), GPIO_MODE_INPUT)))
            return releaseAfterFailure();
        ready_ = true;
        return true;
    }
    bool holdPowerOff() {
        if (!writeLow()) return releaseAfterFailure();
        // A successful later write is not evidence that an earlier fault cleared.
        return ready_;
    }
    bool setWakeRequest(bool active) {
        if (!ready_ || !holdPowerOff()) return false;
        if (!check(gpio_set_level(gpio_num_t(wakeRequestN), active ? 0 : 1)))
            return releaseAfterFailure();
        wakeRequested_ = active; // Requested state, never physical TS feedback.
        return true;
    }
    bool holdLow() { return holdPowerOff() && setWakeRequest(false); }
    bool wakeRequested() const { return ready_ && wakeRequested_; }
    bool ready() const { return ready_; }
    esp_err_t error() const { return error_; }
    esp_err_t releaseError() const { return releaseError_; }
    bool pulseHeartbeat() {
        if (!ready_) return false;
        // ARM/cycle/charge stay LOW. A separately timed cell wake may continue
        // while control progress earns a heartbeat. This method does not start
        // or extend that wake. HIGH on OD releases the sink to the pullup.
        if (!holdPowerOff()) return false;
        if (!check(gpio_set_level(gpio_num_t(watchdogHeartbeat), 0))) return releaseAfterFailure();
        delayMicroseconds(2); // TPS3851 minimum WDI LOW pulse is 50ns.
        return releaseHeartbeat();
    }
    bool releaseHeartbeat() {
        if (!ready_) return false;
        if (!check(gpio_set_level(gpio_num_t(watchdogHeartbeat), 1))) return releaseAfterFailure();
        return true;
    }
private:
    bool initialized_ = false, ready_ = false, wakeRequested_ = false;
    esp_err_t error_ = ESP_OK, releaseError_ = ESP_OK;
    bool check(esp_err_t result) {
        if (error_ == ESP_OK && result != ESP_OK) error_ = result;
        return result == ESP_OK;
    }
    bool writeLow() {
        const bool arm = check(gpio_set_level(gpio_num_t(motorArmRequest), 0));
        const bool cycle = check(gpio_set_level(gpio_num_t(logicCycleRequest), 0));
        const bool charge = check(gpio_set_level(gpio_num_t(chargeRequest), 0));
        return arm && cycle && charge;
    }
    static esp_err_t configure(uint64_t mask, gpio_mode_t mode) {
        gpio_config_t config{};
        config.pin_bit_mask = mask;
        config.mode = mode;
        config.pull_up_en = GPIO_PULLUP_DISABLE;
        config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        config.intr_type = GPIO_INTR_DISABLE;
        return gpio_config(&config);
    }
    bool releaseAfterFailure() {
        ready_ = false;
        wakeRequested_ = false;
        // External ARM/gate/charge pulldowns and WDI/wake pullups supply defaults.
        // A failed SDK release is reported, never claimed as physical isolation.
        const int pins[] = {motorArmRequest, logicCycleRequest, chargeRequest,
                            watchdogHeartbeat, wakeRequestN};
        for (int pin : pins) {
            const auto result = gpio_set_direction(gpio_num_t(pin), GPIO_MODE_INPUT);
            if (releaseError_ == ESP_OK && result != ESP_OK) releaseError_ = result;
        }
        return false;
    }
};
} // namespace pico
