#pragma once
#include <driver/gpio.h>

namespace c6_gpio {
// These pins belong to IDF throughout their lifetime. Arduino's digitalWrite()
// rejects writes before pinMode() registers ownership. Never requests HIGH.
template<uint64_t PinMask>
class SafeLowOutputs {
    static_assert(PinMask && !(PinMask >> 31), "Use ESP32-C6 GPIOs only");
public:
    bool begin() {
        ready_ = false;
        error_ = writeLow(); // Preload every selected latch before ANY output enable.
        if (error_ != ESP_OK) return releaseAfterFailure();
        gpio_config_t config{};
        config.pin_bit_mask = PinMask;
        config.mode = GPIO_MODE_OUTPUT;
        config.pull_up_en = GPIO_PULLUP_DISABLE;
        config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        config.intr_type = GPIO_INTR_DISABLE;
        error_ = gpio_config(&config);
        if (error_ != ESP_OK) return releaseAfterFailure();
        releaseError_ = ESP_OK;
        ready_ = true;
        return true;
    }

    bool holdLow() {
        const esp_err_t result = writeLow();
        if (result != ESP_OK) {
            if (error_ == ESP_OK) error_ = result;
            return releaseAfterFailure();
        }
        return ready_; // A later successful write cannot clear a latched failure.
    }

    bool ready() const { return ready_; }
    esp_err_t error() const { return error_; }
    esp_err_t releaseError() const { return releaseError_; }

private:
    bool ready_ = false;
    esp_err_t error_ = ESP_OK, releaseError_ = ESP_OK;
    esp_err_t writeLow() {
        for (int pin = 0; pin < 31; ++pin) if (PinMask & (1ULL << pin)) {
            const auto result = gpio_set_level(static_cast<gpio_num_t>(pin), 0);
            if (result != ESP_OK) return result;
        }
        return ESP_OK;
    }
    bool releaseAfterFailure() {
        ready_ = false;
        // Release every selected output, including after partial configuration.
        // External pull-downs bias released outputs; callers still require a default-off power stage.
        // Retain errors: a failed SDK release cannot prove physical LOW.
        releaseError_ = ESP_OK;
        for (int pin = 0; pin < 31; ++pin) if (PinMask & (1ULL << pin)) {
            const auto result = gpio_set_direction(static_cast<gpio_num_t>(pin), GPIO_MODE_INPUT);
            if (releaseError_ == ESP_OK && result != ESP_OK) releaseError_ = result;
        }
        return false;
    }
};
} // namespace c6_gpio
