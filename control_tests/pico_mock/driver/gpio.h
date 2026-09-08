#pragma once
#include <Arduino.h>

using esp_err_t = int;
using gpio_num_t = int;
#ifndef WBB_CONTROL_NATIVE_TEST
constexpr int ESP_OK = 0;
#endif
enum gpio_mode_t { GPIO_MODE_INPUT = 1, GPIO_MODE_OUTPUT = 2, GPIO_MODE_OUTPUT_OD = 6 };
enum gpio_pullup_t { GPIO_PULLUP_DISABLE = 0, GPIO_PULLUP_ENABLE = 1 };
enum gpio_pulldown_t { GPIO_PULLDOWN_DISABLE = 0, GPIO_PULLDOWN_ENABLE = 1 };
enum gpio_int_type_t { GPIO_INTR_DISABLE = 0 };
struct gpio_config_t {
    uint64_t pin_bit_mask = 0;
    gpio_mode_t mode = GPIO_MODE_INPUT;
    gpio_pullup_t pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_pulldown_t pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_int_type_t intr_type = GPIO_INTR_DISABLE;
};
namespace pico_gpio_mock {
struct Event { const char* op; int pin, value, mode, error, armLevel, armMode; uint32_t atMs,atUs; };
inline std::array<int, 64> driverModes{};
inline std::vector<Event> events;
inline std::string failOp;
inline int failPin = -1, releaseFailPin = -1, failValue = -1;
inline int record(const char* op, int pin, int value, int mode) {
    int error = 0;
    if ((failOp == op && pin == failPin && (failValue < 0 || value == failValue)) || (std::string(op) == "direction" && pin == releaseFailPin)) {
        error = -42;
        if (failOp == op && pin == failPin) failPin = -1;
        if (std::string(op) == "direction" && pin == releaseFailPin) releaseFailPin = -1;
    }
    events.push_back({op, pin, value, mode, error, levels[16], driverModes[16],fakeNow,fakeNow*1000+fakeSubMicros});
    return error;
}
}
inline esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level) {
    int result = pico_gpio_mock::record("level", pin, level, pico_gpio_mock::driverModes.at(pin));
    if (!result) { levels.at(pin) = level; pinOperations.push_back({"gpio", pin, int(level)}); }
    return result;
}
inline esp_err_t gpio_set_direction(gpio_num_t pin, gpio_mode_t mode) {
    int result = pico_gpio_mock::record("direction", pin, levels.at(pin), mode);
    if (!result) {
        pico_gpio_mock::driverModes.at(pin) = mode;
        modes.at(pin) = mode == GPIO_MODE_INPUT ? INPUT : OUTPUT;
    }
    return result;
}
inline esp_err_t gpio_config(const gpio_config_t* config) {
    assert(config->pin_bit_mask && !(config->pin_bit_mask >> 49));
    assert(config->pull_up_en == GPIO_PULLUP_DISABLE && config->pull_down_en == GPIO_PULLDOWN_DISABLE);
    assert(config->intr_type == GPIO_INTR_DISABLE);
    for (int pin = 0; pin < 49; ++pin) if (config->pin_bit_mask & (1ULL << pin)) {
        int result = pico_gpio_mock::record("config", pin, levels.at(pin), config->mode);
        if (result) return result; // Partial configuration must also be cleaned up.
        pico_gpio_mock::driverModes.at(pin) = config->mode;
        modes.at(pin) = config->mode == GPIO_MODE_INPUT ? INPUT : OUTPUT;
    }
    return 0;
}
inline int gpio_get_level(gpio_num_t pin) {
    assert(pico_gpio_mock::driverModes.at(pin) == GPIO_MODE_INPUT);
    pico_gpio_mock::record("read", pin, levels.at(pin), GPIO_MODE_INPUT);
    return levels.at(pin);
}
