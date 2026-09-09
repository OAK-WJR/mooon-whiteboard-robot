#pragma once
#include <Arduino.h>

using esp_err_t = int;
using gpio_num_t = int;
#ifndef WBB_CONTROL_NATIVE_TEST
constexpr int ESP_OK = 0;
#endif
enum gpio_mode_t { GPIO_MODE_DISABLE = 0, GPIO_MODE_INPUT = 1, GPIO_MODE_OUTPUT = 2 };
enum gpio_pullup_t { GPIO_PULLUP_DISABLE = 0, GPIO_PULLUP_ENABLE = 1 };
enum gpio_pulldown_t { GPIO_PULLDOWN_DISABLE = 0, GPIO_PULLDOWN_ENABLE = 1 };
enum gpio_int_type_t { GPIO_INTR_DISABLE = 0 };
struct gpio_config_t {
    uint64_t pin_bit_mask = 0;
    gpio_mode_t mode = GPIO_MODE_DISABLE;
    gpio_pullup_t pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_pulldown_t pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_int_type_t intr_type = GPIO_INTR_DISABLE;
};

namespace gpio_mock {
struct Event {
    const char* operation;
    int pin, level, result;
    int requestLevel = 0;
    bool requestDriven = false;
};
inline std::vector<Event> events;
inline std::array<bool, 64> outputEnabled{}, arduinoOwned{};
inline int requestPin = 21;
inline int levelFailures = 0, outputFailures = 0, releaseFailures = 0;
inline int levelFailurePin = -1, outputFailurePin = -1, releaseFailurePin = -1;
constexpr int failure = -42;
inline int consume(int& failures) { if (failures) { --failures; return failure; } return 0; }
}

inline esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level) {
    const int result = gpio_mock::levelFailurePin == -1 || gpio_mock::levelFailurePin == pin
        ? gpio_mock::consume(gpio_mock::levelFailures) : 0;
    gpio_mock::events.push_back({"idf_set_level", pin, int(level), result});
    if (!result) {
        levels.at(pin) = level;
        pinOperations.push_back({"gpio", pin, int(level)});
    }
    return result;
}

inline esp_err_t gpio_set_direction(gpio_num_t pin, gpio_mode_t mode) {
    const bool output = mode == GPIO_MODE_OUTPUT;
    const int target = output ? gpio_mock::outputFailurePin : gpio_mock::releaseFailurePin;
    const int result = target == -1 || target == pin
        ? gpio_mock::consume(output ? gpio_mock::outputFailures : gpio_mock::releaseFailures) : 0;
    gpio_mock::events.push_back({output ? "output_enable" : "output_disable", pin, levels.at(pin), result,
                                levels[gpio_mock::requestPin], gpio_mock::outputEnabled[gpio_mock::requestPin]});
    if (!result) {
        gpio_mock::outputEnabled.at(pin) = output;
        modes.at(pin) = output ? OUTPUT : INPUT;
    }
    return result;
}

inline esp_err_t gpio_config(const gpio_config_t* config) {
    assert(config->pin_bit_mask && !(config->pin_bit_mask >> 31));
    for (int pin = 0; pin < 31; ++pin) if (config->pin_bit_mask & (1ULL << pin)) {
        if (pin == gpio_mock::requestPin) {
            assert(config->pull_up_en == GPIO_PULLUP_DISABLE);
            assert(config->pull_down_en == GPIO_PULLDOWN_DISABLE);
            assert(config->intr_type == GPIO_INTR_DISABLE);
        }
        const int result = gpio_set_direction(pin, config->mode);
        if (result) return result;
    }
    return 0;
}
