// Actual driver with Arduino 3.3.9 GPIO/peripheral ownership semantics.
// Shared mocks stay untouched; only this translation unit replaces their APIs.
#ifndef WBB_TOP_BOARD_R3
#error Build this regression with the R3 defines and public config defaults.
#endif
#define digitalWrite permissiveDigitalWrite
#define pinMode permissivePinMode
#define ledcAttach permissiveLedcAttach
#define ledcWrite permissiveLedcWrite
#define ledcDetach permissiveLedcDetach
#include "mock/Arduino.h"
#undef digitalWrite
#undef pinMode
#undef ledcAttach
#undef ledcWrite
#undef ledcDetach
#include "mock/driver/gpio.h"
#include <iostream>
#include <stdexcept>

namespace boundary {
enum Owner { None, GPIO, LEDC, Other };
inline std::array<Owner, 64> owner{};
inline std::array<int, 64> retainedDuty{};
inline int failClearPin = -1, failRegisterPin = -1;
inline unsigned attachCalls = 0;
inline void event(const char* op, int pin) {
    gpio_mock::events.push_back({op, pin, levels.at(pin), 0,
                                levels[21], gpio_mock::outputEnabled[21]});
}
}

inline bool ledcDetach(unsigned pin) {
    pinOperations.push_back({"detach", int(pin), 0});
    if (boundary::owner.at(pin) != boundary::LEDC || int(pin) == failDetachPin) return false;
    // Pinned ledcDetachBus reconnects SIG_GPIO_OUT_IDX without zeroing its latch.
    boundary::event(gpio_mock::outputEnabled[pin] ? "detach_driven" : "detach_disabled", pin);
    boundary::retainedDuty[pin] = duties[pin];
    attached[pin] = false; duties[pin] = 0; boundary::owner[pin] = boundary::None;
    return true;
}
inline void digitalWrite(unsigned pin, int value) {
    if (boundary::owner.at(pin) != boundary::GPIO) {
        boundary::event("arduino_rejected", pin); return;
    }
    (void)gpio_set_level(pin, value);
}
inline void pinMode(unsigned pin, int mode) {
    if (boundary::owner.at(pin) == boundary::LEDC && !ledcDetach(pin)) return;
    if (boundary::owner[pin] == boundary::Other && int(pin) == boundary::failClearPin) return;
    gpio_config_t config{}; config.pin_bit_mask = 1ULL << pin;
    config.mode = mode == OUTPUT ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
    if (gpio_config(&config) != ESP_OK) return;
    boundary::owner[pin] = boundary::GPIO; gpio_mock::arduinoOwned[pin] = true;
}
inline bool ledcAttach(unsigned pin, unsigned, unsigned) {
    ++boundary::attachCalls; pinOperations.push_back({"attach", int(pin), 0});
    if (int(pin) == failAttachPin || boundary::owner.at(pin) == boundary::LEDC) return false;
    if (boundary::owner[pin] == boundary::Other && int(pin) == boundary::failClearPin) return false;
    boundary::owner[pin] = boundary::None; gpio_mock::arduinoOwned[pin] = false;
    // Model the output-enable boundary inside channel configuration, followed
    // by matrix connection with retained channel duty. ARM must contain PWM.
    if (gpio_set_direction(pin, GPIO_MODE_OUTPUT) != ESP_OK) return false;
    duties[pin] = boundary::retainedDuty[pin];
    boundary::event("attach_pwm", pin);
    if (int(pin) == boundary::failRegisterPin) {
        // Pinned attach cleans up its newly-created handle if registration fails.
        boundary::event("detach_driven", pin); duties[pin] = 0; return false;
    }
    attached[pin] = true; boundary::owner[pin] = boundary::LEDC; return true;
}
inline bool ledcWrite(unsigned pin, unsigned duty) {
    pinOperations.push_back({"write", int(pin), int(duty)});
    if (boundary::owner.at(pin) != boundary::LEDC || int(pin) == failWritePin) return false;
    duties[pin] = duty; return true;
}

#ifdef MOTOR_DRIVER_SOURCE
#include MOTOR_DRIVER_SOURCE
#else
#include "../src/motor_driver.cpp"
#endif

static constexpr int pins[] = {MOTOR0_FI_PIN, MOTOR0_BI_PIN, MOTOR1_FI_PIN,
                               MOTOR1_BI_PIN, MOTOR2_FI_PIN, MOTOR2_BI_PIN};
static void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
static void fresh(bool warm = false) {
    failAttachPin = failWritePin = failDetachPin = -1;
    boundary::failClearPin = boundary::failRegisterPin = -1; boundary::attachCalls = 0;
    boundary::owner.fill(boundary::None); boundary::retainedDuty.fill(73);
    levels.fill(LOW); duties.fill(0); attached.fill(false); modes.fill(INPUT);
    gpio_mock::outputEnabled.fill(false); gpio_mock::arduinoOwned.fill(false);
    gpio_mock::levelFailures = gpio_mock::outputFailures = gpio_mock::releaseFailures = 0;
    gpio_mock::levelFailurePin = gpio_mock::outputFailurePin = gpio_mock::releaseFailurePin = -1;
    for (int pin : pins) { levels[pin] = HIGH; gpio_mock::outputEnabled[pin] = warm; }
    levels[21] = LOW; gpio_mock::outputEnabled[21] = true; // Main's required precondition.
    gpio_mock::events.clear(); pinOperations.clear();
}
static void checkOrder() {
    for (const auto& e : gpio_mock::events) {
        require(e.pin >= 0 && e.pin <= 5, "driver touched non-motor pin");
        if (!std::strcmp(e.operation, "output_enable"))
            require(e.level == LOW, "output enabled with HIGH GPIO latch");
        if (!std::strcmp(e.operation, "detach_driven"))
            require(e.level == LOW, "detach exposed HIGH GPIO latch");
        if (!std::strcmp(e.operation, "output_enable") || !std::strcmp(e.operation, "attach_pwm"))
            require(e.requestLevel == LOW && e.requestDriven, "motor enable without driven-LOW ARM request");
    }
}
static void begin(MotorDriver& motor, int index) {
    motor.begin(pins[index / 2 * 2], pins[index / 2 * 2 + 1], 0, 1, 20000, 8);
}
static void cold(bool warm) {
    fresh(warm); TriMotorDriver motors; motors.begin(); checkOrder();
    require(motors.ready(), "normal R3 begin failed");
    for (int pin : pins) {
        require(levels[pin] == LOW && duties[pin] == 0, "begin did not zero latch and PWM");
        require(boundary::owner[pin] == boundary::LEDC, "LEDC ownership missing");
    }
}
static void detachOrder(int index) {
    fresh();
    // Give the old implementation Arduino ownership so this negative control
    // isolates forceLow's detach bug instead of failing at its earlier init bug.
    for (int pin : pins) boundary::owner[pin] = boundary::GPIO;
    MotorDriver motor; begin(motor, index); require(motor.ready(), "test begin failed");
    require(motor.setSpeed(index % 2 ? -80 : 80), "test PWM failed");
    gpio_mock::events.clear();
    for (int pin : pins) levels[pin] = HIGH;
    failWritePin = pins[index]; require(!motor.stop(), "write failure accepted");
    checkOrder(); require(!motor.ready(), "write failure did not latch");
}

#ifndef MOTOR_DRIVER_BASELINE
static void faultCase(const std::string& kind, int index) {
    fresh(); const int pin = pins[index]; MotorDriver motor;
    const bool runtime = kind == "cleanup_preload" || kind == "detach" || kind == "release" || kind == "ownership_lost";
    if (kind == "preload") { gpio_mock::levelFailurePin = pin; gpio_mock::levelFailures = 1; }
    if (kind == "config") { gpio_mock::outputFailurePin = pin; gpio_mock::outputFailures = 1; }
    if (kind == "attach") failAttachPin = pin;
    if (kind == "register") boundary::failRegisterPin = pin;
    if (kind == "ownership") { boundary::owner[pin] = boundary::Other; boundary::failClearPin = pin; }
    if (kind == "zero") failWritePin = pin;
    begin(motor, index);
    if (runtime) {
        require(motor.ready(), "runtime test begin failed");
        require(motor.setSpeed(index % 2 ? -80 : 80), "runtime test PWM failed");
        for (int p : pins) levels[p] = HIGH;
        gpio_mock::events.clear(); failWritePin = pin;
        if (kind == "cleanup_preload") { gpio_mock::levelFailurePin = pin; gpio_mock::levelFailures = 1; }
        if (kind == "detach") failDetachPin = pin;
        if (kind == "release") { gpio_mock::releaseFailurePin = pin; gpio_mock::releaseFailures = 1; }
        if (kind == "ownership_lost") boundary::owner[pin] = boundary::Other;
        require(!motor.stop(), "runtime fault accepted");
    }
    checkOrder(); require(!motor.ready() && motor.fault(), "failure did not latch not-ready");
    if (kind == "preload" || kind == "config") {
        require((motor.fault() & (MotorDriver::InitFailed | MotorDriver::GpioFailed)) == 9, "GPIO init fault mask lost");
        require(motor.gpioError() == gpio_mock::failure, "GPIO init error lost");
        require(boundary::attachCalls == 0, "failed GPIO setup still attached LEDC");
    }
    if (kind == "preload") for (const auto& e : gpio_mock::events)
        require(std::strcmp(e.operation, "output_enable"), "preload failure still enabled output");
    if (kind == "cleanup_preload") {
        require((motor.fault() & MotorDriver::GpioFailed) && motor.gpioError() == gpio_mock::failure, "cleanup preload error lost");
        require(attached[pin], "failed preload still detached to unknown latch");
    }
    if (kind == "detach" || kind == "ownership_lost")
        require(motor.fault() & MotorDriver::DetachFailed, "detach/ownership fault lost");
    if (kind == "release") {
        require((motor.fault() & MotorDriver::ReleaseFailed) && motor.releaseError() == gpio_mock::failure, "release fault lost");
        require(gpio_mock::outputEnabled[pin], "failed release falsely reported disabled");
    } else for (int i = index / 2 * 2; i < index / 2 * 2 + 2; ++i)
        require(!gpio_mock::outputEnabled[pins[i]], "fault left a driver output enabled");
    if (runtime) for (const auto& e : gpio_mock::events)
        require(std::strcmp(e.operation, "output_enable"), "fault cleanup enabled output");
    const auto fault = motor.fault(); const auto error = motor.gpioError(), release = motor.releaseError();
    const auto attachments = boundary::attachCalls;
    gpio_mock::events.clear();
    failAttachPin = failWritePin = failDetachPin = -1;
    boundary::failClearPin = boundary::failRegisterPin = -1;
    begin(motor, index); require(!motor.ready(), "begin cleared a latched fault");
    require(!motor.stop() && !motor.setSpeed(80), "later success rearmed driver");
    require((motor.fault() & fault) == fault && motor.gpioError() == error && motor.releaseError() == release,
            "later cleanup erased fault evidence");
    require(boundary::attachCalls == attachments, "latched driver reattached PWM");
    for (const auto& e : gpio_mock::events)
        require(std::strcmp(e.operation, "output_enable"), "latched driver re-enabled GPIO");
    checkOrder();
}
#endif

int main(int argc, char** argv) {
    try {
        const std::string scenario = argc > 1 ? argv[1] : "cold";
        if (scenario == "cold" || scenario == "warm") cold(scenario == "warm");
        else if (scenario.rfind("detach_order_", 0) == 0) detachOrder(scenario.back() - '0');
#ifndef MOTOR_DRIVER_BASELINE
        else {
            const auto split = scenario.rfind('_');
            require(split != std::string::npos, "unknown scenario");
            faultCase(scenario.substr(0, split), scenario.back() - '0');
        }
#endif
        std::cout << scenario << " PASS\n"; return 0;
    } catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << '\n'; return 1;
    }
}
