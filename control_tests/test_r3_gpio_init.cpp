// Runs each real R3 entry point with the pinned Arduino ownership rule and an
// initially HIGH GPIO21 output latch. Build twice with/without R3_GPIO_TEST_DIAG.
#define WBB_TOP_BOARD_R3 1
#define CONFIG_IDF_TARGET_ESP32C6 1
#define ARDUINO_USB_CDC_ON_BOOT 0
#ifndef R3_GPIO_TEST_DIAG
#define WBB_TOP_BOARD_V2 1
#define WBB_CONTROL_NATIVE_TEST 1
#endif

// Preserve shared mocks for other tests. Replace just their permissive GPIO API
// in this regression with Arduino 3.3.9's reject-before-ownership behavior.
#define digitalWrite permissiveDigitalWrite
#define pinMode permissivePinMode
#define digitalRead permissiveDigitalRead
#include "mock/Arduino.h"
#undef digitalWrite
#undef pinMode
#undef digitalRead
#include "mock/driver/gpio.h"
#include <ArduinoJson.h>
#include <iostream>
#include <stdexcept>

inline unsigned gpioReadCalls = 0;
inline int digitalRead(unsigned pin) {
    ++gpioReadCalls;
    if (modes.at(pin) != INPUT_PULLUP) throw std::runtime_error("read from uninitialized encoder");
    return permissiveDigitalRead(pin);
}

inline void digitalWrite(unsigned pin, int value) {
    if (!gpio_mock::arduinoOwned.at(pin)) {
        gpio_mock::events.push_back({"arduino_rejected", int(pin), value, -1});
        return;
    }
    // IDF does not require Arduino ownership; the Arduino wrapper does.
    (void)gpio_set_level(pin, value);
}
inline void pinMode(unsigned pin, int mode) {
    if (mode == OUTPUT) {
        gpio_config_t config{};
        config.pin_bit_mask = 1ULL << pin;
        config.mode = GPIO_MODE_OUTPUT;
        if (gpio_config(&config)) return;
    } else {
        permissivePinMode(pin, mode);
    }
    gpio_mock::arduinoOwned.at(pin) = true;
}

#ifndef R3_GPIO_TEST_DIAG
#define RobotController GPIOTestRobotBase
#define ControlTransport GPIOTestTransportBase
#include "runtime_stubs.h"
#undef RobotController
#undef ControlTransport
class RobotController : public GPIOTestRobotBase {
public:
    unsigned beginCalls = 0, statusCalls = 0;
    void begin() { ++beginCalls; GPIOTestRobotBase::begin(); }
    void handleCommand(JsonDocument& d) {
        if (!beginCalls) throw std::runtime_error("command reached uninitialized robot");
        GPIOTestRobotBase::handleCommand(d);
    }
    void fillStatus(JsonDocument& d) {
        if (!beginCalls) throw std::runtime_error("status reached uninitialized robot");
        ++statusCalls; GPIOTestRobotBase::fillStatus(d);
    }
    void update(float dt) {
        if (!beginCalls) throw std::runtime_error("update reached uninitialized robot");
        GPIOTestRobotBase::update(dt);
    }
};
class ControlTransport : public GPIOTestTransportBase {
public:
    unsigned beginCalls = 0, pollCalls = 0;
    bool begin(const char* name, const char* ssid, const char* password) {
        ++beginCalls; return GPIOTestTransportBase::begin(name, ssid, password);
    }
    void poll(Receiver receiver) { ++pollCalls; GPIOTestTransportBase::poll(receiver); }
};
#endif

#ifdef R3_GPIO_ENTRYPOINT
#include R3_GPIO_ENTRYPOINT
#elif defined(R3_GPIO_TEST_DIAG)
#include "../c6_selftest/src/main.cpp"
#else
#include "../src/top_v2_main.cpp"
#endif

static void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
static const char* failureReason() {
#ifdef R3_GPIO_TEST_DIAG
    return lastError;
#else
    return reason;
#endif
}
static void checkTrace() {
    std::array<bool, 64> preloaded{};
    for (const auto& event : gpio_mock::events) {
        require(event.pin != 12 && event.pin != 13, "NC GPIO12/13 was touched");
        if (event.pin <= 5 && !std::strcmp(event.operation, "output_enable")) {
            require(event.requestLevel == LOW && event.requestDriven,
                    "FI/BI output enable preceded driven-LOW arm request");
        }
        // Both entry points must leave FI/BI ownership with checked IDF/LEDC
        // initialization. A redundant Arduino setup loop can glitch before the
        // actual driver runs, even when its later initialization is correct.
        const bool managed = event.pin == 21 || event.pin <= 5;
        if (!managed) continue;
        require(std::strcmp(event.operation, "arduino_rejected"), "safe-low write rejected before Arduino ownership");
        if (!std::strcmp(event.operation, "idf_set_level") && !event.result) {
            require(event.level == LOW, "managed safe-low GPIO was written HIGH");
            preloaded[event.pin] = true;
        }
        if (!std::strcmp(event.operation, "output_enable")) {
            require(preloaded[event.pin] && event.level == LOW, "output enabled before successful LOW preload");
        }
        require(!gpio_mock::arduinoOwned[event.pin], "managed GPIO ownership mixed between IDF and Arduino");
    }
}

static JsonDocument uartRequest(const char* op) {
    Serial.tx.clear();
#ifdef R3_GPIO_TEST_DIAG
    Serial.feed(std::string(op) + '\n');
#else
    JsonDocument request;
    request["id"] = 1; request["cmd"] = op; request["boot"] = boot;
    std::string wire; serializeJson(request, wire); Serial.feed(wire + '\n');
#endif
    for (int budget = 0; budget < 32 && Serial.available(); ++budget) loop();
    require(!Serial.available(), "UART RX did not drain");
    require(!Serial.tx.empty(), "fault status/reboot reply missing from UART");
    JsonDocument frame;
    require(!deserializeJson(frame, Serial.tx), "invalid UART JSON");
#ifndef R3_GPIO_TEST_DIAG
    require(frame["payload"].is<const char*>() && frame["mac"].is<const char*>(), "runtime envelope identity changed");
    const std::string payload = frame["payload"].as<std::string>();
    require(!deserializeJson(frame, payload), "invalid runtime payload");
#endif
    return frame;
}

static void checkFaultRecovery(const char* expectedReason) {
    require(Serial.uartMode && Serial.baud == 115200 && Serial.rxPin == 17 && Serial.txPin == 16,
            "fault recovery UART not initialized");
    const unsigned reads = gpioReadCalls;
#ifndef R3_GPIO_TEST_DIAG
    const auto robotCalls = robot.statusCalls, updates = robot.updates;
    const auto wirelessCalls = transport.beginCalls, polls = transport.pollCalls;
    const auto feeds = watchdogFeeds;
#endif
    const auto start = gpio_mock::events.size();
    auto d = uartRequest("status");
    require(d["reason"] == expectedReason && d["gpio_fault_latched"] == true, "latched GPIO fault absent from status");
    require(d["motion_enabled"] == false && d["charge_enabled"] == false && d["power_disconnect"] == false,
            "fault status changed R3 safety capabilities");
    require(d["profile"] == "top_board_v2_r3_review", "R3 profile identity changed");
#ifdef R3_GPIO_TEST_DIAG
    require(d["firmware"] == "wbb-r3-review-diag" && d["serial_link"] == "cp2102n_uart0", "diagnostic identity changed");
    require(d["encoder_inputs_available"] == false, "faulted encoder data marked available");
    for (const auto& motor : d["motors"].as<JsonArray>()) {
        require(motor["a"].isNull() && motor["b"].isNull(), "unavailable encoder value fabricated");
    }
    const char* field = !std::strcmp(expectedReason, "GPIO21_SAFE_LOW_FAILED") ? "arm_gpio_error" : "motor_gpio_error";
    require(d[field] == gpio_mock::failure, "diagnostic IDF error lost");
#else
    require(d["firmware"] == "wbb-control-2" && d["protocol"] == 2, "runtime identity changed");
    require(d["gpio_error"] == gpio_mock::failure && d["read_only"] == true, "runtime GPIO error lost");
    require(d["telemetry_available"] == false && d["x"].isNull() && d["encoder_counts"][0].isNull(),
            "faulted robot telemetry fabricated");
    require(d["pwm_ready"] == false && d["pwm_command"][0] == 0 && d["physical_motors"][2] == 3,
            "runtime inhibited-command identity changed");
#endif
    for (const char* op : {"hello", "stop", "claim", "provision", "area", "arm", "charge"}) {
        auto response = uartRequest(op);
        require(!rebootPending && restartCalls == 0, "read-only/denied command rebooted");
        (void)response;
    }
    // A malformed reboot cannot become intentional recovery.
    Serial.feed(std::string("reboot\0ignored\n", 15));
    while (Serial.available()) loop();
    require(!rebootPending, "malformed reboot accepted");
    fakeNow += 4000; loop();
    require(restartCalls == 0, "fault loop rebooted without an intentional command");
    d = uartRequest("status");
    require(d["reason"] == expectedReason && d["gpio_fault_latched"] == true, "status/stop/malformed input cleared GPIO fault");
    require(gpioReadCalls == reads, "fault path read encoder inputs");
#ifndef R3_GPIO_TEST_DIAG
    require(robot.statusCalls == robotCalls && robot.updates == updates, "fault path used robot status/update");
    require(transport.beginCalls == wirelessCalls && transport.pollCalls == polls, "fault path started/polled wireless");
    require(storedConfig == "{}", "fault path wrote provisioning data");
    if (watchdogReady) require(watchdogFeeds > feeds, "fault UART task stopped servicing its CPU watchdog");
#endif
    // TX pressure drops a frame without clearing the fault or blocking RX.
    Serial.writeCapacity = 0; Serial.tx.clear();
    Serial.feed("hello\n"); while (Serial.available()) loop();
    require(Serial.tx.empty(), "fault UART TX ignored capacity");
    Serial.writeCapacity = 8192;
    d = uartRequest("status"); require(d["gpio_fault_latched"] == true, "TX pressure cleared fault");
    fakeNow = 0xfffffff0U;
    uartRequest("reboot"); const auto requested = rebootAt;
    require(rebootPending && restartCalls == 0, "intentional reboot not scheduled");
    uartRequest("reboot"); require(rebootAt == requested, "duplicate reboot extended delay");
    d = uartRequest("status"); require(d["gpio_fault_latched"] == true && d["reboot_pending"] == true,
                                      "reboot request cleared latched fault");
    fakeNow = requested + 99; loop(); require(restartCalls == 0, "reboot fired too early");
    Serial.writeCapacity = 0; fakeNow = requested + 100; loop();
    require(restartCalls == 1, "intentional reboot not executed with TX pressure/wraparound");
    for (size_t i = start; i < gpio_mock::events.size(); ++i)
        require(std::strcmp(gpio_mock::events[i].operation, "output_enable"), "recovery re-enabled GPIO outputs");
}
static void checkFailure() {
    require(!std::strcmp(failureReason(), "GPIO21_SAFE_LOW_FAILED"), "safe-low failure not reported");
    require(!gpio_mock::outputEnabled[21], "failed GPIO21 init/write retained output enable");
    const auto count = gpio_mock::events.size();
    for (int i = 0; i < 3; ++i) loop();
    require(!gpio_mock::outputEnabled[21], "loop re-enabled a failed request");
    for (size_t i = count; i < gpio_mock::events.size(); ++i) {
        require(std::strcmp(gpio_mock::events[i].operation, "output_enable"), "latched failure retried output enable");
    }
    require(!rebootPending && restartCalls == 0, "failed initialization permitted reboot processing");
    checkFaultRecovery("GPIO21_SAFE_LOW_FAILED");
}
#ifdef R3_GPIO_TEST_DIAG
static void checkMotorFailure() {
    require(!std::strcmp(lastError, "MOTOR_GPIO_SAFE_LOW_FAILED"), "FI/BI failure not reported");
    for (int pin = 0; pin < 6; ++pin) require(!gpio_mock::outputEnabled[pin], "failed FI/BI bank retained output enable");
    const auto count = gpio_mock::events.size();
    for (int i = 0; i < 3; ++i) loop();
    require(levels[21] == LOW && gpio_mock::outputEnabled[21], "FI/BI failure lost driven-LOW arm request");
    for (size_t i = count; i < gpio_mock::events.size(); ++i) {
        require(std::strcmp(gpio_mock::events[i].operation, "output_enable"), "failed FI/BI bank re-enabled");
    }
    require(!rebootPending && restartCalls == 0, "FI/BI failure permitted reboot processing");
    checkFaultRecovery("MOTOR_GPIO_SAFE_LOW_FAILED");
}
#endif

int main(int argc, char** argv) {
    try {
        const std::string scenario = argc > 1 ? argv[1] : "cold";
        levels[21] = HIGH; // Must not accidentally pass because the mock starts at zero.
        for (int pin = 0; pin < 6; ++pin) levels[pin] = HIGH;
        gpio_mock::outputEnabled[21] = scenario != "cold";
        gpio_mock::levelFailurePin = gpio_mock::outputFailurePin = 21;
        if (scenario == "preload_failure") gpio_mock::levelFailures = 1;
        if (scenario == "config_failure") gpio_mock::outputFailures = 1;
#ifdef R3_GPIO_TEST_DIAG
        const bool motorFailure = scenario.rfind("motor_", 0) == 0;
        const int failedMotorPin = motorFailure ? scenario.back() - '0' : -1;
        if (scenario.rfind("motor_preload_", 0) == 0) {
            gpio_mock::levelFailurePin = failedMotorPin; gpio_mock::levelFailures = 1;
        }
        if (scenario.rfind("motor_config_", 0) == 0) {
            gpio_mock::outputFailurePin = failedMotorPin; gpio_mock::outputFailures = 1;
        }
#endif
        setup();
        checkTrace(); // The old entry points fail here even if a later write repairs LOW.
#ifndef R3_GPIO_TEST_DIAG
        if (scenario == "provision_failure" || scenario == "overrun_failure" || scenario == "wireless_reboot_failure") {
            gpio_mock::levelFailures = 1;
            const auto updates = robot.updates;
            if (scenario == "provision_failure") {
                JsonDocument request;
                request["id"] = 1; request["cmd"] = "provision"; request["boot"] = boot;
                request["ssid"] = "test"; request["password"] = "12345678";
                request["key"] = std::string(64, 'a');
                std::string wire; serializeJson(request, wire);
                command(control::Link::USB, 0, wire.c_str());
                require(storedConfig == "{}", "GPIO failure during provisioning still wrote NVS");
            } else if (scenario == "overrun_failure") {
                fakeNow = lastControlAt + maxControlIntervalMs + 1; loop();
                require(robot.updates == updates, "GPIO failure during overrun still updated robot");
            } else {
                transport.wifiUp = true;
                require(owner.claim(control::Link::WiFi, 7, 1, 1, millis()), "test lease rejected");
                rebootPending = true; rebootLink = control::Link::WiFi;
                rebootAt = rebootQuietAt = millis();
                fakeNow += 100; loop();
                require(restartCalls == 0, "new GPIO fault allowed pending wireless reboot");
                loop(); require(!rebootPending, "fault did not cancel wireless reboot for USB recovery");
            }
            checkFailure(); checkTrace();
            std::cout << scenario << " PASS (fault transition contained; USB recovery available)\n";
            return 0;
        }
#endif
#ifdef R3_GPIO_TEST_DIAG
        if (motorFailure) {
            if (scenario.rfind("motor_write_", 0) == 0) {
                gpio_mock::levelFailurePin = failedMotorPin; gpio_mock::levelFailures = 1;
                loop();
            } else {
                require(Serial.baud == 115200, "fault recovery UART not initialized");
                if (scenario.rfind("motor_preload_", 0) == 0) {
                    for (const auto& event : gpio_mock::events) {
                        require(event.pin == 21 || std::strcmp(event.operation, "output_enable"),
                                "FI/BI preload failure still attempted output enable");
                    }
                }
            }
            checkMotorFailure(); checkTrace();
            std::cout << scenario << " PASS (all six diagnostic outputs released; request LOW)\n";
            return 0;
        }
#endif
        if (scenario == "preload_failure" || scenario == "config_failure") {
            require(Serial.baud == 115200, "fault recovery UART not initialized");
#ifndef R3_GPIO_TEST_DIAG
            require(robot.beginCalls == 0 && transport.beginCalls == 0, "failed GPIO initialization started robot/wireless");
#endif
            for (const auto& event : gpio_mock::events) {
                require(event.pin == 21, "failed GPIO21 initialization configured another pin");
                if (scenario == "preload_failure") {
                    require(std::strcmp(event.operation, "output_enable"), "failed preload still attempted output enable");
                }
            }
            checkFailure();
        } else {
            require(levels[21] == LOW && gpio_mock::outputEnabled[21], "startup request is not driven LOW");
            require(Serial.uartMode && Serial.rxPin == 17 && Serial.txPin == 16, "R3 UART mapping changed");
            if (scenario == "write_failure") {
                gpio_mock::levelFailures = 1;
                loop();
                checkFailure();
            } else {
#ifdef R3_GPIO_TEST_DIAG
                command("stop");
                for (const auto& p : r3::headers) {
                    require(levels[p.fi] == LOW && levels[p.bi] == LOW, "diagnostic FI/BI not LOW");
                    require(modes[p.encA] == INPUT_PULLUP && modes[p.encB] == INPUT_PULLUP,
                            "diagnostic encoder direction changed");
                }
                require(Serial.tx.find("\"motion_enabled\":false,\"charge_enabled\":false") != std::string::npos,
                        "diagnostic motion/charge gates changed");
#else
                halt("TEST_STOP");
                require(!motionQualified && !robot.moving, "runtime motion gate changed");
#endif
                for (int i = 0; i < 3; ++i) loop();
                require(levels[21] == LOW && gpio_mock::outputEnabled[21], "subsequent request LOW writes failed");
            }
        }
        checkTrace();
        std::cout << scenario << " PASS (real R3 entry point; strict Arduino ownership)\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
