#pragma once
// Current fourteen-sheet power_integration/controller_contract.json MCU1 contacts,
// including recovery/status/wake/charge nets recorded in pin_nets (not just the older
// 16-entry gpio_assignments table). Electrical handoff only; not a release.
#if !defined(WBB_TOP_BOARD_PICO) || WBB_TOP_BOARD_PICO != 1
#error "Select the explicit top_board_v2_pico environment"
#endif
#if defined(WBB_TOP_BOARD_R3)
#error "Pico and R3 profiles are mutually exclusive"
#endif
#if !defined(CONFIG_IDF_TARGET_ESP32S3) || !CONFIG_IDF_TARGET_ESP32S3 || defined(CONFIG_IDF_TARGET_ESP32C6)
#error "Pico requires ESP32-S3"
#endif
#if !defined(WBB_TOP_BOARD_V2) || WBB_TOP_BOARD_V2 != 1 || defined(WBB_R3_DIAGNOSTIC) || defined(MOTOR_TEST_MODE)
#error "Pico requires the gated integrated runtime; motor test and R3 diagnostic are forbidden"
#endif
#if !defined(ARDUINO_USB_CDC_ON_BOOT) || ARDUINO_USB_CDC_ON_BOOT != 0
#error "Pico factory CH343P requires UART0 Serial (ARDUINO_USB_CDC_ON_BOOT=0)"
#endif

#include <stdint.h>
namespace pico {
constexpr const char* profile = "top_board_v2_pico_review";
constexpr int uartTx = 43, uartRx = 44;
constexpr int motorArmRequest = 16, watchdogHeartbeat = 17;
constexpr int i2cSda = 1, i2cScl = 2;
constexpr int logicFaultN = 18, logicPgood = 38, logicCycleRequest = 42;
constexpr int wakeRequestN = 39, chargeRequest = 41; // Safe defaults only; GPIO40 stays NC.
struct MotorPins { uint8_t fi, bi, encA, encB, physical; const char* header; };
constexpr MotorPins headers[] = {
    {4, 5, 10, 11, 1, "M0"},
    {6, 7, 12, 13, 3, "M1"},
    {8, 9, 14, 15, 2, "M2"},
};
// Kinematics/telemetry use physical Motor1/2/3, not connector numbering.
constexpr int physicalOrder[] = {0, 2, 1};
constexpr bool reserved(int pin) {
    return pin == 0 || pin == 3 || pin == 45 || pin == 46 || // straps
           pin == 19 || pin == 20 || pin == 43 || pin == 44 || // USB/UART
           pin == 21 || (pin >= 33 && pin <= 37); // LED/memory
}
constexpr bool isMotorPin(int pin) {
    for (const auto& p : headers) if (pin == p.fi || pin == p.bi) return true;
    return false;
}
constexpr bool isMotorPair(int fi, int bi) {
    for (const auto& p : headers) if (fi == p.fi && bi == p.bi) return true;
    return false;
}
constexpr bool pinsAreDistinct() {
    const int pins[] = {headers[0].fi, headers[0].bi, headers[0].encA, headers[0].encB,
                        headers[1].fi, headers[1].bi, headers[1].encA, headers[1].encB,
                        headers[2].fi, headers[2].bi, headers[2].encA, headers[2].encB,
                        motorArmRequest, watchdogHeartbeat, i2cSda, i2cScl,
                        logicFaultN, logicPgood, logicCycleRequest, wakeRequestN, chargeRequest};
    for (unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
        if (pins[i] < 1 || pins[i] > 48 || reserved(pins[i])) return false;
        for (unsigned j = 0; j < i; ++j) if (pins[i] == pins[j]) return false;
    }
    return true;
}
static_assert(pinsAreDistinct(), "Pico application pins alias each other or reserved pins");
} // namespace pico

#undef MOTOR0_FI_PIN
#undef MOTOR0_BI_PIN
#undef MOTOR1_FI_PIN
#undef MOTOR1_BI_PIN
#undef MOTOR2_FI_PIN
#undef MOTOR2_BI_PIN
#undef ENC0_A_PIN
#undef ENC0_B_PIN
#undef ENC1_A_PIN
#undef ENC1_B_PIN
#undef ENC2_A_PIN
#undef ENC2_B_PIN
#define MOTOR0_FI_PIN pico::headers[pico::physicalOrder[0]].fi
#define MOTOR0_BI_PIN pico::headers[pico::physicalOrder[0]].bi
#define MOTOR1_FI_PIN pico::headers[pico::physicalOrder[1]].fi
#define MOTOR1_BI_PIN pico::headers[pico::physicalOrder[1]].bi
#define MOTOR2_FI_PIN pico::headers[pico::physicalOrder[2]].fi
#define MOTOR2_BI_PIN pico::headers[pico::physicalOrder[2]].bi
#define ENC0_A_PIN pico::headers[pico::physicalOrder[0]].encA
#define ENC0_B_PIN pico::headers[pico::physicalOrder[0]].encB
#define ENC1_A_PIN pico::headers[pico::physicalOrder[1]].encA
#define ENC1_B_PIN pico::headers[pico::physicalOrder[1]].encB
#define ENC2_A_PIN pico::headers[pico::physicalOrder[2]].encA
#define ENC2_B_PIN pico::headers[pico::physicalOrder[2]].encB
// Raw count orientation only. Do not inherit an old harness's polarity fix.
#undef ENCODER_DIR_M0
#undef ENCODER_DIR_M1
#undef ENCODER_DIR_M2
#define ENCODER_DIR_M0 (+1)
#define ENCODER_DIR_M1 (+1)
#define ENCODER_DIR_M2 (+1)
