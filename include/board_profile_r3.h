#pragma once
// Review profile for kicad/r3_system/core_contract.json, not a released board ID.
// GPIO21 is a request with a 10k pulldown; it is NOT a motor power disconnect.
#if !defined(WBB_TOP_BOARD_R3) || WBB_TOP_BOARD_R3 != 1
#error "Select the explicit R3 review build environment"
#endif
#if !defined(CONFIG_IDF_TARGET_ESP32C6) || !CONFIG_IDF_TARGET_ESP32C6
#error "R3 requires ESP32-C6"
#endif
#if !defined(ARDUINO_USB_CDC_ON_BOOT) || ARDUINO_USB_CDC_ON_BOOT != 0
#error "R3 CP2102N requires UART0 Serial (ARDUINO_USB_CDC_ON_BOOT=0)"
#endif

#include <stdint.h>
namespace r3 {
constexpr const char* profile = "top_board_v2_r3_review";
constexpr int uartTx = 16, uartRx = 17;
constexpr int motorArmRequest = 21;
constexpr int watchdogHeartbeat = 12;
constexpr int i2cSda = 14, i2cScl = 22;
constexpr int led = 15;
// GPIO12 is wired to the external watchdog, reserved for checked open-drain service.
// Its pulse service is not yet implemented; GPIO13 remains NC.
// J2 pin3 is UART RX17, not AUX GPIO6.
// No AUX output or charger/I2C initialization is provided by this profile.
struct MotorPins { uint8_t fi, bi, encA, encB, physical; const char* header; };
// Diagnostic display order is by header, intentionally different from kinematics.
constexpr MotorPins headers[] = {
    {0, 1, 7, 6, 1, "M0"},
    {2, 3, 18, 19, 3, "M1"},
    {4, 5, 20, 23, 2, "M2"},
};
constexpr int physicalOrder[] = {0, 2, 1}; // Physical Motor1/2/3 = M0/M2/M1.
constexpr bool pinsAreDistinct() {
    const int used[] = {0, 1, 7, 6, 2, 3, 18, 19, 4, 5, 20, 23,
                        uartTx, uartRx, motorArmRequest, watchdogHeartbeat, i2cSda, i2cScl, led};
    for (unsigned i = 0; i < sizeof(used) / sizeof(used[0]); ++i) {
        if (used[i] == 13 || used[i] == 8 || used[i] == 9) return false;
        for (unsigned j = 0; j < i; ++j) if (used[i] == used[j]) return false;
    }
    return true;
}
static_assert(pinsAreDistinct(), "R3 pins must not alias UART, NC or boot straps");
} // namespace r3

// Override even a stale board configuration, in every firmware translation unit.
// The R3 environments pre-include public defaults; private config.h stays untouched.
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
#define MOTOR0_FI_PIN r3::headers[0].fi
#define MOTOR0_BI_PIN r3::headers[0].bi
#define MOTOR1_FI_PIN r3::headers[2].fi
#define MOTOR1_BI_PIN r3::headers[2].bi
#define MOTOR2_FI_PIN r3::headers[1].fi
#define MOTOR2_BI_PIN r3::headers[1].bi
#define ENC0_A_PIN r3::headers[0].encA
#define ENC0_B_PIN r3::headers[0].encB
#define ENC1_A_PIN r3::headers[2].encA
#define ENC1_B_PIN r3::headers[2].encB
#define ENC2_A_PIN r3::headers[1].encA
#define ENC2_B_PIN r3::headers[1].encB
