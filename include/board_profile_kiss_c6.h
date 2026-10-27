#pragma once
// C6-MINI top schematic; power/PCB/harness not released.
#if !defined(WBB_TOP_BOARD_KISS_C6) || WBB_TOP_BOARD_KISS_C6 != 1
#error "Select the explicit top_board_v2_kiss_c6 environment"
#endif
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO)
#error "KISS C6 cannot be combined with another board profile"
#endif
#if !defined(CONFIG_IDF_TARGET_ESP32C6) || !CONFIG_IDF_TARGET_ESP32C6 || defined(CONFIG_IDF_TARGET_ESP32S3)
#error "KISS C6 requires ESP32-C6"
#endif
#if !defined(WBB_TOP_BOARD_V2) || WBB_TOP_BOARD_V2 != 1 || defined(WBB_R3_DIAGNOSTIC) || defined(MOTOR_TEST_MODE)
#error "KISS C6 requires the gated integrated runtime"
#endif
#if !defined(ARDUINO_USB_CDC_ON_BOOT) || ARDUINO_USB_CDC_ON_BOOT != 0
#error "KISS C6 CH343G requires UART0 Serial"
#endif

#include <stdint.h>
namespace kiss_c6 {
constexpr const char* profile = "top_board_v2_kiss_c6_review";
constexpr int uartTx = 16, uartRx = 17;
// GPIO map follows the 2026-10-27 escape re-route: each header takes the module pads nearest to it.
constexpr int motorArmRequest = 0;
// Factory ADC conversion uses these pins. Divider calibration/off-state review pending.
constexpr int batterySense = 4, motorSense = 5;
// Sensor head (J1): I2C on the unused native-USB pads, IR LED switch on the GPIO15 strap (R25 pull-up = off at boot).
constexpr int i2cSda = 12, i2cScl = 13, irEnableN = 15;
struct MotorPins { uint8_t fi, bi, encA, encB, physical; const char* header; };
constexpr MotorPins headers[] = {
    {22, 23, 20, 21, 1, "M0"},
    {1, 6, 2, 3, 3, "M1"},
    {14, 7, 19, 18, 2, "M2"},
};
constexpr int physicalOrder[] = {0, 2, 1};
constexpr bool isMotorPin(int pin) {
    for (const auto& p : headers) if (pin == p.fi || pin == p.bi) return true;
    return false;
}
constexpr bool isMotorPair(int fi, int bi) {
    for (const auto& p : headers) if (fi == p.fi && bi == p.bi) return true;
    return false;
}
constexpr bool pinsAreDistinct() {
    const int used[] = {headers[0].fi,headers[0].bi,headers[0].encA,headers[0].encB,
                        headers[1].fi,headers[1].bi,headers[1].encA,headers[1].encB,
                        headers[2].fi,headers[2].bi,headers[2].encA,headers[2].encB,uartTx,uartRx,
                        motorArmRequest,batterySense,motorSense};
    for (unsigned i=0; i<sizeof(used)/sizeof(used[0]); ++i) {
        if (used[i]<0 || used[i]>23 || used[i]==8 || used[i]==9 ||
            used[i]==10 || used[i]==11 || used[i]==12 || used[i]==13 || used[i]==15) return false;
        for (unsigned j=0; j<i; ++j) if (used[i]==used[j]) return false;
    }
    return true;
}
static_assert(pinsAreDistinct(), "KISS C6 pins alias reserved or application pins");
} // namespace kiss_c6

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
#define MOTOR0_FI_PIN kiss_c6::headers[0].fi
#define MOTOR0_BI_PIN kiss_c6::headers[0].bi
#define MOTOR1_FI_PIN kiss_c6::headers[2].fi
#define MOTOR1_BI_PIN kiss_c6::headers[2].bi
#define MOTOR2_FI_PIN kiss_c6::headers[1].fi
#define MOTOR2_BI_PIN kiss_c6::headers[1].bi
#define ENC0_A_PIN kiss_c6::headers[0].encA
#define ENC0_B_PIN kiss_c6::headers[0].encB
#define ENC1_A_PIN kiss_c6::headers[2].encA
#define ENC1_B_PIN kiss_c6::headers[2].encB
#define ENC2_A_PIN kiss_c6::headers[1].encA
#define ENC2_B_PIN kiss_c6::headers[1].encB
// Raw count orientation; final motor gearing and harness calibration pending.
#undef ENCODER_DIR_M0
#undef ENCODER_DIR_M1
#undef ENCODER_DIR_M2
#define ENCODER_DIR_M0 (+1)
#define ENCODER_DIR_M1 (+1)
#define ENCODER_DIR_M2 (+1)
