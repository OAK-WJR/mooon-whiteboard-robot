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
#if !defined(ARDUINO_USB_CDC_ON_BOOT) || ARDUINO_USB_CDC_ON_BOOT != 1
#error "KISS C6 has no USB-UART bridge (CH343G removed 2026-10-29): Serial is the C6's native USB CDC"
#endif

#include <stdint.h>
namespace kiss_c6 {
constexpr const char* profile = "top_board_v2_kiss_c6_review";
// GPIO map follows the 2026-10-27 escape re-route: each header takes the module pads nearest to it.
constexpr int motorArmRequest = 0;
// Factory ADC conversion uses these pins. Divider calibration/off-state review pending.
// batterySense: VBAT divider (120k/10k). motorSense (2026-10-28 board rev): total motor current from INA180A1
// (gain 20) on the 0.1 ohm VMOT shunt R49, via 1k/100nF -> 2.0 V per A. Linear (datasheet gain error) to ~1.4 A
// (VOUT <= VS-0.5 V); the output rails near 3.27 V (~1.6 A). It is no longer VMOT.
constexpr int batterySense = 4, motorSense = 5;
// Sensor head (J1), 2026-10-29: GPIO12/13 are the native USB again, so I2C SDA moved to the GPIO15 strap (sampled
// only if the JTAG-select eFuse is burned) and SCL to GPIO16 (the ROM boot log toggles it with SDA high: no START).
// The IR LED switch moved to GPIO17, which stays quiet at boot; R25 still pulls it up (IR off).
constexpr int i2cSda = 15, i2cScl = 16, irEnableN = 17;
// USB power sense (2026-10-29): VBUS -> R52 1k -> USB_DIV -> R10 2.2k -> GND, isolated from IO8 by Q1 (2N7002, gate 3V3,
// drain USB_DIV, source IO8) with R53 100k IO8 -> 3V3. USB present: USB_DIV ~3.4 V (3.0 V at a 4.4 V sag), Q1 off, IO8 = 3.3 V.
// No USB: VBUS floats except for D1's reverse leakage (SS14, up to ~0.3 mA hot), which the stiff divider turns into <=0.66 V,
// so Q1 conducts and IO8 <= 0.72 V (< VIL 0.825 V). 3V3 off while charging: Q1 off, nothing drives the unpowered IO8.
// IO8 is a strap that only matters for manual download mode, which always has USB plugged in (so it reads high).
// The board reports "battery" vs "external_power" over USB and Wi-Fi; whether the IP2326 is still charging is not
// observable (no charger status pin reaches the MCU). Charging draws ~1.2 A at 5 V (0.6 A into the 2S pack). Since
// 2026-10-30 a CC gate (LM393 U8 vs a 0.65 V CJ431 threshold, Q2 on IP2326 EN) enables charging only on a USB-C source
// advertising >=1.5 A; a Default-USB source or a USB-A cable powers the board but does not charge.
constexpr int usbPowerSense = 8;
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
                        headers[2].fi,headers[2].bi,headers[2].encA,headers[2].encB,i2cScl,irEnableN,
                        motorArmRequest,batterySense,motorSense};
    for (unsigned i=0; i<sizeof(used)/sizeof(used[0]); ++i) {
        if (used[i]<0 || used[i]>23 || used[i]==8 || used[i]==9 ||
            used[i]==10 || used[i]==11 || used[i]==12 || used[i]==13 || used[i]==15) return false;
        for (unsigned j=0; j<i; ++j) if (used[i]==used[j]) return false;
    }
    return true;
}
static_assert(pinsAreDistinct(), "KISS C6 pins alias reserved or application pins");
static_assert(i2cSda == 15 && !isMotorPin(i2cSda) && i2cSda != i2cScl && i2cSda != irEnableN,
              "SDA is the only application pin allowed on the GPIO15 strap");
static_assert(usbPowerSense == 8 && !isMotorPin(usbPowerSense), "USB power sense is the only input on the GPIO8 strap");
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
