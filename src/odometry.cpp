#include "odometry.h"
#include "firmware_config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
// History: this file used to #include <driver/adc.h> and call adc_power_acquire() to suppress
// ESP32 Errata 3.11 noise on v1 board GPIO36/39. The v2 board moved ENC1 to GPIO22/23,
// and the API is gone in Arduino core 3.x / new IDF, so it was removed.

// ============================================================
// Encoder interrupts (global)
// A/B phase 4x (quadrature) counting
//
// Encoder 1 (GPIO36/39) may get false interrupts in WiFi mode
// (ESP32 SAR ADC hardware bug); debounce with a state machine: only accept valid
// A/B phase state transitions, ignore invalid jumps
// ============================================================

// Fixed-width modular counters: signed ++/-- is undefined at the limit.
// One lock also makes read/reset consistent with ISRs on dual-core ESP32.
static volatile uint32_t enc_counts_[3] = {0, 0, 0};
static portMUX_TYPE encoder_mux_ = portMUX_INITIALIZER_UNLOCKED;
static constexpr int8_t kEncoderDirSign[3] = {
    ENCODER_DIR_M0, ENCODER_DIR_M1, ENCODER_DIR_M2
};
static_assert((ENCODER_DIR_M0 == 1 || ENCODER_DIR_M0 == -1) &&
              (ENCODER_DIR_M1 == 1 || ENCODER_DIR_M1 == -1) &&
              (ENCODER_DIR_M2 == 1 || ENCODER_DIR_M2 == -1),
              "Encoder directions must be +1 or -1");

// Map the 32-bit window to signed telemetry without an out-of-range cast or
// negating INT32_MIN. The same conversion decodes modular count differences.
static long signed_count(uint32_t value) {
    if (value <= INT32_MAX) return static_cast<long>(value);
    return -1L - static_cast<long>(UINT32_MAX - value);
}

// --- Encoder 1 state machine debounce (GPIO36/39 WiFi interference guard) ---
// Valid A/B phase transitions: 00→01→11→10→00 (forward) / the reverse order (reverse)
// prev_state encoding: (A << 1) | B
static volatile uint8_t enc1_prev_state_ = 0;

// State transition lookup table: index = (prev_state << 2) | new_state
// Values: 0=invalid, +1=forward, -1=reverse
static const int8_t ENC_STATE_TABLE[16] = {
     0, +1, -1,  0,   // prev=00: →01=+1, →10=-1, →00/11=invalid
    -1,  0,  0, +1,   // prev=01: →00=-1, →11=+1, →01/10=invalid
    +1,  0,  0, -1,   // prev=10: →00=+1, →11=-1, →10/01=invalid
     0, -1, +1,  0    // prev=11: →01=-1, →10=+1, →11/00=invalid
};

// --- Encoder 0 ISR (GPIO34/35, standard 4x counting) ---
static void IRAM_ATTR enc0_a_isr() {
    taskENTER_CRITICAL_ISR(&encoder_mux_);
    if (digitalRead(ENC0_A_PIN) == digitalRead(ENC0_B_PIN))
        enc_counts_[0] = enc_counts_[0] - 1U;
    else
        enc_counts_[0] = enc_counts_[0] + 1U;
    taskEXIT_CRITICAL_ISR(&encoder_mux_);
}
static void IRAM_ATTR enc0_b_isr() {
    taskENTER_CRITICAL_ISR(&encoder_mux_);
    if (digitalRead(ENC0_A_PIN) == digitalRead(ENC0_B_PIN))
        enc_counts_[0] = enc_counts_[0] + 1U;
    else
        enc_counts_[0] = enc_counts_[0] - 1U;
    taskEXIT_CRITICAL_ISR(&encoder_mux_);
}

// --- Encoder 1 ISR (GPIO36/39, state machine debounce) ---
// adc_power_acquire() removes hardware noise; the state machine filters leftover glitches
static void IRAM_ATTR enc1_isr() {
    taskENTER_CRITICAL_ISR(&encoder_mux_);
    uint8_t a = digitalRead(ENC1_A_PIN);
    uint8_t b = digitalRead(ENC1_B_PIN);
    uint8_t new_state = (a << 1) | b;
    uint8_t idx = (enc1_prev_state_ << 2) | new_state;
    int8_t delta = ENC_STATE_TABLE[idx];
    if (delta != 0) {
        if (delta > 0) enc_counts_[1] = enc_counts_[1] + 1U;
        else enc_counts_[1] = enc_counts_[1] - 1U;
        enc1_prev_state_ = new_state;
    }
    taskEXIT_CRITICAL_ISR(&encoder_mux_);
}

// --- Encoder 2 ISR (GPIO18/19, standard 4x counting) ---
static void IRAM_ATTR enc2_a_isr() {
    taskENTER_CRITICAL_ISR(&encoder_mux_);
    if (digitalRead(ENC2_A_PIN) == digitalRead(ENC2_B_PIN))
        enc_counts_[2] = enc_counts_[2] - 1U;
    else
        enc_counts_[2] = enc_counts_[2] + 1U;
    taskEXIT_CRITICAL_ISR(&encoder_mux_);
}
static void IRAM_ATTR enc2_b_isr() {
    taskENTER_CRITICAL_ISR(&encoder_mux_);
    if (digitalRead(ENC2_A_PIN) == digitalRead(ENC2_B_PIN))
        enc_counts_[2] = enc_counts_[2] + 1U;
    else
        enc_counts_[2] = enc_counts_[2] - 1U;
    taskEXIT_CRITICAL_ISR(&encoder_mux_);
}

void encoders_init() {
#if defined(CONFIG_IDF_TARGET_ESP32C6) || defined(WBB_TOP_BOARD_PICO)
    // C6/Pico profile encoder pins support input pullups. The Pico mapping is
    // GPIO10/11 here; UART43/44 and ARM16 are never encoder inputs.
    pinMode(ENC0_A_PIN, INPUT_PULLUP);
    pinMode(ENC0_B_PIN, INPUT_PULLUP);
#else
    // Motor0 encoder: GPIO34/35 — input-only pins, no internal pull-up (driven by the encoder's push-pull output)
    pinMode(ENC0_A_PIN, INPUT);
    pinMode(ENC0_B_PIN, INPUT);
#endif
    attachInterrupt(digitalPinToInterrupt(ENC0_A_PIN), enc0_a_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC0_B_PIN), enc0_b_isr, CHANGE);

    // Motor1 encoder: GPIO22/23 (v2 board) — normal digital pins, both support internal pull-up, no external resistor needed
    pinMode(ENC1_A_PIN, INPUT_PULLUP);   // GPIO22
    pinMode(ENC1_B_PIN, INPUT_PULLUP);   // GPIO23
    taskENTER_CRITICAL(&encoder_mux_);
    enc1_prev_state_ = (digitalRead(ENC1_A_PIN) << 1) | digitalRead(ENC1_B_PIN);
    taskEXIT_CRITICAL(&encoder_mux_);
    attachInterrupt(digitalPinToInterrupt(ENC1_A_PIN), enc1_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC1_B_PIN), enc1_isr, CHANGE);

    // Motor2 encoder: GPIO19/18 — confirmed by measurement, supports internal pull-up
    pinMode(ENC2_A_PIN, INPUT_PULLUP);   // GPIO19
    pinMode(ENC2_B_PIN, INPUT_PULLUP);   // GPIO18
    attachInterrupt(digitalPinToInterrupt(ENC2_A_PIN), enc2_a_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_B_PIN), enc2_b_isr, CHANGE);
}

void encoders_read(volatile long counts[3]) {
    uint32_t snapshot[3];
    taskENTER_CRITICAL(&encoder_mux_);
    for (int i = 0; i < 3; ++i) snapshot[i] = enc_counts_[i];
    taskEXIT_CRITICAL(&encoder_mux_);
    for (int i = 0; i < 3; ++i) {
        uint32_t directed = kEncoderDirSign[i] > 0 ? snapshot[i] : uint32_t(0) - snapshot[i];
        counts[i] = signed_count(directed);
    }
}

void encoders_reset() {
    taskENTER_CRITICAL(&encoder_mux_);
    enc_counts_[0] = 0;
    enc_counts_[1] = 0;
    enc_counts_[2] = 0;
    taskEXIT_CRITICAL(&encoder_mux_);
}

// ============================================================
// Odometry class
// ============================================================

Odometry::Odometry(const Kinematics& kin, int encoder_cpr)
    : kin_(kin)
    , encoder_cpr_(encoder_cpr) {}

void Odometry::begin() {
    encoders_init();
    reset();
}

void Odometry::reset() {
    encoders_reset();
    pose_.x = 0; pose_.y = 0; pose_.theta = 0;
    velocity_.vx = 0; velocity_.vy = 0; velocity_.omega = 0;
    wheel_velocity_.v[0] = 0; wheel_velocity_.v[1] = 0; wheel_velocity_.v[2] = 0;
    prev_counts_[0] = 0;
    prev_counts_[1] = 0;
    prev_counts_[2] = 0;
}

void Odometry::update(float dt) {
    if (!isfinite(dt) || dt <= 0.0f) return;

    // Read the current encoder counts
    volatile long counts[3];
    encoders_read(counts);

    // Compute each wheel's linear speed
    // 4x counting: actual pulse count = encoder_cpr_ * 4
    float cpr4 = static_cast<float>(encoder_cpr_) * 4.0f;
    float wheel_circumference = 2.0f * M_PI * kin_.getWheelRadius();

    WheelVelocity wv;
    for (int i = 0; i < 3; i++) {
        // Unsigned subtraction preserves a small positive/negative movement at
        // both the signed reporting boundary and the full 32-bit rollover.
        uint32_t current = static_cast<uint32_t>(counts[i]);
        long delta = signed_count(current - prev_counts_[i]);
        prev_counts_[i] = current;

        float revolutions = (float)delta / cpr4;
        float raw_vel = (revolutions * wheel_circumference) / dt;

        // Wheel speed sanity gate: above the physical max speed = encoder noise/crosstalk (especially M1 PWM crosstalk
        // into ENC1); drop this frame (treat as 0) so a phantom constant speed can't push the PID into saturation.
        if (fabsf(raw_vel) > MAX_WHEEL_VEL) raw_vel = 0.0f;

        // First-order low-pass filter: smooths the wheel speed measurement, so the PID overreacts less to noise.
        // M1 (index 1, ENC1 GPIO36/39) has heavy crosstalk noise; flatten it with a heavier low-pass (smaller alpha),
        // otherwise the PID amplifies the noise into erratic PWM (measured jerk 56 vs 6/12 for M0/M2).
        const float alpha = (i == 1) ? 0.15f : 0.3f;
        wv.v[i] = alpha * raw_vel + (1.0f - alpha) * wheel_velocity_.v[i];
    }

    wheel_velocity_ = wv;

    // Forward kinematics: wheel speeds -> body velocity
    velocity_ = kin_.forward(wv);

    // Pose integration (midpoint integration)
    float half_dtheta = velocity_.omega * dt * 0.5f;
    float cos_th = cosf(pose_.theta + half_dtheta);
    float sin_th = sinf(pose_.theta + half_dtheta);

    pose_.x     += (velocity_.vx * cos_th - velocity_.vy * sin_th) * dt;
    pose_.y     += (velocity_.vx * sin_th + velocity_.vy * cos_th) * dt;
    pose_.theta += velocity_.omega * dt;

    // Normalize the angle to [-PI, PI]
    while (pose_.theta >  M_PI) pose_.theta -= 2.0f * M_PI;
    while (pose_.theta < -M_PI) pose_.theta += 2.0f * M_PI;
}

void Odometry::getEncoderCounts(volatile long counts[3]) const {
    encoders_read(counts);
}
