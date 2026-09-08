#include "odometry.h"
#include "firmware_config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
// 历史: 这里曾 #include <driver/adc.h> 调 adc_power_acquire(), 用来压 v1 板
// GPIO36/39 上的 ESP32 Errata 3.11 噪声。v2 板已把 ENC1 挪到 GPIO22/23,
// 且该 API 在 Arduino core 3.x / 新 IDF 中已移除, 故删除。

// ============================================================
// 编码器中断 (全局)
// AB 相四倍频计数
//
// 编码器 1 (GPIO36/39) 在 WiFi 模式下可能产生虚假中断
// (ESP32 SAR ADC 硬件 Bug), 用状态机去抖: 只接受合法的
// AB 相状态转移, 忽略非法跳变
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

// --- 编码器 1 状态机去抖 (GPIO36/39 WiFi 干扰防护) ---
// AB 相合法状态转移表: 00→01→11→10→00 (正转) / 反向 (反转)
// prev_state 编码: (A << 1) | B
static volatile uint8_t enc1_prev_state_ = 0;

// 状态转移查找表: index = (prev_state << 2) | new_state
// 值: 0=非法, +1=正转, -1=反转
static const int8_t ENC_STATE_TABLE[16] = {
     0, +1, -1,  0,   // prev=00: →01=+1, →10=-1, →00/11=无效
    -1,  0,  0, +1,   // prev=01: →00=-1, →11=+1, →01/10=无效
    +1,  0,  0, -1,   // prev=10: →00=+1, →11=-1, →10/01=无效
     0, -1, +1,  0    // prev=11: →01=-1, →10=+1, →11/00=无效
};

// --- 编码器 0 ISR (GPIO34/35, 标准四倍频) ---
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

// --- 编码器 1 ISR (GPIO36/39, 状态机去抖) ---
// adc_power_acquire() 消除硬件噪声, 状态机过滤残余毛刺
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

// --- 编码器 2 ISR (GPIO18/19, 标准四倍频) ---
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
    // Motor0 编码器: GPIO34/35 — 仅输入引脚, 无内部上拉(靠编码器推挽输出驱动)
    pinMode(ENC0_A_PIN, INPUT);
    pinMode(ENC0_B_PIN, INPUT);
#endif
    attachInterrupt(digitalPinToInterrupt(ENC0_A_PIN), enc0_a_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC0_B_PIN), enc0_b_isr, CHANGE);

    // Motor1 编码器: GPIO22/23 (v2 板) — 普通数字脚, 均支持内部上拉, 免外部电阻
    pinMode(ENC1_A_PIN, INPUT_PULLUP);   // GPIO22
    pinMode(ENC1_B_PIN, INPUT_PULLUP);   // GPIO23
    taskENTER_CRITICAL(&encoder_mux_);
    enc1_prev_state_ = (digitalRead(ENC1_A_PIN) << 1) | digitalRead(ENC1_B_PIN);
    taskEXIT_CRITICAL(&encoder_mux_);
    attachInterrupt(digitalPinToInterrupt(ENC1_A_PIN), enc1_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC1_B_PIN), enc1_isr, CHANGE);

    // Motor2 编码器: GPIO19/18 — 实测确认, 支持内部上拉
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
// Odometry 类
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

    // 读取当前编码器计数
    volatile long counts[3];
    encoders_read(counts);

    // 计算各轮线速度
    // 四倍频: 实际脉冲数 = encoder_cpr_ * 4
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

        // 轮速合理性闸: 超过物理最大速度 = 编码器噪声/串扰 (尤其 M1 PWM 串扰
        // 进 ENC1), 丢弃这帧 (按 0 处理), 防止幽灵恒速把 PID 顶到饱和。
        if (fabsf(raw_vel) > MAX_WHEEL_VEL) raw_vel = 0.0f;

        // 一阶低通滤波: 平滑轮速测量，减少 PID 对噪声的过度反应。
        // M1(索引1, ENC1 GPIO36/39) 串扰噪声大, 用更重的低通(更小 alpha)压平,
        // 否则噪声经 PID 放大成 PWM 乱跳(实测 jerk 56 vs M0/M2 的 6/12)。
        const float alpha = (i == 1) ? 0.15f : 0.3f;
        wv.v[i] = alpha * raw_vel + (1.0f - alpha) * wheel_velocity_.v[i];
    }

    wheel_velocity_ = wv;

    // 正运动学: 轮速 -> 机体速度
    velocity_ = kin_.forward(wv);

    // 位姿积分 (中间值积分)
    float half_dtheta = velocity_.omega * dt * 0.5f;
    float cos_th = cosf(pose_.theta + half_dtheta);
    float sin_th = sinf(pose_.theta + half_dtheta);

    pose_.x     += (velocity_.vx * cos_th - velocity_.vy * sin_th) * dt;
    pose_.y     += (velocity_.vx * sin_th + velocity_.vy * cos_th) * dt;
    pose_.theta += velocity_.omega * dt;

    // 归一化角度到 [-PI, PI]
    while (pose_.theta >  M_PI) pose_.theta -= 2.0f * M_PI;
    while (pose_.theta < -M_PI) pose_.theta += 2.0f * M_PI;
}

void Odometry::getEncoderCounts(volatile long counts[3]) const {
    encoders_read(counts);
}
