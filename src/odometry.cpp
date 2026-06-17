#include "odometry.h"
#include "config.h"
#include <math.h>
#include <driver/adc.h>  // adc_power_acquire(): 修复 GPIO36/39 噪声 (ESP32 Errata 3.11)

// ============================================================
// 编码器中断 (全局)
// AB 相四倍频计数
//
// 编码器 1 (GPIO36/39) 在 WiFi 模式下可能产生虚假中断
// (ESP32 SAR ADC 硬件 Bug), 用状态机去抖: 只接受合法的
// AB 相状态转移, 忽略非法跳变
// ============================================================

static volatile long enc_counts_[3] = {0, 0, 0};
static constexpr int8_t kEncoderDirSign[3] = {
    ENCODER_DIR_M0, ENCODER_DIR_M1, ENCODER_DIR_M2
};

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
    if (digitalRead(ENC0_A_PIN) == digitalRead(ENC0_B_PIN))
        enc_counts_[0]--;
    else
        enc_counts_[0]++;
}
static void IRAM_ATTR enc0_b_isr() {
    if (digitalRead(ENC0_A_PIN) == digitalRead(ENC0_B_PIN))
        enc_counts_[0]++;
    else
        enc_counts_[0]--;
}

// --- 编码器 1 ISR (GPIO36/39, 状态机去抖) ---
// adc_power_acquire() 消除硬件噪声, 状态机过滤残余毛刺
static void IRAM_ATTR enc1_isr() {
    uint8_t a = digitalRead(ENC1_A_PIN);
    uint8_t b = digitalRead(ENC1_B_PIN);
    uint8_t new_state = (a << 1) | b;
    uint8_t idx = (enc1_prev_state_ << 2) | new_state;
    int8_t delta = ENC_STATE_TABLE[idx];
    if (delta != 0) {
        enc_counts_[1] += delta;
        enc1_prev_state_ = new_state;
    }
}

// --- 编码器 2 ISR (GPIO18/19, 标准四倍频) ---
static void IRAM_ATTR enc2_a_isr() {
    if (digitalRead(ENC2_A_PIN) == digitalRead(ENC2_B_PIN))
        enc_counts_[2]--;
    else
        enc_counts_[2]++;
}
static void IRAM_ATTR enc2_b_isr() {
    if (digitalRead(ENC2_A_PIN) == digitalRead(ENC2_B_PIN))
        enc_counts_[2]++;
    else
        enc_counts_[2]--;
}

void encoders_init() {
    // 修复 ESP32 Errata 3.11: ADC 电源常开, 消除 GPIO36/39 噪声毛刺
    adc_power_acquire();

    // Motor0 编码器: GPIO34/35 — 仅输入引脚, 无内部上拉
    pinMode(ENC0_A_PIN, INPUT);
    pinMode(ENC0_B_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENC0_A_PIN), enc0_a_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC0_B_PIN), enc0_b_isr, CHANGE);

    // Motor1 编码器: GPIO36 + GPIO2 — 实测确认
    // GPIO36: 仅输入, 无内部上拉, adc_power_acquire() 已消除噪声
    // GPIO2: strapping pin, 有内部上拉, 启动时需保持高电平
    pinMode(ENC1_A_PIN, INPUT);          // GPIO36
    pinMode(ENC1_B_PIN, INPUT_PULLUP);   // GPIO2
    enc1_prev_state_ = (digitalRead(ENC1_A_PIN) << 1) | digitalRead(ENC1_B_PIN);
    attachInterrupt(digitalPinToInterrupt(ENC1_A_PIN), enc1_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC1_B_PIN), enc1_isr, CHANGE);

    // Motor2 编码器: GPIO19/18 — 实测确认, 支持内部上拉
    pinMode(ENC2_A_PIN, INPUT_PULLUP);   // GPIO19
    pinMode(ENC2_B_PIN, INPUT_PULLUP);   // GPIO18
    attachInterrupt(digitalPinToInterrupt(ENC2_A_PIN), enc2_a_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_B_PIN), enc2_b_isr, CHANGE);
}

void encoders_read(volatile long counts[3]) {
    noInterrupts();
    counts[0] = enc_counts_[0] * kEncoderDirSign[0];
    counts[1] = enc_counts_[1] * kEncoderDirSign[1];
    counts[2] = enc_counts_[2] * kEncoderDirSign[2];
    interrupts();
}

void encoders_reset() {
    noInterrupts();
    enc_counts_[0] = 0;
    enc_counts_[1] = 0;
    enc_counts_[2] = 0;
    interrupts();
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
    if (dt <= 0.0f) return;

    // 读取当前编码器计数
    volatile long counts[3];
    encoders_read(counts);

    // 计算各轮线速度
    // 四倍频: 实际脉冲数 = encoder_cpr_ * 4
    float cpr4 = (float)(encoder_cpr_ * 4);
    float wheel_circumference = 2.0f * M_PI * kin_.getWheelRadius();

    WheelVelocity wv;
    for (int i = 0; i < 3; i++) {
        long delta = counts[i] - prev_counts_[i];
        prev_counts_[i] = counts[i];

        float revolutions = (float)delta / cpr4;
        float raw_vel = (revolutions * wheel_circumference) / dt;

        // 轮速合理性闸: 超过物理最大速度 = 编码器噪声/串扰 (尤其 M1 PWM 串扰
        // 进 ENC1), 丢弃这帧 (按 0 处理), 防止幽灵恒速把 PID 顶到饱和。
        if (fabsf(raw_vel) > MAX_WHEEL_VEL) raw_vel = 0.0f;

        // 一阶低通滤波: 平滑轮速测量，减少 PID 对噪声的过度反应
        // alpha=0.3 → 截止频率 ≈ 50Hz * 0.3 / (2π) ≈ 2.4Hz
        constexpr float VELOCITY_FILTER_ALPHA = 0.3f;
        wv.v[i] = VELOCITY_FILTER_ALPHA * raw_vel
                 + (1.0f - VELOCITY_FILTER_ALPHA) * wheel_velocity_.v[i];
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
