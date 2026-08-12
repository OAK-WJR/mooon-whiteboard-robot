#include "motor_driver.h"
#include "config.h"

// Arduino-ESP32 core 3.x 改了 LEDC 接口: 通道由内核自动分配, 一律按"引脚"操作。
// core 2.x 仍需手工指定通道。用下面两个包装抹平差异 (C6 只能跑 3.x)。
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define LEDC_SETUP(pin, ch, freq, res)  ledcAttach((pin), (freq), (res))
  #define LEDC_WRITE(pin, ch, duty)       ledcWrite((pin), (duty))
#else
  #define LEDC_SETUP(pin, ch, freq, res)  do { ledcSetup((ch), (freq), (res)); \
                                               ledcAttachPin((pin), (ch)); } while (0)
  #define LEDC_WRITE(pin, ch, duty)       ledcWrite((ch), (duty))
#endif

// ---- MotorDriver (RZ7899-MS: FI + BI 双引脚控制) ----

void MotorDriver::begin(uint8_t fi_pin, uint8_t bi_pin,
                         uint8_t fi_channel, uint8_t bi_channel,
                         uint32_t pwm_freq, uint8_t pwm_res) {
    fi_pin_     = fi_pin;
    bi_pin_     = bi_pin;
    fi_channel_ = fi_channel;
    bi_channel_ = bi_channel;
    pwm_max_    = (1 << pwm_res) - 1;

    LEDC_SETUP(fi_pin_, fi_channel_, pwm_freq, pwm_res);
    LEDC_SETUP(bi_pin_, bi_channel_, pwm_freq, pwm_res);

    stop();
}

void MotorDriver::setSpeed(int16_t speed) {
    if (speed > 0) {
        // 正转: FI=PWM, BI=LOW
        LEDC_WRITE(bi_pin_, bi_channel_, 0);
        LEDC_WRITE(fi_pin_, fi_channel_, constrain(speed, 0, pwm_max_));
    } else if (speed < 0) {
        // 反转: FI=LOW, BI=PWM
        LEDC_WRITE(fi_pin_, fi_channel_, 0);
        LEDC_WRITE(bi_pin_, bi_channel_, constrain(-speed, 0, pwm_max_));
    } else {
        stop();
    }
}

void MotorDriver::stop() {
    // 刹车: FI=LOW, BI=LOW
    LEDC_WRITE(fi_pin_, fi_channel_, 0);
    LEDC_WRITE(bi_pin_, bi_channel_, 0);
}

// ---- TriMotorDriver (3x RZ7899-MS) ----

void TriMotorDriver::begin() {
    // 每个 RZ7899 需要 2 个 LEDC 通道 (FI + BI)
    // Motor0: channel 0 (FI), channel 1 (BI)
    motors_[0].begin(MOTOR0_FI_PIN, MOTOR0_BI_PIN,
                     0, 1, PWM_FREQUENCY, PWM_RESOLUTION);
    // Motor1: channel 2 (FI), channel 3 (BI)
    motors_[1].begin(MOTOR1_FI_PIN, MOTOR1_BI_PIN,
                     2, 3, PWM_FREQUENCY, PWM_RESOLUTION);
    // Motor2: channel 4 (FI), channel 5 (BI)
    motors_[2].begin(MOTOR2_FI_PIN, MOTOR2_BI_PIN,
                     4, 5, PWM_FREQUENCY, PWM_RESOLUTION);
}

void TriMotorDriver::setSpeeds(int16_t s0, int16_t s1, int16_t s2) {
    motors_[0].setSpeed(s0);
    motors_[1].setSpeed(s1);
    motors_[2].setSpeed(s2);
}

void TriMotorDriver::stopAll() {
    motors_[0].stop();
    motors_[1].stop();
    motors_[2].stop();
}
