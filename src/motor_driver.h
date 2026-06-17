#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

/**
 * 单个直流电机驱动 (RZ7899-MS 风格)
 * 双引脚控制: FI (正向输入) + BI (反向输入)
 *
 * 正转: FI=PWM, BI=LOW
 * 反转: FI=LOW, BI=PWM
 * 刹车: FI=LOW, BI=LOW
 */
class MotorDriver {
public:
    MotorDriver() = default;

    /**
     * 初始化电机引脚
     * @param fi_pin    正向输入引脚 (FI)
     * @param bi_pin    反向输入引脚 (BI)
     * @param fi_channel ESP32 LEDC 通道 (FI 用)
     * @param bi_channel ESP32 LEDC 通道 (BI 用)
     * @param pwm_freq  PWM 频率
     * @param pwm_res   PWM 分辨率 (bits)
     */
    void begin(uint8_t fi_pin, uint8_t bi_pin,
               uint8_t fi_channel, uint8_t bi_channel,
               uint32_t pwm_freq, uint8_t pwm_res);

    /**
     * 设置电机速度
     * @param speed -255 ~ +255, 正值正转, 负值反转, 0 停止
     */
    void setSpeed(int16_t speed);

    /** 停止 (刹车: FI=LOW, BI=LOW) */
    void stop();

private:
    uint8_t fi_pin_      = 0;
    uint8_t bi_pin_      = 0;
    uint8_t fi_channel_  = 0;
    uint8_t bi_channel_  = 0;
    int16_t pwm_max_     = 255;
};

/**
 * 三电机驱动管理 (3x RZ7899-MS)
 */
class TriMotorDriver {
public:
    void begin();
    void setSpeeds(int16_t s0, int16_t s1, int16_t s2);
    void stopAll();

    MotorDriver& motor(int idx) { return motors_[idx]; }

private:
    MotorDriver motors_[3];
};

#endif // MOTOR_DRIVER_H
