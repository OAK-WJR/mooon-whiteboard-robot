#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "kinematics.h"

/**
 * 三轮全向轮里程计
 *
 * 使用 ESP32 硬件中断读取 3 个编码器 AB 相脉冲,
 * 每个控制周期计算轮速并积分得到位姿。
 */

struct Pose2D {
    float x     = 0.0f;  // m
    float y     = 0.0f;  // m
    float theta = 0.0f;  // rad
};

class Odometry {
public:
    /**
     * @param kin        运动学实例引用
     * @param encoder_cpr 编码器每圈脉冲数 (含减速比)
     */
    Odometry(const Kinematics& kin, int encoder_cpr);

    /** 初始化编码器引脚和中断 */
    void begin();

    /** 每个控制周期调用一次, 更新轮速和位姿 */
    void update(float dt);

    /** 获取当前位姿 */
    const Pose2D& getPose() const { return pose_; }

    /** 获取当前机体速度 */
    const BodyVelocity& getVelocity() const { return velocity_; }

    /** 获取各轮线速度 (直接从编码器计算, 用于 PID 反馈) */
    const WheelVelocity& getWheelVelocity() const { return wheel_velocity_; }

    /** 重置位姿 */
    void reset();

    /** 设置位姿 (视觉校正) */
    void setPose(float x, float y, float theta) {
        pose_.x = x; pose_.y = y; pose_.theta = theta;
    }

    /** 方向校正后的 32 位有符号计数窗口；累计越界时回绕 (调试用) */
    void getEncoderCounts(volatile long counts[3]) const;

private:
    const Kinematics& kin_;
    int encoder_cpr_;

    Pose2D pose_;
    BodyVelocity velocity_;
    WheelVelocity wheel_velocity_;

    // 上一次的编码器计数 (用于计算差值)
    // A sample interval must contain fewer than 2^31 net encoder edges.
    uint32_t prev_counts_[3] = {0, 0, 0};
};

// ---- 编码器中断相关 (全局) ----

/** 初始化编码器引脚和中断服务函数 */
void encoders_init();

/** 获取当前编码器计数 (线程安全) */
void encoders_read(volatile long counts[3]);

/** 重置编码器计数 */
void encoders_reset();

#endif // ODOMETRY_H
