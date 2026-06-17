#ifndef KINEMATICS_H
#define KINEMATICS_H

/**
 * 三轮全向轮运动学 (120° 配置)
 *
 * 轮子布局 (俯视):
 *        wheel1 (240°/后)
 *        /    \
 *   wheel0    wheel2
 *   (120°)    (0°/360°)
 *
 * 正运动学: 轮速 -> 机体速度
 * 逆运动学: 机体速度 -> 轮速
 */

struct BodyVelocity {
    float vx;    // m/s, 前进方向
    float vy;    // m/s, 左方向
    float omega; // rad/s, 逆时针为正
};

struct WheelVelocity {
    float v[3];  // 三个轮的线速度 (m/s)
};

class Kinematics {
public:
    /**
     * @param wheel_radius  轮半径 (m)
     * @param robot_radius  轮心到底盘中心距离 (m)
     */
    Kinematics(float wheel_radius, float robot_radius);

    /** 逆运动学: 机体速度 -> 轮线速度 */
    WheelVelocity inverse(const BodyVelocity& body) const;

    /** 正运动学: 轮线速度 -> 机体速度 */
    BodyVelocity forward(const WheelVelocity& wheels) const;

    float getWheelRadius() const { return wheel_radius_; }
    float getRobotRadius() const { return robot_radius_; }

private:
    float wheel_radius_;
    float robot_radius_;

    // 逆运动学矩阵系数 (预计算)
    float sin60_;
    float cos60_;
};

#endif // KINEMATICS_H
