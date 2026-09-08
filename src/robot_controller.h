#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "kinematics.h"
#include "odometry.h"
#include "motor_driver.h"
#include "pid_controller.h"
#include "path_planner.h"
#include "trajectory_follower.h"
#include <ArduinoJson.h>
#include "firmware_config.h"

enum RobotState {
    STATE_IDLE,
    STATE_PLANNING,
    STATE_READY,
    STATE_RUNNING,
    STATE_DONE,
    STATE_WHEEL_TEST    // 单轮定速测试 (PID 调参用)
};

class RobotController {
public:
    void begin();

    // Hardware/task initialization failures must be able to stop all outputs.
    void stopMotors();
    void update(float dt);           // 50Hz 控制循环调用
    void handleCommand(JsonDocument& doc);  // 处理 UDP 命令
    void fillStatus(JsonDocument& doc);     // 填充状态回传

    RobotState getState() const { return state_; }

    // --- 运行时调参 (串口) ---
    // motor: -1=全部三轮, 0..2=单轮; which: 'p' 'i' 'd' 'f' 'l'(I限幅)
    void tunePid(int motor, char which, float val);
    void setLookAhead(float d)     { follower_.setLookAhead(d); }
    void setGoalTolerance(float d) { follower_.setGoalTolerance(d); }
    void setLinearGain(float kp)   { follower_.setLinearGain(kp); }
    void printParams(Print& out) const;     // 打印所有可调参数
    void printTelemetry(Print& out) const;  // 打印每轮 目标/实际/PWM
    void printGains(Print& out) const;      // 机读增益行 (调参器解析)

    // 单轮定速: idx 0..2 让该轮定速跟踪 v (m/s), 其余轮停; idx<0 退出测试。
    // 跑的是与导航完全相同的 FF+PID+静摩擦+slew 管线, 供实时调参。
    void setWheelTest(int idx, float v);

private:
    RobotState state_ = STATE_IDLE;

    Kinematics kinematics_{WHEEL_RADIUS, ROBOT_RADIUS};
    Odometry odometry_{kinematics_, ENCODER_CPR};
    TriMotorDriver motors_;
    PIDController pid_[3];
    PathPlanner planner_;
    TrajectoryFollower follower_;

    float max_speed_ = MAX_LINEAR_SPEED;

    // 单轮测试状态
    int   wheel_test_idx_ = -1;
    float wheel_test_v_   = 0.0f;

    // 遥测快照 (control loop 每拍更新, 供串口打印)
    float   last_target_[3] = {0, 0, 0};
    float   last_actual_[3] = {0, 0, 0};
    int16_t last_pwm_[3]    = {0, 0, 0};

    // 堵转保护状态 (每轮独立)
    uint16_t      stall_cnt_[3]    = {0, 0, 0};  // 连续"用力但不动"的拍数
    uint8_t       stall_trips_[3]  = {0, 0, 0};  // 已触发次数, 达 STALL_MAX_TRIPS 则锁定
    uint32_t      stall_started_[3] = {0, 0, 0};  // 冷却开始时刻，按 32 位回绕求已用时间
    bool          stall_cooling_[3] = {false, false, false};  // 时间值 0 也是合法时刻

    // 目标轮速 -> FF+PID+静摩擦+slew -> PWM (导航与单轮测试共用同一管线)
    void driveWheels(const WheelVelocity& target, const BodyVelocity& body, float dt);
};

#endif
