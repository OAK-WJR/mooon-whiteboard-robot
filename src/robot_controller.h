#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "kinematics.h"
#include "odometry.h"
#include "motor_driver.h"
#include "pid_controller.h"
#include "path_planner.h"
#include "trajectory_follower.h"
#include "wifi_comm.h"
#include "config.h"

enum RobotState {
    STATE_IDLE,
    STATE_PLANNING,
    STATE_READY,
    STATE_RUNNING,
    STATE_DONE
};

class RobotController {
public:
    void begin();
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

private:
    RobotState state_ = STATE_IDLE;

    Kinematics kinematics_{WHEEL_RADIUS, ROBOT_RADIUS};
    Odometry odometry_{kinematics_, ENCODER_CPR};
    TriMotorDriver motors_;
    PIDController pid_[3];
    PathPlanner planner_;
    TrajectoryFollower follower_;

    float max_speed_ = MAX_LINEAR_SPEED;

    // 遥测快照 (control loop 每拍更新, 供串口打印)
    float   last_target_[3] = {0, 0, 0};
    float   last_actual_[3] = {0, 0, 0};
    int16_t last_pwm_[3]    = {0, 0, 0};

    void stopMotors();
};

#endif
