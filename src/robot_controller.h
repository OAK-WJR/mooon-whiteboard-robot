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

private:
    RobotState state_ = STATE_IDLE;

    Kinematics kinematics_{WHEEL_RADIUS, ROBOT_RADIUS};
    Odometry odometry_{kinematics_, ENCODER_CPR};
    TriMotorDriver motors_;
    PIDController pid_[3];
    PathPlanner planner_;
    TrajectoryFollower follower_;

    float max_speed_ = MAX_LINEAR_SPEED;

    void stopMotors();
    void controlLoop(float dt);
};

#endif
