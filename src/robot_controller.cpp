#include "robot_controller.h"
#include <Arduino.h>

static const char* stateStr(RobotState s) {
    switch (s) {
        case STATE_IDLE:     return "idle";
        case STATE_PLANNING: return "planning";
        case STATE_READY:    return "ready";
        case STATE_RUNNING:  return "running";
        case STATE_DONE:     return "done";
        default:             return "unknown";
    }
}

void RobotController::begin() {
    motors_.begin();
    odometry_.begin();

    const float pid_kp[3]   = {PID_KP_M0, PID_KP_M1, PID_KP_M2};
    const float pid_ki[3]   = {PID_KI_M0, PID_KI_M1, PID_KI_M2};
    const float pid_kd[3]   = {PID_KD_M0, PID_KD_M1, PID_KD_M2};
    const float pid_kf[3]   = {PID_KF_M0, PID_KF_M1, PID_KF_M2};
    const float pid_ilim[3] = {PID_ILIM_M0, PID_ILIM_M1, PID_ILIM_M2};

    for (int i = 0; i < 3; i++) {
        pid_[i].setGains(pid_kp[i], pid_ki[i], pid_kd[i], pid_kf[i]);
        pid_[i].setOutputLimits(-PWM_MAX, PWM_MAX);
        pid_[i].setIntegralLimit(pid_ilim[i]);
        pid_[i].setSlewRate(1500.0f);
    }

    Serial.println("RobotController initialized.");
}

void RobotController::stopMotors() {
    for (int i = 0; i < 3; i++) {
        pid_[i].reset();
        motors_.motor(i).stop();
    }
}

void RobotController::handleCommand(JsonDocument& doc) {
    const char* cmd = doc["cmd"] | "";

    if (strcmp(cmd, "area") == 0) {
        float x = doc["x"] | 0.0f;
        float y = doc["y"] | 0.0f;
        float w = doc["w"] | 0.1f;
        float h = doc["h"] | 0.1f;
        float spacing = doc["spacing"] | 0.02f;

        state_ = STATE_PLANNING;
        int n = planner_.generateZigzag(x, y, w, h, spacing);
        Serial.printf("Path: %d waypoints for area (%.2f,%.2f,%.2f,%.2f) spacing=%.3f\n",
                      n, x, y, w, h, spacing);

        if (n > 0) {
            follower_.setPath(planner_.getPath(), planner_.getCount());
            follower_.setSpeed(max_speed_);
            state_ = STATE_READY;
        } else {
            state_ = STATE_IDLE;
        }
    }
    else if (strcmp(cmd, "start") == 0) {
        if (state_ == STATE_READY || state_ == STATE_DONE) {
            follower_.setPath(planner_.getPath(), planner_.getCount());
            follower_.setSpeed(max_speed_);
            for (int i = 0; i < 3; i++) pid_[i].reset();
            state_ = STATE_RUNNING;
            Serial.println("Running!");
        }
    }
    else if (strcmp(cmd, "stop") == 0) {
        stopMotors();
        follower_.reset();
        state_ = STATE_IDLE;
        Serial.println("Stopped.");
    }
    else if (strcmp(cmd, "pos") == 0) {
        // 视觉位置校正
        float x = doc["x"] | 0.0f;
        float y = doc["y"] | 0.0f;
        float theta = doc["theta"] | 0.0f;
        odometry_.setPose(x, y, theta);
        Serial.printf("Pose corrected: (%.3f, %.3f, %.3f)\n", x, y, theta);
    }
    else if (strcmp(cmd, "set_speed") == 0) {
        max_speed_ = doc["v"] | MAX_LINEAR_SPEED;
        follower_.setSpeed(max_speed_);
        Serial.printf("Speed set: %.3f m/s\n", max_speed_);
    }
    else if (strcmp(cmd, "status") == 0) {
        // status 请求，fillStatus 会处理
    }
}

void RobotController::update(float dt) {
    odometry_.update(dt);

    if (state_ != STATE_RUNNING) return;

    const Pose2D& pose = odometry_.getPose();
    BodyVelocity target_body;

    bool running = follower_.compute(pose.x, pose.y, pose.theta, target_body);

    if (!running) {
        stopMotors();
        state_ = STATE_DONE;
        Serial.println("Path complete.");
        return;
    }

    // 逆运动学 → 轮速度
    WheelVelocity target_wheels = kinematics_.inverse(target_body);
    const WheelVelocity& actual_wheels = odometry_.getWheelVelocity();

    // FF+PID
    for (int i = 0; i < 3; i++) {
        float target_v = target_wheels.v[i];
        float actual_v = actual_wheels.v[i];

        if (fabsf(target_v) < MOTOR_MIN_SPEED && fabsf(target_body.vx) < MOTOR_MIN_SPEED
            && fabsf(target_body.vy) < MOTOR_MIN_SPEED) {
            pid_[i].reset();
            motors_.motor(i).stop();
            continue;
        }

        float pwm_f = pid_[i].compute(target_v, actual_v, dt);
        int16_t pwm = constrain((int16_t)pwm_f, -PWM_MAX, PWM_MAX);
        motors_.motor(i).setSpeed(pwm);
    }
}

void RobotController::fillStatus(JsonDocument& doc) {
    const Pose2D& pose = odometry_.getPose();
    doc["state"] = stateStr(state_);
    doc["x"] = pose.x;
    doc["y"] = pose.y;
    doc["theta"] = pose.theta;
    doc["progress"] = follower_.getProgress();
    doc["wp"] = follower_.getCurrentWaypoint();
    doc["wp_total"] = follower_.getTotalWaypoints();
}
