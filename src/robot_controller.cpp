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
    else if (strcmp(cmd, "goto") == 0) {
        // 点到点: 从当前位置直线导航到目标点, 立即执行
        float x = doc["x"] | 0.0f;
        float y = doc["y"] | 0.0f;
        const Pose2D& pose = odometry_.getPose();
        state_ = STATE_PLANNING;
        int n = planner_.generateGoto(pose.x, pose.y, x, y);
        Serial.printf("Goto (%.3f,%.3f)->(%.3f,%.3f): %d pts\n", pose.x, pose.y, x, y, n);
        if (n >= 2) {
            follower_.setPath(planner_.getPath(), planner_.getCount());
            follower_.setSpeed(max_speed_);
            for (int i = 0; i < 3; i++) pid_[i].reset();
            state_ = STATE_RUNNING;
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

    // 即使空闲也刷新实际轮速快照, 方便串口验证编码器
    {
        const WheelVelocity& wv = odometry_.getWheelVelocity();
        for (int i = 0; i < 3; i++) last_actual_[i] = wv.v[i];
    }

    if (state_ != STATE_RUNNING) {
        for (int i = 0; i < 3; i++) { last_target_[i] = 0; last_pwm_[i] = 0; }
        return;
    }

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

        last_target_[i] = target_v;

        if (fabsf(target_v) < MOTOR_MIN_SPEED && fabsf(target_body.vx) < MOTOR_MIN_SPEED
            && fabsf(target_body.vy) < MOTOR_MIN_SPEED) {
            pid_[i].reset();
            motors_.motor(i).stop();
            last_pwm_[i] = 0;
            continue;
        }

        float pwm_f = pid_[i].compute(target_v, actual_v, dt);
        // 静摩擦前馈: 命令运动时叠加固定 PWM, 掀过启动摩擦 (低速也能动)
        if (fabsf(target_v) > MOTOR_MIN_SPEED)
            pwm_f += (target_v > 0 ? MOTOR_STICTION_PWM : -MOTOR_STICTION_PWM);
        int16_t pwm = constrain((int16_t)pwm_f, -PWM_MAX, PWM_MAX);
        motors_.motor(i).setSpeed(pwm);
        last_pwm_[i] = pwm;
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

// ============================================================
// 运行时调参 (串口)
// ============================================================

void RobotController::tunePid(int motor, char which, float val) {
    int from = (motor < 0) ? 0 : motor;
    int to   = (motor < 0) ? 2 : motor;
    if (from < 0 || to > 2) return;

    for (int i = from; i <= to; i++) {
        switch (which) {
            case 'p': pid_[i].setKp(val); break;
            case 'i': pid_[i].setKi(val); break;
            case 'd': pid_[i].setKd(val); break;
            case 'f': pid_[i].setKf(val); break;
            case 'l': pid_[i].setIntegralLimit(val); break;
            default: Serial.printf("ERR 未知增益 '%c'\n", which); return;
        }
    }
    Serial.printf("OK %s K%c=%.2f\n",
                  motor < 0 ? "all" : (String("M") + motor).c_str(), which, val);
}

void RobotController::printParams(Print& out) const {
    out.println(F("------ 可调参数 ------"));
    for (int i = 0; i < 3; i++) {
        out.printf("M%d  Kp=%.1f Ki=%.1f Kd=%.1f Kf=%.1f Ilim=%.0f\n",
                   i, pid_[i].getKp(), pid_[i].getKi(), pid_[i].getKd(),
                   pid_[i].getKf(), pid_[i].getIntegralLimit());
    }
    out.printf("speed=%.3f m/s  lookahead=%.3f m  goal_tol=%.3f m  kp_lin=%.2f\n",
               follower_.getMaxSpeed(), follower_.getLookAhead(),
               follower_.getGoalTolerance(), follower_.getLinearGain());
    out.println(F("----------------------"));
}

void RobotController::printTelemetry(Print& out) const {
    out.printf("M0 t=%.3f a=%.3f pwm=%d | M1 t=%.3f a=%.3f pwm=%d | M2 t=%.3f a=%.3f pwm=%d\n",
               last_target_[0], last_actual_[0], last_pwm_[0],
               last_target_[1], last_actual_[1], last_pwm_[1],
               last_target_[2], last_actual_[2], last_pwm_[2]);
}
