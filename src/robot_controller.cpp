#include "robot_controller.h"
#include <Arduino.h>

static const char* stateStr(RobotState s) {
    switch (s) {
        case STATE_IDLE:     return "idle";
        case STATE_PLANNING: return "planning";
        case STATE_READY:    return "ready";
        case STATE_RUNNING:  return "running";
        case STATE_DONE:     return "done";
        case STATE_WHEEL_TEST: return "wheeltest";
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
        last_pwm_[i]     = 0;
        stall_cnt_[i]    = 0;   // stopping clears the stall state; locked wheels are unlocked here too
        stall_trips_[i]  = 0;
        stall_started_[i] = 0;
        stall_cooling_[i] = false;
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

        stopMotors();
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
        // Point to point: navigate in a straight line from the current position to the target point, run right away
        float x = doc["x"] | 0.0f;
        float y = doc["y"] | 0.0f;
        const Pose2D& pose = odometry_.getPose();
        stopMotors();
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
        // Vision position correction
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
        // status request, fillStatus handles it
    }
}

void RobotController::update(float dt) {
    odometry_.update(dt);

    // Refresh the actual wheel speed snapshot even when idle, to make checking the encoders over serial easy
    {
        const WheelVelocity& wv = odometry_.getWheelVelocity();
        for (int i = 0; i < 3; i++) last_actual_[i] = wv.v[i];
    }

    // Single-wheel fixed-speed test: bypass navigation, make the selected wheel track the set speed directly (for tuning)
    if (state_ == STATE_WHEEL_TEST) {
        WheelVelocity tw = {{0, 0, 0}};
        if (wheel_test_idx_ >= 0 && wheel_test_idx_ <= 2)
            tw.v[wheel_test_idx_] = wheel_test_v_;
        BodyVelocity tb = {wheel_test_v_, 0.0f, 0.0f};  // nonzero -> keeps the active wheel from being stopped by the deadband gate
        driveWheels(tw, tb, dt);
        return;
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

    // Inverse kinematics → wheel speeds → drive pipeline shared with the single-wheel test
    WheelVelocity target_wheels = kinematics_.inverse(target_body);
    driveWheels(target_wheels, target_body, dt);
}

// Target wheel speed -> FF+PID+static friction+slew -> PWM. Shared by navigation and the single-wheel test so both behave the same.
void RobotController::driveWheels(const WheelVelocity& target, const BodyVelocity& body, float dt) {
    if (!motors_.ready()) {
        stopMotors();
        state_ = STATE_IDLE;
        return;
    }
    const WheelVelocity& actual_wheels = odometry_.getWheelVelocity();

    uint32_t now = millis();

    for (int i = 0; i < 3; i++) {
        float target_v = target.v[i];
        float actual_v = actual_wheels.v[i];

        last_target_[i] = target_v;

        if (fabsf(target_v) < MOTOR_MIN_SPEED && fabsf(body.vx) < MOTOR_MIN_SPEED
            && fabsf(body.vy) < MOTOR_MIN_SPEED) {
            pid_[i].reset();
            if (!motors_.motor(i).stop()) { stopMotors(); state_=STATE_IDLE; return; }
            last_pwm_[i] = 0;
            stall_cnt_[i] = 0;              // it was not asked to move, so not a stall
            continue;
        }

        // --- Stall protection ---
        // Keep PWM stopped while cooling down. Uses elapsed time, so a full cooldown is waited out even across millis() wraparound.
        if (stall_cooling_[i]) {
            if (uint32_t(now - stall_started_[i]) < uint32_t(STALL_COOLDOWN_MS)) {
                pid_[i].reset();
                if (!motors_.motor(i).stop()) { stopMotors(); state_=STATE_IDLE; return; }
                last_pwm_[i] = 0;
                continue;
            }
            stall_cooling_[i] = false;
            stall_cnt_[i]    = 0;
            Serial.printf("STALL M%d cooldown over, retrying (attempt %d)\n", i, stall_trips_[i]);
        }
        // Limit reached: lock this wheel until stop/a new command clears it
        if (stall_trips_[i] >= STALL_MAX_TRIPS) {
            pid_[i].reset();
            if (!motors_.motor(i).stop()) { stopMotors(); state_=STATE_IDLE; return; }
            last_pwm_[i] = 0;
            continue;
        }
        // Criterion: pushing (high PWM) but not moving (speed≈0)
        if (abs(last_pwm_[i]) >= STALL_PWM_THRESH && fabsf(actual_v) < STALL_VEL_THRESH) {
            if (++stall_cnt_[i] >= STALL_TRIP_TICKS) {
                stall_trips_[i]++;
                stall_started_[i] = now;
                stall_cooling_[i] = true;
                stall_cnt_[i]    = 0;
                pid_[i].reset();
                if (!motors_.motor(i).stop()) { stopMotors(); state_=STATE_IDLE; return; }
                last_pwm_[i] = 0;
                Serial.printf("STALL M%d Stall! PWM=%d but wheel speed≈0 for %dms -> PWM off %dms (total %d/%d)\n",
                              i, STALL_PWM_THRESH, STALL_TRIP_TICKS * CONTROL_PERIOD_MS,
                              STALL_COOLDOWN_MS, stall_trips_[i], STALL_MAX_TRIPS);
                if (stall_trips_[i] >= STALL_MAX_TRIPS)
                    Serial.printf("STALL M%d stalled %d times in a row, wheel locked (send x/stop to reset)\n",
                                  i, STALL_MAX_TRIPS);
                continue;
            }
        } else {
            stall_cnt_[i] = 0;              // clear once it is turning
        }

        float pwm_f = pid_[i].compute(target_v, actual_v, dt);
        // Smooth Coulomb friction feedforward: full breakaway compensation at rest, removed linearly as speed builds (no hard switch -> no jitter).
        // frac: 1 at |actual|=0, 0 at MOTOR_BREAKAWAY_VEL, linear in between.
        static const float kStiction[3] =
            {MOTOR_STICTION_M0, MOTOR_STICTION_M1, MOTOR_STICTION_M2};
        if (fabsf(target_v) > MOTOR_MIN_SPEED) {
            float frac = 1.0f - fminf(fabsf(actual_v) / MOTOR_BREAKAWAY_VEL, 1.0f);
            pwm_f += (target_v > 0 ? 1.0f : -1.0f) * kStiction[i] * frac;
        }
        // Output rate limit (slew): at most this much PWM change per tick, flattens jitter directly (including the static friction term).
        // M1 (heavy ENC1 crosstalk) gets a tighter limit -> PWM stops jumping around; M0/M2 get a looser limit to stay responsive.
        const float max_dpwm = (i == 1) ? 12.0f : 45.0f;
        float prev = (float)last_pwm_[i];
        pwm_f = fmaxf(prev - max_dpwm, fminf(prev + max_dpwm, pwm_f));
        int16_t pwm = constrain((int16_t)pwm_f, -PWM_MAX, PWM_MAX);
        if (!motors_.motor(i).setSpeed(pwm)) { stopMotors(); state_=STATE_IDLE; return; }
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
    doc["pwm_ready"] = motors_.ready();
    JsonArray pwmChannels = doc["motor_pwm_ready"].to<JsonArray>();
    for (int i = 0; i < 3; ++i) pwmChannels.add(motors_.motor(i).ready());
#ifdef WBB_TOP_BOARD_V2
    JsonArray faults = doc["motor_pwm_fault"].to<JsonArray>();
    for (int i = 0; i < 3; ++i) faults.add(motors_.motor(i).fault());
    volatile long counts[3];
    odometry_.getEncoderCounts(counts);
    JsonArray encoders = doc["encoder_counts"].to<JsonArray>();
    JsonArray measured = doc["wheel_speed_mps"].to<JsonArray>();
    JsonArray commanded = doc["pwm_command"].to<JsonArray>();
    for (int i = 0; i < 3; ++i) {
        encoders.add(long(counts[i]));
        measured.add(last_actual_[i]);
        commanded.add(last_pwm_[i]);
    }
    // Main control/kinematics arrays use physical Motor1/2/3; header order differs.
    JsonArray physical = doc["physical_motors"].to<JsonArray>();
    for (int number : {1, 2, 3}) physical.add(number);
    JsonArray headers = doc["header_order"].to<JsonArray>();
    for (const char* header : {"M0", "M2", "M1"}) headers.add(header);
#endif
}

// ============================================================
// Runtime tuning (serial)
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
            default: Serial.printf("ERR unknown gain '%c'\n", which); return;
        }
    }
    Serial.printf("OK %s K%c=%.2f\n",
                  motor < 0 ? "all" : (String("M") + motor).c_str(), which, val);
}

void RobotController::printParams(Print& out) const {
    out.println(F("------ Tunable parameters ------"));
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

// Machine-readable gain lines: one line per motor, so the tuner can read the current values at startup
void RobotController::printGains(Print& out) const {
    for (int i = 0; i < 3; i++) {
        out.printf("GAINS M%d kp=%.3f ki=%.3f kd=%.3f kf=%.3f il=%.1f\n",
                   i, pid_[i].getKp(), pid_[i].getKi(), pid_[i].getKd(),
                   pid_[i].getKf(), pid_[i].getIntegralLimit());
    }
}

void RobotController::setWheelTest(int idx, float v) {
    if (idx < 0 || idx > 2) {           // exit the test
        wheel_test_idx_ = -1;
        wheel_test_v_   = 0.0f;
        stopMotors();
        state_ = STATE_IDLE;
        Serial.println("WHEELTEST off");
        return;
    }
    wheel_test_idx_ = idx;
    wheel_test_v_   = v;
    for (int i = 0; i < 3; i++) pid_[i].reset();   // clear the integral for a clean start
    state_ = STATE_WHEEL_TEST;
    Serial.printf("WHEELTEST M%d v=%.3f\n", idx, v);
}
