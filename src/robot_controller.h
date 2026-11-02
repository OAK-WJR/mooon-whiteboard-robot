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
    STATE_WHEEL_TEST    // Single-wheel fixed-speed test (for PID tuning)
};

class RobotController {
public:
    void begin();

    // Hardware/task initialization failures must be able to stop all outputs.
    void stopMotors();
    void update(float dt);           // called by the 50Hz control loop
    void handleCommand(JsonDocument& doc);  // handle a UDP command
    void fillStatus(JsonDocument& doc);     // fill the status report

    RobotState getState() const { return state_; }

    // --- Runtime tuning (serial) ---
    // motor: -1=all three wheels, 0..2=one wheel; which: 'p' 'i' 'd' 'f' 'l'(I limit)
    void tunePid(int motor, char which, float val);
    void setLookAhead(float d)     { follower_.setLookAhead(d); }
    void setGoalTolerance(float d) { follower_.setGoalTolerance(d); }
    void setLinearGain(float kp)   { follower_.setLinearGain(kp); }
    void printParams(Print& out) const;     // print all tunable parameters
    void printTelemetry(Print& out) const;  // print target/actual/PWM for each wheel
    void printGains(Print& out) const;      // machine-readable gain lines (parsed by the tuner)

    // Single-wheel fixed speed: idx 0..2 makes that wheel track speed v (m/s), the other wheels stop; idx<0 exits the test.
    // Runs exactly the same FF+PID+static friction+slew pipeline as navigation, for live tuning.
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

    // Single-wheel test state
    int   wheel_test_idx_ = -1;
    float wheel_test_v_   = 0.0f;

    // Telemetry snapshot (updated every control loop tick, for serial printing)
    float   last_target_[3] = {0, 0, 0};
    float   last_actual_[3] = {0, 0, 0};
    int16_t last_pwm_[3]    = {0, 0, 0};

    // Stall protection state (separate per wheel)
    uint16_t      stall_cnt_[3]    = {0, 0, 0};  // consecutive ticks of "pushing but not moving"
    uint8_t       stall_trips_[3]  = {0, 0, 0};  // trip count; locks at STALL_MAX_TRIPS
    uint32_t      stall_started_[3] = {0, 0, 0};  // cooldown start time; elapsed time computed with 32-bit wraparound
    bool          stall_cooling_[3] = {false, false, false};  // a time value of 0 is also a valid time

    // target wheel speed -> FF+PID+static friction+slew -> PWM (navigation and the single-wheel test share one pipeline)
    void driveWheels(const WheelVelocity& target, const BodyVelocity& body, float dt);
};

#endif
