/**
 * WhiteBoardBot full simulation test suite
 *
 * Runs on the desktop (native), no ESP32 hardware needed:
 *   pio test -e native
 *
 * Test modules:
 *   1. Kinematics — forward/inverse consistency / symmetry / extreme values / direction sweep
 *   2. PID    — step / limits / anti-windup / direction change / disturbance / different motor models
 *   3. Odometry — straight / strafe / rotation / arc / long-run drift / reverse
 *   4. Full chain — closed-loop tracking / accel-decel / emergency stop / circular motion / saturation recovery / three-phase motion
 */

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <cfloat>

// ---- Project modules (pure math, no Arduino dependency) ----
#include "config.h"
#include "kinematics.h"
#include "pid_controller.h"

// Include the implementation files directly (avoids link problems; these are pure math with no Arduino dependency)
#include "kinematics.cpp"
#include "pid_controller.cpp"

// Pose2D is defined in odometry.h, but that header depends on Arduino
struct Pose2D {
    float x     = 0.0f;
    float y     = 0.0f;
    float theta = 0.0f;
};

// ============================================================
// Simple test framework
// ============================================================
static int tests_run    = 0;
static int tests_passed = 0;
static int tests_failed = 0;
static int section_run  = 0;
static int section_fail = 0;

#define ASSERT_NEAR(actual, expected, tol, msg) do { \
    tests_run++; section_run++; \
    float _a = (actual), _e = (expected), _t = (tol); \
    if (fabsf(_a - _e) <= _t) { tests_passed++; } \
    else { tests_failed++; section_fail++; \
        printf("  FAIL: %s\n    actual=%.6f  expected=%.6f  tol=%.6f\n", \
               msg, (double)_a, (double)_e, (double)_t); } \
} while(0)

#define ASSERT_TRUE(cond, msg) do { \
    tests_run++; section_run++; \
    if (cond) { tests_passed++; } \
    else { tests_failed++; section_fail++; printf("  FAIL: %s\n", msg); } \
} while(0)

#define SECTION_BEGIN(name) do { \
    section_run = 0; section_fail = 0; \
    printf("\n=== %s ===\n", name); \
} while(0)

#define SECTION_END(name) do { \
    printf("  [%s] %d/%d passed\n", name, section_run - section_fail, section_run); \
} while(0)

// ============================================================
// Simulated odometry (does not use Arduino encoder interrupts)
// ============================================================
class SimOdometry {
public:
    SimOdometry(const Kinematics& kin, int encoder_cpr)
        : kin_(kin), encoder_cpr_(encoder_cpr) {}

    void reset() {
        pose_ = {0, 0, 0};
        velocity_ = {0, 0, 0};
        for (int i = 0; i < 3; i++) {
            simulated_counts_[i] = 0;
            prev_counts_[i] = 0;
            fractional_counts_[i] = 0;
        }
    }

    void simulateWheelVelocities(const float wheel_v[3], float dt) {
        float cpr4 = (float)(encoder_cpr_ * 4);
        float wheel_circumference = 2.0f * M_PI * kin_.getWheelRadius();
        for (int i = 0; i < 3; i++) {
            float revolutions = wheel_v[i] * dt / wheel_circumference;
            fractional_counts_[i] += revolutions * cpr4;
            simulated_counts_[i] = (long)fractional_counts_[i];
        }
    }

    void update(float dt) {
        if (dt <= 0.0f) return;
        float cpr4 = (float)(encoder_cpr_ * 4);
        float wheel_circumference = 2.0f * M_PI * kin_.getWheelRadius();
        WheelVelocity wv;
        for (int i = 0; i < 3; i++) {
            long delta = simulated_counts_[i] - prev_counts_[i];
            prev_counts_[i] = simulated_counts_[i];
            float revolutions = (float)delta / cpr4;
            wv.v[i] = (revolutions * wheel_circumference) / dt;
        }
        velocity_ = kin_.forward(wv);
        float half_dtheta = velocity_.omega * dt * 0.5f;
        float cos_th = cosf(pose_.theta + half_dtheta);
        float sin_th = sinf(pose_.theta + half_dtheta);
        pose_.x     += (velocity_.vx * cos_th - velocity_.vy * sin_th) * dt;
        pose_.y     += (velocity_.vx * sin_th + velocity_.vy * cos_th) * dt;
        pose_.theta += velocity_.omega * dt;
        while (pose_.theta >  M_PI) pose_.theta -= 2.0f * M_PI;
        while (pose_.theta < -M_PI) pose_.theta += 2.0f * M_PI;
    }

    const Pose2D& getPose() const { return pose_; }
    const BodyVelocity& getVelocity() const { return velocity_; }

private:
    const Kinematics& kin_;
    int encoder_cpr_;
    Pose2D pose_;
    BodyVelocity velocity_;
    long simulated_counts_[3] = {0, 0, 0};
    long prev_counts_[3] = {0, 0, 0};
    double fractional_counts_[3] = {0, 0, 0};
};

// ============================================================
// Closed-loop simulator (reused by several tests)
// ============================================================
struct SimState {
    Kinematics      kin;
    SimOdometry     odom;
    PIDController   pid[3];
    float           motor_v[3];   // actual motor speed
    float           motor_tau;    // motor time constant
    float           dt;

    SimState()
        : kin(WHEEL_RADIUS, ROBOT_RADIUS)
        , odom(kin, ENCODER_CPR)
        , motor_tau(0.05f)
        , dt(1.0f / CONTROL_RATE_HZ)
    {
        motor_v[0] = motor_v[1] = motor_v[2] = 0;
        for (int i = 0; i < 3; i++) {
            pid[i].setGains(PID_KP, PID_KI, PID_KD, PID_KF);
            pid[i].setOutputLimits(-PWM_MAX, PWM_MAX);
            pid[i].setIntegralLimit(PID_INTEGRAL_LIMIT);
        }
    }

    void reset() {
        odom.reset();
        for (int i = 0; i < 3; i++) { motor_v[i] = 0; pid[i].reset(); }
    }

    // Run one closed-loop control step, return the PWM array
    void step(const BodyVelocity& target, float pwm_out[3]) {
        odom.update(dt);
        WheelVelocity tw = kin.inverse(target);
        const BodyVelocity& ab = odom.getVelocity();
        WheelVelocity aw = kin.inverse(ab);
        for (int i = 0; i < 3; i++) {
            pwm_out[i] = pid[i].compute(tw.v[i], aw.v[i], dt);
            pwm_out[i] = fmaxf(-PWM_MAX, fminf(PWM_MAX, pwm_out[i]));
            float mv = pwm_out[i] / PID_KF;
            motor_v[i] += (mv - motor_v[i]) * dt / motor_tau;
        }
        odom.simulateWheelVelocities(motor_v, dt);
    }

    // Run N steps
    void run(const BodyVelocity& target, int steps) {
        float pwm[3];
        for (int i = 0; i < steps; i++) step(target, pwm);
    }

    // Run for the given number of seconds
    void runSeconds(const BodyVelocity& target, float seconds) {
        run(target, (int)(seconds / dt));
    }
};

// ############################################################
//                    1. Kinematics tests
// ############################################################
static void test_kinematics_basic() {
    SECTION_BEGIN("Kinematics: basic forward/inverse consistency");
    Kinematics kin(WHEEL_RADIUS, ROBOT_RADIUS);

    // Pure forward
    {
        BodyVelocity b = {0.1f, 0, 0};
        WheelVelocity w = kin.inverse(b);
        ASSERT_NEAR(w.v[0], -w.v[2], 1e-6f, "pure forward: w0=-w2");
        ASSERT_NEAR(w.v[1], 0, 1e-6f, "pure forward: w1=0");
        BodyVelocity r = kin.forward(w);
        ASSERT_NEAR(r.vx, 0.1f, 1e-5f, "pure forward recover vx");
        ASSERT_NEAR(r.vy, 0, 1e-5f, "pure forward recover vy");
        ASSERT_NEAR(r.omega, 0, 1e-5f, "pure forward recover omega");
    }

    // Pure strafe
    {
        BodyVelocity b = {0, 0.1f, 0};
        WheelVelocity w = kin.inverse(b);
        ASSERT_NEAR(w.v[0], w.v[2], 1e-6f, "pure strafe: w0=w2");
        ASSERT_NEAR(w.v[1], -0.1f, 1e-5f, "pure strafe: w1=-vy");
        BodyVelocity r = kin.forward(w);
        ASSERT_NEAR(r.vx, 0, 1e-5f, "pure strafe recover vx");
        ASSERT_NEAR(r.vy, 0.1f, 1e-5f, "pure strafe recover vy");
    }

    // Pure rotation
    {
        BodyVelocity b = {0, 0, 1.0f};
        WheelVelocity w = kin.inverse(b);
        float ev = ROBOT_RADIUS * 1.0f;
        ASSERT_NEAR(w.v[0], ev, 1e-5f, "pure rotation: w0=R*omega");
        ASSERT_NEAR(w.v[1], ev, 1e-5f, "pure rotation: w1=R*omega");
        ASSERT_NEAR(w.v[2], ev, 1e-5f, "pure rotation: w2=R*omega");
        BodyVelocity r = kin.forward(w);
        ASSERT_NEAR(r.omega, 1.0f, 1e-4f, "pure rotation recover omega");
    }

    // Zero speed
    {
        BodyVelocity b = {0, 0, 0};
        WheelVelocity w = kin.inverse(b);
        ASSERT_NEAR(w.v[0], 0, 1e-8f, "zero speed w0");
        ASSERT_NEAR(w.v[1], 0, 1e-8f, "zero speed w1");
        ASSERT_NEAR(w.v[2], 0, 1e-8f, "zero speed w2");
    }
    SECTION_END("basic forward/inverse");
}

static void test_kinematics_sweep() {
    SECTION_BEGIN("Kinematics: 360° direction sweep");
    Kinematics kin(WHEEL_RADIUS, ROBOT_RADIUS);

    // Sweep all directions at 0.1 m/s (0°~350°), check forward/inverse consistency
    float speed = 0.1f;
    for (int deg = 0; deg < 360; deg += 10) {
        float rad = (float)deg * M_PI / 180.0f;
        BodyVelocity b = {speed * cosf(rad), speed * sinf(rad), 0};
        WheelVelocity w = kin.inverse(b);
        BodyVelocity r = kin.forward(w);
        char msg[64];
        snprintf(msg, sizeof(msg), "direction %d°: vx consistent", deg);
        ASSERT_NEAR(r.vx, b.vx, 1e-4f, msg);
        snprintf(msg, sizeof(msg), "direction %d°: vy consistent", deg);
        ASSERT_NEAR(r.vy, b.vy, 1e-4f, msg);
    }
    printf("  36 directions × 2 axes = 72 forward/inverse consistency checks\n");
    SECTION_END("direction sweep");
}

static void test_kinematics_extreme() {
    SECTION_BEGIN("Kinematics: extreme values and limits");
    Kinematics kin(WHEEL_RADIUS, ROBOT_RADIUS);

    // Negative values
    {
        BodyVelocity b = {-0.15f, -0.08f, -2.0f};
        WheelVelocity w = kin.inverse(b);
        BodyVelocity r = kin.forward(w);
        ASSERT_NEAR(r.vx, b.vx, 1e-4f, "negative vx");
        ASSERT_NEAR(r.vy, b.vy, 1e-4f, "negative vy");
        ASSERT_NEAR(r.omega, b.omega, 1e-3f, "negative omega");
    }

    // Max speed (all wheels near MOTOR_MAX_SPEED)
    {
        float max_v = MOTOR_MAX_SPEED;
        BodyVelocity b = {max_v, 0, 0};
        WheelVelocity w = kin.inverse(b);
        // No wheel speed magnitude should exceed sqrt(3)/2 * max_v (set by the kinematics matrix)
        for (int i = 0; i < 3; i++) {
            ASSERT_TRUE(fabsf(w.v[i]) <= max_v * 1.1f, "max speed: wheel speed within limit");
        }
    }

    // Very small values (near zero but not zero)
    {
        BodyVelocity b = {1e-6f, 1e-6f, 1e-6f};
        WheelVelocity w = kin.inverse(b);
        BodyVelocity r = kin.forward(w);
        ASSERT_NEAR(r.vx, b.vx, 1e-5f, "tiny value vx");
        ASSERT_NEAR(r.vy, b.vy, 1e-5f, "tiny value vy");
    }

    // Linear superposition: inverse(a+b) == inverse(a) + inverse(b)
    {
        BodyVelocity a = {0.05f, 0.03f, 0.2f};
        BodyVelocity b = {-0.02f, 0.04f, -0.1f};
        BodyVelocity ab = {a.vx+b.vx, a.vy+b.vy, a.omega+b.omega};
        WheelVelocity wa = kin.inverse(a);
        WheelVelocity wb = kin.inverse(b);
        WheelVelocity wab = kin.inverse(ab);
        for (int i = 0; i < 3; i++) {
            char msg[64];
            snprintf(msg, sizeof(msg), "linear superposition w%d", i);
            ASSERT_NEAR(wab.v[i], wa.v[i] + wb.v[i], 1e-5f, msg);
        }
    }
    SECTION_END("extreme values");
}

// ############################################################
//                    2. PID controller tests
// ############################################################
static void test_pid_basic() {
    SECTION_BEGIN("PID: basic functions");

    // Pure feedforward
    {
        PIDController pid;
        pid.setGains(0, 0, 0, PID_KF);
        pid.setOutputLimits(-255, 255);
        float out = pid.compute(0.1f, 0.1f, 0.02f);
        ASSERT_NEAR(out, PID_KF * 0.1f, 0.01f, "pure feedforward output=Kf*target");
    }

    // Output limits
    {
        PIDController pid;
        pid.setGains(PID_KP, PID_KI, PID_KD, PID_KF);
        pid.setOutputLimits(-255, 255);
        float out = pid.compute(1.0f, 0, 0.02f);
        ASSERT_TRUE(out <= 255.0f, "upper limit");
        out = pid.compute(-1.0f, 0, 0.02f);
        ASSERT_TRUE(out >= -255.0f, "lower limit");
    }

    // reset
    {
        PIDController pid;
        pid.setGains(PID_KP, PID_KI, PID_KD, PID_KF);
        pid.setOutputLimits(-255, 255);
        pid.compute(0.1f, 0, 0.02f);
        pid.reset();
        ASSERT_NEAR(pid.getIntegral(), 0, 1e-8f, "reset integral=0");
        ASSERT_NEAR(pid.getLastOutput(), 0, 1e-8f, "reset output=0");
    }

    // dt<=0 does not update
    {
        PIDController pid;
        pid.setGains(PID_KP, PID_KI, PID_KD, PID_KF);
        pid.setOutputLimits(-255, 255);
        float out1 = pid.compute(0.1f, 0, 0.02f);
        float out2 = pid.compute(0.1f, 0, 0.0f);
        ASSERT_NEAR(out2, out1, 1e-8f, "dt=0 no update");
        float out3 = pid.compute(0.1f, 0, -0.01f);
        ASSERT_NEAR(out3, out1, 1e-8f, "dt<0 no update");
    }

    // Pure P control
    {
        PIDController pid;
        pid.setGains(100.0f, 0, 0, 0);
        pid.setOutputLimits(-1000, 1000);
        float out = pid.compute(0.1f, 0.05f, 0.02f);
        ASSERT_NEAR(out, 100.0f * 0.05f, 0.01f, "pure P: output=Kp*error");
    }
    SECTION_END("PID basics");
}

static void test_pid_step_response() {
    SECTION_BEGIN("PID: step response");

    PIDController pid;
    pid.setGains(PID_KP, PID_KI, PID_KD, PID_KF);
    pid.setOutputLimits(-255, 255);
    pid.setIntegralLimit(PID_INTEGRAL_LIMIT);

    float target = 0.10f;
    float actual = 0.0f;
    float dt = 1.0f / CONTROL_RATE_HZ;
    float tau = 0.05f;

    bool settled = false;
    int settle_step = -1;
    float max_overshoot = 0;

    printf("  step response target=%.2f m/s:\n", (double)target);
    for (int i = 0; i < 200; i++) {
        float pwm = pid.compute(target, actual, dt);
        float mv = pwm / PID_KF;
        actual += (mv - actual) * dt / tau;

        float overshoot = (actual - target) / target;
        if (overshoot > max_overshoot) max_overshoot = overshoot;

        if (i < 10 || i % 50 == 0) {
            printf("    step %3d: actual=%.4f pwm=%.1f [FF=%.1f P=%.1f I=%.1f D=%.1f]\n",
                   i, (double)actual, (double)pwm,
                   (double)pid.getFFTerm(), (double)pid.getPTerm(),
                   (double)pid.getITerm(), (double)pid.getDTerm());
        }

        if (!settled && fabsf(actual - target) < target * 0.01f) {
            settled = true;
            settle_step = i;
        }
    }

    ASSERT_TRUE(settled, "settles to 1% within 200 steps");
    ASSERT_NEAR(actual, target, target * 0.02f, "steady-state error < 2%");
    ASSERT_TRUE(max_overshoot < 0.15f, "overshoot < 15%");
    printf("  settled: step %d (%.0f ms), overshoot: %.1f%%\n",
           settle_step, (double)(settle_step * dt * 1000),
           (double)(max_overshoot * 100));
    SECTION_END("step response");
}

static void test_pid_antiwindup() {
    SECTION_BEGIN("PID: Anti-windup (integral limit check)");

    float dt = 0.02f;

    // Test 1: the integral limit really works (uses a small limit to speed up the test)
    {
        float test_limit = 0.5f;  // small limit, saturates quickly
        PIDController pid;
        pid.setGains(0, PID_KI, 0, 0);  // I only, easier to observe
        pid.setOutputLimits(-10000, 10000);  // widen the output limit to test only the integral limit
        pid.setIntegralLimit(test_limit);

        // Large error held for 100 steps; check the limited I term (in PWM output units).
        for (int i = 0; i < 100; i++) {
            pid.compute(1.0f, 0.0f, dt);
        }
        float integral = pid.getITerm(); // setIntegralLimit is a limit in PWM output units.
        printf("  integral after buildup: %.3f (limit: ±%.1f)\n",
               (double)integral, (double)test_limit);
        ASSERT_TRUE(fabsf(integral) <= test_limit + 0.001f,
                    "integral is limited");
        ASSERT_NEAR(integral, test_limit, 0.01f,
                    "integral reaches the limit");
    }

    // Test 2: after the integral is limited, the error reverses and the integral unwinds
    {
        float test_limit = 0.5f;
        PIDController pid;
        pid.setGains(0, PID_KI, 0, 0);
        pid.setOutputLimits(-10000, 10000);
        pid.setIntegralLimit(test_limit);

        // First saturate to the positive limit
        for (int i = 0; i < 100; i++) pid.compute(1.0f, 0, dt);
        ASSERT_NEAR(pid.getITerm(), test_limit, 0.01f, "I term first saturates to the limit");

        // Reverse the error; the integral should fall and saturate at the negative limit
        for (int i = 0; i < 200; i++) pid.compute(-1.0f, 0, dt);
        float integral = pid.getITerm();
        printf("  integral after reversal: %.3f\n", (double)integral);
        ASSERT_NEAR(integral, -test_limit, 0.01f,
                    "integral unwinds to the negative limit after reversal");
    }

    // Test 3: closed-loop saturation recovery (with real parameters)
    {
        PIDController pid;
        pid.setGains(PID_KP, PID_KI, PID_KD, PID_KF);
        pid.setOutputLimits(-255, 255);
        pid.setIntegralLimit(PID_INTEGRAL_LIMIT);

        float actual = 0, tau = 0.05f;

        // First track 0.15 m/s and hold steady for 1s
        for (int i = 0; i < 50; i++) {
            float pwm = pid.compute(0.15f, actual, dt);
            actual += (pwm / PID_KF - actual) * dt / tau;
        }

        // Suddenly switch to 0.02 m/s
        bool settled = false;
        for (int i = 0; i < 200; i++) {
            float pwm = pid.compute(0.02f, actual, dt);
            actual += (pwm / PID_KF - actual) * dt / tau;
            if (!settled && fabsf(actual - 0.02f) < 0.003f) {
                settled = true;
                printf("  closed-loop saturation recovery: step %d (%.0f ms)\n",
                       i, (double)(i * dt * 1000));
            }
        }
        ASSERT_TRUE(settled, "settles after switching from high to low speed");
        ASSERT_NEAR(actual, 0.02f, 0.005f, "steady-state accuracy after switch");
    }
    // Disabling integral control must not divide by zero and poison future output.
    {
        PIDController pid;
        pid.setGains(20, 0, 0, 100);
        for(int i=0;i<10;++i) {
            ASSERT_TRUE(std::isfinite(pid.compute(.1f,0,.02f)), "Ki=0 output is finite");
            ASSERT_NEAR(pid.getITerm(),0,.0001f,"Ki=0 integral term cleared");
        }
        pid.setKi(2);
        ASSERT_TRUE(std::isfinite(pid.compute(.1f,0,.02f)), "output is finite after Ki restored");
    }
    SECTION_END("Anti-windup");
}

static void test_pid_direction_change() {
    SECTION_BEGIN("PID: direction change (positive→negative)");

    PIDController pid;
    pid.setGains(PID_KP, PID_KI, PID_KD, PID_KF);
    pid.setOutputLimits(-255, 255);
    pid.setIntegralLimit(PID_INTEGRAL_LIMIT);

    float actual = 0;
    float dt = 1.0f / CONTROL_RATE_HZ;
    float tau = 0.05f;

    // First go forward for 1 second
    for (int i = 0; i < CONTROL_RATE_HZ; i++) {
        float pwm = pid.compute(0.1f, actual, dt);
        actual += (pwm / PID_KF - actual) * dt / tau;
    }
    ASSERT_NEAR(actual, 0.1f, 0.01f, "forward steady state");
    printf("  forward steady state: actual=%.4f\n", (double)actual);

    // Jump to negative
    bool settled = false;
    int settle_step = -1;
    for (int i = 0; i < CONTROL_RATE_HZ * 2; i++) {
        float pwm = pid.compute(-0.1f, actual, dt);
        actual += (pwm / PID_KF - actual) * dt / tau;
        if (!settled && fabsf(actual - (-0.1f)) < 0.005f) {
            settled = true;
            settle_step = i;
        }
    }
    ASSERT_TRUE(settled, "settles after direction change");
    ASSERT_NEAR(actual, -0.1f, 0.01f, "negative steady state");
    printf("  direction change done: step %d (%.0f ms)\n",
           settle_step, (double)(settle_step * dt * 1000));
    SECTION_END("direction change");
}

static void test_pid_disturbance() {
    SECTION_BEGIN("PID: disturbance recovery");

    PIDController pid;
    pid.setGains(PID_KP, PID_KI, PID_KD, PID_KF);
    pid.setOutputLimits(-255, 255);
    pid.setIntegralLimit(PID_INTEGRAL_LIMIT);

    float actual = 0;
    float dt = 1.0f / CONTROL_RATE_HZ;
    float tau = 0.05f;
    float target = 0.1f;

    // First settle for 1 second
    for (int i = 0; i < CONTROL_RATE_HZ; i++) {
        float pwm = pid.compute(target, actual, dt);
        actual += (pwm / PID_KF - actual) * dt / tau;
    }
    printf("  steady state: actual=%.4f (target=%.4f)\n", (double)actual, (double)target);

    // Apply a step disturbance: speed suddenly drops by 50%
    actual *= 0.5f;
    printf("  apply disturbance: actual→%.4f\n", (double)actual);

    bool recovered = false;
    int recover_step = -1;
    for (int i = 0; i < CONTROL_RATE_HZ * 2; i++) {
        float pwm = pid.compute(target, actual, dt);
        actual += (pwm / PID_KF - actual) * dt / tau;
        if (!recovered && fabsf(actual - target) < target * 0.02f) {
            recovered = true;
            recover_step = i;
        }
    }
    ASSERT_TRUE(recovered, "recovers after disturbance");
    ASSERT_NEAR(actual, target, target * 0.03f, "steady-state accuracy after disturbance");
    printf("  recovered: step %d (%.0f ms)\n",
           recover_step, (double)(recover_step * dt * 1000));
    SECTION_END("disturbance recovery");
}

static void test_pid_different_motors() {
    SECTION_BEGIN("PID: different motor characteristics (fast/slow)");

    float dt = 1.0f / CONTROL_RATE_HZ;
    float target = 0.1f;

    // Fast motor (tau=20ms)
    {
        PIDController pid;
        pid.setGains(PID_KP, PID_KI, PID_KD, PID_KF);
        pid.setOutputLimits(-255, 255);
        pid.setIntegralLimit(PID_INTEGRAL_LIMIT);

        float actual = 0;
        bool settled = false;
        int step = -1;
        for (int i = 0; i < 200; i++) {
            float pwm = pid.compute(target, actual, dt);
            actual += (pwm / PID_KF - actual) * dt / 0.02f;
            if (!settled && fabsf(actual - target) < target * 0.01f) {
                settled = true; step = i;
            }
        }
        ASSERT_TRUE(settled, "fast motor (τ=20ms): settles");
        printf("  fast motor (τ=20ms): settle step %d\n", step);
    }

    // Slow motor (tau=150ms)
    {
        PIDController pid;
        pid.setGains(PID_KP, PID_KI, PID_KD, PID_KF);
        pid.setOutputLimits(-255, 255);
        pid.setIntegralLimit(PID_INTEGRAL_LIMIT);

        float actual = 0;
        bool settled = false;
        int step = -1;
        for (int i = 0; i < 500; i++) {
            float pwm = pid.compute(target, actual, dt);
            actual += (pwm / PID_KF - actual) * dt / 0.15f;
            if (!settled && fabsf(actual - target) < target * 0.01f) {
                settled = true; step = i;
            }
        }
        ASSERT_TRUE(settled, "slow motor (τ=150ms): settles");
        ASSERT_NEAR(actual, target, target * 0.05f, "slow motor steady-state accuracy");
        printf("  slow motor (τ=150ms): settle step %d\n", step);
    }
    SECTION_END("motor characteristics");
}

static void test_pid_multiple_speeds() {
    SECTION_BEGIN("PID: step at several speeds");

    float dt = 1.0f / CONTROL_RATE_HZ;
    float tau = 0.05f;
    float speeds[] = {0.02f, 0.05f, 0.10f, 0.15f, 0.20f};

    for (float target : speeds) {
        PIDController pid;
        pid.setGains(PID_KP, PID_KI, PID_KD, PID_KF);
        pid.setOutputLimits(-255, 255);
        pid.setIntegralLimit(PID_INTEGRAL_LIMIT);

        float actual = 0;
        bool settled = false;
        for (int i = 0; i < 300; i++) {
            float pwm = pid.compute(target, actual, dt);
            actual += (pwm / PID_KF - actual) * dt / tau;
            if (!settled && fabsf(actual - target) < target * 0.02f) {
                settled = true;
            }
        }
        char msg[64];
        snprintf(msg, sizeof(msg), "target=%.2f: settles", target);
        ASSERT_TRUE(settled, msg);
        snprintf(msg, sizeof(msg), "target=%.2f: steady-state accuracy", target);
        ASSERT_NEAR(actual, target, target * 0.05f, msg);
    }
    SECTION_END("several speeds");
}

// ############################################################
//                    3. Odometry tests
// ############################################################
static void test_odometry_basic() {
    SECTION_BEGIN("Odometry: basic motion");
    Kinematics kin(WHEEL_RADIUS, ROBOT_RADIUS);
    SimOdometry odom(kin, ENCODER_CPR);
    float dt = 1.0f / CONTROL_RATE_HZ;

    // Straight forward
    {
        odom.reset();
        BodyVelocity cmd = {0.1f, 0, 0};
        WheelVelocity wv = kin.inverse(cmd);
        for (int i = 0; i < CONTROL_RATE_HZ; i++) {
            odom.simulateWheelVelocities(wv.v, dt);
            odom.update(dt);
        }
        const Pose2D& p = odom.getPose();
        printf("  straight forward 1s: x=%.4f y=%.4f θ=%.4f\n",
               (double)p.x, (double)p.y, (double)p.theta);
        ASSERT_NEAR(p.x, 0.1f, 0.01f, "straight x≈0.1");
        ASSERT_NEAR(p.y, 0, 0.005f, "straight y≈0");
        ASSERT_NEAR(p.theta, 0, 0.01f, "straight θ≈0");
    }

    // Pure strafe
    {
        odom.reset();
        BodyVelocity cmd = {0, 0.05f, 0};
        WheelVelocity wv = kin.inverse(cmd);
        for (int i = 0; i < CONTROL_RATE_HZ; i++) {
            odom.simulateWheelVelocities(wv.v, dt);
            odom.update(dt);
        }
        const Pose2D& p = odom.getPose();
        printf("  pure strafe 1s: x=%.4f y=%.4f θ=%.4f\n",
               (double)p.x, (double)p.y, (double)p.theta);
        ASSERT_NEAR(p.x, 0, 0.005f, "strafe x≈0");
        ASSERT_NEAR(p.y, 0.05f, 0.01f, "strafe y≈0.05");
    }

    // Rotate in place
    {
        odom.reset();
        BodyVelocity cmd = {0, 0, 1.0f};
        WheelVelocity wv = kin.inverse(cmd);
        for (int i = 0; i < CONTROL_RATE_HZ; i++) {
            odom.simulateWheelVelocities(wv.v, dt);
            odom.update(dt);
        }
        const Pose2D& p = odom.getPose();
        printf("  rotate in place 1s: x=%.4f y=%.4f θ=%.4f\n",
               (double)p.x, (double)p.y, (double)p.theta);
        ASSERT_NEAR(p.theta, 1.0f, 0.1f, "rotation θ≈1 rad");
        ASSERT_NEAR(p.x, 0, 0.01f, "rotation x≈0");
        ASSERT_NEAR(p.y, 0, 0.01f, "rotation y≈0");
    }
    SECTION_END("basic motion");
}

static void test_odometry_arc() {
    SECTION_BEGIN("Odometry: arc path");
    Kinematics kin(WHEEL_RADIUS, ROBOT_RADIUS);
    SimOdometry odom(kin, ENCODER_CPR);
    float dt = 1.0f / CONTROL_RATE_HZ;

    // Turn radius R = vx / omega
    float vx = 0.05f;
    float omega = 1.0f;
    float R_turn = vx / omega;  // = 0.05m
    printf("  arc: vx=%.2f omega=%.2f → R=%.3f m\n",
           (double)vx, (double)omega, (double)R_turn);

    // Drive π/2 (a quarter turn)
    float duration = (M_PI / 2.0f) / omega;  // ≈ 1.57 s
    BodyVelocity cmd = {vx, 0, omega};
    WheelVelocity wv = kin.inverse(cmd);

    int steps = (int)(duration / dt);
    for (int i = 0; i < steps; i++) {
        odom.simulateWheelVelocities(wv.v, dt);
        odom.update(dt);
    }

    const Pose2D& p = odom.getPose();
    // Quarter arc: expect x ≈ R*sin(π/2) = R, y ≈ R*(1-cos(π/2)) = R
    printf("  quarter arc: x=%.4f y=%.4f θ=%.3f (expect x≈%.3f y≈%.3f θ≈%.3f)\n",
           (double)p.x, (double)p.y, (double)p.theta,
           (double)R_turn, (double)R_turn, M_PI/2.0);
    ASSERT_NEAR(p.theta, (float)(M_PI/2.0), 0.15f, "arc θ≈π/2");
    ASSERT_NEAR(p.x, R_turn, R_turn * 0.2f, "arc x≈R");
    ASSERT_NEAR(p.y, R_turn, R_turn * 0.2f, "arc y≈R");
    SECTION_END("arc path");
}

static void test_odometry_full_rotation() {
    SECTION_BEGIN("Odometry: full 360° rotation");
    Kinematics kin(WHEEL_RADIUS, ROBOT_RADIUS);
    SimOdometry odom(kin, ENCODER_CPR);
    float dt = 1.0f / CONTROL_RATE_HZ;

    float omega = 2.0f;  // 2 rad/s
    float duration = 2.0f * M_PI / omega;  // one full turn
    BodyVelocity cmd = {0, 0, omega};
    WheelVelocity wv = kin.inverse(cmd);

    int steps = (int)(duration / dt);
    for (int i = 0; i < steps; i++) {
        odom.simulateWheelVelocities(wv.v, dt);
        odom.update(dt);
    }

    const Pose2D& p = odom.getPose();
    printf("  360° rotation: x=%.4f y=%.4f θ=%.4f (expect θ≈0 or ±2π)\n",
           (double)p.x, (double)p.y, (double)p.theta);
    // θ should come back near 0 (normalized to [-π, π])
    ASSERT_NEAR(p.theta, 0, 0.2f, "360° θ returns");
    ASSERT_NEAR(p.x, 0, 0.01f, "360° x≈0");
    ASSERT_NEAR(p.y, 0, 0.01f, "360° y≈0");
    SECTION_END("360° rotation");
}

static void test_odometry_reverse() {
    SECTION_BEGIN("Odometry: reverse motion");
    Kinematics kin(WHEEL_RADIUS, ROBOT_RADIUS);
    SimOdometry odom(kin, ENCODER_CPR);
    float dt = 1.0f / CONTROL_RATE_HZ;

    // Forward 1s first, then back 1s
    {
        odom.reset();
        BodyVelocity fwd = {0.1f, 0, 0};
        WheelVelocity wf = kin.inverse(fwd);
        for (int i = 0; i < CONTROL_RATE_HZ; i++) {
            odom.simulateWheelVelocities(wf.v, dt);
            odom.update(dt);
        }
        float x_mid = odom.getPose().x;
        printf("  after forward 1s: x=%.4f\n", (double)x_mid);

        BodyVelocity bwd = {-0.1f, 0, 0};
        WheelVelocity wb = kin.inverse(bwd);
        for (int i = 0; i < CONTROL_RATE_HZ; i++) {
            odom.simulateWheelVelocities(wb.v, dt);
            odom.update(dt);
        }
        const Pose2D& p = odom.getPose();
        printf("  after round trip: x=%.4f y=%.4f θ=%.4f\n",
               (double)p.x, (double)p.y, (double)p.theta);
        ASSERT_NEAR(p.x, 0, 0.015f, "round trip x≈0");
        ASSERT_NEAR(p.y, 0, 0.005f, "round trip y≈0");
    }
    SECTION_END("reverse motion");
}

static void test_odometry_long_drift() {
    SECTION_BEGIN("Odometry: long-run drift (10s)");
    Kinematics kin(WHEEL_RADIUS, ROBOT_RADIUS);
    SimOdometry odom(kin, ENCODER_CPR);
    float dt = 1.0f / CONTROL_RATE_HZ;

    BodyVelocity cmd = {0.08f, 0, 0};
    WheelVelocity wv = kin.inverse(cmd);
    float duration = 10.0f;
    int steps = (int)(duration * CONTROL_RATE_HZ);

    for (int i = 0; i < steps; i++) {
        odom.simulateWheelVelocities(wv.v, dt);
        odom.update(dt);
    }

    const Pose2D& p = odom.getPose();
    float expected_x = cmd.vx * duration;
    printf("  10s straight: x=%.4f (expect %.4f), y=%.6f, θ=%.6f\n",
           (double)p.x, (double)expected_x, (double)p.y, (double)p.theta);
    ASSERT_NEAR(p.x, expected_x, expected_x * 0.02f, "10s x error<2%");
    ASSERT_NEAR(p.y, 0, 0.01f, "10s y drift<1cm");
    ASSERT_NEAR(p.theta, 0, 0.05f, "10s θ drift<0.05rad");
    SECTION_END("long-run drift");
}

// ############################################################
//                    4. Full-chain closed-loop simulation
// ############################################################
static void test_closed_loop_tracking() {
    SECTION_BEGIN("Closed loop: basic tracking (diagonal motion)");

    SimState sim;
    BodyVelocity target = {0.08f, -0.02f, 0};
    float duration = 3.0f;

    printf("  target: vx=%.3f vy=%.3f omega=%.3f, %.1f s\n",
           (double)target.vx, (double)target.vy, (double)target.omega, (double)duration);

    float pwm[3];
    int steps = (int)(duration / sim.dt);
    for (int step = 0; step < steps; step++) {
        sim.step(target, pwm);
        if (step < 3 || step % 50 == 0 || step == steps-1) {
            const Pose2D& p = sim.odom.getPose();
            const BodyVelocity& v = sim.odom.getVelocity();
            printf("    t=%.2f: pos(%.4f,%.4f) vel(%.4f,%.4f)\n",
                   (double)(step * sim.dt), (double)p.x, (double)p.y,
                   (double)v.vx, (double)v.vy);
        }
    }

    const Pose2D& fp = sim.odom.getPose();
    float ex = target.vx * duration, ey = target.vy * duration;
    ASSERT_NEAR(fp.x, ex, ex * 0.15f, "tracking x<15%");
    ASSERT_NEAR(fp.y, ey, fabsf(ey) * 0.15f + 0.005f, "tracking y<15%");
    ASSERT_NEAR(fp.theta, 0, 0.05f, "tracking θ drift<0.05 rad");

    const BodyVelocity& fv = sim.odom.getVelocity();
    ASSERT_NEAR(fv.vx, target.vx, 0.01f, "vx tracking error<0.01");
    ASSERT_NEAR(fv.vy, target.vy, 0.01f, "vy tracking error<0.01");
    SECTION_END("basic tracking");
}

static void test_closed_loop_accel_decel() {
    SECTION_BEGIN("Closed loop: trapezoidal accel/decel");

    SimState sim;
    float pwm[3];

    // Phase 1: accelerate to 0.1 m/s (1s)
    printf("  phase 1: accelerate → 0.1 m/s\n");
    BodyVelocity target = {0.1f, 0, 0};
    sim.run(target, CONTROL_RATE_HZ);

    const BodyVelocity& v1 = sim.odom.getVelocity();
    ASSERT_NEAR(v1.vx, 0.1f, 0.01f, "vx≈0.1 after accel");

    // Phase 2: constant speed 2s
    printf("  phase 2: constant speed 2s\n");
    sim.run(target, CONTROL_RATE_HZ * 2);
    const Pose2D& p2 = sim.odom.getPose();
    printf("    pos after constant speed: (%.4f, %.4f)\n", (double)p2.x, (double)p2.y);

    // Phase 3: decelerate to 0 (1s)
    printf("  phase 3: decelerate → 0\n");
    BodyVelocity stop = {0, 0, 0};
    sim.run(stop, CONTROL_RATE_HZ);

    const BodyVelocity& v3 = sim.odom.getVelocity();
    const Pose2D& p3 = sim.odom.getPose();
    printf("    after stop: vel=(%.4f,%.4f) pos=(%.4f,%.4f)\n",
           (double)v3.vx, (double)v3.vy, (double)p3.x, (double)p3.y);

    // Total distance: about 0.1 * 3 = 0.3m (accel/decel phases add about half)
    ASSERT_NEAR(v3.vx, 0, 0.005f, "vx≈0 after stop");
    ASSERT_TRUE(p3.x > 0.2f && p3.x < 0.35f, "total distance in a reasonable range");
    SECTION_END("accel/decel");
}

static void test_closed_loop_emergency_stop() {
    SECTION_BEGIN("Closed loop: emergency stop (simulated cmd_vel timeout)");

    SimState sim;
    float pwm[3];

    // Full speed forward 1s
    BodyVelocity target = {MOTOR_MAX_SPEED * 0.8f, 0, 0};
    sim.run(target, CONTROL_RATE_HZ);

    const BodyVelocity& v1 = sim.odom.getVelocity();
    printf("  full speed: vx=%.4f\n", (double)v1.vx);
    ASSERT_TRUE(v1.vx > target.vx * 0.5f, "running at full speed");

    // Emergency stop (target set to zero)
    BodyVelocity zero = {0, 0, 0};
    bool stopped = false;
    int stop_step = -1;
    for (int i = 0; i < CONTROL_RATE_HZ; i++) {
        sim.step(zero, pwm);
        const BodyVelocity& v = sim.odom.getVelocity();
        if (!stopped && fabsf(v.vx) < 0.005f) {
            stopped = true;
            stop_step = i;
        }
    }
    ASSERT_TRUE(stopped, "emergency stop succeeded");
    printf("  emergency stop: %d steps (%.0f ms)\n", stop_step, (double)(stop_step * sim.dt * 1000));
    SECTION_END("emergency stop");
}

static void test_closed_loop_circle() {
    SECTION_BEGIN("Closed loop: circular motion (draw a circle)");

    SimState sim;
    float pwm[3];

    // Draw a full circle: vx=0.05, omega=1.0 → R=0.05m, period=2π s
    BodyVelocity target = {0.05f, 0, 1.0f};
    float period = 2.0f * M_PI / 1.0f;  // ≈6.28s
    int steps = (int)(period / sim.dt);

    printf("  circle: vx=%.2f omega=%.2f R=%.3fm, period=%.2fs\n",
           (double)target.vx, (double)target.omega,
           (double)(target.vx / target.omega), (double)period);

    for (int step = 0; step < steps; step++) {
        sim.step(target, pwm);
        if (step % (steps/4) == 0) {
            const Pose2D& p = sim.odom.getPose();
            printf("    t=%.1f: pos(%.4f,%.4f) θ=%.2f\n",
                   (double)(step * sim.dt), (double)p.x, (double)p.y, (double)p.theta);
        }
    }

    const Pose2D& fp = sim.odom.getPose();
    printf("  circle end point: (%.4f,%.4f) θ=%.3f\n",
           (double)fp.x, (double)fp.y, (double)fp.theta);

    // Back near the start point
    float dist = sqrtf(fp.x * fp.x + fp.y * fp.y);
    ASSERT_TRUE(dist < 0.03f, "circle returns to origin (distance<3cm)");
    // θ should come back near 0
    ASSERT_NEAR(fp.theta, 0, 0.3f, "circle θ returns");
    SECTION_END("circular motion");
}

static void test_closed_loop_saturation_recovery() {
    SECTION_BEGIN("Closed loop: PWM saturation and recovery");

    SimState sim;
    float pwm[3];

    // Command a target above the motor max speed
    BodyVelocity over_target = {MOTOR_MAX_SPEED * 1.5f, 0, 0};
    printf("  overspeed target: vx=%.3f (max=%.3f)\n",
           (double)over_target.vx, (double)MOTOR_MAX_SPEED);

    // Run 1s; PWM should saturate
    bool saturated = false;
    for (int i = 0; i < CONTROL_RATE_HZ; i++) {
        sim.step(over_target, pwm);
        if (fabsf(pwm[0]) >= PWM_MAX - 1 || fabsf(pwm[2]) >= PWM_MAX - 1) {
            saturated = true;
        }
    }
    ASSERT_TRUE(saturated, "PWM reaches saturation");

    // Back to normal speed
    BodyVelocity normal = {0.05f, 0, 0};
    bool recovered = false;
    for (int i = 0; i < CONTROL_RATE_HZ * 2; i++) {
        sim.step(normal, pwm);
        const BodyVelocity& v = sim.odom.getVelocity();
        if (!recovered && fabsf(v.vx - normal.vx) < 0.01f) {
            recovered = true;
            printf("  saturation recovery: step %d (%.0f ms)\n", i, (double)(i * sim.dt * 1000));
        }
    }
    ASSERT_TRUE(recovered, "normal tracking resumes after saturation");
    SECTION_END("saturation recovery");
}

static void test_closed_loop_three_phase() {
    SECTION_BEGIN("Closed loop: three-phase motion (forward→turn→strafe)");

    SimState sim;

    // Phase 1: forward 0.08 m/s, 2s
    BodyVelocity phase1 = {0.08f, 0, 0};
    sim.runSeconds(phase1, 2.0f);
    const Pose2D& p1 = sim.odom.getPose();
    printf("  phase 1 (forward): pos(%.4f,%.4f) θ=%.3f\n",
           (double)p1.x, (double)p1.y, (double)p1.theta);
    ASSERT_NEAR(p1.x, 0.16f, 0.03f, "forward x≈0.16");

    // Phase 2: rotate in place 90° (π/2 rad)
    float omega_turn = 2.0f;
    float turn_duration = (M_PI / 2.0f) / omega_turn;
    BodyVelocity phase2 = {0, 0, omega_turn};
    sim.runSeconds(phase2, turn_duration);
    const Pose2D& p2 = sim.odom.getPose();
    printf("  phase 2 (turn 90°): pos(%.4f,%.4f) θ=%.3f (expect≈%.3f)\n",
           (double)p2.x, (double)p2.y, (double)p2.theta, M_PI/2.0);
    ASSERT_NEAR(p2.theta, (float)(M_PI/2.0), 0.15f, "turn θ≈π/2");

    // Phase 3: forward (now the global y direction) 0.08 m/s, 2s
    BodyVelocity phase3 = {0.08f, 0, 0};
    sim.runSeconds(phase3, 2.0f);
    const Pose2D& p3 = sim.odom.getPose();
    printf("  phase 3 (forward after turn): pos(%.4f,%.4f) θ=%.3f\n",
           (double)p3.x, (double)p3.y, (double)p3.theta);

    // After the 90° turn, moving forward should be mostly in the y direction
    ASSERT_TRUE(p3.y > 0.10f, "forward after turn: y increases");
    // x should stay about the same as at the end of phase 1 (no longer increasing)
    ASSERT_NEAR(p3.x, p1.x, 0.04f, "forward after turn: x about unchanged");
    SECTION_END("three-phase motion");
}

static void test_closed_loop_pure_rotation_track() {
    SECTION_BEGIN("Closed loop: pure rotation tracking accuracy");

    SimState sim;

    // Rotate in the positive direction at 2 rad/s, 1s
    BodyVelocity target = {0, 0, 2.0f};
    sim.runSeconds(target, 1.0f);

    const Pose2D& p = sim.odom.getPose();
    const BodyVelocity& v = sim.odom.getVelocity();
    printf("  rotate 1s: θ=%.4f (expect 2.0) omega=%.4f\n",
           (double)p.theta, (double)v.omega);
    ASSERT_NEAR(p.theta, 2.0f, 0.2f, "rotation tracking θ");
    ASSERT_NEAR(p.x, 0, 0.01f, "rotation x≈0");
    ASSERT_NEAR(p.y, 0, 0.01f, "rotation y≈0");

    // Rotate back the other way
    BodyVelocity reverse = {0, 0, -2.0f};
    sim.runSeconds(reverse, 1.0f);

    const Pose2D& p2 = sim.odom.getPose();
    printf("  rotated back: θ=%.4f (expect ≈0)\n", (double)p2.theta);
    ASSERT_NEAR(p2.theta, 0, 0.3f, "rotated back θ≈0");
    SECTION_END("pure rotation tracking");
}

// ############################################################
//                         main
// ############################################################
int main() {
    printf("============================================\n");
    printf("  WhiteBoardBot full simulation test\n");
    printf("  WHEEL_R=%.4f  ROBOT_R=%.6f  CPR=%d\n",
           (double)WHEEL_RADIUS, (double)ROBOT_RADIUS, ENCODER_CPR);
    printf("  RATE=%dHz  PID: Kf=%.0f Kp=%.0f Ki=%.0f Kd=%.0f\n",
           CONTROL_RATE_HZ, (double)PID_KF, (double)PID_KP,
           (double)PID_KI, (double)PID_KD);
    printf("============================================\n");

    // 1. Kinematics
    test_kinematics_basic();
    test_kinematics_sweep();
    test_kinematics_extreme();

    // 2. PID
    test_pid_basic();
    test_pid_step_response();
    test_pid_antiwindup();
    test_pid_direction_change();
    test_pid_disturbance();
    test_pid_different_motors();
    test_pid_multiple_speeds();

    // 3. Odometry
    test_odometry_basic();
    test_odometry_arc();
    test_odometry_full_rotation();
    test_odometry_reverse();
    test_odometry_long_drift();

    // 4. Full-chain closed loop
    test_closed_loop_tracking();
    test_closed_loop_accel_decel();
    test_closed_loop_emergency_stop();
    test_closed_loop_circle();
    test_closed_loop_saturation_recovery();
    test_closed_loop_three_phase();
    test_closed_loop_pure_rotation_track();

    printf("\n############################################\n");
    printf("  Summary: %d/%d passed, %d failed\n",
           tests_passed, tests_run, tests_failed);
    printf("############################################\n");

    return tests_failed > 0 ? 1 : 0;
}
