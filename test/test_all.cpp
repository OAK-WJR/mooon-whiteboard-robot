/**
 * WhiteBoardBot 全面模拟测试套件
 *
 * 在桌面 (native) 环境运行, 不需要 ESP32 硬件:
 *   pio test -e native
 *
 * 测试模块:
 *   1. 运动学 — 正逆一致性 / 对称性 / 极端值 / 各方向扫描
 *   2. PID    — 阶跃 / 限幅 / Anti-windup / 换向 / 扰动 / 不同电机模型
 *   3. 里程计 — 直线 / 横移 / 旋转 / 圆弧 / 长时间漂移 / 反向
 *   4. 全链路 — 闭环跟踪 / 加减速 / 急停 / 圆周运动 / 饱和恢复 / 三段运动
 */

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <cfloat>

// ---- 项目模块 (纯数学, 不依赖 Arduino) ----
#include "config.h"
#include "kinematics.h"
#include "pid_controller.h"

// 直接包含实现文件 (避免链接问题, 这些是纯数学不依赖 Arduino)
#include "kinematics.cpp"
#include "pid_controller.cpp"

// Pose2D 在 odometry.h 中定义, 但那个头文件依赖 Arduino
struct Pose2D {
    float x     = 0.0f;
    float y     = 0.0f;
    float theta = 0.0f;
};

// ============================================================
// 简易测试框架
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
    printf("  [%s] %d/%d 通过\n", name, section_run - section_fail, section_run); \
} while(0)

// ============================================================
// 模拟里程计 (不使用 Arduino 的编码器中断)
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
// 闭环模拟器 (复用于多个测试)
// ============================================================
struct SimState {
    Kinematics      kin;
    SimOdometry     odom;
    PIDController   pid[3];
    float           motor_v[3];   // 电机实际速度
    float           motor_tau;    // 电机时间常数
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

    // 执行一步闭环控制, 返回 PWM 数组
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

    // 运行 N 步
    void run(const BodyVelocity& target, int steps) {
        float pwm[3];
        for (int i = 0; i < steps; i++) step(target, pwm);
    }

    // 运行指定秒数
    void runSeconds(const BodyVelocity& target, float seconds) {
        run(target, (int)(seconds / dt));
    }
};

// ############################################################
//                    1. 运动学测试
// ############################################################
static void test_kinematics_basic() {
    SECTION_BEGIN("运动学: 基础正逆一致性");
    Kinematics kin(WHEEL_RADIUS, ROBOT_RADIUS);

    // 纯前进
    {
        BodyVelocity b = {0.1f, 0, 0};
        WheelVelocity w = kin.inverse(b);
        ASSERT_NEAR(w.v[0], -w.v[2], 1e-6f, "纯前进: w0=-w2");
        ASSERT_NEAR(w.v[1], 0, 1e-6f, "纯前进: w1=0");
        BodyVelocity r = kin.forward(w);
        ASSERT_NEAR(r.vx, 0.1f, 1e-5f, "纯前进 recover vx");
        ASSERT_NEAR(r.vy, 0, 1e-5f, "纯前进 recover vy");
        ASSERT_NEAR(r.omega, 0, 1e-5f, "纯前进 recover omega");
    }

    // 纯横移
    {
        BodyVelocity b = {0, 0.1f, 0};
        WheelVelocity w = kin.inverse(b);
        ASSERT_NEAR(w.v[0], w.v[2], 1e-6f, "纯横移: w0=w2");
        ASSERT_NEAR(w.v[1], -0.1f, 1e-5f, "纯横移: w1=-vy");
        BodyVelocity r = kin.forward(w);
        ASSERT_NEAR(r.vx, 0, 1e-5f, "纯横移 recover vx");
        ASSERT_NEAR(r.vy, 0.1f, 1e-5f, "纯横移 recover vy");
    }

    // 纯旋转
    {
        BodyVelocity b = {0, 0, 1.0f};
        WheelVelocity w = kin.inverse(b);
        float ev = ROBOT_RADIUS * 1.0f;
        ASSERT_NEAR(w.v[0], ev, 1e-5f, "纯旋转: w0=R*omega");
        ASSERT_NEAR(w.v[1], ev, 1e-5f, "纯旋转: w1=R*omega");
        ASSERT_NEAR(w.v[2], ev, 1e-5f, "纯旋转: w2=R*omega");
        BodyVelocity r = kin.forward(w);
        ASSERT_NEAR(r.omega, 1.0f, 1e-4f, "纯旋转 recover omega");
    }

    // 零速度
    {
        BodyVelocity b = {0, 0, 0};
        WheelVelocity w = kin.inverse(b);
        ASSERT_NEAR(w.v[0], 0, 1e-8f, "零速度 w0");
        ASSERT_NEAR(w.v[1], 0, 1e-8f, "零速度 w1");
        ASSERT_NEAR(w.v[2], 0, 1e-8f, "零速度 w2");
    }
    SECTION_END("基础正逆");
}

static void test_kinematics_sweep() {
    SECTION_BEGIN("运动学: 360° 方向扫描");
    Kinematics kin(WHEEL_RADIUS, ROBOT_RADIUS);

    // 以 0.1 m/s 扫描所有方向 (0°~350°), 验证正逆一致性
    float speed = 0.1f;
    for (int deg = 0; deg < 360; deg += 10) {
        float rad = (float)deg * M_PI / 180.0f;
        BodyVelocity b = {speed * cosf(rad), speed * sinf(rad), 0};
        WheelVelocity w = kin.inverse(b);
        BodyVelocity r = kin.forward(w);
        char msg[64];
        snprintf(msg, sizeof(msg), "方向 %d°: vx 一致", deg);
        ASSERT_NEAR(r.vx, b.vx, 1e-4f, msg);
        snprintf(msg, sizeof(msg), "方向 %d°: vy 一致", deg);
        ASSERT_NEAR(r.vy, b.vy, 1e-4f, msg);
    }
    printf("  36 个方向 × 2 轴 = 72 项正逆一致性\n");
    SECTION_END("方向扫描");
}

static void test_kinematics_extreme() {
    SECTION_BEGIN("运动学: 极端值与边界");
    Kinematics kin(WHEEL_RADIUS, ROBOT_RADIUS);

    // 负值
    {
        BodyVelocity b = {-0.15f, -0.08f, -2.0f};
        WheelVelocity w = kin.inverse(b);
        BodyVelocity r = kin.forward(w);
        ASSERT_NEAR(r.vx, b.vx, 1e-4f, "负值 vx");
        ASSERT_NEAR(r.vy, b.vy, 1e-4f, "负值 vy");
        ASSERT_NEAR(r.omega, b.omega, 1e-3f, "负值 omega");
    }

    // 最大速度 (所有轮接近 MOTOR_MAX_SPEED)
    {
        float max_v = MOTOR_MAX_SPEED;
        BodyVelocity b = {max_v, 0, 0};
        WheelVelocity w = kin.inverse(b);
        // 任何轮速的绝对值不应超过 sqrt(3)/2 * max_v (由运动学矩阵决定)
        for (int i = 0; i < 3; i++) {
            ASSERT_TRUE(fabsf(w.v[i]) <= max_v * 1.1f, "最大速度: 轮速不超限");
        }
    }

    // 非常小的值 (接近零但不为零)
    {
        BodyVelocity b = {1e-6f, 1e-6f, 1e-6f};
        WheelVelocity w = kin.inverse(b);
        BodyVelocity r = kin.forward(w);
        ASSERT_NEAR(r.vx, b.vx, 1e-5f, "微小值 vx");
        ASSERT_NEAR(r.vy, b.vy, 1e-5f, "微小值 vy");
    }

    // 线性叠加: inverse(a+b) == inverse(a) + inverse(b)
    {
        BodyVelocity a = {0.05f, 0.03f, 0.2f};
        BodyVelocity b = {-0.02f, 0.04f, -0.1f};
        BodyVelocity ab = {a.vx+b.vx, a.vy+b.vy, a.omega+b.omega};
        WheelVelocity wa = kin.inverse(a);
        WheelVelocity wb = kin.inverse(b);
        WheelVelocity wab = kin.inverse(ab);
        for (int i = 0; i < 3; i++) {
            char msg[64];
            snprintf(msg, sizeof(msg), "线性叠加 w%d", i);
            ASSERT_NEAR(wab.v[i], wa.v[i] + wb.v[i], 1e-5f, msg);
        }
    }
    SECTION_END("极端值");
}

// ############################################################
//                    2. PID 控制器测试
// ############################################################
static void test_pid_basic() {
    SECTION_BEGIN("PID: 基础功能");

    // 纯前馈
    {
        PIDController pid;
        pid.setGains(0, 0, 0, PID_KF);
        pid.setOutputLimits(-255, 255);
        float out = pid.compute(0.1f, 0.1f, 0.02f);
        ASSERT_NEAR(out, PID_KF * 0.1f, 0.01f, "纯前馈 output=Kf*target");
    }

    // 输出限幅
    {
        PIDController pid;
        pid.setGains(PID_KP, PID_KI, PID_KD, PID_KF);
        pid.setOutputLimits(-255, 255);
        float out = pid.compute(1.0f, 0, 0.02f);
        ASSERT_TRUE(out <= 255.0f, "上限幅");
        out = pid.compute(-1.0f, 0, 0.02f);
        ASSERT_TRUE(out >= -255.0f, "下限幅");
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

    // dt<=0 不更新
    {
        PIDController pid;
        pid.setGains(PID_KP, PID_KI, PID_KD, PID_KF);
        pid.setOutputLimits(-255, 255);
        float out1 = pid.compute(0.1f, 0, 0.02f);
        float out2 = pid.compute(0.1f, 0, 0.0f);
        ASSERT_NEAR(out2, out1, 1e-8f, "dt=0 不更新");
        float out3 = pid.compute(0.1f, 0, -0.01f);
        ASSERT_NEAR(out3, out1, 1e-8f, "dt<0 不更新");
    }

    // 纯 P 控制
    {
        PIDController pid;
        pid.setGains(100.0f, 0, 0, 0);
        pid.setOutputLimits(-1000, 1000);
        float out = pid.compute(0.1f, 0.05f, 0.02f);
        ASSERT_NEAR(out, 100.0f * 0.05f, 0.01f, "纯P: output=Kp*error");
    }
    SECTION_END("PID 基础");
}

static void test_pid_step_response() {
    SECTION_BEGIN("PID: 阶跃响应");

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

    printf("  阶跃响应 target=%.2f m/s:\n", (double)target);
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

    ASSERT_TRUE(settled, "200 步内收敛到 1%");
    ASSERT_NEAR(actual, target, target * 0.02f, "稳态误差 < 2%");
    ASSERT_TRUE(max_overshoot < 0.15f, "超调量 < 15%");
    printf("  收敛: step %d (%.0f ms), 超调: %.1f%%\n",
           settle_step, (double)(settle_step * dt * 1000),
           (double)(max_overshoot * 100));
    SECTION_END("阶跃响应");
}

static void test_pid_antiwindup() {
    SECTION_BEGIN("PID: Anti-windup (积分限幅验证)");

    float dt = 0.02f;

    // 测试 1: 积分限幅确实生效 (使用小限幅值加速测试)
    {
        float test_limit = 0.5f;  // 小限幅, 便于快速饱和
        PIDController pid;
        pid.setGains(0, PID_KI, 0, 0);  // 只有 I, 便于观察
        pid.setOutputLimits(-10000, 10000);  // 放宽输出限幅, 只测积分限幅
        pid.setIntegralLimit(test_limit);

        // 持续大误差让积分累积: error=1.0, dt=0.02 → 每步 +0.02
        // 到 0.5 需要 25 步, 跑 100 步确保饱和
        for (int i = 0; i < 100; i++) {
            pid.compute(1.0f, 0.0f, dt);
        }
        float integral = pid.getIntegral();
        printf("  累积后积分: %.3f (限幅: ±%.1f)\n",
               (double)integral, (double)test_limit);
        ASSERT_TRUE(fabsf(integral) <= test_limit + 0.001f,
                    "积分被限幅");
        ASSERT_NEAR(integral, test_limit, 0.01f,
                    "积分达到上限");
    }

    // 测试 2: 积分限幅后误差反向, 积分能退绕
    {
        float test_limit = 0.5f;
        PIDController pid;
        pid.setGains(0, PID_KI, 0, 0);
        pid.setOutputLimits(-10000, 10000);
        pid.setIntegralLimit(test_limit);

        // 先饱和到正上限
        for (int i = 0; i < 100; i++) pid.compute(1.0f, 0, dt);
        ASSERT_NEAR(pid.getIntegral(), test_limit, 0.01f, "先饱和到上限");

        // 反向误差, 积分应该下降并饱和到负上限
        for (int i = 0; i < 200; i++) pid.compute(-1.0f, 0, dt);
        float integral = pid.getIntegral();
        printf("  反向后积分: %.3f\n", (double)integral);
        ASSERT_NEAR(integral, -test_limit, 0.01f,
                    "反向后积分退绕到负上限");
    }

    // 测试 3: 闭环饱和恢复 (用真实参数)
    {
        PIDController pid;
        pid.setGains(PID_KP, PID_KI, PID_KD, PID_KF);
        pid.setOutputLimits(-255, 255);
        pid.setIntegralLimit(PID_INTEGRAL_LIMIT);

        float actual = 0, tau = 0.05f;

        // 先追踪 0.15 m/s 稳定 1s
        for (int i = 0; i < 50; i++) {
            float pwm = pid.compute(0.15f, actual, dt);
            actual += (pwm / PID_KF - actual) * dt / tau;
        }

        // 突然切到 0.02 m/s
        bool settled = false;
        for (int i = 0; i < 200; i++) {
            float pwm = pid.compute(0.02f, actual, dt);
            actual += (pwm / PID_KF - actual) * dt / tau;
            if (!settled && fabsf(actual - 0.02f) < 0.003f) {
                settled = true;
                printf("  闭环饱和恢复: step %d (%.0f ms)\n",
                       i, (double)(i * dt * 1000));
            }
        }
        ASSERT_TRUE(settled, "从高速切低速后收敛");
        ASSERT_NEAR(actual, 0.02f, 0.005f, "切换后稳态精度");
    }
    SECTION_END("Anti-windup");
}

static void test_pid_direction_change() {
    SECTION_BEGIN("PID: 换向 (正→负)");

    PIDController pid;
    pid.setGains(PID_KP, PID_KI, PID_KD, PID_KF);
    pid.setOutputLimits(-255, 255);
    pid.setIntegralLimit(PID_INTEGRAL_LIMIT);

    float actual = 0;
    float dt = 1.0f / CONTROL_RATE_HZ;
    float tau = 0.05f;

    // 先正向 1 秒
    for (int i = 0; i < CONTROL_RATE_HZ; i++) {
        float pwm = pid.compute(0.1f, actual, dt);
        actual += (pwm / PID_KF - actual) * dt / tau;
    }
    ASSERT_NEAR(actual, 0.1f, 0.01f, "正向稳态");
    printf("  正向稳态: actual=%.4f\n", (double)actual);

    // 突变到负向
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
    ASSERT_TRUE(settled, "换向后收敛");
    ASSERT_NEAR(actual, -0.1f, 0.01f, "负向稳态");
    printf("  换向完成: step %d (%.0f ms)\n",
           settle_step, (double)(settle_step * dt * 1000));
    SECTION_END("换向");
}

static void test_pid_disturbance() {
    SECTION_BEGIN("PID: 扰动恢复");

    PIDController pid;
    pid.setGains(PID_KP, PID_KI, PID_KD, PID_KF);
    pid.setOutputLimits(-255, 255);
    pid.setIntegralLimit(PID_INTEGRAL_LIMIT);

    float actual = 0;
    float dt = 1.0f / CONTROL_RATE_HZ;
    float tau = 0.05f;
    float target = 0.1f;

    // 先稳定 1 秒
    for (int i = 0; i < CONTROL_RATE_HZ; i++) {
        float pwm = pid.compute(target, actual, dt);
        actual += (pwm / PID_KF - actual) * dt / tau;
    }
    printf("  稳态: actual=%.4f (target=%.4f)\n", (double)actual, (double)target);

    // 施加阶跃扰动: 速度突然降低 50%
    actual *= 0.5f;
    printf("  施加扰动: actual→%.4f\n", (double)actual);

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
    ASSERT_TRUE(recovered, "扰动后恢复");
    ASSERT_NEAR(actual, target, target * 0.03f, "扰动后稳态精度");
    printf("  恢复: step %d (%.0f ms)\n",
           recover_step, (double)(recover_step * dt * 1000));
    SECTION_END("扰动恢复");
}

static void test_pid_different_motors() {
    SECTION_BEGIN("PID: 不同电机特性 (快/慢)");

    float dt = 1.0f / CONTROL_RATE_HZ;
    float target = 0.1f;

    // 快电机 (tau=20ms)
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
        ASSERT_TRUE(settled, "快电机(τ=20ms): 收敛");
        printf("  快电机(τ=20ms): settle step %d\n", step);
    }

    // 慢电机 (tau=150ms)
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
        ASSERT_TRUE(settled, "慢电机(τ=150ms): 收敛");
        ASSERT_NEAR(actual, target, target * 0.05f, "慢电机稳态精度");
        printf("  慢电机(τ=150ms): settle step %d\n", step);
    }
    SECTION_END("电机特性");
}

static void test_pid_multiple_speeds() {
    SECTION_BEGIN("PID: 多速度点阶跃");

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
        snprintf(msg, sizeof(msg), "target=%.2f: 收敛", target);
        ASSERT_TRUE(settled, msg);
        snprintf(msg, sizeof(msg), "target=%.2f: 稳态精度", target);
        ASSERT_NEAR(actual, target, target * 0.05f, msg);
    }
    SECTION_END("多速度点");
}

// ############################################################
//                    3. 里程计测试
// ############################################################
static void test_odometry_basic() {
    SECTION_BEGIN("里程计: 基础运动");
    Kinematics kin(WHEEL_RADIUS, ROBOT_RADIUS);
    SimOdometry odom(kin, ENCODER_CPR);
    float dt = 1.0f / CONTROL_RATE_HZ;

    // 直线前进
    {
        odom.reset();
        BodyVelocity cmd = {0.1f, 0, 0};
        WheelVelocity wv = kin.inverse(cmd);
        for (int i = 0; i < CONTROL_RATE_HZ; i++) {
            odom.simulateWheelVelocities(wv.v, dt);
            odom.update(dt);
        }
        const Pose2D& p = odom.getPose();
        printf("  直线前进 1s: x=%.4f y=%.4f θ=%.4f\n",
               (double)p.x, (double)p.y, (double)p.theta);
        ASSERT_NEAR(p.x, 0.1f, 0.01f, "直线 x≈0.1");
        ASSERT_NEAR(p.y, 0, 0.005f, "直线 y≈0");
        ASSERT_NEAR(p.theta, 0, 0.01f, "直线 θ≈0");
    }

    // 纯横移
    {
        odom.reset();
        BodyVelocity cmd = {0, 0.05f, 0};
        WheelVelocity wv = kin.inverse(cmd);
        for (int i = 0; i < CONTROL_RATE_HZ; i++) {
            odom.simulateWheelVelocities(wv.v, dt);
            odom.update(dt);
        }
        const Pose2D& p = odom.getPose();
        printf("  纯横移 1s: x=%.4f y=%.4f θ=%.4f\n",
               (double)p.x, (double)p.y, (double)p.theta);
        ASSERT_NEAR(p.x, 0, 0.005f, "横移 x≈0");
        ASSERT_NEAR(p.y, 0.05f, 0.01f, "横移 y≈0.05");
    }

    // 原地旋转
    {
        odom.reset();
        BodyVelocity cmd = {0, 0, 1.0f};
        WheelVelocity wv = kin.inverse(cmd);
        for (int i = 0; i < CONTROL_RATE_HZ; i++) {
            odom.simulateWheelVelocities(wv.v, dt);
            odom.update(dt);
        }
        const Pose2D& p = odom.getPose();
        printf("  原地旋转 1s: x=%.4f y=%.4f θ=%.4f\n",
               (double)p.x, (double)p.y, (double)p.theta);
        ASSERT_NEAR(p.theta, 1.0f, 0.1f, "旋转 θ≈1 rad");
        ASSERT_NEAR(p.x, 0, 0.01f, "旋转 x≈0");
        ASSERT_NEAR(p.y, 0, 0.01f, "旋转 y≈0");
    }
    SECTION_END("基础运动");
}

static void test_odometry_arc() {
    SECTION_BEGIN("里程计: 圆弧路径");
    Kinematics kin(WHEEL_RADIUS, ROBOT_RADIUS);
    SimOdometry odom(kin, ENCODER_CPR);
    float dt = 1.0f / CONTROL_RATE_HZ;

    // 转弯半径 R = vx / omega
    float vx = 0.05f;
    float omega = 1.0f;
    float R_turn = vx / omega;  // = 0.05m
    printf("  圆弧: vx=%.2f omega=%.2f → R=%.3f m\n",
           (double)vx, (double)omega, (double)R_turn);

    // 走 π/2 (四分之一圈)
    float duration = (M_PI / 2.0f) / omega;  // ≈ 1.57 s
    BodyVelocity cmd = {vx, 0, omega};
    WheelVelocity wv = kin.inverse(cmd);

    int steps = (int)(duration / dt);
    for (int i = 0; i < steps; i++) {
        odom.simulateWheelVelocities(wv.v, dt);
        odom.update(dt);
    }

    const Pose2D& p = odom.getPose();
    // 四分之一圆弧: 期望 x ≈ R*sin(π/2) = R, y ≈ R*(1-cos(π/2)) = R
    printf("  四分之一圆弧: x=%.4f y=%.4f θ=%.3f (expect x≈%.3f y≈%.3f θ≈%.3f)\n",
           (double)p.x, (double)p.y, (double)p.theta,
           (double)R_turn, (double)R_turn, M_PI/2.0);
    ASSERT_NEAR(p.theta, (float)(M_PI/2.0), 0.15f, "圆弧 θ≈π/2");
    ASSERT_NEAR(p.x, R_turn, R_turn * 0.2f, "圆弧 x≈R");
    ASSERT_NEAR(p.y, R_turn, R_turn * 0.2f, "圆弧 y≈R");
    SECTION_END("圆弧路径");
}

static void test_odometry_full_rotation() {
    SECTION_BEGIN("里程计: 完整 360° 旋转");
    Kinematics kin(WHEEL_RADIUS, ROBOT_RADIUS);
    SimOdometry odom(kin, ENCODER_CPR);
    float dt = 1.0f / CONTROL_RATE_HZ;

    float omega = 2.0f;  // 2 rad/s
    float duration = 2.0f * M_PI / omega;  // 一整圈
    BodyVelocity cmd = {0, 0, omega};
    WheelVelocity wv = kin.inverse(cmd);

    int steps = (int)(duration / dt);
    for (int i = 0; i < steps; i++) {
        odom.simulateWheelVelocities(wv.v, dt);
        odom.update(dt);
    }

    const Pose2D& p = odom.getPose();
    printf("  360° 旋转: x=%.4f y=%.4f θ=%.4f (expect θ≈0 or ±2π)\n",
           (double)p.x, (double)p.y, (double)p.theta);
    // θ 应该回到接近 0 (归一化到 [-π, π])
    ASSERT_NEAR(p.theta, 0, 0.2f, "360° θ 回归");
    ASSERT_NEAR(p.x, 0, 0.01f, "360° x≈0");
    ASSERT_NEAR(p.y, 0, 0.01f, "360° y≈0");
    SECTION_END("360° 旋转");
}

static void test_odometry_reverse() {
    SECTION_BEGIN("里程计: 反向运动");
    Kinematics kin(WHEEL_RADIUS, ROBOT_RADIUS);
    SimOdometry odom(kin, ENCODER_CPR);
    float dt = 1.0f / CONTROL_RATE_HZ;

    // 先前进 1s, 再后退 1s
    {
        odom.reset();
        BodyVelocity fwd = {0.1f, 0, 0};
        WheelVelocity wf = kin.inverse(fwd);
        for (int i = 0; i < CONTROL_RATE_HZ; i++) {
            odom.simulateWheelVelocities(wf.v, dt);
            odom.update(dt);
        }
        float x_mid = odom.getPose().x;
        printf("  前进 1s 后: x=%.4f\n", (double)x_mid);

        BodyVelocity bwd = {-0.1f, 0, 0};
        WheelVelocity wb = kin.inverse(bwd);
        for (int i = 0; i < CONTROL_RATE_HZ; i++) {
            odom.simulateWheelVelocities(wb.v, dt);
            odom.update(dt);
        }
        const Pose2D& p = odom.getPose();
        printf("  往返后: x=%.4f y=%.4f θ=%.4f\n",
               (double)p.x, (double)p.y, (double)p.theta);
        ASSERT_NEAR(p.x, 0, 0.015f, "往返 x≈0");
        ASSERT_NEAR(p.y, 0, 0.005f, "往返 y≈0");
    }
    SECTION_END("反向运动");
}

static void test_odometry_long_drift() {
    SECTION_BEGIN("里程计: 长时间漂移 (10s)");
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
    printf("  10s 直线: x=%.4f (expect %.4f), y=%.6f, θ=%.6f\n",
           (double)p.x, (double)expected_x, (double)p.y, (double)p.theta);
    ASSERT_NEAR(p.x, expected_x, expected_x * 0.02f, "10s x 误差<2%");
    ASSERT_NEAR(p.y, 0, 0.01f, "10s y 漂移<1cm");
    ASSERT_NEAR(p.theta, 0, 0.05f, "10s θ 漂移<0.05rad");
    SECTION_END("长时间漂移");
}

// ############################################################
//                    4. 全链路闭环模拟
// ############################################################
static void test_closed_loop_tracking() {
    SECTION_BEGIN("闭环: 基础跟踪 (斜向运动)");

    SimState sim;
    BodyVelocity target = {0.08f, -0.02f, 0};
    float duration = 3.0f;

    printf("  目标: vx=%.3f vy=%.3f omega=%.3f, %.1f 秒\n",
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
    ASSERT_NEAR(fp.x, ex, ex * 0.15f, "跟踪 x<15%");
    ASSERT_NEAR(fp.y, ey, fabsf(ey) * 0.15f + 0.005f, "跟踪 y<15%");
    ASSERT_NEAR(fp.theta, 0, 0.05f, "跟踪 θ 漂移<0.05 rad");

    const BodyVelocity& fv = sim.odom.getVelocity();
    ASSERT_NEAR(fv.vx, target.vx, 0.01f, "vx 跟踪误差<0.01");
    ASSERT_NEAR(fv.vy, target.vy, 0.01f, "vy 跟踪误差<0.01");
    SECTION_END("基础跟踪");
}

static void test_closed_loop_accel_decel() {
    SECTION_BEGIN("闭环: 加减速梯形速度");

    SimState sim;
    float pwm[3];

    // 阶段1: 加速到 0.1 m/s (1s)
    printf("  阶段 1: 加速 → 0.1 m/s\n");
    BodyVelocity target = {0.1f, 0, 0};
    sim.run(target, CONTROL_RATE_HZ);

    const BodyVelocity& v1 = sim.odom.getVelocity();
    ASSERT_NEAR(v1.vx, 0.1f, 0.01f, "加速后 vx≈0.1");

    // 阶段2: 匀速 2s
    printf("  阶段 2: 匀速 2s\n");
    sim.run(target, CONTROL_RATE_HZ * 2);
    const Pose2D& p2 = sim.odom.getPose();
    printf("    匀速后 pos: (%.4f, %.4f)\n", (double)p2.x, (double)p2.y);

    // 阶段3: 减速到 0 (1s)
    printf("  阶段 3: 减速 → 0\n");
    BodyVelocity stop = {0, 0, 0};
    sim.run(stop, CONTROL_RATE_HZ);

    const BodyVelocity& v3 = sim.odom.getVelocity();
    const Pose2D& p3 = sim.odom.getPose();
    printf("    停止后: vel=(%.4f,%.4f) pos=(%.4f,%.4f)\n",
           (double)v3.vx, (double)v3.vy, (double)p3.x, (double)p3.y);

    // 总距离: 约 0.1 * 3 = 0.3m (加减速阶段贡献约一半)
    ASSERT_NEAR(v3.vx, 0, 0.005f, "停止后 vx≈0");
    ASSERT_TRUE(p3.x > 0.2f && p3.x < 0.35f, "总位移在合理范围");
    SECTION_END("加减速");
}

static void test_closed_loop_emergency_stop() {
    SECTION_BEGIN("闭环: 急停 (模拟 cmd_vel 超时)");

    SimState sim;
    float pwm[3];

    // 全速前进 1s
    BodyVelocity target = {MOTOR_MAX_SPEED * 0.8f, 0, 0};
    sim.run(target, CONTROL_RATE_HZ);

    const BodyVelocity& v1 = sim.odom.getVelocity();
    printf("  全速: vx=%.4f\n", (double)v1.vx);
    ASSERT_TRUE(v1.vx > target.vx * 0.5f, "全速运行中");

    // 急停 (目标归零)
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
    ASSERT_TRUE(stopped, "急停成功");
    printf("  急停: %d 步 (%.0f ms)\n", stop_step, (double)(stop_step * sim.dt * 1000));
    SECTION_END("急停");
}

static void test_closed_loop_circle() {
    SECTION_BEGIN("闭环: 圆周运动 (画圆)");

    SimState sim;
    float pwm[3];

    // 画一个完整的圆: vx=0.05, omega=1.0 → R=0.05m, 周期=2π s
    BodyVelocity target = {0.05f, 0, 1.0f};
    float period = 2.0f * M_PI / 1.0f;  // ≈6.28s
    int steps = (int)(period / sim.dt);

    printf("  画圆: vx=%.2f omega=%.2f R=%.3fm, 周期=%.2fs\n",
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
    printf("  画圆终点: (%.4f,%.4f) θ=%.3f\n",
           (double)fp.x, (double)fp.y, (double)fp.theta);

    // 回到起点附近
    float dist = sqrtf(fp.x * fp.x + fp.y * fp.y);
    ASSERT_TRUE(dist < 0.03f, "画圆回到原点 (距离<3cm)");
    // θ 应回到接近 0
    ASSERT_NEAR(fp.theta, 0, 0.3f, "画圆 θ 回归");
    SECTION_END("圆周运动");
}

static void test_closed_loop_saturation_recovery() {
    SECTION_BEGIN("闭环: PWM 饱和与恢复");

    SimState sim;
    float pwm[3];

    // 命令一个超过电机最大速度的目标
    BodyVelocity over_target = {MOTOR_MAX_SPEED * 1.5f, 0, 0};
    printf("  过速目标: vx=%.3f (max=%.3f)\n",
           (double)over_target.vx, (double)MOTOR_MAX_SPEED);

    // 运行 1s, PWM 应该饱和
    bool saturated = false;
    for (int i = 0; i < CONTROL_RATE_HZ; i++) {
        sim.step(over_target, pwm);
        if (fabsf(pwm[0]) >= PWM_MAX - 1 || fabsf(pwm[2]) >= PWM_MAX - 1) {
            saturated = true;
        }
    }
    ASSERT_TRUE(saturated, "PWM 达到饱和");

    // 恢复到正常速度
    BodyVelocity normal = {0.05f, 0, 0};
    bool recovered = false;
    for (int i = 0; i < CONTROL_RATE_HZ * 2; i++) {
        sim.step(normal, pwm);
        const BodyVelocity& v = sim.odom.getVelocity();
        if (!recovered && fabsf(v.vx - normal.vx) < 0.01f) {
            recovered = true;
            printf("  饱和恢复: step %d (%.0f ms)\n", i, (double)(i * sim.dt * 1000));
        }
    }
    ASSERT_TRUE(recovered, "饱和后恢复正常跟踪");
    SECTION_END("饱和恢复");
}

static void test_closed_loop_three_phase() {
    SECTION_BEGIN("闭环: 三段运动 (前进→转弯→横移)");

    SimState sim;

    // 阶段 1: 前进 0.08 m/s, 2s
    BodyVelocity phase1 = {0.08f, 0, 0};
    sim.runSeconds(phase1, 2.0f);
    const Pose2D& p1 = sim.odom.getPose();
    printf("  阶段1 (前进): pos(%.4f,%.4f) θ=%.3f\n",
           (double)p1.x, (double)p1.y, (double)p1.theta);
    ASSERT_NEAR(p1.x, 0.16f, 0.03f, "前进 x≈0.16");

    // 阶段 2: 原地旋转 90° (π/2 rad)
    float omega_turn = 2.0f;
    float turn_duration = (M_PI / 2.0f) / omega_turn;
    BodyVelocity phase2 = {0, 0, omega_turn};
    sim.runSeconds(phase2, turn_duration);
    const Pose2D& p2 = sim.odom.getPose();
    printf("  阶段2 (转90°): pos(%.4f,%.4f) θ=%.3f (expect≈%.3f)\n",
           (double)p2.x, (double)p2.y, (double)p2.theta, M_PI/2.0);
    ASSERT_NEAR(p2.theta, (float)(M_PI/2.0), 0.15f, "转弯 θ≈π/2");

    // 阶段 3: 前进 (此时全局 y 方向) 0.08 m/s, 2s
    BodyVelocity phase3 = {0.08f, 0, 0};
    sim.runSeconds(phase3, 2.0f);
    const Pose2D& p3 = sim.odom.getPose();
    printf("  阶段3 (转后前进): pos(%.4f,%.4f) θ=%.3f\n",
           (double)p3.x, (double)p3.y, (double)p3.theta);

    // 经过 90° 转弯后前进, 应该主要在 y 方向移动
    ASSERT_TRUE(p3.y > 0.10f, "转后前进: y 增大");
    // x 应该和阶段1 结束时差不多 (不再增加)
    ASSERT_NEAR(p3.x, p1.x, 0.04f, "转后前进: x 基本不变");
    SECTION_END("三段运动");
}

static void test_closed_loop_pure_rotation_track() {
    SECTION_BEGIN("闭环: 纯旋转跟踪精度");

    SimState sim;

    // 正向旋转 2 rad/s, 1s
    BodyVelocity target = {0, 0, 2.0f};
    sim.runSeconds(target, 1.0f);

    const Pose2D& p = sim.odom.getPose();
    const BodyVelocity& v = sim.odom.getVelocity();
    printf("  旋转 1s: θ=%.4f (expect 2.0) omega=%.4f\n",
           (double)p.theta, (double)v.omega);
    ASSERT_NEAR(p.theta, 2.0f, 0.2f, "旋转跟踪 θ");
    ASSERT_NEAR(p.x, 0, 0.01f, "旋转 x≈0");
    ASSERT_NEAR(p.y, 0, 0.01f, "旋转 y≈0");

    // 反向旋转回来
    BodyVelocity reverse = {0, 0, -2.0f};
    sim.runSeconds(reverse, 1.0f);

    const Pose2D& p2 = sim.odom.getPose();
    printf("  反转回: θ=%.4f (expect ≈0)\n", (double)p2.theta);
    ASSERT_NEAR(p2.theta, 0, 0.3f, "反转回 θ≈0");
    SECTION_END("纯旋转跟踪");
}

// ############################################################
//                         main
// ############################################################
int main() {
    printf("============================================\n");
    printf("  WhiteBoardBot 全面模拟测试\n");
    printf("  WHEEL_R=%.4f  ROBOT_R=%.6f  CPR=%d\n",
           (double)WHEEL_RADIUS, (double)ROBOT_RADIUS, ENCODER_CPR);
    printf("  RATE=%dHz  PID: Kf=%.0f Kp=%.0f Ki=%.0f Kd=%.0f\n",
           CONTROL_RATE_HZ, (double)PID_KF, (double)PID_KP,
           (double)PID_KI, (double)PID_KD);
    printf("============================================\n");

    // 1. 运动学
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

    // 3. 里程计
    test_odometry_basic();
    test_odometry_arc();
    test_odometry_full_rotation();
    test_odometry_reverse();
    test_odometry_long_drift();

    // 4. 全链路闭环
    test_closed_loop_tracking();
    test_closed_loop_accel_decel();
    test_closed_loop_emergency_stop();
    test_closed_loop_circle();
    test_closed_loop_saturation_recovery();
    test_closed_loop_three_phase();
    test_closed_loop_pure_rotation_track();

    printf("\n############################################\n");
    printf("  总结: %d/%d 通过, %d 失败\n",
           tests_passed, tests_run, tests_failed);
    printf("############################################\n");

    return tests_failed > 0 ? 1 : 0;
}
