// WhiteBoardBot PID 自动调参 (仿真, 主机运行)
//
// 用真实的 PIDController 类 + 每个电机各自的一阶模型 (kf_true, tau),
// 对 Kp/Ki/Kd/Kf 做坐标下降, 最小化阶跃响应代价 (跟踪误差 + 超调 + 稳态误差
// + PWM 抖动), 为三个电机各自输出一套增益。
//
// 三个电机的物理特性不同 (kf_true: PWM/(m/s) 直流增益, tau: 时间常数) —— 这里给的是
// 合理的示例值, 体现真实变差; 真机标定时把测得的 kf_true/tau 填进 MOTORS[] 即可。
//
// 编译运行:
//   c++ -std=c++17 -O2 -I src -I include -DNATIVE_TEST \
//       tools/autotune.cpp src/pid_controller.cpp -o .pio/autotune && .pio/autotune
//
// 真机识别 kf_true/tau 的方法 (motor_test 固件):
//   选电机, 开环给固定 PWM, 读稳态轮速 -> kf_true = PWM / 轮速;
//   阶跃后到 63% 稳态值的时间 ≈ tau。

#include <cstdio>
#include <cmath>
#include <initializer_list>
#include "config.h"
#include "pid_controller.h"

static const float DT   = 1.0f / CONTROL_RATE_HZ;   // 控制周期 (50Hz -> 0.02s)
static const float CPR4 = ENCODER_CPR * 4.0f;
static const float CIRC = 2.0f * 3.14159265f * WHEEL_RADIUS;
static const float MPC  = CIRC / CPR4;               // 每个编码器计数对应的米数

// 每个电机的真实一阶模型 (kf_true = 稳态 PWM/(m/s), tau = 时间常数)
struct MotorModel { const char* name; float kf_true; float tau; };
// 真机实测 (开环 PWM=200, 2026-06-16): kf_true = PWM/(m/s)。
// tau 暂用 50ms 估值 (未实测), 三轮增益差异由实测 kf_true 驱动。
static const MotorModel MOTORS[3] = {
    {"M0", 504.0f, 0.050f},
    {"M1", 636.0f, 0.050f},
    {"M2", 515.0f, 0.050f},
};

struct Gains { float kp, ki, kd, kf, ilim; };

// 仿真一次阶跃, 返回代价。复刻固件回路: PID -> 一阶电机 -> 编码器量化 -> EMA 测速反馈。
static float simulateCost(const Gains& g, const MotorModel& m, float target) {
    PIDController pid;
    pid.setGains(g.kp, g.ki, g.kd, g.kf);
    pid.setOutputLimits(-PWM_MAX, PWM_MAX);
    pid.setIntegralLimit(g.ilim);
    pid.setSlewRate(1500.0f);

    float motor_v = 0.0f;      // 电机真实速度
    double enc = 0.0;          // 编码器累计计数 (浮点累计, 取整量化)
    long prev_counts = 0;
    float meas = 0.0f;         // EMA 滤波后的测速 (反馈)

    const int N = 150;         // 3.0s
    float itae = 0, overshoot = 0;
    float pwm_sum = 0, pwm_sq = 0; int tail = 0;

    for (int k = 0; k < N; k++) {
        float pwm = pid.compute(target, meas, DT);
        if (pwm >  PWM_MAX) pwm =  PWM_MAX;
        if (pwm < -PWM_MAX) pwm = -PWM_MAX;

        // 一阶电机: 稳态速度 = pwm / kf_true
        float mv = pwm / m.kf_true;
        motor_v += (mv - motor_v) * DT / m.tau;

        // 编码器量化 + EMA 测速 (与固件 odometry 一致: 0.6 旧 + 0.4 新)
        enc += motor_v / MPC * DT;
        long counts = (long)enc;
        float raw = (float)(counts - prev_counts) * MPC / DT;
        prev_counts = counts;
        meas = 0.6f * meas + 0.4f * raw;

        float err = target - motor_v;
        float t = k * DT;
        itae += fabsf(err) * t;
        if (motor_v > target) overshoot = fmaxf(overshoot, motor_v - target);

        if (k >= N - 25) { pwm_sum += pwm; pwm_sq += pwm * pwm; tail++; }
    }

    float steady_err = fabsf(target - motor_v);
    float pwm_mean = pwm_sum / tail;
    float pwm_std  = sqrtf(fmaxf(0.0f, pwm_sq / tail - pwm_mean * pwm_mean));

    return itae
         + 300.0f * steady_err
         +  80.0f * fmaxf(0.0f, overshoot - 0.02f * target)
         +   2.0f * pwm_std;      // 强烈惩罚极限环抖动
}

// 多个工作点取平均, 避免只在单一速度过拟合
static float cost(const Gains& g, const MotorModel& m) {
    const float targets[3] = {0.05f, 0.10f, 0.18f};
    float c = 0;
    for (float t : targets) c += simulateCost(g, m, t);
    return c / 3.0f;
}

// 坐标下降: 每个参数乘法步进搜索
static Gains tune(const MotorModel& m) {
    Gains g{200.0f, 20.0f, 0.0f, m.kf_true, 250.0f};   // 从合理初值起步, Kf≈真实直流增益
    float best = cost(g, m);

    float* params[] = {&g.kp, &g.ki, &g.kd, &g.kf, &g.ilim};
    const int NP = 5;

    for (int pass = 0; pass < 60; pass++) {
        bool improved = false;
        for (int p = 0; p < NP; p++) {
            float& val = *params[p];
            for (float factor : {1.4f, 1.0f / 1.4f, 1.15f, 1.0f / 1.15f}) {
                float old = val;
                float cand = old * factor;
                if (p == 2 && old < 1e-3f) cand = (factor > 1) ? 1.0f : 0.0f;  // Kd 可从 0 起步
                if (cand < 0) continue;
                val = cand;
                float c = cost(g, m);
                if (c < best - 1e-4f) { best = c; improved = true; }
                else val = old;
            }
        }
        // Kd 单独尝试归零 (很多情况下最优 Kd≈0)
        float old = g.kd; g.kd = 0;
        float c = cost(g, m);
        if (c < best - 1e-4f) best = c; else g.kd = old;
        if (!improved) break;
    }
    return g;
}

int main() {
    printf("==== WhiteBoardBot PID 自动调参 (仿真) ====\n");
    printf("控制 %dHz  CPR4=%.0f  每格=%.2e m\n\n", (int)CONTROL_RATE_HZ, (double)CPR4, (double)MPC);

    Gains g[3];
    for (int i = 0; i < 3; i++) {
        g[i] = tune(MOTORS[i]);
        printf("%s  kf_true=%.0f tau=%.0fms  ->  Kp=%.1f Ki=%.1f Kd=%.1f Kf=%.1f Ilim=%.0f\n",
               MOTORS[i].name, (double)MOTORS[i].kf_true, (double)(MOTORS[i].tau * 1000),
               (double)g[i].kp, (double)g[i].ki, (double)g[i].kd,
               (double)g[i].kf, (double)g[i].ilim);
    }

    printf("\n---- 粘贴到 include/config.example.h ----\n");
    const char* sfx[3] = {"M0", "M1", "M2"};
    for (int i = 0; i < 3; i++) {
        printf("#define PID_KP_%s           %.1f\n", sfx[i], (double)g[i].kp);
        printf("#define PID_KI_%s           %.1f\n", sfx[i], (double)g[i].ki);
        printf("#define PID_KD_%s           %.1f\n", sfx[i], (double)g[i].kd);
        printf("#define PID_KF_%s           %.1f\n", sfx[i], (double)g[i].kf);
        printf("#define PID_ILIM_%s         %.0f\n\n", sfx[i], (double)g[i].ilim);
    }
    printf("#define PID_KF              %.1f\n", (double)g[1].kf);
    printf("#define PID_KP              %.1f\n", (double)g[1].kp);
    printf("#define PID_KI              %.1f\n", (double)g[1].ki);
    printf("#define PID_KD              %.1f\n", (double)g[1].kd);
    printf("#define PID_INTEGRAL_LIMIT  %.0f\n", (double)g[1].ilim);
    return 0;
}
