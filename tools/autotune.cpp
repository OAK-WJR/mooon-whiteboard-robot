// WhiteBoardBot PID auto-tuning (simulation, runs on the host)
//
// Uses the real PIDController class + a first-order model for each motor (kf_true, tau),
// runs coordinate descent on Kp/Ki/Kd/Kf to minimize the step response cost (tracking error + overshoot + steady-state error
// + PWM jitter), and prints one set of gains for each of the three motors.
//
// The three motors differ physically (kf_true: PWM/(m/s) DC gain, tau: time constant) — the values here are
// reasonable examples that show real variation; when calibrating the real robot, just put the measured kf_true/tau into MOTORS[].
//
// Build and run:
//   c++ -std=c++17 -O2 -I src -I include -DNATIVE_TEST \
//       tools/autotune.cpp src/pid_controller.cpp -o .pio/autotune && .pio/autotune
//
// How to identify kf_true/tau on the real robot (motor_test firmware):
//   select a motor, apply a fixed open-loop PWM, read the steady-state wheel speed -> kf_true = PWM / wheel speed;
//   time after the step to reach 63% of the steady-state value ≈ tau.

#include <cstdio>
#include <cmath>
#include <initializer_list>
#include "config.h"
#include "pid_controller.h"

static const float DT   = 1.0f / CONTROL_RATE_HZ;   // control period (50Hz -> 0.02s)
static const float CPR4 = ENCODER_CPR * 4.0f;
static const float CIRC = 2.0f * 3.14159265f * WHEEL_RADIUS;
static const float MPC  = CIRC / CPR4;               // meters per encoder count

// True first-order model of each motor (kf_true = steady-state PWM/(m/s), tau = time constant)
struct MotorModel { const char* name; float kf_true; float tau; };
// Measured on the real robot (open-loop PWM=200, 2026-06-16): kf_true = PWM/(m/s).
// tau uses a 50ms estimate for now (not measured); gain differences between the three wheels come from the measured kf_true.
static const MotorModel MOTORS[3] = {
    {"M0", 504.0f, 0.050f},
    {"M1", 636.0f, 0.050f},
    {"M2", 515.0f, 0.050f},
};

struct Gains { float kp, ki, kd, kf, ilim; };

// Simulate one step and return its cost. Mirrors the firmware loop: PID -> first-order motor -> encoder quantization -> EMA speed feedback.
static float simulateCost(const Gains& g, const MotorModel& m, float target) {
    PIDController pid;
    pid.setGains(g.kp, g.ki, g.kd, g.kf);
    pid.setOutputLimits(-PWM_MAX, PWM_MAX);
    pid.setIntegralLimit(g.ilim);
    pid.setSlewRate(1500.0f);

    float motor_v = 0.0f;      // true motor speed
    double enc = 0.0;          // cumulative encoder count (accumulated as float, quantized by rounding)
    long prev_counts = 0;
    float meas = 0.0f;         // EMA-filtered measured speed (feedback)

    const int N = 150;         // 3.0s
    float itae = 0, overshoot = 0;
    float pwm_sum = 0, pwm_sq = 0; int tail = 0;

    for (int k = 0; k < N; k++) {
        float pwm = pid.compute(target, meas, DT);
        if (pwm >  PWM_MAX) pwm =  PWM_MAX;
        if (pwm < -PWM_MAX) pwm = -PWM_MAX;

        // First-order motor: steady-state speed = pwm / kf_true
        float mv = pwm / m.kf_true;
        motor_v += (mv - motor_v) * DT / m.tau;

        // Encoder quantization + EMA speed (same as firmware odometry: 0.6 old + 0.4 new)
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
         +   2.0f * pwm_std;      // strongly penalize limit-cycle jitter
}

// Average over several operating points to avoid overfitting to a single speed
static float cost(const Gains& g, const MotorModel& m) {
    const float targets[3] = {0.05f, 0.10f, 0.18f};
    float c = 0;
    for (float t : targets) c += simulateCost(g, m, t);
    return c / 3.0f;
}

// Coordinate descent: multiplicative step search on each parameter
static Gains tune(const MotorModel& m) {
    Gains g{200.0f, 20.0f, 0.0f, m.kf_true, 250.0f};   // start from reasonable initial values, Kf≈true DC gain
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
                if (p == 2 && old < 1e-3f) cand = (factor > 1) ? 1.0f : 0.0f;  // Kd can start from 0
                if (cand < 0) continue;
                val = cand;
                float c = cost(g, m);
                if (c < best - 1e-4f) { best = c; improved = true; }
                else val = old;
            }
        }
        // Separately try Kd at zero (in many cases the best Kd≈0)
        float old = g.kd; g.kd = 0;
        float c = cost(g, m);
        if (c < best - 1e-4f) best = c; else g.kd = old;
        if (!improved) break;
    }
    return g;
}

int main() {
    printf("==== WhiteBoardBot PID auto-tuning (simulation) ====\n");
    printf("Control %dHz  CPR4=%.0f  per count=%.2e m\n\n", (int)CONTROL_RATE_HZ, (double)CPR4, (double)MPC);

    Gains g[3];
    for (int i = 0; i < 3; i++) {
        g[i] = tune(MOTORS[i]);
        printf("%s  kf_true=%.0f tau=%.0fms  ->  Kp=%.1f Ki=%.1f Kd=%.1f Kf=%.1f Ilim=%.0f\n",
               MOTORS[i].name, (double)MOTORS[i].kf_true, (double)(MOTORS[i].tau * 1000),
               (double)g[i].kp, (double)g[i].ki, (double)g[i].kd,
               (double)g[i].kf, (double)g[i].ilim);
    }

    printf("\n---- Paste into include/config.example.h ----\n");
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
