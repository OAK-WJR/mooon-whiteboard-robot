#include "pid_controller.h"

static float clampf(float val, float lo, float hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

void PIDController::setGains(float kp, float ki, float kd, float kf) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    kf_ = kf;
}

void PIDController::setOutputLimits(float min_output, float max_output) {
    min_output_ = min_output;
    max_output_ = max_output;
}

void PIDController::setIntegralLimit(float max_integral) {
    max_integral_ = max_integral;
}

float PIDController::compute(float target, float measured, float dt) {
    if (dt <= 0.0f) {
        return last_output_;
    }

    float error = target - measured;

    // ---- Feedforward ----
    // Based on a linear motor speed→voltage model: PWM ≈ Kf * target_speed
    ff_term_ = kf_ * target;

    // ---- P term ----
    p_term_ = kp_ * error;

    // ---- I term (anti-windup: stop integrating when the output saturates) ----
    float pre_output = ff_term_ + p_term_ + ki_ * integral_;
    bool saturated = (pre_output >= max_output_ && error > 0) ||
                     (pre_output <= min_output_ && error < 0);
    if (ki_ == 0.0f) {
        integral_ = 0.0f;
        i_term_ = 0.0f;
    } else {
      if (!saturated) {
        integral_ += error * dt;
      }
      i_term_ = clampf(ki_ * integral_, -max_integral_, max_integral_);
      integral_ = i_term_ / ki_;
    }

    // ---- D term (derivative on measurement, avoids spikes on setpoint jumps) ----
    if (first_run_) {
        d_term_ = 0.0f;
        prev_measured_ = measured;
        first_run_ = false;
    } else {
        // d(measurement)/dt, negated because measured rises as error falls
        float d_measured = (measured - prev_measured_) / dt;
        d_term_ = -kd_ * d_measured;
        prev_measured_ = measured;
    }

    prev_error_ = error;

    // ---- Total output ----
    float output = ff_term_ + p_term_ + i_term_ + d_term_;
    output = clampf(output, min_output_, max_output_);

    // PWM rate limit: at most max_slew_rate_ * dt change per control cycle
    // Keeps the motor from jumping suddenly
    float max_delta = max_slew_rate_ * dt;
    float delta = output - last_output_;
    if (delta > max_delta) output = last_output_ + max_delta;
    else if (delta < -max_delta) output = last_output_ - max_delta;

    last_output_ = output;

    return output;
}

void PIDController::reset() {
    integral_      = 0.0f;
    prev_error_    = 0.0f;
    prev_measured_ = 0.0f;
    first_run_     = true;
    ff_term_       = 0.0f;
    p_term_        = 0.0f;
    i_term_        = 0.0f;
    d_term_        = 0.0f;
    last_output_   = 0.0f;
}
