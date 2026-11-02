#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/**
 * Feedforward + PID controller
 *
 * output = Kf * target + Kp * error + Ki * integral(error) + Kd * d(error)/dt
 *
 * Features:
 * - Feedforward (FF): fast response, based on the motor speed-voltage model
 * - Proportional (P): corrects static error
 * - Integral (I): removes steady-state error, with anti-windup (integral limit)
 * - Derivative (D): derivative on measurement, avoids derivative spikes from setpoint jumps
 * - Output limits
 */
class PIDController {
public:
    PIDController() = default;

    /**
     * Set PID gains
     * @param kp proportional gain
     * @param ki integral gain
     * @param kd derivative gain
     * @param kf feedforward gain (target is multiplied directly by kf)
     */
    void setGains(float kp, float ki, float kd, float kf = 0.0f);

    /**
     * Set output limits
     * @param min_output minimum output (usually -PWM_MAX)
     * @param max_output maximum output (usually +PWM_MAX)
     */
    void setOutputLimits(float min_output, float max_output);

    /**
     * Set the integral limit (anti-windup)
     * @param max_integral upper bound on the integral term's absolute value
     */
    void setIntegralLimit(float max_integral);

    /**
     * Set the PWM output rate limit (slew rate)
     * @param max_slew PWM/s, max change per second
     */
    void setSlewRate(float max_slew) { max_slew_rate_ = max_slew; }

    /**
     * Compute the control output
     * @param target   target value (desired speed, m/s)
     * @param measured measured value (actual speed, m/s)
     * @param dt       time step (s)
     * @return control output (PWM value)
     */
    float compute(float target, float measured, float dt);

    /** Reset internal state (integral, last error) */
    void reset();

    // Per-gain getter/setter (runtime tuning)
    void setKp(float kp) { kp_ = kp; }
    void setKi(float ki) { ki_ = ki; }
    void setKd(float kd) { kd_ = kd; }
    void setKf(float kf) { kf_ = kf; }
    float getKp() const { return kp_; }
    float getKi() const { return ki_; }
    float getKd() const { return kd_; }
    float getKf() const { return kf_; }
    float getIntegralLimit() const { return max_integral_; }

    // For debugging/tuning
    float getLastError() const { return prev_error_; }
    float getIntegral() const { return integral_; }
    float getLastOutput() const { return last_output_; }
    float getFFTerm() const { return ff_term_; }
    float getPTerm() const { return p_term_; }
    float getITerm() const { return i_term_; }
    float getDTerm() const { return d_term_; }

private:
    // Gains
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;
    float kf_ = 0.0f;

    // Limits
    float min_output_    = -255.0f;
    float max_output_    =  255.0f;
    float max_integral_  =  500.0f;
    float max_slew_rate_ = 5000.0f;  // PWM/s, no limit by default

    // Internal state
    float integral_      = 0.0f;
    float prev_error_    = 0.0f;
    float prev_measured_ = 0.0f;
    bool  first_run_     = true;

    // Per-term values (for debugging)
    float ff_term_       = 0.0f;
    float p_term_        = 0.0f;
    float i_term_        = 0.0f;
    float d_term_        = 0.0f;
    float last_output_   = 0.0f;
};

#endif // PID_CONTROLLER_H
