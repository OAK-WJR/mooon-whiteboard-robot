#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/**
 * 前馈 + PID 控制器
 *
 * 输出 = Kf * target + Kp * error + Ki * integral(error) + Kd * d(error)/dt
 *
 * 特性:
 * - 前馈 (FF): 快速响应, 基于电机速度-电压模型
 * - 比例 (P): 修正静态误差
 * - 积分 (I): 消除稳态误差, 带 anti-windup (积分限幅)
 * - 微分 (D): 基于测量值微分 (derivative on measurement), 避免 setpoint 突变引起的微分尖峰
 * - 输出限幅
 */
class PIDController {
public:
    PIDController() = default;

    /**
     * 设置 PID 增益
     * @param kp 比例增益
     * @param ki 积分增益
     * @param kd 微分增益
     * @param kf 前馈增益 (target 直接乘以 kf)
     */
    void setGains(float kp, float ki, float kd, float kf = 0.0f);

    /**
     * 设置输出限幅
     * @param min_output 最小输出 (通常 -PWM_MAX)
     * @param max_output 最大输出 (通常 +PWM_MAX)
     */
    void setOutputLimits(float min_output, float max_output);

    /**
     * 设置积分限幅 (anti-windup)
     * @param max_integral 积分项绝对值上限
     */
    void setIntegralLimit(float max_integral);

    /**
     * 设置 PWM 输出变化率限制 (slew rate)
     * @param max_slew PWM/s, 每秒最大变化量
     */
    void setSlewRate(float max_slew) { max_slew_rate_ = max_slew; }

    /**
     * 计算控制输出
     * @param target   目标值 (期望速度, m/s)
     * @param measured 测量值 (实际速度, m/s)
     * @param dt       时间步长 (s)
     * @return 控制输出 (PWM 值)
     */
    float compute(float target, float measured, float dt);

    /** 重置内部状态 (积分、上次误差) */
    void reset();

    // 单项增益 getter/setter (运行时调参)
    void setKp(float kp) { kp_ = kp; }
    void setKi(float ki) { ki_ = ki; }
    void setKd(float kd) { kd_ = kd; }
    void setKf(float kf) { kf_ = kf; }
    float getKp() const { return kp_; }
    float getKi() const { return ki_; }
    float getKd() const { return kd_; }
    float getKf() const { return kf_; }
    float getIntegralLimit() const { return max_integral_; }

    // 调试/调参用
    float getLastError() const { return prev_error_; }
    float getIntegral() const { return integral_; }
    float getLastOutput() const { return last_output_; }
    float getFFTerm() const { return ff_term_; }
    float getPTerm() const { return p_term_; }
    float getITerm() const { return i_term_; }
    float getDTerm() const { return d_term_; }

private:
    // 增益
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;
    float kf_ = 0.0f;

    // 限幅
    float min_output_    = -255.0f;
    float max_output_    =  255.0f;
    float max_integral_  =  500.0f;
    float max_slew_rate_ = 5000.0f;  // PWM/s, 默认不限制

    // 内部状态
    float integral_      = 0.0f;
    float prev_error_    = 0.0f;
    float prev_measured_ = 0.0f;
    bool  first_run_     = true;

    // 各项分解值 (调试用)
    float ff_term_       = 0.0f;
    float p_term_        = 0.0f;
    float i_term_        = 0.0f;
    float d_term_        = 0.0f;
    float last_output_   = 0.0f;
};

#endif // PID_CONTROLLER_H
