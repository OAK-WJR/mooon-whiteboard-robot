#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

/**
 * Single DC motor driver (RZ7889-MS)
 * Two-pin control: FI (forward input) + BI (reverse input)
 *
 * Forward: FI=PWM, BI=LOW
 * Reverse: FI=LOW, BI=PWM
 * Coast stop: FI=LOW, BI=LOW; RZ7889-MS active braking is both pins HIGH.
 */
class MotorDriver {
public:
    MotorDriver() = default;

    /**
     * Initialize motor pins
     * @param fi_pin    forward input pin (FI)
     * @param bi_pin    reverse input pin (BI)
     * @param fi_channel ESP32 LEDC channel (for FI)
     * @param bi_channel ESP32 LEDC channel (for BI)
     * @param pwm_freq  PWM frequency
     * @param pwm_res   PWM resolution (bits)
     */
    void begin(uint8_t fi_pin, uint8_t bi_pin,
               uint8_t fi_channel, uint8_t bi_channel,
               uint32_t pwm_freq, uint8_t pwm_res);

    /**
     * Set motor speed
     * @param speed -255 ~ +255, positive = forward, negative = reverse, 0 = stop
     */
    bool setSpeed(int16_t speed);

    /** Coast stop (FI=LOW, BI=LOW) */
    bool stop();

    bool ready() const { return ready_; }
    enum Fault : uint8_t { None=0, InitFailed=1, WriteFailed=2, DetachFailed=4
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
        , GpioFailed=8, ReleaseFailed=16
#endif
    };
    uint8_t fault() const { return fault_; }
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
    int32_t gpioError() const { return gpio_error_; }
    int32_t releaseError() const { return release_error_; }
#endif

private:
    uint8_t fi_pin_      = 0;
    uint8_t bi_pin_      = 0;
    uint8_t fi_channel_  = 0;
    uint8_t bi_channel_  = 0;
    int16_t pwm_max_     = 255;
    bool ready_         = false;
    bool initialized_   = false;
    bool fi_attached_   = false;
    bool bi_attached_   = false;
    uint8_t fault_      = None;
    void forceLow();
    bool writeDuty(uint8_t pin, uint8_t channel, uint32_t duty);
    bool writeFailed();
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
    int32_t gpio_error_ = 0, release_error_ = 0;
    bool preloadLow(uint8_t pin);
    void releaseOutput(uint8_t pin);
    void releaseLeg(uint8_t pin, bool& attached);
#endif
};

/**
 * Three-motor driver manager (3x RZ7889-MS)
 */
class TriMotorDriver {
public:
    void begin();
    void setSpeeds(int16_t s0, int16_t s1, int16_t s2);
    void stopAll();

    bool ready() const { return motors_[0].ready() && motors_[1].ready() && motors_[2].ready(); }

    MotorDriver& motor(int idx) { return motors_[idx]; }

private:
    MotorDriver motors_[3];
};

#endif // MOTOR_DRIVER_H
