#include "motor_driver.h"
#include "firmware_config.h"
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
#include <driver/gpio.h>
#endif

// Arduino-ESP32 core 3.x changed the LEDC API: channels are assigned by the core; everything works by "pin".
// core 2.x still needs channels set by hand; the write API gives no failure feedback.

// ---- MotorDriver (RZ7889-MS: FI + BI two-pin control) ----

void MotorDriver::begin(uint8_t fi_pin, uint8_t bi_pin,
                         uint8_t fi_channel, uint8_t bi_channel,
                         uint32_t pwm_freq, uint8_t pwm_res) {
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
    // One initialization per driver lifetime: no call can clear a latched fault.
    if (initialized_) return;
#else
    if (initialized_) forceLow();
    if (fi_attached_ || bi_attached_) return; // A failed detach cannot be cleared by retrying attach.
#endif
    fi_pin_     = fi_pin;
    bi_pin_     = bi_pin;
    fi_channel_ = fi_channel;
    bi_channel_ = bi_channel;
    initialized_ = true;
    fault_ = None;

#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
    // Caller must already hold the selected ARM request LOW. GPIO and LEDC are serialized
    // here; do not use Arduino digitalWrite on these IDF-owned GPIO latches.
#if defined(WBB_TOP_BOARD_KISS_C6)
    if (!kiss_c6::isMotorPair(fi_pin_, bi_pin_) ||
#elif defined(WBB_TOP_BOARD_PICO)
    if (!pico::isMotorPair(fi_pin_, bi_pin_) ||
#else
    if (fi_pin_ > 30 || bi_pin_ > 30 || fi_pin_ == bi_pin_ ||
#endif
        !pwm_freq || !pwm_res || pwm_res > 14) {
        fault_ |= InitFailed;
        forceLow();
        return;
    }
    const bool fiLow = preloadLow(fi_pin_);
    const bool biLow = preloadLow(bi_pin_);
    if (!fiLow || !biLow) {
        fault_ |= InitFailed;
        forceLow();
        return;
    }
    gpio_config_t config{};
    config.pin_bit_mask = (1ULL << fi_pin_) | (1ULL << bi_pin_);
    config.mode = GPIO_MODE_OUTPUT;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    const esp_err_t result = gpio_config(&config);
    if (result != ESP_OK) {
        gpio_error_ = result; fault_ |= InitFailed | GpioFailed;
        forceLow();
        return;
    }
#else
    digitalWrite(fi_pin_, LOW);
    digitalWrite(bi_pin_, LOW);
    pinMode(fi_pin_, OUTPUT);
    pinMode(bi_pin_, OUTPUT);
    if (fi_pin_ == bi_pin_ || !pwm_freq || !pwm_res || pwm_res > 14) {
        fault_ |= InitFailed;
        forceLow();
        return;
    }
#endif
    pwm_max_ = (1 << pwm_res) - 1;
#if defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
    // The current electrical handoff does not qualify FI/BI modulation.
    // Keep the checked LOW GPIOs; attach no PWM peripheral at all.
    return;
#endif
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
    // Arduino 3.3.9 attach may reuse retained channel duty. Zero each successful
    // attachment immediately; ARM-low hardware must contain the attach interval.
    fi_attached_ = ledcAttach(fi_pin_, pwm_freq, pwm_res);
    if (!fi_attached_) fault_ |= InitFailed;
    else if (!writeDuty(fi_pin_, fi_channel_, 0)) fault_ |= InitFailed | WriteFailed;
    if (fault_ == None) {
        bi_attached_ = ledcAttach(bi_pin_, pwm_freq, pwm_res);
        if (!bi_attached_) fault_ |= InitFailed;
        else if (!writeDuty(bi_pin_, bi_channel_, 0)) fault_ |= InitFailed | WriteFailed;
    }
    if (fault_ != None) { forceLow(); return; }
    ready_ = true;
    return;
#else
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    const bool fiReady = fi_attached_ = ledcAttach(fi_pin_, pwm_freq, pwm_res);
    const bool biReady = bi_attached_ = ledcAttach(bi_pin_, pwm_freq, pwm_res);
    ready_ = fiReady && biReady;
    if (!ready_) {
        fault_ |= InitFailed;
        forceLow();
        Serial.printf("FAULT PWM_INIT: FI GPIO%u=%d BI GPIO%u=%d\n",
                      fi_pin_, fiReady, bi_pin_, biReady);
    }
#else
    const double fiHz = ledcSetup(fi_channel_, pwm_freq, pwm_res);
    const double biHz = ledcSetup(bi_channel_, pwm_freq, pwm_res);
    ready_ = fiHz > 0 && biHz > 0;
    if (ready_) {
        ledcAttachPin(fi_pin_, fi_channel_);
        ledcAttachPin(bi_pin_, bi_channel_);
        fi_attached_ = bi_attached_ = true;
    } else {
        fault_ |= InitFailed;
        Serial.println("FAULT PWM_INIT: LEDC setup failed");
    }
#endif

    stop();
#endif
}

bool MotorDriver::writeDuty(uint8_t pin, uint8_t channel, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    (void)channel;
    return ledcWrite(pin, duty);
#else
    (void)pin;
    ledcWrite(channel, duty); // Core 2.x exposes no write-result feedback.
    return true;
#endif
}

void MotorDriver::forceLow() {
    ready_ = false;
    if (!initialized_) return;
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
    // Fault cleanup never enables a GPIO output. A detach itself reconnects the
    // GPIO latch, so preload before detach, not merely before gpio_config.
    releaseLeg(fi_pin_, fi_attached_);
    releaseLeg(bi_pin_, bi_attached_);
#else
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    if (fi_attached_) {
        if (ledcDetach(fi_pin_)) fi_attached_ = false;
        else fault_ |= DetachFailed;
    }
    if (bi_attached_) {
        if (ledcDetach(bi_pin_)) bi_attached_ = false;
        else fault_ |= DetachFailed;
    }
#else
    if (fi_attached_) ledcDetachPin(fi_pin_);
    if (bi_attached_) ledcDetachPin(bi_pin_);
    fi_attached_ = bi_attached_ = false;
#endif
    pinMode(fi_pin_, OUTPUT);
    pinMode(bi_pin_, OUTPUT);
    digitalWrite(fi_pin_, LOW);
    digitalWrite(bi_pin_, LOW);
    // API attempts are not proof of physical motor-power isolation.
#endif
}

#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
bool MotorDriver::preloadLow(uint8_t pin) {
    const esp_err_t result = gpio_set_level(static_cast<gpio_num_t>(pin), 0);
    if (result == ESP_OK) return true;
    if (!gpio_error_) gpio_error_ = result;
    fault_ |= GpioFailed;
    return false;
}

void MotorDriver::releaseOutput(uint8_t pin) {
    const esp_err_t result = gpio_set_direction(static_cast<gpio_num_t>(pin), GPIO_MODE_INPUT);
    if (result != ESP_OK) {
        if (!release_error_) release_error_ = result;
        fault_ |= ReleaseFailed;
    }
}

void MotorDriver::releaseLeg(uint8_t pin, bool& attached) {
#if defined(WBB_TOP_BOARD_KISS_C6)
    if (!kiss_c6::isMotorPin(pin)) return;
#elif defined(WBB_TOP_BOARD_PICO)
    if (!pico::isMotorPin(pin)) return;
#else
    if (pin > 30) return; // Invalid begin() must never address/shift an invalid pin.
#endif
    if (preloadLow(pin) && attached) {
        if (ledcDetach(pin)) attached = false;
        else fault_ |= DetachFailed;
    }
    // On preload failure, defer detach rather than expose an unknown latch.
    // On detach/ownership failure, retain the flag and still attempt checked
    // output disable. Neither GPIO release nor a later retry clears the fault.
    releaseOutput(pin);
}
#endif

bool MotorDriver::writeFailed() {
    fault_ |= WriteFailed;
    forceLow();
    return false;
}

bool MotorDriver::setSpeed(int16_t speed) {
#if defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
    const bool stopped = stop();
    return speed == 0 && stopped; // No path to a nonzero duty in this profile.
#endif
    if (!ready_) { forceLow(); return false; }
    if (speed > 0) {
        // Forward: FI=PWM, BI=LOW
        if (!writeDuty(bi_pin_, bi_channel_, 0)) return writeFailed();
        if (!writeDuty(fi_pin_, fi_channel_, constrain(speed, 0, pwm_max_))) return writeFailed();
    } else if (speed < 0) {
        // Reverse: FI=LOW, BI=PWM
        if (!writeDuty(fi_pin_, fi_channel_, 0)) return writeFailed();
        if (!writeDuty(bi_pin_, bi_channel_, constrain(-int32_t(speed), 0, pwm_max_))) return writeFailed();
    } else {
        return stop();
    }
    return true;
}

bool MotorDriver::stop() {
    // RZ7889-MS: FI=LOW, BI=LOW is a high-impedance coast stop.
#if defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
#ifdef WBB_TOP_BOARD_KISS_C6
    if (!initialized_ || !kiss_c6::isMotorPair(fi_pin_, bi_pin_)) return false;
#else
    if (!initialized_ || !pico::isMotorPair(fi_pin_, bi_pin_)) return false;
#endif
    if (fault_ != None) { forceLow(); return false; }
    const bool fiLow = preloadLow(fi_pin_);
    const bool biLow = preloadLow(bi_pin_);
    if (!fiLow || !biLow) { forceLow(); return false; }
    return true; // Still not PWM-ready, armed, or measured as power-isolated.
#endif
    if (!ready_) {
        forceLow();
        return false;
    }
    const bool fiOK = writeDuty(fi_pin_, fi_channel_, 0);
    const bool biOK = writeDuty(bi_pin_, bi_channel_, 0);
    if (!fiOK || !biOK) return writeFailed();
    return true;
}

// ---- TriMotorDriver (3x RZ7889-MS) ----

void TriMotorDriver::begin() {
    // Each RZ7889 needs 2 LEDC channels (FI + BI)
    // Motor0: channel 0 (FI), channel 1 (BI)
    motors_[0].begin(MOTOR0_FI_PIN, MOTOR0_BI_PIN,
                     0, 1, PWM_FREQUENCY, PWM_RESOLUTION);
    // Motor1: channel 2 (FI), channel 3 (BI)
    motors_[1].begin(MOTOR1_FI_PIN, MOTOR1_BI_PIN,
                     2, 3, PWM_FREQUENCY, PWM_RESOLUTION);
    // Motor2: channel 4 (FI), channel 5 (BI)
    motors_[2].begin(MOTOR2_FI_PIN, MOTOR2_BI_PIN,
                     4, 5, PWM_FREQUENCY, PWM_RESOLUTION);
}

void TriMotorDriver::setSpeeds(int16_t s0, int16_t s1, int16_t s2) {
    if (!ready()) { stopAll(); return; }
    const int16_t speeds[3] = {s0, s1, s2};
    for (int i=0; i<3; ++i) {
        if (!motors_[i].setSpeed(speeds[i])) { stopAll(); return; }
    }
}

void TriMotorDriver::stopAll() {
    motors_[0].stop();
    motors_[1].stop();
    motors_[2].stop();
}
