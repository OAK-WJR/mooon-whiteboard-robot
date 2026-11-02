#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "kinematics.h"

/**
 * Three-wheel omni wheel odometry
 *
 * Uses ESP32 hardware interrupts to read A/B phase pulses from the 3 encoders,
 * computes wheel speeds each control cycle and integrates them into a pose.
 */

struct Pose2D {
    float x     = 0.0f;  // m
    float y     = 0.0f;  // m
    float theta = 0.0f;  // rad
};

class Odometry {
public:
    /**
     * @param kin        reference to the kinematics instance
     * @param encoder_cpr encoder pulses per revolution (including gear ratio)
     */
    Odometry(const Kinematics& kin, int encoder_cpr);

    /** Initialize encoder pins and interrupts */
    void begin();

    /** Call once per control cycle; updates wheel speeds and pose */
    void update(float dt);

    /** Get the current pose */
    const Pose2D& getPose() const { return pose_; }

    /** Get the current body velocity */
    const BodyVelocity& getVelocity() const { return velocity_; }

    /** Get each wheel's linear speed (computed directly from the encoders, used for PID feedback) */
    const WheelVelocity& getWheelVelocity() const { return wheel_velocity_; }

    /** Reset the pose */
    void reset();

    /** Set the pose (vision correction) */
    void setPose(float x, float y, float theta) {
        pose_.x = x; pose_.y = y; pose_.theta = theta;
    }

    /** Direction-corrected 32-bit signed count window; wraps around on overflow (for debugging) */
    void getEncoderCounts(volatile long counts[3]) const;

private:
    const Kinematics& kin_;
    int encoder_cpr_;

    Pose2D pose_;
    BodyVelocity velocity_;
    WheelVelocity wheel_velocity_;

    // Previous encoder counts (for computing differences)
    // A sample interval must contain fewer than 2^31 net encoder edges.
    uint32_t prev_counts_[3] = {0, 0, 0};
};

// ---- Encoder interrupts (global) ----

/** Initialize encoder pins and interrupt service routines */
void encoders_init();

/** Get the current encoder counts (thread-safe) */
void encoders_read(volatile long counts[3]);

/** Reset the encoder counts */
void encoders_reset();

#endif // ODOMETRY_H
