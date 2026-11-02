#ifndef KINEMATICS_H
#define KINEMATICS_H

/**
 * Three-wheel omni wheel kinematics (120° layout)
 *
 * Wheel layout (top view):
 *        wheel1 (240°/rear)
 *        /    \
 *   wheel0    wheel2
 *   (120°)    (0°/360°)
 *
 * Forward kinematics: wheel speeds -> body velocity
 * Inverse kinematics: body velocity -> wheel speeds
 */

struct BodyVelocity {
    float vx;    // m/s, forward
    float vy;    // m/s, left
    float omega; // rad/s, counterclockwise positive
};

struct WheelVelocity {
    float v[3];  // linear speed of the three wheels (m/s)
};

class Kinematics {
public:
    /**
     * @param wheel_radius  wheel radius (m)
     * @param robot_radius  distance from wheel center to chassis center (m)
     */
    Kinematics(float wheel_radius, float robot_radius);

    /** Inverse kinematics: body velocity -> wheel linear speeds */
    WheelVelocity inverse(const BodyVelocity& body) const;

    /** Forward kinematics: wheel linear speeds -> body velocity */
    BodyVelocity forward(const WheelVelocity& wheels) const;

    float getWheelRadius() const { return wheel_radius_; }
    float getRobotRadius() const { return robot_radius_; }

private:
    float wheel_radius_;
    float robot_radius_;

    // Inverse kinematics matrix coefficients (precomputed)
    float sin60_;
    float cos60_;
};

#endif // KINEMATICS_H
