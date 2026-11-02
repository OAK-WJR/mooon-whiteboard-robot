#include "kinematics.h"
#include <math.h>

// sin(60°) = sqrt(3)/2 ≈ 0.866
// cos(60°) = 0.5
static constexpr float SIN60 = 0.86602540378f;
static constexpr float COS60 = 0.5f;

Kinematics::Kinematics(float wheel_radius, float robot_radius)
    : wheel_radius_(wheel_radius)
    , robot_radius_(robot_radius)
    , sin60_(SIN60)
    , cos60_(COS60) {}

/**
 * Inverse kinematics (body -> wheel)
 *
 * | v0 |   | -sin60   cos60   d |   | vx    |
 * | v1 | = |  0       -1      d | * | vy    |
 * | v2 |   |  sin60   cos60   d |   | omega |
 */
WheelVelocity Kinematics::inverse(const BodyVelocity& body) const {
    WheelVelocity w;
    w.v[0] = -sin60_ * body.vx + cos60_ * body.vy + robot_radius_ * body.omega;
    w.v[1] =                       -1.0f * body.vy + robot_radius_ * body.omega;
    w.v[2] =  sin60_ * body.vx + cos60_ * body.vy + robot_radius_ * body.omega;
    return w;
}

/**
 * Forward kinematics (wheel -> body)
 *
 * vx    = (sqrt(3)/3) * (v2 - v0)
 * vy    = (1/3) * (v0 + v2) - (2/3) * v1
 * omega = (1/(3*d)) * (v0 + v1 + v2)
 */
BodyVelocity Kinematics::forward(const WheelVelocity& wheels) const {
    BodyVelocity body;
    float v0 = wheels.v[0];
    float v1 = wheels.v[1];
    float v2 = wheels.v[2];

    body.vx    = (SIN60 / 1.5f) * (v2 - v0);            // sqrt(3)/3 = sin60 / 1.5
    body.vy    = (1.0f / 3.0f) * (v0 + v2) - (2.0f / 3.0f) * v1;
    body.omega = (1.0f / (3.0f * robot_radius_)) * (v0 + v1 + v2);

    return body;
}
