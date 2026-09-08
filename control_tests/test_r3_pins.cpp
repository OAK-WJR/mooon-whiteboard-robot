// Public config must win over the local include/config.h in real consumers.
#include <cassert>
#include <iostream>
#include "config.example.h"
#include "config.h"
#include "../src/kinematics.cpp"
#include "../src/motor_driver.cpp"
#include "../src/odometry.cpp"
int main() {
    TriMotorDriver motors; motors.begin();
    Kinematics kinematics(WHEEL_RADIUS, ROBOT_RADIUS);
    Odometry odometry(kinematics, ENCODER_CPR); odometry.begin();
    assert(motors.ready());
    assert(modes[6] == INPUT_PULLUP && interruptHandlers[6]);
    assert(!interruptHandlers[16] && !interruptHandlers[17]);
    for (int p : {6, 7, 12, 13, 16, 17, 18, 19, 20, 23}) assert(modes[p] != OUTPUT);
    for (const auto& op : pinOperations) {
        assert(op.pin >= 0 && op.pin <= 5 && op.value == 0);
    }
    const int physical[3][4] = {
        {MOTOR0_FI_PIN, MOTOR0_BI_PIN, ENC0_A_PIN, ENC0_B_PIN},
        {MOTOR1_FI_PIN, MOTOR1_BI_PIN, ENC1_A_PIN, ENC1_B_PIN},
        {MOTOR2_FI_PIN, MOTOR2_BI_PIN, ENC2_A_PIN, ENC2_B_PIN},
    };
    std::cout << "{\"physical_order\":[";
    for (unsigned i = 0; i < 3; ++i) {
        if (i) std::cout << ',';
        std::cout << '[' << physical[i][0] << ',' << physical[i][1] << ',' << physical[i][2] << ',' << physical[i][3] << ']';
    }
    std::cout << "],\"diagnostic_order\":[";
    for (unsigned i = 0; i < 3; ++i) {
        const auto& p = r3::headers[i];
        if (i) std::cout << ',';
        std::cout << '[' << int(p.fi) << ',' << int(p.bi) << ',' << int(p.encA) << ',' << int(p.encB) << ']';
    }
    std::cout << "],\"watchdog_heartbeat_gpio\":" << r3::watchdogHeartbeat << "}\n";
}
