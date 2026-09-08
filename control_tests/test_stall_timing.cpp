// Actual controller/driver, synthetic stationary encoder and 32-bit millis clock.
#include <cassert>
#include <cstdint>
#include <iostream>
#include <string>
#include "../src/kinematics.cpp"
#include "../src/pid_controller.cpp"
#include "../src/path_planner.cpp"
#include "../src/trajectory_follower.cpp"
#include "../src/odometry.cpp"
#include "../src/motor_driver.cpp"
#include "../src/robot_controller.cpp"

static constexpr int pins[] = {MOTOR0_FI_PIN, MOTOR0_BI_PIN, MOTOR1_FI_PIN,
                               MOTOR1_BI_PIN, MOTOR2_FI_PIN, MOTOR2_BI_PIN};
static void fresh(RobotController& robot, uint32_t start) {
    failAttachPin = failWritePin = failDetachPin = -1;
    levels.fill(LOW); duties.fill(0); attached.fill(false);
    pinOperations.clear(); Serial.tx.clear(); fakeNow = start;
    robot.begin();
}
static int pwm(RobotController& robot, int wheel) {
    JsonDocument status; robot.fillStatus(status);
    return status["pwm_command"][wheel].as<int>();
}
static int reachStall(RobotController& robot, int wheel) {
    robot.setWheelTest(wheel, MAX_WHEEL_VEL);
    bool running = false;
    for (int tick = 1; tick <= 1000; ++tick) {
        fakeNow += CONTROL_PERIOD_MS;
        robot.update(float(CONTROL_PERIOD_MS) / 1000.0f);
        int value = pwm(robot, wheel);
        if (running && value == 0) {
            assert(Serial.tx.find("堵转!") != std::string::npos);
            return tick;
        }
        running |= value > 0;
    }
    assert(false && "stationary motor did not reach stall protection");
    return 0;
}
static void stopped() { for (int pin : pins) assert(duties[pin] == 0); }

int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "normal";
    uint32_t tripAt = 10000;
    if (mode == "wrap") tripAt = UINT32_MAX - uint32_t(STALL_COOLDOWN_MS / 2);
    else if (mode == "zero-deadline") tripAt = uint32_t(0) - uint32_t(STALL_COOLDOWN_MS);
    else if (mode != "normal") return 2;
    for (int wheel = 0; wheel < 3; ++wheel) {
        RobotController measured;
        fresh(measured, 0);
        int ticks = reachStall(measured, wheel);
        RobotController robot;
        fresh(robot, tripAt - uint32_t(ticks * CONTROL_PERIOD_MS));
        assert(reachStall(robot, wheel) == ticks);
        assert(fakeNow == tripAt);
        stopped();
        for (uint32_t elapsed : {1U, uint32_t(STALL_COOLDOWN_MS / 2),
                                 uint32_t(STALL_COOLDOWN_MS - 1)}) {
            fakeNow = tripAt + elapsed;
            robot.update(float(CONTROL_PERIOD_MS) / 1000.0f);
            stopped();
        }
        fakeNow = tripAt + uint32_t(STALL_COOLDOWN_MS);
        robot.update(float(CONTROL_PERIOD_MS) / 1000.0f);
        assert(pwm(robot, wheel) > 0);
        assert(duties[pins[wheel * 2]] > 0);
        for (int other = 0; other < 3; ++other) if (other != wheel) {
            assert(duties[pins[other * 2]] == 0);
            assert(duties[pins[other * 2 + 1]] == 0);
        }
        JsonDocument stop; stop["cmd"] = "stop";
        robot.handleCommand(stop); stopped();
        fakeNow += uint32_t(STALL_COOLDOWN_MS * 2);
        robot.update(float(CONTROL_PERIOD_MS) / 1000.0f);
        stopped();
    }
    std::cout << "Three stall cooldowns " << mode << ", retry boundary and explicit stop PASS\n";
}
