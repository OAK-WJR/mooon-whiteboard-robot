// Executes production encoder ISRs and odometry with synthetic GPIO edges.
// Boundary seeds avoid waiting billions of pulses; GPIO/interrupt timing is mocked.
#include <cassert>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <type_traits>
#include "../src/kinematics.cpp"
#include "../src/odometry.cpp"

static constexpr int aPins[] = {ENC0_A_PIN, ENC1_A_PIN, ENC2_A_PIN};
static constexpr int bPins[] = {ENC0_B_PIN, ENC1_B_PIN, ENC2_B_PIN};
static constexpr int directions[] = {ENCODER_DIR_M0, ENCODER_DIR_M1, ENCODER_DIR_M2};
static int phase[3]{};
using Counter = std::remove_cv_t<std::remove_reference_t<decltype(enc_counts_[0])>>;

static void fresh(Odometry& odometry) {
    levels.fill(LOW);
    interruptHandlers.fill(nullptr);
    for (int& p : phase) p = 0;
    odometry.begin();
}

static void steps(int wheel, int direction, int count) {
    // Existing raw encoder polarity differs for the middle decoder.
    static constexpr int positive[2][4] = {{0, 2, 3, 1}, {0, 1, 3, 2}};
    for (int n = 0; n < count; ++n) {
        int old = positive[wheel == 1][phase[wheel]];
        phase[wheel] = (phase[wheel] + (direction > 0 ? 1 : 3)) % 4;
        int next = positive[wheel == 1][phase[wheel]];
        levels[aPins[wheel]] = (next >> 1) & 1;
        levels[bPins[wheel]] = next & 1;
        int changed = (old ^ next) == 2 ? aPins[wheel] : bPins[wheel];
        assert(interruptHandlers[changed]);
        interruptHandlers[changed]();
    }
}

// Independent, wide-integer oracle for the public signed 32-bit count window.
static long reported(int64_t value) {
    int64_t result = value % 4294967296LL;
    if (result < 0) result += 4294967296LL;
    if (result > 2147483647LL) result -= 4294967296LL;
    return static_cast<long>(result);
}

static void near(float actual, float expected) {
    if (!std::isfinite(actual) || std::fabs(actual - expected) > 1e-6f) {
        std::cerr << "actual=" << actual << " expected=" << expected << '\n';
        std::abort();
    }
}

int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "boundaries";
    Kinematics kin(WHEEL_RADIUS, ROBOT_RADIUS);
    Odometry odometry(kin, ENCODER_CPR);
    fresh(odometry);
    if (mode == "overflow") {
        for (int wheel = 0; wheel < 3; ++wheel) {
            fresh(odometry);
            enc_counts_[wheel] = std::numeric_limits<Counter>::max();
            steps(wheel, 1, 1);
            assert(enc_counts_[wheel] == 0);
            steps(wheel, -1, 1);
            assert(enc_counts_[wheel] == std::numeric_limits<Counter>::max());
        }
        std::cout << "All three ISR counters wrap in both directions PASS\n";
    } else if (mode == "directions") {
        const uint32_t seeds[] = {0, 1, 0x7fffffffU, 0x80000000U, 0xfffffffeU, 0xffffffffU};
        for (uint32_t seed : seeds) {
            for (int wheel = 0; wheel < 3; ++wheel) enc_counts_[wheel] = seed;
            volatile long counts[3];
            odometry.getEncoderCounts(counts);
            for (int wheel = 0; wheel < 3; ++wheel)
                assert(counts[wheel] == reported(int64_t(seed) * directions[wheel]));
        }
        std::cout << "18 direction-corrected signed telemetry boundaries PASS\n";
    } else if (mode == "invalid-dt") {
        for (float dt : {0.0f, -0.02f, std::numeric_limits<float>::quiet_NaN(),
                         std::numeric_limits<float>::infinity()}) {
            fresh(odometry);
            steps(0, 1, 4);
            odometry.update(dt);
            near(odometry.getPose().x, 0);
            near(odometry.getPose().y, 0);
            near(odometry.getPose().theta, 0);
            for (float v : odometry.getWheelVelocity().v) near(v, 0);
            odometry.update(0.02f);
            float expected = 0.3f * directions[0] * 4.0f *
                (2.0f * float(M_PI) * WHEEL_RADIUS) / (float(ENCODER_CPR) * 4.0f) / 0.02f;
            near(odometry.getWheelVelocity().v[0], expected);
        }
        std::cout << "Invalid time intervals leave state and encoder baseline intact PASS\n";
    } else if (mode == "boundaries") {
        const uint32_t seeds[] = {4, 0x7ffffffcU, 0x80000004U, 0xfffffffcU};
        int cases = 0;
        for (int wheel = 0; wheel < 3; ++wheel) {
            for (uint32_t seed : seeds) {
                for (int direction : {-1, 1}) {
                    fresh(odometry);
                    enc_counts_[wheel] = seed;
                    odometry.update(0.02f);
                    float previous = odometry.getWheelVelocity().v[wheel];
                    steps(wheel, direction, 8);
                    odometry.update(0.02f);
                    float alpha = wheel == 1 ? 0.15f : 0.3f;
                    float raw = direction * directions[wheel] * 8.0f *
                        (2.0f * float(M_PI) * WHEEL_RADIUS) / (float(ENCODER_CPR) * 4.0f) / 0.02f;
                    near(odometry.getWheelVelocity().v[wheel], alpha * raw + (1.0f - alpha) * previous);
                    volatile long counts[3];
                    odometry.getEncoderCounts(counts);
                    assert(counts[wheel] == reported((int64_t(seed) + direction * 8) * directions[wheel]));
                    assert(std::isfinite(odometry.getPose().x));
                    assert(std::isfinite(odometry.getPose().y));
                    assert(std::isfinite(odometry.getPose().theta));
                    ++cases;
                }
            }
        }
        // Exercise ordinary operation through registered interrupt handlers too.
        fresh(odometry);
        for (int tick = 0; tick < 10000; ++tick) {
            for (int wheel = 0; wheel < 3; ++wheel) steps(wheel, 1, 4);
            odometry.update(0.02f);
        }
        volatile long counts[3];
        odometry.getEncoderCounts(counts);
        for (int wheel = 0; wheel < 3; ++wheel) {
            assert(counts[wheel] == 40000 * directions[wheel]);
            float speed = directions[wheel] * (2.0f * float(M_PI) * WHEEL_RADIUS) /
                          float(ENCODER_CPR) / 0.02f;
            near(odometry.getWheelVelocity().v[wheel], speed);
        }
        odometry.reset();
        odometry.getEncoderCounts(counts);
        for (int wheel = 0; wheel < 3; ++wheel) {
            assert(counts[wheel] == 0);
            near(odometry.getWheelVelocity().v[wheel], 0);
        }
        std::cout << cases << " rollover integrations, 120000 GPIO edges and reset PASS\n";
    } else {
        return 2;
    }
}
