// Desktop closed-loop tests for wbb_core.h with a crude gear-motor plant.
// Build: c++ -std=c++17 -O2 -Iinclude tests/sim_test.cpp -o /tmp/sim && /tmp/sim
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "wbb_core.h"

using namespace wbb;

static int failures = 0;
#define CHECK(cond, ...)                                     \
    do {                                                     \
        if (!(cond)) {                                       \
            std::printf("  FAIL %s:%d: ", __FILE__, __LINE__); \
            std::printf(__VA_ARGS__);                        \
            std::printf("\n");                               \
            failures++;                                      \
        }                                                    \
    } while (0)

// Slow-decay drive: steady speed = K * (duty + load - friction). Zero duty = shorted motor.
struct MotorPlant {
    float K = 11000, tau = 0.08f, stiction = 0.06f, coulomb = 0.04f, v = 0;
    double pos = 0;
    int sign = 1;  // -1 simulates a reversed encoder
    void step(float u, float load, float dt) {
        const float drive = u + load;
        if (std::fabs(v) < 30 && std::fabs(drive) < stiction) { v = 0; return; }
        const float dir = std::fabs(v) > 1 ? (v > 0 ? 1.f : -1.f) : (drive > 0 ? 1.f : -1.f);
        const float target = K * (drive - coulomb * dir);
        v += (target - v) * dt / tau;
        pos += v * dt;
    }
};

struct Rig {
    MotorPlant m[3];
    Motion motion;
    Pose truth;           // chassis pose from actual wheel motion (no slip)
    float gravity = 0;    // duty-equivalent wheel load at full alignment
    float boardYaw = kPi / 2;  // body x (at origin) points up the board
    SlipComp realSlip;         // what the board really does to the chassis
    float velocityFilter[3] = {};
    double lastPos[3] = {};
    uint32_t now = 0;
    float maxErr = 0, maxLag = 0;
    void loads(float out[3]) {
        // Gravity is -Y on the board. Express it in the body frame.
        float wx, wy, bx, by;
        rotate(-boardYaw, 0, -1, wx, wy);
        rotate(-truth.yaw, wx, wy, bx, by);
        const float n[3][2] = {{-0.866025f, 0.5f}, {0, -1}, {0.866025f, 0.5f}};
        for (int i = 0; i < 3; i++) out[i] = gravity * (2.f / 3.f) * (n[i][0] * bx + n[i][1] * by) * 1.5f;
    }
    void run(float seconds, bool calibrating = false, Calibrator* cal = nullptr) {
        const float dt = 0.01f;
        for (int k = 0; k < int(seconds / dt + 0.5f); k++) {
            now += 10;
            int32_t counts[3];
            float delta[3];
            for (int i = 0; i < 3; i++) {
                counts[i] = int32_t(std::floor(m[i].pos)) * m[i].sign;
                delta[i] = float(m[i].pos - lastPos[i]) * m[i].sign;
                lastPos[i] = m[i].pos;
                velocityFilter[i] += 0.4f * (delta[i] / dt - velocityFilter[i]);
            }
            motion.observe(delta);
            motion.record(now);
            float u[3];
            (void)counts;
            if (calibrating) cal->tick(dt, velocityFilter, u);
            else motion.tick(now, dt, velocityFilter, u);
            for (int i = 0; i < 3; i++) maxErr = std::fmax(maxErr, std::fabs(motion.wheel[i].err));
            maxLag = std::fmax(maxLag, motion.mode == Mode::Active ? motion.lagMetric() : 0.f);
            float load[3];
            loads(load);
            double before[3] = {m[0].pos, m[1].pos, m[2].pos};
            for (int i = 0; i < 3; i++) m[i].step(u[i], load[i], dt);
            float d[3] = {float(m[0].pos - before[0]), float(m[1].pos - before[1]), float(m[2].pos - before[2])};
            if (realSlip.neutral()) {
                integrate(truth, fromWheels(d));
            } else {
                const Twist b = fromWheels(d);
                float wx, wy, X, Y;
                rotate(truth.yaw + 0.5f * b.w, b.x, b.y, wx, wy);
                rotate(boardYaw, wx, wy, X, Y);
                const float right = X, side = std::fabs(X), vertical = Y;
                X = X * realSlip.side + (vertical > 0 ? realSlip.upSide * vertical : -realSlip.downSide * vertical);
                Y = Y * (Y > 0 ? realSlip.up : realSlip.down) - realSlip.sagPerM * side;
                rotate(-boardYaw, X, Y, wx, wy);
                truth.x += wx; truth.y += wy; truth.yaw = wrap(truth.yaw + b.w + realSlip.yawPerM * right);
            }
        }
    }
    void enable() { motion.enable(); }
    // Board-frame manual command, like the web page in "board" mode.
    void drive(float bx, float by, float w = 0) {
        float wx, wy;
        rotate(-boardYaw, bx, by, wx, wy);
        motion.driveWorld(now, wx, wy, w);
    }
    void boardPose(float& X, float& Y) const { rotate(boardYaw, truth.x, truth.y, X, Y); }
};

static void testHoldOnWall() {
    std::puts("hold on wall");
    for (float tau : {0.05f, 0.12f, 0.3f}) {
        Rig r;
        r.gravity = 0.32f;
        for (auto& m : r.m) m.tau = tau;
        r.enable();
        r.run(6);
        float X, Y;
        r.boardPose(X, Y);
        std::printf("  tau=%.2f sag=%.2f mm drift x=%.2f mm max wheel err=%.0f counts\n", tau, Y * 1000, X * 1000, r.maxErr);
        CHECK(r.motion.mode == Mode::Active, "mode %d fault %s", int(r.motion.mode), r.motion.fault);
        CHECK(std::fabs(Y) < 0.012f, "sag %.4f", Y);
        // The final position must stop creeping.
        float Y0 = Y;
        r.run(4);
        r.boardPose(X, Y);
        CHECK(std::fabs(Y - Y0) < 0.0005f, "creep %.5f", Y - Y0);
    }
}

static void testStraightDriveUp() {
    std::puts("straight drive up the wall, mismatched motors");
    Rig r;
    r.gravity = 0.3f;
    r.m[0].K = 13500; r.m[1].K = 7500; r.m[1].tau = 0.15f; r.m[2].stiction = 0.12f;
    r.enable();
    r.run(1);
    float X0, Y0;
    r.boardPose(X0, Y0);
    for (int k = 0; k < 200; k++) { r.drive(0, 0.15f); r.run(0.01f); }
    r.drive(0, 0);  // key released
    r.run(1.5f);
    float X, Y;
    r.boardPose(X, Y);
    std::printf("  travelled %.1f mm, sideways %.2f mm, yaw %.2f deg, sync min seen? source=%d\n",
                (Y - Y0) * 1000, (X - X0) * 1000, r.truth.yaw * 57.3f, int(r.motion.source));
    CHECK(r.motion.mode == Mode::Active, "fault %s", r.motion.fault);
    CHECK(std::fabs((Y - Y0) - 0.30f) < 0.035f, "travel %.4f", Y - Y0);
    CHECK(std::fabs(X - X0) < 0.003f, "sideways %.4f", X - X0);
    CHECK(std::fabs(r.truth.yaw) < 0.03f, "yaw %.4f", r.truth.yaw);
    CHECK(r.motion.source == Source::None, "still moving");
}

static void testOverspeedSync() {
    std::puts("command faster than the weakest motor can go");
    Rig r;
    r.gravity = 0.3f;
    r.m[1].K = 5000;
    r.enable();
    r.motion.lim.speed = 0.4f;
    float minSync = 1;
    for (int k = 0; k < 300; k++) { r.drive(0.4f, 0); r.run(0.01f); minSync = std::fmin(minSync, r.motion.sync); }
    r.run(2);
    float X, Y;
    r.boardPose(X, Y);
    std::printf("  x %.1f mm, y %.2f mm, yaw %.2f deg, min sync %.2f, max err %.0f\n", X * 1000, Y * 1000,
                r.truth.yaw * 57.3f, minSync, r.maxErr);
    CHECK(r.motion.mode == Mode::Active, "fault %s", r.motion.fault);
    CHECK(minSync < 0.9f, "sync never engaged");
    CHECK(std::fabs(Y) < 0.012f, "lateral %.4f", Y);
    CHECK(std::fabs(r.truth.yaw) < 0.05f, "yaw %.4f", r.truth.yaw);
}

static void testCoverageReturnsHome() {
    std::puts("coverage plan on the wall and return to origin");
    Rig r;
    r.gravity = 0.3f;
    r.m[1].K = 9000;
    r.enable();
    CoverageConfig c;
    c.width = 0.5f; c.height = 0.3f; c.eraser = 0.07f; c.overlap = 0.2f; c.speed = 0.1f;
    float xs[600], ys[600];
    const int n = planCoverage(c, xs, ys, 600);
    CHECK(n > 4, "plan %d", n);
    CHECK(std::fabs(xs[n - 1]) < 1e-6f && std::fabs(ys[n - 1]) < 1e-6f, "plan must end at origin");
    r.motion.path.clear();
    for (int i = 0; i < n; i++) {
        float wx, wy;
        rotate(-r.boardYaw, xs[i], ys[i], wx, wy);
        r.motion.path.add({0, wx, wy, 0});
    }
    r.motion.path.speed = c.speed;
    CHECK(r.motion.startPath(), "start");
    float t = 0, maxLane = 0;
    while (r.motion.source == Source::Path && t < 120) {
        r.run(0.05f);
        t += 0.05f;
        float X, Y;
        r.boardPose(X, Y);
        maxLane = std::fmax(maxLane, X);
    }
    r.run(1);
    float X, Y;
    r.boardPose(X, Y);
    std::printf("  %d waypoints, %.0f s, %.2f m, reached x=%.3f, home error %.2f mm, yaw %.2f deg\n", n, t,
                r.motion.path.total, maxLane, std::hypot(X, Y) * 1000, r.truth.yaw * 57.3f);
    CHECK(r.motion.mode == Mode::Active, "fault %s", r.motion.fault);
    CHECK(r.motion.source == Source::None, "path not finished");
    CHECK(std::hypot(X, Y) < 0.003f, "home error %.4f", std::hypot(X, Y));
    CHECK(std::fabs(maxLane - 0.5f) < 0.01f, "lane length %.3f", maxLane);
}

static void testPauseResume() {
    std::puts("pause and resume a path");
    Rig r;
    r.gravity = 0.3f;
    r.enable();
    r.motion.path.clear();
    r.motion.path.add({0, 0, -0.3f, 0});  // board right when the robot points up
    r.motion.path.speed = 0.1f;
    r.motion.startPath();
    r.run(1.0f);
    r.motion.hold();
    r.run(1.0f);
    float X1, Y1;
    r.boardPose(X1, Y1);
    r.run(1.0f);
    float X2, Y2;
    r.boardPose(X2, Y2);
    CHECK(r.motion.path.paused(), "not paused");
    CHECK(std::fabs(X2 - X1) < 0.0005f, "moved while paused %.5f", X2 - X1);
    r.motion.resumePath();
    r.run(5);
    float X, Y;
    r.boardPose(X, Y);
    std::printf("  paused at %.1f mm, finished at %.1f mm\n", X1 * 1000, X * 1000);
    CHECK(std::fabs(X - 0.3f) < 0.002f, "end %.4f", X);
    CHECK(r.motion.source == Source::None, "not done");
}

static void testRunawayFaults() {
    std::puts("reversed encoder is caught");
    Rig r;
    r.m[2].sign = -1;
    r.enable();
    r.drive(0.1f, 0);
    float t = 0;
    while (r.motion.mode == Mode::Active && t < 5) { r.drive(0.1f, 0); r.run(0.01f); t += 0.01f; }
    std::printf("  fault %s motor %d after %.2f s\n", r.motion.fault, r.motion.faultMotor, t);
    CHECK(r.motion.mode == Mode::Fault, "no fault");
}

static void testBlockedWheelPauses() {
    std::puts("blocked wheel pauses travel instead of falling");
    Rig r;
    r.gravity = 0.3f;
    r.enable();
    r.m[0].stiction = 5;  // jammed
    float t = 0;
    while (t < 4) { r.drive(0.1f, 0); r.run(0.01f); t += 0.01f; }
    std::printf("  stuck=%d sync=%.2f mode=%d err0=%.0f\n", r.motion.stuck, r.motion.sync, int(r.motion.mode),
                r.motion.wheel[0].err);
    CHECK(r.motion.mode == Mode::Active, "fault %s", r.motion.fault);
    CHECK(r.motion.stuck, "stuck not reported");
}

static void testCalibration() {
    std::puts("feed-forward calibration on suspended wheels");
    Rig r;
    r.m[0].K = 12000; r.m[1].K = 8000; r.m[2].K = 10000;
    Calibrator cal;
    cal.begin();
    r.run(20, true, &cal);
    CHECK(cal.done, "not done");
    WheelFF ff[3];
    CHECK(cal.fit(ff), "fit failed %s", cal.error);
    for (int i = 0; i < 3; i++) {
        std::printf("  M%d kv %.2e/%.2e (true %.2e) ks %.3f/%.3f\n", i + 1, ff[i].kv[0], ff[i].kv[1], 1 / r.m[i].K,
                    ff[i].ks[0], ff[i].ks[1]);
        CHECK(std::fabs(ff[i].kv[0] * r.m[i].K - 1) < 0.1f, "kv");
    }
    Rig bad;
    bad.m[1].sign = -1;
    Calibrator c2;
    c2.begin();
    bad.run(20, true, &c2);
    WheelFF f2[3];
    CHECK(!c2.fit(f2) && c2.errorMotor == 2, "reversed encoder not reported");
}

static void testPlanner() {
    std::puts("planner geometry");
    CoverageConfig c;
    c.width = 1.0f; c.height = 0.5f; c.eraser = 0.1f; c.overlap = 0;
    float xs[100], ys[100];
    int n = planCoverage(c, xs, ys, 100);
    // 6 lanes at y=0,.1,..,.5: 6 lane runs + 5 steps + home
    CHECK(n == 12, "n=%d", n);  // even lane count ends at the start edge: straight down home
    CHECK(std::fabs(xs[0] - 1) < 1e-6f && ys[0] == 0, "first");
    CHECK(std::fabs(ys[n - 2] - 0.5f) < 1e-6f, "last lane");
    c.sx = -1; c.sy = -1; c.vertical = 1;
    n = planCoverage(c, xs, ys, 100);
    CHECK(n > 0 && ys[0] < 0 && xs[0] == 0, "vertical mirrored");
    c.eraser = 0;
    CHECK(planCoverage(c, xs, ys, 100) == -1, "invalid accepted");
    c.eraser = 0.01f; c.overlap = 0; c.width = 3; c.height = 3;
    CHECK(planCoverage(c, xs, ys, 100) == -1, "overflow accepted");
}

static void runSquare(Rig& r, float& upErr, float& sideSag, float& homeErr) {
    r.enable();
    r.run(0.5f);
    auto& p = r.motion.path;
    float pts[4][2] = {{0, 0.3f}, {0.4f, 0.3f}, {0.4f, 0}, {0, 0}};  // up, right, down, left (board)
    p.clear();
    for (auto& q : pts) { float wx, wy; rotate(-r.boardYaw, q[0], q[1], wx, wy); p.add({0, wx, wy, 0}); }
    p.speed = 0.1f;
    r.motion.startPath();
    float X, Y;
    // Measure at each corner (the chassis is momentarily stopped there).
    while (r.motion.path.index < 1) r.run(0.01f);
    r.boardPose(X, Y);
    upErr = Y - 0.3f;
    while (r.motion.path.index < 2) r.run(0.01f);
    r.boardPose(X, Y);
    sideSag = Y - 0.3f;
    while (r.motion.source == Source::Path) r.run(0.05f);
    r.run(0.5f);
    r.boardPose(X, Y);
    homeErr = std::hypot(X, Y);
}

static void testSlipCompensation() {
    std::puts("manual slip compensation");
    SlipComp board;
    board.up = 0.9f; board.down = 1.08f; board.side = 0.95f; board.sagPerM = 0.03f; board.yawPerM = 0.05f;
    board.upSide = -0.04f; board.downSide = 0.017f;
    float up0, sag0, home0, up1, sag1, home1;
    {
        Rig r; r.gravity = 0.3f; r.realSlip = board;
        runSquare(r, up0, sag0, home0);
    }
    {
        Rig r; r.gravity = 0.3f; r.realSlip = board;
        r.motion.slip = board; r.motion.slip.mount = r.boardYaw;
        runSquare(r, up1, sag1, home1);
        CHECK(r.motion.mode == Mode::Active, "fault %s", r.motion.fault);
    }
    std::printf("  without: up %.1f mm, sag after 40 cm sideways %.1f mm, home %.1f mm\n", up0 * 1000, sag0 * 1000, home0 * 1000);
    std::printf("  with:    up %.1f mm, sag after 40 cm sideways %.1f mm, home %.1f mm\n", up1 * 1000, sag1 * 1000, home1 * 1000);
    CHECK(std::fabs(up1) < 0.002f && std::fabs(sag1) < 0.002f && home1 < 0.003f, "compensation ineffective");
    CHECK(std::fabs(up0) > 0.02f, "uncompensated case should be visibly wrong");
}

static void testCameraFusion() {
    std::puts("camera fixes (10 Hz, 120 ms late) correct unmodelled slip");
    for (bool camera : {false, true}) {
        Rig r;
        r.gravity = 0.3f;
        r.realSlip.up = 0.88f; r.realSlip.down = 1.08f; r.realSlip.side = 0.97f;
        r.realSlip.sagPerM = 0.11f; r.realSlip.yawPerM = -0.18f; r.realSlip.upSide = -0.03f;
        r.enable();
        CoverageConfig c;
        c.width = 0.38f; c.height = 0.70f; c.eraser = 0.08f; c.overlap = 0.25f; c.speed = 0.08f;
        float xs[600], ys[600];
        const int n = planCoverage(c, xs, ys, 600);
        r.motion.path.clear();
        for (int i = 0; i < n; i++) { float wx, wy; rotate(-r.boardYaw, xs[i], ys[i], wx, wy); r.motion.path.add({0, wx, wy, 0}); }
        r.motion.path.speed = c.speed;
        r.motion.startPath();
        std::vector<std::pair<uint32_t, Pose>> seen;  // camera samples waiting to arrive
        float worst = 0, t = 0;
        while (r.motion.source == Source::Path && t < 200) {
            r.run(0.01f);
            t += 0.01f;
            if (camera && r.now % 100 == 0) seen.push_back({r.now, r.truth});
            while (!seen.empty() && r.now - seen.front().first >= 120) {
                r.motion.fuse(seen.front().second, seen.front().first, r.now);
                seen.erase(seen.begin());
            }
            // Distance between where the plan wants the robot and where it really is.
            worst = std::fmax(worst, std::hypot(r.truth.x - r.motion.ref.x, r.truth.y - r.motion.ref.y));
        }
        r.run(1.5f);
        float X, Y;
        r.boardPose(X, Y);
        std::printf("  camera %s: home error %.1f mm, worst deviation from plan %.1f mm, fixes %u rejected %u\n",
                    camera ? "on " : "off", std::hypot(X, Y) * 1000, worst * 1000, r.motion.fixes, r.motion.fixRejects);
        if (camera) {
            CHECK(std::hypot(X, Y) < 0.006f, "home %.4f", std::hypot(X, Y));
            CHECK(worst < 0.02f, "deviation %.4f", worst);
            CHECK(r.motion.mode == Mode::Active, "fault %s", r.motion.fault);
        }
    }
}

// Spin-erase on the wall: gravity rotates in the body frame while the robot turns.
static float spinOnWall(float rate, float speed, float* jitter, float* worstLag, float* sagMm) {
    Rig r;
    r.gravity = 0.32f;
    r.enable();
    r.run(2);
    float X0, Y0;
    r.boardPose(X0, Y0);
    const float yaw0 = r.truth.yaw;
    float wx, wy;
    rotate(-r.boardYaw, 0.20f, 0, wx, wy);
    r.motion.path.clear();
    r.motion.path.add({2, r.motion.ref.x + wx, r.motion.ref.y + wy, 8 * 2 * kPi});
    r.motion.path.speed = speed; r.motion.path.accel = 0.05f;
    r.motion.path.yawRate = rate; r.motion.path.yawAccel = 3.f;
    r.motion.startPath();
    float minSync = 1, worst = 0, maxDev = 0;
    int reversals = 0;
    float lastU[3] = {}, lastDu[3] = {};
    for (int k = 0; k < 20000 && r.motion.source == Source::Path; k++) {
        r.run(0.01f);
        minSync = std::fmin(minSync, r.motion.sync);
        worst = std::fmax(worst, r.motion.lagMetric());
        float X, Y;
        r.boardPose(X, Y);
        maxDev = std::fmax(maxDev, std::fabs(Y - Y0));
        for (int i = 0; i < 3; i++) {
            const float du = r.motion.wheel[i].u - lastU[i];
            if (du * lastDu[i] < 0 && std::fabs(du) > 0.03f) reversals++;
            if (std::fabs(du) > 1e-4f) lastDu[i] = du;
            lastU[i] = r.motion.wheel[i].u;
        }
    }
    r.run(1.5f);
    float X, Y;
    r.boardPose(X, Y);
    *jitter = float(reversals);
    *worstLag = worst;
    *sagMm = maxDev * 1000;
    std::printf("  rate %.1f rad/s: min sync %.2f, worst lag %.1f, duty reversals %d, max sag while spinning %.2f mm, end %.2f/%.2f mm yaw %.2f deg\n",
                rate, minSync, worst, reversals, maxDev * 1000, (X - X0 - 0.20f) * 1000, (Y - Y0) * 1000, wrap(r.truth.yaw - yaw0) * 57.3f);
    return minSync;
}

// One wheel with a sticky gearbox: settle time and overshoot after small moves.
static void testStickyWheel() {
    std::puts("sticky wheel (breakaway 0.30 duty on M2): small moves on the wall");
    Rig r;
    r.gravity = 0.25f;
    r.m[1].stiction = 0.30f;
    r.enable();
    r.run(2);
    float worstSettle = 0, worstOver = 0, worstFinal = 0;
    for (auto [bx, by] : {std::pair<float, float>{0.03f, 0}, {-0.03f, 0}, {0, 0.03f}, {0, -0.03f}}) {
        float wx, wy;
        rotate(-r.boardYaw, bx, by, wx, wy);
        r.motion.path.clear();
        r.motion.path.add({0, r.motion.ref.x + wx, r.motion.ref.y + wy, 0});
        r.motion.path.speed = 0.04f; r.motion.path.accel = 0.15f;
        r.motion.startPath();
        int k = 0;
        for (; k < 1500 && r.motion.source == Source::Path; k++) r.run(0.01f);
        // after the reference stops: time until every wheel is within 2 counts for good
        float settle = 0, over = 0;
        int lastBad = 0;
        for (int j = 0; j < 400; j++) {
            r.run(0.01f);
            for (int i = 0; i < 3; i++) {
                if (std::fabs(r.motion.wheel[i].err) > 4) lastBad = j;
                over = std::fmax(over, std::fabs(r.motion.wheel[i].err));
            }
        }
        settle = (lastBad + 1) * 0.01f;
        float fe = 0;
        for (int i = 0; i < 3; i++) fe = std::fmax(fe, std::fabs(r.motion.wheel[i].err));
        std::printf("    move (%+.2f,%+.2f): settle %.2f s, peak %.0f, final errs %.1f %.1f %.1f, u2 %+.2f boost2 %+.2f\n", bx, by, settle, over,
                    r.motion.wheel[0].err, r.motion.wheel[1].err, r.motion.wheel[2].err, r.motion.wheel[1].u, r.motion.wheel[1].boost);
        worstSettle = std::fmax(worstSettle, settle);
        worstOver = std::fmax(worstOver, over);
        worstFinal = std::fmax(worstFinal, fe);
    }
    std::printf("  worst settle %.2f s, worst error after the stop %.0f counts, worst final error %.0f counts\n", worstSettle, worstOver, worstFinal);
    CHECK(worstSettle < 4.0f, "settles in %.2f s", worstSettle);
    CHECK(worstFinal <= 4.f, "final error %.0f counts", worstFinal);
}

static void testSpinOnWall() {
    std::puts("spin-erase on the vertical wall (8 turns over 20 cm)");
    for (float rate : {1.2f, 2.5f}) {
        float jit, lag, sag;
        const float ms = spinOnWall(rate, rate * 0.20f / (8 * 2 * kPi), &jit, &lag, &sag);
        CHECK(ms > 0.5f, "reference stalls (min sync %.2f)", ms);
        CHECK(sag < 3.f, "sags %.2f mm while spinning", sag);
    }
}

static void testSpinErase() {
    std::puts("spin-erase: 20 cm across while turning 2 turns, then back turning -2 turns");
    Rig r;
    r.gravity = 0.3f;
    r.enable();
    r.run(1);
    float X0, Y0;
    r.boardPose(X0, Y0);
    const float yaw0 = r.truth.yaw;
    auto spin = [&](float bx, float by, float turns) {
        float wx, wy;
        rotate(-r.boardYaw, bx, by, wx, wy);
        r.motion.path.clear();
        r.motion.path.add({2, r.motion.ref.x + wx, r.motion.ref.y + wy, turns * 2 * kPi});
        r.motion.path.speed = 0.05f; r.motion.path.accel = 0.1f;
        r.motion.path.yawRate = 2.5f; r.motion.path.yawAccel = 4.f;
        CHECK(r.motion.startPath(), "start");
        float maxW = 0, lastYaw = r.motion.ref.yaw;
        for (int k = 0; k < 4000 && r.motion.source == Source::Path; k++) {
            r.run(0.01f);
            maxW = std::fmax(maxW, std::fabs(wrap(r.motion.ref.yaw - lastYaw)) / 0.01f);
            lastYaw = r.motion.ref.yaw;
        }
        r.run(1.0f);
        return maxW;
    };
    const float w1 = spin(0.20f, 0, 2.f);
    float X, Y;
    r.boardPose(X, Y);
    std::printf("  out: x %.2f mm  y %.2f mm  yaw %.2f deg  (max ref turn rate %.2f rad/s)\n", (X - X0) * 1000, (Y - Y0) * 1000,
                wrap(r.truth.yaw - yaw0) * 57.3f, w1);
    CHECK(std::fabs((X - X0) - 0.20f) < 0.003f, "out travel %.4f", X - X0);
    CHECK(w1 <= 2.5f * 1.05f, "turn rate %.2f", w1);
    spin(-0.20f, 0, -2.f);
    r.boardPose(X, Y);
    std::printf("  back: error %.2f mm, %.2f mm, yaw %.2f deg\n", (X - X0) * 1000, (Y - Y0) * 1000, wrap(r.truth.yaw - yaw0) * 57.3f);
    CHECK(r.motion.mode == Mode::Active, "fault %s", r.motion.fault);
    CHECK(std::hypot(X - X0, Y - Y0) < 0.002f, "return error %.4f", std::hypot(X - X0, Y - Y0));
    CHECK(std::fabs(wrap(r.truth.yaw - yaw0)) < 0.01f, "return yaw %.4f", r.truth.yaw - yaw0);
}

int main() {
    testStickyWheel();
    testSpinOnWall();
    testSpinErase();
    testPlanner();
    testHoldOnWall();
    testStraightDriveUp();
    testOverspeedSync();
    testCoverageReturnsHome();
    testPauseResume();
    testRunawayFaults();
    testBlockedWheelPauses();
    testCalibration();
    testSlipCompensation();
    testCameraFusion();
    std::printf(failures ? "\n%d FAILED\n" : "\nALL PASSED\n", failures);
    return failures ? 1 : 0;
}
