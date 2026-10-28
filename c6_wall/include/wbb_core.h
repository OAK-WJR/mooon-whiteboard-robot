// Hardware-independent control core for the C6 whiteboard robot.
// Compiles on the ESP32-C6 and on a desktop (tests/sim_test.cpp).
//
// Design: every wheel is a POSITION servo that tracks a reference trajectory
// generated from one body reference pose. Because all three wheels follow the
// same kinematic reference, the chassis moves straight and stops exactly where
// the reference stops. The integral term is never cleared by stopping, so it
// keeps the torque that holds the robot against gravity on a vertical board.
#pragma once
#include <cmath>
#include <cstdint>

namespace wbb {

constexpr float kMetersPerCount = 0.11938052f / 2800.f;  // 38 mm wheel, 700 CPR x4
// Wheel contact radius (centre to wheel). Runtime-settable. Camera-measured
// 2026-10-26: 50.4 mm, and negative (positive wheel rotation turns it clockwise).
// The inherited +33.7 mm was wrong in both size and sign.
inline float gRobotRadius = -0.0504f;
// Negative when positive wheel rotation turns the chassis clockwise (measured on this
// robot: camera saw -20 deg for +30 deg of encoder yaw, i.e. radius -50.9 mm).
inline bool validRadius(float r) { return std::isfinite(r) && std::fabs(r) >= 0.02f && std::fabs(r) <= 0.12f; }
constexpr float kPi = 3.14159265f;

inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline float wrap(float a) {
    while (a > kPi) a -= 2 * kPi;
    while (a < -kPi) a += 2 * kPi;
    return a;
}

struct Pose { float x = 0, y = 0, yaw = 0; };
struct Twist { float x = 0, y = 0, w = 0; };

// Body twist (m/s, rad/s) -> wheel counts/s. Also maps displacements to counts.
// Body frame: x forward (W key), y left (A key), yaw counter-clockwise.
inline void toWheels(const Twist& v, float out[3]) {
    out[0] = (-0.866025404f * v.x + 0.5f * v.y + gRobotRadius * v.w) / kMetersPerCount;
    out[1] = (-v.y + gRobotRadius * v.w) / kMetersPerCount;
    out[2] = (0.866025404f * v.x + 0.5f * v.y + gRobotRadius * v.w) / kMetersPerCount;
}
inline Twist fromWheels(const float c[3]) {
    return {(c[2] - c[0]) * kMetersPerCount / 1.732050808f,
            (c[0] - 2 * c[1] + c[2]) * kMetersPerCount / 3,
            (c[0] + c[1] + c[2]) * kMetersPerCount / (3 * gRobotRadius)};
}
inline void rotate(float angle, float x, float y, float& ox, float& oy) {
    const float c = std::cos(angle), s = std::sin(angle);
    ox = c * x - s * y;
    oy = s * x + c * y;
}
// SE(2) exponential: exact for a constant body twist over the step.
inline void integrate(Pose& p, const Twist& d) {
    const float a = std::fabs(d.w) < 1e-4f ? 1 - d.w * d.w / 6 : std::sin(d.w) / d.w;
    const float b = std::fabs(d.w) < 1e-4f ? d.w / 2 : (1 - std::cos(d.w)) / d.w;
    float wx, wy;
    rotate(p.yaw, a * d.x - b * d.y, b * d.x + a * d.y, wx, wy);
    p.x += wx;
    p.y += wy;
    p.yaw = wrap(p.yaw + d.w);
}

// ---------------------------------------------------------------- wheel servo
struct ServoGains {
    // Measured 2026-10-26: 0.008 cut start-up yaw from 1.3 to 0.2 deg; 0.012 oscillated
    // full-scale on the real gearboxes (backlash), although the simulation was stable.
    float kp = 0.008f;    // duty per count of position error
    float ki = 0.02f;     // duty per count*second
    // 0.0004 made a lightly loaded wheel chatter inside its gearbox backlash
    // (60 output reversals in 2 s holds); 0.00015 gave none, start-up yaw unchanged.
    float kd = 0.00015f;  // duty per count/s of velocity error
    float imax = 0.75f;   // load/gravity memory limit (duty)
    float umax = 1.0f;
};
// Per-wheel feed-forward, index 0 forward, 1 reverse. duty = kv*|v| + ks.
struct WheelFF {
    float kv[2] = {8.5e-5f, 8.5e-5f};
    float ks[2] = {0.06f, 0.06f};
};
inline bool validGains(const ServoGains& g) {
    return std::isfinite(g.kp) && std::isfinite(g.ki) && std::isfinite(g.kd) && std::isfinite(g.imax) &&
           g.kp >= 0 && g.kp <= 0.05f && g.ki >= 0 && g.ki <= 0.2f && g.kd >= 0 && g.kd <= 0.005f &&
           g.imax >= 0 && g.imax <= 1 && g.umax > 0.1f && g.umax <= 1;
}
inline bool validFF(const WheelFF& f) {
    for (int d = 0; d < 2; d++)
        if (!std::isfinite(f.kv[d]) || !std::isfinite(f.ks[d]) || f.kv[d] < 1e-5f || f.kv[d] > 5e-4f ||
            f.ks[d] < 0 || f.ks[d] > 0.5f)
            return false;
    return true;
}

struct WheelServo {
    float vref = 0;  // reference velocity, counts/s
    float integ = 0, u = 0, err = 0;
    // Breakaway help for a sticky gearbox: while the reference stands still and the wheel is
    // stuck with an error, push harder quickly; drop it the moment the wheel moves, so the
    // stored push does not throw the wheel past its target (measured 2026-10-27: wheel 2 needed
    // 0.30 duty to break free, the others 0.08-0.15, and it hung 1.6 s then jumped 19 counts).
    float boost = 0;
    // Each breakaway raises the error needed for the next one (x1.6): big errors still get pushed
    // home, but a wheel that jumps +-3.5 counts past the target is left alone instead of being
    // thrown back and forth forever (limit cycle seen in simulation and as shaking on the robot).
    float boostThreshold = 2.5f;
    static constexpr float kBoostRate = 1.2f;   // duty per second while stuck
    static constexpr float kBoostMax = 0.45f;
    void reset() { vref = 0; err = 0; u = 0; boost = 0; boostThreshold = 2.5f; }
    // error: this wheel's share of the pose error, in counts.
    // load: this wheel's share of the world-frame load memory (gravity), duty.
    // kiShare: fraction of g.ki integrated per wheel (the rest lives in the world frame).
    float update(float error, float velocity, float dt, const ServoGains& g, const WheelFF& f, float load = 0,
                 float kiShare = 1) {
        err = error;
        const int d = vref >= 0 ? 0 : 1;
        const float ff = f.kv[d] * vref + f.ks[d] * clampf(vref / 400.f, -1.f, 1.f);
        const float pd = g.kp * err + g.kd * (vref - velocity);
        const float pre = ff + pd + integ + load + boost;
        const bool windup = (pre > g.umax && err > 0) || (pre < -g.umax && err < 0);
        // A sub-count dead zone at standstill stops stiction hunting.
        const bool settled = std::fabs(vref) < 1.f && std::fabs(err) < 1.5f;
        if (!windup && !settled) integ = clampf(integ + kiShare * g.ki * err * dt, -g.imax, g.imax);
        const bool still = std::fabs(vref) < 1.f;
        const bool stuck = std::fabs(velocity) < 25.f && std::fabs(err) > boostThreshold;
        if (std::fabs(boost) < 0.005f) boost = 0;  // halving never reaches zero by itself
        if (!still) boostThreshold = 2.5f;           // the reference moves again: fresh start for the next stop
        if (std::fabs(velocity) > 60.f) {            // broke free: drop it at once
            if (std::fabs(boost) > 0.05f) boostThreshold *= 1.6f;
            boost = 0;
        }
        if (still && stuck && !windup && boost * err >= 0)
            boost = clampf(boost + std::copysign(kBoostRate * dt, err), -kBoostMax, kBoostMax);
        else
            boost *= 0.5f;  // settled or overshot: gone within a few ticks
        u = clampf(ff + pd + integ + load + boost, -g.umax, g.umax);
        return u;
    }
};

// ---------------------------------------------------------------- path runner
struct Segment {
    // 0: straight line to (x, y) keeping yaw, 1: rotate in place to yaw,
    // 2: straight line to (x, y) while turning by yaw (signed, may exceed one turn) in proportion
    //    to the distance travelled (spin-erase: the wheels sweep the board as it moves).
    uint8_t kind;
    float x, y, yaw;
};
constexpr int kMaxSegments = 520;

struct PathRunner {
    Segment seg[kMaxSegments];
    int count = 0, index = 0;
    bool running = false, pauseRequested = false;
    float speed = 0.08f, accel = 0.25f, yawRate = 0.8f, yawAccel = 2.0f;
    float s = 0, v = 0, length = 0;  // progress on the current segment (m or rad)
    float done = 0, total = 0;       // translation progress over the whole path (m)
    Pose start;

    void clear() { count = index = 0; running = pauseRequested = false; s = v = length = done = total = 0; }
    bool add(const Segment& g) {
        if (count >= kMaxSegments) return false;
        seg[count++] = g;
        return true;
    }
    void begin(const Pose& ref) {
        index = 0; s = v = 0; done = 0; running = count > 0; pauseRequested = false;
        total = 0;
        Pose p = ref;
        for (int i = 0; i < count; i++) {
            if (seg[i].kind != 1) { total += std::hypot(seg[i].x - p.x, seg[i].y - p.y); p.x = seg[i].x; p.y = seg[i].y; }
            if (seg[i].kind == 1) p.yaw = seg[i].yaw;
            if (seg[i].kind == 2) p.yaw = wrap(p.yaw + seg[i].yaw);
        }
        startSegment(ref);
    }
    bool paused() const { return pauseRequested && v == 0; }
    void startSegment(const Pose& ref) {
        start = ref; s = 0;
        if (index >= count) { running = false; v = 0; return; }
        const Segment& g = seg[index];
        length = g.kind != 1 ? std::hypot(g.x - ref.x, g.y - ref.y) : std::fabs(wrap(g.yaw - ref.yaw));
    }
    // Advances the reference pose by effective time dte (already sync-scaled).
    void step(Pose& ref, float dte) {
        if (!running) return;
        while (index < count && length < 1e-5f) { finishSegment(ref); if (!running) return; }
        const Segment& g = seg[index];
        const bool line = g.kind != 1;
        float vmax = pauseRequested ? 0.f : (line ? speed : yawRate), a = line ? accel : yawAccel;
        if (g.kind == 2 && std::fabs(g.yaw) > 1e-6f && length > 1e-5f) {
            // Turning rate = |turn| / length * v: keep it within the rotation limits.
            const float k = std::fabs(g.yaw) / length;  // rad per m
            vmax = std::fmin(vmax, yawRate / k);
            a = std::fmin(a, yawAccel / k);
        }
        const float remaining = length - s;
        const float cap = std::fmin(vmax, std::sqrt(2 * a * remaining));
        if (v < cap) v = std::fmin(cap, v + a * dte);
        else v = std::fmax(cap, v - a * dte);
        float ds = v * dte;
        if (ds >= remaining || remaining - ds < 2e-6f) ds = remaining;
        s += ds;
        if (line) {
            const float f = s / length;
            ref.x = start.x + (g.x - start.x) * f;
            ref.y = start.y + (g.y - start.y) * f;
            ref.yaw = g.kind == 2 ? wrap(start.yaw + g.yaw * f) : start.yaw;
            done += ds;
        } else {
            ref.yaw = wrap(start.yaw + std::copysign(s, wrap(g.yaw - start.yaw)));
        }
        if (s >= length) finishSegment(ref);
    }
    void finishSegment(Pose& ref) {
        const Segment& g = seg[index];
        if (g.kind == 0) { ref.x = g.x; ref.y = g.y; }
        else if (g.kind == 1) ref.yaw = wrap(g.yaw);
        else { ref.x = g.x; ref.y = g.y; ref.yaw = wrap(start.yaw + g.yaw); }
        v = 0;
        index++;
        startSegment(ref);
    }
};

// ---------------------------------------------------------------- coverage plan
struct CoverageConfig {
    float width = 0.60f, height = 0.40f;  // travel range of the robot centre from the origin (m)
    float eraser = 0.08f, overlap = 0.25f;
    float speed = 0.08f;
    uint8_t vertical = 0;  // 0: horizontal lanes, 1: vertical lanes
    int8_t sx = 1, sy = 1; // +1 right/up from the origin, -1 left/down
};
inline bool validCoverage(const CoverageConfig& c) {
    return std::isfinite(c.width) && std::isfinite(c.height) && std::isfinite(c.eraser) && std::isfinite(c.overlap) &&
           std::isfinite(c.speed) && c.width >= 0 && c.width <= 3 && c.height >= 0 && c.height <= 3 &&
           c.eraser >= 0.01f && c.eraser <= 0.5f && c.overlap >= 0 && c.overlap <= 0.8f && c.speed >= 0.02f &&
           c.speed <= 0.3f && c.vertical <= 1 && (c.sx == 1 || c.sx == -1) && (c.sy == 1 || c.sy == -1);
}
// Boustrophedon waypoints in the board frame (X right, Y up). Starts after the
// origin and ends back at the origin. Returns the number of points, -1 on error.
inline int planCoverage(const CoverageConfig& c, float xs[], float ys[], int max) {
    if (!validCoverage(c)) return -1;
    const float along = c.vertical ? c.height : c.width, across = c.vertical ? c.width : c.height;
    const float pitch = c.eraser * (1 - c.overlap);
    const int lanes = across < 1e-4f ? 1 : int(std::ceil(across / pitch - 1e-4f)) + 1;
    const float spacing = lanes > 1 ? across / (lanes - 1) : 0;
    int n = 0;
    auto put = [&](float a, float b) {
        if (n >= max) return false;
        float x = c.vertical ? b : a, y = c.vertical ? a : b;
        xs[n] = c.sx * x; ys[n] = c.sy * y; n++;
        return true;
    };
    float at = 0;  // current position along the lane
    for (int k = 0; k < lanes; k++) {
        const float b = k * spacing;
        if (k > 0 && !put(at, b)) return -1;  // step across to the next lane
        at = (k % 2 == 0) ? along : 0;
        if (along > 1e-4f && !put(at, b)) return -1;
    }
    // Back to the origin along measured directions only: across the lane, then down
    // the start edge. A long diagonal mixes slip modes and returned 3.4 cm off.
    if (at > 1e-4f && !put(0, (lanes - 1) * spacing)) return -1;
    if (!put(0, 0)) return -1;
    return n;
}

// ---------------------------------------------------------------- motion
enum class Mode : uint8_t { Off = 0, Active, Calibrate, Fault };
enum class Source : uint8_t { None = 0, Manual, Path };

struct Limits {
    float speed = 0.25f;      // m/s hard cap for manual commands
    float accel = 0.35f;      // m/s^2
    float yawRate = 1.2f;     // rad/s
    float yawAccel = 4.0f;    // rad/s^2
    // Pose lag, normalised: translation/1 mm or yaw/0.5 deg, whichever is worse.
    float syncSoft = 2.5f;    // all wheels start slowing together
    float syncHard = 8.f;     // the reference waits for the chassis
    float runawayM = 0.12f;   // pose error that means the chassis is driven away: fault
    float runawayYaw = 0.8f;
    uint32_t leaseMs = 450;   // manual command lifetime; expiry = smooth stop + hold
};

// Manual slip model for a vertical board. Wheel encoders cannot see slip, so the
// user measures it once and the odometry is corrected: the servo then turns the
// wheels further until the CORRECTED pose reaches the reference.
// Ratios are actual chassis travel / wheel-measured travel, in the board frame.
struct SlipComp {
    float up = 1, down = 1, side = 1;
    float sagPerM = 0;   // m the chassis sinks per m of sideways travel (either direction)
    float yawPerM = 0;   // rad the chassis turns (CCW +) per m travelled to the RIGHT; moving left turns it back
    float upSide = 0;    // m the chassis drifts RIGHT per m travelled up
    float downSide = 0;  // m the chassis drifts RIGHT per m travelled down
    float mount = kPi / 2;  // board angle of the world x axis (set by the firmware)
    // Appended 2026-10-27 (the NVS blob is append-only): left/right and up/down asymmetry.
    float sideLeft = 0;  // extra ratio for LEFTward travel (left ratio = side + sideLeft)
    float sagLeft = 0;   // extra m sunk per m travelled LEFT
    float yawLeft = 0;   // extra rad (CCW +) per m travelled LEFT
    float yawUp = 0;     // rad (CCW +) per m travelled UP
    float yawDown = 0;   // rad (CCW +) per m travelled DOWN
    bool neutral() const {
        return up == 1 && down == 1 && side == 1 && sagPerM == 0 && yawPerM == 0 && upSide == 0 && downSide == 0 &&
               sideLeft == 0 && sagLeft == 0 && yawLeft == 0 && yawUp == 0 && yawDown == 0;
    }
};
inline bool validSlip(const SlipComp& c) {
    auto ratio = [](float r) { return std::isfinite(r) && r >= 0.5f && r <= 1.5f; };
    return ratio(c.up) && ratio(c.down) && ratio(c.side) && std::isfinite(c.sagPerM) && std::isfinite(c.yawPerM) &&
           std::fabs(c.sagPerM) <= 0.3f && std::fabs(c.yawPerM) <= 0.6f && std::isfinite(c.upSide) &&
           std::isfinite(c.downSide) && std::fabs(c.upSide) <= 0.2f && std::fabs(c.downSide) <= 0.2f &&
           std::isfinite(c.sideLeft) && std::fabs(c.sideLeft) <= 0.2f && std::isfinite(c.sagLeft) && std::fabs(c.sagLeft) <= 0.2f &&
           std::isfinite(c.yawLeft) && std::fabs(c.yawLeft) <= 0.6f && std::isfinite(c.yawUp) && std::fabs(c.yawUp) <= 0.6f &&
           std::isfinite(c.yawDown) && std::fabs(c.yawDown) <= 0.6f;
}

struct Motion {
    Mode mode = Mode::Off;
    Source source = Source::None;
    const char* fault = "";
    int faultMotor = 0;
    WheelServo wheel[3];
    ServoGains gains;
    WheelFF ff[3];
    Limits lim;
    SlipComp slip;
    Pose ref, odom;
    Twist vel;                 // world-frame reference velocity
    Twist manual;              // world-frame manual target velocity
    uint32_t manualMs = 0;
    float sync = 1, stuckTime = 0;
    bool stuck = false;
    PathRunner path;
    // Gravity memory in the WORLD frame (duty-equivalent force, x/y). Gravity does not turn
    // when the robot turns, so this stays valid while spinning; it is projected on the wheels
    // through the current heading. The per-wheel integrals only keep wheel-specific friction.
    float loadX = 0, loadY = 0;
    float kiWheelShare = 0.2f;  // fraction of gains.ki integrated per wheel
    float yawNormSpin = 0.035f; // rad of heading lag per unit of lag while spin-erasing (2 deg)

    // External (camera) pose fusion. The camera sees the true chassis, wheels do not see
    // slip. A fix measured at time t is compared with the odometry recorded at t and the
    // difference is blended into the current odometry; the servo then removes it.
    struct Stamp { uint32_t ms = 0; Pose p; };
    static constexpr int kHistory = 128;  // 1.28 s at 100 Hz
    Stamp history[kHistory];
    int historyHead = 0;
    uint32_t fixes = 0, fixRejects = 0;
    Pose lastFixDelta;
    void record(uint32_t now) {
        history[historyHead] = {now, odom};
        historyHead = (historyHead + 1) % kHistory;
    }
    // Returns false when the fix is too old or implausible (left unapplied).
    bool fuse(const Pose& cam, uint32_t camMs, uint32_t now, float gain = 0.35f, float yawGain = 0.35f) {
        if (uint32_t(now - camMs) > 1000) { fixRejects++; return false; }
        const Stamp* best = nullptr;
        for (const auto& h : history)
            if (h.ms && (!best || std::labs(int32_t(h.ms - camMs)) < std::labs(int32_t(best->ms - camMs)))) best = &h;
        if (!best || std::labs(int32_t(best->ms - camMs)) > 30) { fixRejects++; return false; }
        const Pose d{cam.x - best->p.x, cam.y - best->p.y, wrap(cam.yaw - best->p.yaw)};
        if (std::hypot(d.x, d.y) > 0.05f || std::fabs(d.yaw) > 0.175f) { fixRejects++; return false; }
        odom.x += gain * d.x;
        odom.y += gain * d.y;
        odom.yaw = wrap(odom.yaw + yawGain * d.yaw);
        // Keep the stored history consistent with the corrected odometry.
        for (auto& h : history) { h.p.x += gain * d.x; h.p.y += gain * d.y; h.p.yaw = wrap(h.p.yaw + yawGain * d.yaw); }
        lastFixDelta = d;
        fixes++;
        return true;
    }

    void enable() {
        if (mode == Mode::Fault) { for (auto& w : wheel) w.integ = 0; loadX = loadY = 0; }
        for (auto& w : wheel) w.reset();
        ref = odom;  // hold exactly where the robot is now
        vel = manual = {};
        source = Source::None;
        path.clear();
        sync = 1; stuckTime = 0; stuck = false;
        fault = ""; faultMotor = 0;
        mode = Mode::Active;
    }
    void release() {
        mode = Mode::Off; source = Source::None; path.clear();
        vel = manual = {};
        for (auto& w : wheel) { w.u = 0; w.vref = 0; }
    }
    void setFault(const char* why, int motor = 0) {
        if (mode == Mode::Fault) return;
        release();
        mode = Mode::Fault; fault = why; faultMotor = motor;
    }
    // Stops any motion smoothly and holds position.
    void hold() {
        if (source == Source::Path) path.pauseRequested = true;
        manual = {};
        if (source == Source::Manual) manualMs = 0;
    }
    bool moving() const {
        return std::fabs(vel.x) + std::fabs(vel.y) + std::fabs(vel.w) > 1e-5f ||
               (source == Source::Path && !path.paused());
    }
    bool driveWorld(uint32_t now, float vx, float vy, float w) {
        if (mode != Mode::Active || source == Source::Path) return false;
        if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(w)) return false;
        const float n = std::hypot(vx, vy);
        if (n > lim.speed) { vx *= lim.speed / n; vy *= lim.speed / n; }
        manual = {vx, vy, clampf(w, -lim.yawRate, lim.yawRate)};
        manualMs = now;
        source = Source::Manual;
        return true;
    }
    bool startPath() {
        if (mode != Mode::Active || path.count == 0) return false;
        manual = vel = {};
        path.begin(ref);
        source = Source::Path;
        stuckTime = 0; stuck = false;
        return true;
    }
    bool resumePath() {
        if (mode != Mode::Active || source != Source::Path) return false;
        path.pauseRequested = false; stuckTime = 0; stuck = false;
        return true;
    }
    void observe(const float delta[3]) {
        const Twist d = fromWheels(delta);
        if (slip.neutral()) { integrate(odom, d); return; }
        float wx, wy, X, Y;
        rotate(odom.yaw + 0.5f * d.w, d.x, d.y, wx, wy);  // body -> world (midpoint heading)
        rotate(slip.mount, wx, wy, X, Y);                  // world -> board
        const float right = X, side = std::fabs(X), vertical = Y;
        const bool left = X < 0;
        X = X * (slip.side + (left ? slip.sideLeft : 0.f)) + (vertical > 0 ? slip.upSide * vertical : -slip.downSide * vertical);
        Y = Y * (Y > 0 ? slip.up : slip.down) - (slip.sagPerM + (left ? slip.sagLeft : 0.f)) * side;
        rotate(-slip.mount, X, Y, wx, wy);
        odom.x += wx;
        odom.y += wy;
        odom.yaw = wrap(odom.yaw + d.w + slip.yawPerM * right + (left ? slip.yawLeft * side : 0.f) +
                        (vertical > 0 ? slip.yawUp * vertical : -slip.yawDown * vertical));
    }

    // Re-bases the world frame on the current odometry pose. Keeps any pose error.
    void setOrigin() {
        float x, y;
        rotate(-odom.yaw, ref.x - odom.x, ref.y - odom.y, x, y);
        ref = {x, y, wrap(ref.yaw - odom.yaw)};
        odom = {};
    }
    float yawNorm = 0.00873f;  // rad of heading lag that counts like 1 mm of position lag
    bool spinning() const {
        return source == Source::Path && path.running && path.index < path.count && path.seg[path.index].kind == 2;
    }
    float lagMetric() const {
        return std::fmax(std::hypot(ref.x - odom.x, ref.y - odom.y) / 0.001f,
                         std::fabs(wrap(ref.yaw - odom.yaw)) / (spinning() ? yawNormSpin : yawNorm));
    }

    // One control period, after observe(). velocity: filtered wheel counts/s.
    // The wheels servo the POSE error (reference - odometry) mapped into wheel
    // space through the actual heading, so a heading error is corrected instead
    // of silently bending the path.
    void tick(uint32_t now, float dt, const float velocity[3], float u[3]) {
        for (int i = 0; i < 3; i++) u[i] = 0;
        if (mode != Mode::Active) return;
        // A stalled loop (flash write, Wi-Fi burst) must not create a large reference jump.
        dt = clampf(dt, 0.001f, 0.03f);

        sync = clampf((lim.syncHard - lagMetric()) / (lim.syncHard - lim.syncSoft), 0.f, 1.f);
        const float dte = dt * sync;

        const Pose before = ref;
        if (source == Source::Manual) {
            const bool fresh = manualMs && uint32_t(now - manualMs) <= lim.leaseMs;
            const Twist target = fresh ? manual : Twist{};
            float dx = target.x - vel.x, dy = target.y - vel.y;
            const float dv = std::hypot(dx, dy), maxDv = lim.accel * dt;
            if (dv > maxDv) { dx *= maxDv / dv; dy *= maxDv / dv; }
            vel.x += dx; vel.y += dy;
            vel.w += clampf(target.w - vel.w, -lim.yawAccel * dt, lim.yawAccel * dt);
            ref.x += vel.x * dte; ref.y += vel.y * dte; ref.yaw = wrap(ref.yaw + vel.w * dte);
            if (!fresh && std::fabs(vel.x) + std::fabs(vel.y) + std::fabs(vel.w) < 1e-6f) {
                vel = {}; source = Source::None;
            }
        } else if (source == Source::Path) {
            path.step(ref, dte);
            if (!path.running) source = Source::None;
        }
        // Velocity actually being commanded this period (feed-forward).
        const Twist v = {(ref.x - before.x) / dt, (ref.y - before.y) / dt, wrap(ref.yaw - before.yaw) / dt};
        if (source != Source::Manual) vel = source == Source::None ? Twist{} : v;

        Twist e, vb;
        e.w = wrap(ref.yaw - odom.yaw);
        rotate(-odom.yaw, ref.x - odom.x, ref.y - odom.y, e.x, e.y);
        vb.w = v.w;
        rotate(-odom.yaw, v.x, v.y, vb.x, vb.y);
        float ec[3], vc[3];
        toWheels(e, ec);
        toWheels(vb, vc);
        // World-frame load memory: integrated only while spin-erasing (gravity turns in the body
        // frame then); otherwise every wheel integrates its own error at the full rate, which also
        // learns its own friction (with only 20 % per wheel, wheel 1 hunted around zero duty while
        // translating: 2-3 Hz chatter, measured 2026-10-27). The stored world load keeps acting.
        const bool spin = spinning();
        const float share = spin ? kiWheelShare : 1.f;
        {
            const float ex = (ref.x - odom.x) / kMetersPerCount, ey = (ref.y - odom.y) / kMetersPerCount;
            const bool settled = !moving() && std::hypot(ex, ey) < 1.5f;
            bool sat = false;
            for (int i = 0; i < 3; i++) sat = sat || std::fabs(wheel[i].u) >= gains.umax * 0.999f;
            if (spin && !settled && !sat) {
                const float k = (1 - kiWheelShare) * gains.ki * dt;
                loadX += k * ex;
                loadY += k * ey;
                const float n = std::hypot(loadX, loadY);
                if (n > gains.imax) { loadX *= gains.imax / n; loadY *= gains.imax / n; }
            }
        }
        float lb[2];
        rotate(-odom.yaw, loadX, loadY, lb[0], lb[1]);
        const float load[3] = {-0.866025404f * lb[0] + 0.5f * lb[1], -lb[1], 0.866025404f * lb[0] + 0.5f * lb[1]};
        for (int i = 0; i < 3; i++) {
            wheel[i].vref = vc[i];
            u[i] = wheel[i].update(ec[i], velocity[i], dt, gains, ff[i], load[i], share);
        }
        if (std::hypot(ref.x - odom.x, ref.y - odom.y) > lim.runawayM || std::fabs(e.w) > lim.runawayYaw) {
            int worst = 0;
            for (int i = 1; i < 3; i++) if (std::fabs(ec[i]) > std::fabs(ec[worst])) worst = i;
            setFault("RUNAWAY", worst + 1);
            for (int k = 0; k < 3; k++) u[k] = 0;
            return;
        }
        // A wheel that cannot follow (blocked, overloaded) pauses travel; the robot keeps holding.
        stuckTime = (source != Source::None && sync < 0.05f) ? stuckTime + dt : 0;
        if (stuckTime > 2.5f && !stuck) { stuck = true; hold(); }
    }
};

// ---------------------------------------------------------------- calibration
// Suspended-wheel identification of the feed-forward: duty = kv*|v| + ks.
struct Calibrator {
    static constexpr int kSteps = 8;
    static constexpr float kDuty[kSteps] = {0.18f, 0.28f, 0.40f, 0.55f, -0.18f, -0.28f, -0.40f, -0.55f};
    static constexpr float kStepTime = 0.7f, kSettle = 0.35f;
    int wheel = 0, step = 0, samples = 0;
    float t = 0, sum = 0;
    float speed[3][kSteps] = {};
    bool active = false, done = false;
    const char* error = "";
    int errorMotor = 0;

    void begin() { wheel = step = samples = 0; t = sum = 0; active = true; done = false; error = ""; errorMotor = 0; }
    void tick(float dt, const float velocity[3], float u[3]) {
        for (int i = 0; i < 3; i++) u[i] = 0;
        if (!active) return;
        t += dt;
        if (t > kSettle) { sum += velocity[wheel]; samples++; }
        if (t >= kStepTime) {
            speed[wheel][step] = samples ? sum / samples : 0;
            t = sum = 0; samples = 0;
            if (++step >= kSteps) { step = 0; if (++wheel >= 3) { active = false; done = true; return; } }
        }
        u[wheel] = kDuty[step];
    }
    // Fits each wheel and direction. Fails on missing or inverted feedback.
    bool fit(WheelFF out[3]) {
        for (int i = 0; i < 3; i++) {
            for (int d = 0; d < 2; d++) {
                float sx = 0, sy = 0, sxx = 0, sxy = 0; int n = 0;
                for (int k = 0; k < 4; k++) {
                    const float duty = kDuty[d * 4 + k], v = speed[i][d * 4 + k];
                    if (v * duty < 0 && std::fabs(v) > 150) { error = "POLARITY"; errorMotor = i + 1; return false; }
                    if (std::fabs(v) < 150) continue;
                    const float x = std::fabs(v), y = std::fabs(duty);
                    sx += x; sy += y; sxx += x * x; sxy += x * y; n++;
                }
                if (n < 2) { error = "NO_MOTION"; errorMotor = i + 1; return false; }
                const float den = n * sxx - sx * sx;
                if (den <= 0) { error = "FIT"; errorMotor = i + 1; return false; }
                const float kv = (n * sxy - sx * sy) / den, ks = (sy - kv * sx) / n;
                out[i].kv[d] = clampf(kv, 1.2e-5f, 4.5e-4f);
                out[i].ks[d] = clampf(ks, 0.f, 0.45f);
            }
        }
        return true;
    }
};

}  // namespace wbb
