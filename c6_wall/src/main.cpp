// WhiteBoardBot wall firmware: Waveshare ESP32-C6-LCD-1.47 + adapter + RZ7889 base.
// Physical wheel order 1,2,3 (header order M0,M2,M1). Control core: include/wbb_core.h.
//
//  * Wheels are position servos: stopping = holding, never coasting.
//  * RZ7889 FI=BI=H is brake. All idle/fault/reboot states brake.
//  * Web page + WebSocket control on port 80; firmware updates over Wi-Fi (POST /ota)
//    with automatic rollback if the new image cannot come back online.
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Adafruit_ST7789.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <SPI.h>
#include <WiFi.h>
#include <driver/gpio.h>
#include <esp_app_desc.h>
#include <esp_http_server.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <soc/gpio_reg.h>
#include "build_info.h"
#include "network_config.h"
#include "wbb_core.h"
#include "web_page.h"

#ifndef WBB_OTA_TOKEN
#error "WBB_OTA_TOKEN comes from include/build_info.h (tools/prebuild.py)"
#endif

using wbb::Mode;
using wbb::Source;

// ------------------------------------------------------------------ hardware
struct Pins { uint8_t fi, bi, a, b; int sign; };
static constexpr Pins kPins[3] = {{0, 1, 16, 17, 1}, {4, 5, 20, 23, 1}, {2, 3, 18, 19, -1}};
static constexpr uint8_t kButton = 9, kBacklight = 22;
static constexpr uint32_t kPwmHz = 20000;
static constexpr uint8_t kPwmBits = 10;
static constexpr uint32_t kFull = 1023;  // Arduino maps max duty to 100 % (constant high)

static volatile uint32_t encCount[3], encBad[3];
static volatile uint8_t encPrev[3];
static portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;
static const int8_t kQuad[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

static void IRAM_ATTR encoderIsr(void* arg) {
    const unsigned i = uintptr_t(arg);
    const uint32_t in = REG_READ(GPIO_IN_REG);
    const uint8_t ab = (((in >> kPins[i].a) & 1) << 1) | ((in >> kPins[i].b) & 1);
    portENTER_CRITICAL_ISR(&encMux);
    const uint8_t old = encPrev[i];
    if (ab != old) {
        if ((ab ^ old) == 3) encBad[i]++;
        else encCount[i] += kQuad[(old << 2) | ab];
        encPrev[i] = ab;
    }
    portEXIT_CRITICAL_ISR(&encMux);
}
static void readEncoders(uint32_t c[3], uint32_t b[3]) {
    portENTER_CRITICAL(&encMux);
    for (int i = 0; i < 3; i++) { c[i] = encCount[i]; b[i] = encBad[i]; }
    portEXIT_CRITICAL(&encMux);
}

static bool pwmReady = false;
static void brakePinsGpio() {
    for (auto& p : kPins)
        for (uint8_t pin : {p.fi, p.bi}) {
            gpio_set_level(gpio_num_t(pin), 1);
            gpio_set_direction(gpio_num_t(pin), GPIO_MODE_INPUT_OUTPUT);
        }
}
static void releasePinHold() {
    for (auto& p : kPins) for (uint8_t pin : {p.fi, p.bi}) gpio_hold_dis(gpio_num_t(pin));
}
static void holdPinsForReboot() {
    // Pads keep their level through the software reset, so the motors stay braked.
    for (auto& p : kPins) for (uint8_t pin : {p.fi, p.bi}) gpio_hold_en(gpio_num_t(pin));
}
static bool attachPwm() {
    bool ok = true;
    for (int i = 0; i < 3; i++) {
        const uint8_t pins[2] = {kPins[i].fi, kPins[i].bi};
        for (int k = 0; k < 2; k++) {
            ok = ok && ledcAttachChannel(pins[k], kPwmHz, kPwmBits, 2 * i + k) && ledcWrite(pins[k], kFull);
            gpio_input_enable(gpio_num_t(pins[k]));
        }
    }
    return ok;
}
// Slow decay: the off-phase brakes. u in [-1, 1]; 0 = brake.
static void writeMotor(int i, float u) {
    if (!pwmReady) return;
    u = wbb::clampf(u, -1, 1);
    const uint32_t off = uint32_t(lroundf((1 - fabsf(u)) * kFull));
    ledcWrite(kPins[i].fi, u >= 0 ? kFull : off);
    ledcWrite(kPins[i].bi, u >= 0 ? off : kFull);
}

// ------------------------------------------------------------------ shared state
static SemaphoreHandle_t lock;
struct Locked {
    Locked() { xSemaphoreTake(lock, portMAX_DELAY); }
    ~Locked() { xSemaphoreGive(lock); }
};
static wbb::Motion motion;
static wbb::Calibrator calibrator;
static const char* calibrationResult = "";
static int calibrationMotor = 0;
static float wheelVel[3], wheelU[3];
static int32_t wheelCounts[3];
static uint32_t wheelBad[3];
static uint32_t controlTicks = 0, controlLateMs = 0;
static int mountDeg = 90;  // board direction the robot's front faced when the origin was set
// Body-frame angle of the robot's physical front (CCW from the kinematic +x axis,
// which lies between wheels 1 and 3). The user's "front" can be any side of the triangle.
static int frontDeg = 0;
static wbb::CoverageConfig cover;
static bool pinConflict = false;
static volatile bool otaBusy = false;
static volatile uint32_t rebootAt = 0;
static volatile bool beacon = false;  // LCD camera target: cyan / magenta halves
static bool lcdReady = false;

// Board angle of the world (odometry) x axis.
static float mountYaw() { return (mountDeg - frontDeg) * wbb::kPi / 180.f; }

// Survives software resets (OTA, crash): the robot resumes holding where it was.
struct ResumeState {
    uint32_t magic;
    uint8_t active;
    uint8_t crashes;
    float integ[3];
    wbb::Pose ref, odom;
    float load[2];  // world-frame gravity memory
    uint32_t crc;
};
__NOINIT_ATTR static ResumeState resumeState;  // LP RAM is heap on this build: use HP noinit
static constexpr uint32_t kResumeMagic = 0x57425231;
static uint32_t resumeCrc(const ResumeState& s) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&s);
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < offsetof(ResumeState, crc); i++) h = (h ^ p[i]) * 16777619u;
    return h;
}
static void saveResume() {  // call with lock held
    resumeState.magic = kResumeMagic;
    resumeState.active = motion.mode == Mode::Active;
    for (int i = 0; i < 3; i++) resumeState.integ[i] = motion.wheel[i].integ;
    resumeState.load[0] = motion.loadX;
    resumeState.load[1] = motion.loadY;
    resumeState.ref = motion.ref;
    resumeState.odom = motion.odom;
    resumeState.crc = resumeCrc(resumeState);
}
static bool resumedHold = false;
struct BootInfo { int reason = 0; bool nvs = false, magic = false, crc = false; int active = -1, crashes = -1; wbb::Pose odom; };
static BootInfo bootInfo;
static Preferences prefs, legacyPrefs;
// Planned reboots (OTA, reboot command) also write the record to NVS: the noinit
// RAM address can move between firmware builds, NVS cannot.
static void saveResumeNvs() { prefs.putBytes("resume", &resumeState, sizeof(resumeState)); }
static void restoreResume() {
    const esp_reset_reason_t why = esp_reset_reason();
    ResumeState stored;
    if (prefs.getBytesLength("resume") == sizeof(stored) && prefs.getBytes("resume", &stored, sizeof(stored)) == sizeof(stored)) {
        if (why == ESP_RST_SW) { resumeState = stored; bootInfo.nvs = true; }
        prefs.remove("resume");  // one-shot: never reuse it after a later crash or power cycle
    }
    bootInfo.reason = why;
    bootInfo.magic = resumeState.magic == kResumeMagic;
    bootInfo.crc = resumeState.crc == resumeCrc(resumeState);
    bootInfo.active = resumeState.active;
    bootInfo.crashes = resumeState.crashes;
    bootInfo.odom = resumeState.odom;
    const bool soft = why == ESP_RST_SW || why == ESP_RST_PANIC || why == ESP_RST_INT_WDT ||
                      why == ESP_RST_TASK_WDT || why == ESP_RST_WDT;
    if (!soft || resumeState.magic != kResumeMagic || resumeState.crc != resumeCrc(resumeState)) {
        memset(&resumeState, 0, sizeof(resumeState));
        return;
    }
    // Repeated crashes: stop resuming and stay braked.
    resumeState.crashes = why == ESP_RST_SW ? 0 : resumeState.crashes + 1;
    motion.odom = resumeState.odom;
    if (resumeState.active && resumeState.crashes < 3) {
        motion.enable();
        motion.ref = resumeState.ref;
        for (int i = 0; i < 3; i++) motion.wheel[i].integ = resumeState.integ[i];
        motion.loadX = resumeState.load[0];
        motion.loadY = resumeState.load[1];
        resumedHold = true;
    }
}

// ------------------------------------------------------------------ settings
struct Settings {
    uint32_t version = 3;
    wbb::ServoGains gains;
    wbb::WheelFF ff[3];
    float speed = 0.20f, accel = 0.35f;
    wbb::CoverageConfig cover;
    int16_t mount = 90;
};
static void loadSettings() {
    Settings s;
    if (prefs.getBytesLength("settings") == sizeof(s) && prefs.getBytes("settings", &s, sizeof(s)) == sizeof(s) &&
        s.version == Settings{}.version) {
        if (wbb::validGains(s.gains)) motion.gains = s.gains;
        for (int i = 0; i < 3; i++) if (wbb::validFF(s.ff[i])) motion.ff[i] = s.ff[i];
        if (s.speed >= 0.02f && s.speed <= 0.4f) motion.lim.speed = s.speed;
        if (s.accel >= 0.05f && s.accel <= 2.f) motion.lim.accel = s.accel;
        if (wbb::validCoverage(s.cover)) cover = s.cover;
        if (s.mount % 90 == 0 && s.mount >= 0 && s.mount < 360) mountDeg = s.mount;
    }
}
static void loadFront() {
    const int f = prefs.getInt("front", 0);
    if (f >= 0 && f < 360) frontDeg = f;
    const float r = prefs.getFloat("radius", wbb::gRobotRadius);
    if (wbb::validRadius(r)) wbb::gRobotRadius = r;
}
static void loadSlip() {
    // Older builds stored a shorter struct (fields are only ever appended): read what
    // is there and keep defaults for the rest, instead of silently dropping it all.
    wbb::SlipComp c;
    const size_t stored = prefs.getBytesLength("slip");
    if (stored >= 6 * sizeof(float) && stored <= sizeof(c) && stored % sizeof(float) == 0) {
        wbb::SlipComp tmp;
        uint8_t raw[sizeof(c)];
        if (prefs.getBytes("slip", raw, stored) == stored) {
            // Layouts only ever grow at the end: copy what is stored, keep defaults for the rest.
            memcpy(&tmp, raw, stored);
            if (wbb::validSlip(tmp)) c = tmp;
        }
    }
    motion.slip = c;
    motion.slip.mount = mountYaw();
}
static bool saveSlip() {  // call WITHOUT the lock
    wbb::SlipComp c;
    { Locked l; c = motion.slip; }
    return prefs.putBytes("slip", &c, sizeof(c)) == sizeof(c);
}
static bool saveSettings() {  // flash write: call WITHOUT the lock
    Settings s;
    {
        Locked l;
        s.gains = motion.gains;
        for (int i = 0; i < 3; i++) s.ff[i] = motion.ff[i];
        s.speed = motion.lim.speed;
        s.accel = motion.lim.accel;
        s.cover = cover;
        s.mount = mountDeg;
    }
    return prefs.putBytes("settings", &s, sizeof(s)) == sizeof(s);
}

// ------------------------------------------------------------------ control task
static void controlTask(void*) {
    uint32_t prev[3], bad[3];
    readEncoders(prev, bad);
    uint32_t lastUs = micros();
    bool buttonWas = false;
    int conflictTicks = 0;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
        const uint32_t nowUs = micros(), now = millis();
        const float dt = (nowUs - lastUs) * 1e-6f;
        lastUs = nowUs;
        uint32_t c[3];
        readEncoders(c, bad);
        float delta[3];
        for (int i = 0; i < 3; i++) {
            delta[i] = float(int32_t(c[i] - prev[i])) * kPins[i].sign;
            prev[i] = c[i];
        }
        const bool button = digitalRead(kButton) == LOW;
        float u[3] = {0, 0, 0};
        Mode mode;
        {
            Locked l;
            const float fdt = wbb::clampf(dt, 0.002f, 0.1f);
            for (int i = 0; i < 3; i++) {
                wheelVel[i] += 0.4f * (delta[i] / fdt - wheelVel[i]);
                wheelCounts[i] = int32_t(c[i]) * kPins[i].sign;
                wheelBad[i] = bad[i];
            }
            if (dt > 0.03f) controlLateMs = now;
            motion.observe(delta);
            motion.record(now);
            if (button && !buttonWas) motion.hold();
            if (motion.mode == Mode::Calibrate) {
                calibrator.tick(dt, wheelVel, u);
                if (calibrator.done) {
                    wbb::WheelFF fitted[3];
                    if (calibrator.fit(fitted)) {
                        for (int i = 0; i < 3; i++) motion.ff[i] = fitted[i];
                        calibrationResult = "OK";
                        calibrationMotor = 0;
                    } else {
                        calibrationResult = calibrator.error;
                        calibrationMotor = calibrator.errorMotor;
                    }
                    calibrator.done = false;
                    motion.mode = Mode::Off;
                }
            } else {
                motion.tick(now, dt, wheelVel, u);
            }
            mode = motion.mode;
            for (int i = 0; i < 3; i++) wheelU[i] = u[i];
            if (pinConflict && mode == Mode::Active) motion.setFault("PIN_CONFLICT");
            saveResume();
            controlTicks++;
        }
        buttonWas = button;
        const bool driving = mode == Mode::Active || mode == Mode::Calibrate;
        for (int i = 0; i < 3; i++) writeMotor(i, driving ? u[i] : 0.f);
        // While braking every pin must read high. A low pin means something else
        // (e.g. a TF card on GPIO4/5) is fighting the driver input.
        if (!driving && pwmReady) {
            bool ok = true;
            for (auto& p : kPins) ok = ok && gpio_get_level(gpio_num_t(p.fi)) && gpio_get_level(gpio_num_t(p.bi));
            conflictTicks = ok ? 0 : conflictTicks + 1;
            if (conflictTicks > 5) pinConflict = true;
        }
    }
}

// ------------------------------------------------------------------ network
static drive::NetworkCredentials* network = nullptr;  // heap: 4.6 KB
static char apName[32], apPass[16], hostName[24];
static httpd_handle_t http = nullptr;
static bool otaPendingVerify = false, otaConfirmed = false;
static uint32_t staConnectedSince = 0;

extern "C" bool verifyRollbackLater() { return true; }  // we confirm only once back online

static void confirmOta(const char* why) {
    if (!otaPendingVerify || otaConfirmed) return;
    if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
        otaConfirmed = true;
        Serial.printf("OTA image confirmed (%s)\n", why);
    }
}

static const char* modeName(Mode m) {
    switch (m) {
        case Mode::Active: return "active";
        case Mode::Calibrate: return "calibrate";
        case Mode::Fault: return "fault";
        default: return "off";
    }
}
static const char* sourceName(Source s) {
    return s == Source::Manual ? "manual" : s == Source::Path ? "path" : "none";
}
static String buildId() {
    char sha[9];
    const uint8_t* h = esp_app_get_description()->app_elf_sha256;
    snprintf(sha, sizeof(sha), "%02x%02x%02x%02x", h[0], h[1], h[2], h[3]);
    return String(WBB_BUILD_ID) + "-" + sha;
}

static String telemetry() {
    JsonDocument d;
    d["t"] = "s";
    d["fw"] = buildId();
    d["up"] = millis();
    {
        Locked l;
        d["mode"] = modeName(motion.mode);
        d["src"] = sourceName(motion.source);
        d["fault"] = motion.fault;
        d["fm"] = motion.faultMotor;
        auto pose = d["pose"].to<JsonArray>();
        pose.add(motion.odom.x); pose.add(motion.odom.y); pose.add(motion.odom.yaw);
        auto ref = d["ref"].to<JsonArray>();
        ref.add(motion.ref.x); ref.add(motion.ref.y); ref.add(motion.ref.yaw);
        d["lag"] = motion.lagMetric();
        auto ld = d["load"].to<JsonArray>();
        ld.add(motion.loadX); ld.add(motion.loadY);
        d["sync"] = motion.sync;
        d["stuck"] = motion.stuck;
        d["moving"] = motion.moving();
        auto w = d["w"].to<JsonArray>();
        for (int i = 0; i < 3; i++) {
            auto row = w.add<JsonArray>();
            row.add(lroundf(motion.wheel[i].err)); row.add(wheelU[i]); row.add(lroundf(wheelVel[i]));
            row.add(motion.wheel[i].integ); row.add(wheelCounts[i]); row.add(wheelBad[i]);
        }
        auto p = d["path"].to<JsonObject>();
        p["run"] = motion.source == Source::Path;
        p["paused"] = motion.source == Source::Path && motion.path.paused();
        p["i"] = motion.path.index;
        p["n"] = motion.path.count;
        p["done"] = motion.path.done;
        p["total"] = motion.path.total;
        auto cal = d["cal"].to<JsonObject>();
        cal["active"] = motion.mode == Mode::Calibrate;
        cal["wheel"] = calibrator.wheel + 1;
        cal["result"] = calibrationResult;
        cal["motor"] = calibrationMotor;
        d["pin_conflict"] = pinConflict;
        d["late_ms"] = controlLateMs;
        d["resumed"] = resumedHold;
        auto fx = d["fix"].to<JsonObject>();
        fx["n"] = motion.fixes; fx["rejected"] = motion.fixRejects;
        fx["dx"] = motion.lastFixDelta.x; fx["dy"] = motion.lastFixDelta.y; fx["dyaw"] = motion.lastFixDelta.yaw;
        auto boot = d["boot"].to<JsonObject>();
        boot["reason"] = bootInfo.reason; boot["nvs"] = bootInfo.nvs; boot["magic"] = bootInfo.magic; boot["crc"] = bootInfo.crc;
        boot["active"] = bootInfo.active; boot["crashes"] = bootInfo.crashes;
        auto bo = boot["odom"].to<JsonArray>();
        bo.add(bootInfo.odom.x); bo.add(bootInfo.odom.y); bo.add(bootInfo.odom.yaw);
        d["mount"] = mountDeg;
        d["front"] = frontDeg;
        d["radius_mm"] = wbb::gRobotRadius * 1000;
        d["beacon"] = bool(beacon);
        auto c = d["cover"].to<JsonObject>();
        c["width"] = cover.width; c["height"] = cover.height; c["eraser"] = cover.eraser;
        c["overlap"] = cover.overlap; c["speed"] = cover.speed; c["vertical"] = cover.vertical;
        c["sx"] = cover.sx; c["sy"] = cover.sy;
        auto sl = d["slip"].to<JsonObject>();
        sl["up"] = motion.slip.up; sl["down"] = motion.slip.down; sl["side"] = motion.slip.side;
        sl["sag"] = motion.slip.sagPerM; sl["yaw"] = motion.slip.yawPerM;
        sl["up_side"] = motion.slip.upSide; sl["down_side"] = motion.slip.downSide; sl["on"] = !motion.slip.neutral();
        sl["side_left"] = motion.slip.sideLeft; sl["sag_left"] = motion.slip.sagLeft; sl["yaw_left"] = motion.slip.yawLeft;
        sl["yaw_up"] = motion.slip.yawUp; sl["yaw_down"] = motion.slip.yawDown;
        auto g = d["gains"].to<JsonObject>();
        g["kp"] = motion.gains.kp; g["ki"] = motion.gains.ki; g["kd"] = motion.gains.kd;
        g["imax"] = motion.gains.imax; g["speed"] = motion.lim.speed; g["accel"] = motion.lim.accel;
        auto ff = g["ff"].to<JsonArray>();
        for (auto& f : motion.ff) {
            auto row = ff.add<JsonArray>();
            row.add(f.kv[0] * 1e5f); row.add(f.kv[1] * 1e5f); row.add(f.ks[0]); row.add(f.ks[1]);
        }
    }
    auto n = d["net"].to<JsonObject>();
    const bool sta = WiFi.status() == WL_CONNECTED;
    n["ssid"] = network ? network->ssid : "";
    n["sta"] = sta;
    n["ip"] = sta ? WiFi.localIP().toString() : String("");
    n["rssi"] = sta ? WiFi.RSSI() : 0;
    n["ap"] = apName;
    n["host"] = String(hostName) + ".local";
    d["ota_pending"] = otaPendingVerify && !otaConfirmed;
    d["ota_busy"] = bool(otaBusy);
    String out;
    serializeJson(d, out);
    return out;
}

// Board-frame vector (X right, Y up) -> world frame (odometry frame of the origin).
static void boardToWorld(float bx, float by, float& wx, float& wy) { wbb::rotate(-mountYaw(), bx, by, wx, wy); }

static bool loadCoverage(const wbb::CoverageConfig& c) {  // lock held
    static float xs[wbb::kMaxSegments], ys[wbb::kMaxSegments];
    const int n = wbb::planCoverage(c, xs, ys, wbb::kMaxSegments - 4);
    if (n < 1) return false;
    auto& path = motion.path;
    path.clear();
    if (std::hypot(motion.ref.x, motion.ref.y) > 0.002f) path.add({0, 0, 0, 0});  // go to the origin first
    if (std::fabs(motion.ref.yaw) > 0.01f) path.add({1, 0, 0, 0});            // square up to the origin heading
    for (int i = 0; i < n; i++) {
        float wx, wy;
        boardToWorld(xs[i], ys[i], wx, wy);
        path.add({0, wx, wy, 0});
    }
    path.speed = c.speed;
    path.accel = std::fmin(0.25f, motion.lim.accel);
    path.yawRate = 0.8f;
    path.yawAccel = 2.0f;
    return true;
}

static float number(JsonVariantConst v, float fallback) {
    if (!v.is<float>() && !v.is<int>()) return fallback;
    const float f = v.as<float>();
    return std::isfinite(f) ? f : fallback;
}

// Runs one JSON command. Returns an error code or nullptr. Flash writes happen
// after the lock is released.
static const char* runCommand(JsonVariantConst d, JsonDocument& reply, bool usb) {
    const char* op = d["op"] | "";
    const uint32_t now = millis();
    bool save = false, saveSlipNow = false, saveFront = false, saveRadius = false;
    const char* error = nullptr;
    if (!strcmp(op, "drive")) {  // hot path: no reply payload
        const float x = number(d["x"], 0), y = number(d["y"], 0), w = number(d["w"], 0);
        const bool body = !strcmp(d["frame"] | "board", "body");
        Locked l;
        if (otaBusy) return "OTA_BUSY";
        float wx, wy;
        if (body) {
            float bx, by;
            wbb::rotate((frontDeg - 90) * wbb::kPi / 180.f, x, y, bx, by);  // keys: W = robot front
            wbb::rotate(motion.ref.yaw, bx, by, wx, wy);
        } else {
            boardToWorld(x, y, wx, wy);
        }
        if (!motion.driveWorld(now, wx, wy, w))
            return motion.mode != Mode::Active ? "NOT_ENABLED" : "PATH_RUNNING";
        return nullptr;
    }
    {
        Locked l;
        if (!strcmp(op, "enable")) {
            if (pinConflict) error = "PIN_CONFLICT";
            else if (motion.mode == Mode::Calibrate) error = "CALIBRATING";
            else if (motion.mode != Mode::Active) motion.enable();
        } else if (!strcmp(op, "release")) {
            if (motion.mode != Mode::Calibrate) motion.release();
        } else if (!strcmp(op, "stop") || !strcmp(op, "hold") || !strcmp(op, "pause")) {
            if (motion.mode == Mode::Calibrate) { calibrator.active = false; motion.mode = Mode::Off; }
            motion.hold();
        } else if (!strcmp(op, "fix")) {  // camera pose in the world frame (m, rad), age in ms
            const float x = number(d["x"], NAN), y = number(d["y"], NAN), yaw = number(d["yaw"], NAN);
            const float age = number(d["age"], 0);
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) || age < 0 || age > 1000) error = "BAD_FIX";
            else if (!motion.fuse({x, y, yaw}, now - uint32_t(age), now, wbb::clampf(number(d["gain"], 0.35f), 0.f, 1.f)))
                error = "FIX_REJECTED";
        } else if (!strcmp(op, "abort")) {  // drop any path (paused or running) and hold here
            if (motion.source == Source::Path) {
                motion.path.clear();
                motion.source = Source::None;
                motion.vel = {};
            }
        } else if (!strcmp(op, "resume")) {
            if (!motion.resumePath()) error = "NO_PATH";
        } else if (!strcmp(op, "origin")) {
            if (motion.moving()) error = "MOVING";
            else { motion.setOrigin(); motion.path.clear(); }
        } else if (!strcmp(op, "cover")) {
            wbb::CoverageConfig c = cover;
            c.width = number(d["width"], c.width);
            c.height = number(d["height"], c.height);
            c.eraser = number(d["eraser"], c.eraser);
            c.overlap = number(d["overlap"], c.overlap);
            c.speed = number(d["speed"], c.speed);
            if (d["vertical"].is<int>()) c.vertical = d["vertical"].as<int>() ? 1 : 0;
            if (d["sx"].is<int>()) c.sx = d["sx"].as<int>() < 0 ? -1 : 1;
            if (d["sy"].is<int>()) c.sy = d["sy"].as<int>() < 0 ? -1 : 1;
            if (d["mount"].is<int>()) {
                const int m = ((d["mount"].as<int>() % 360) + 360) % 360;
                if (m % 90) error = "BAD_MOUNT";
                else { mountDeg = m; motion.slip.mount = mountYaw(); }
            }
            if (!error && !wbb::validCoverage(c)) error = "BAD_COVER";
            if (!error) { cover = c; save = true; }
        } else if (!strcmp(op, "start")) {
            if (motion.mode != Mode::Active) error = "NOT_ENABLED";
            else if (otaBusy) error = "OTA_BUSY";
            else if (motion.source == Source::Path) error = "PATH_RUNNING";
            else if (!loadCoverage(cover)) error = "PLAN_TOO_LARGE";
            else if (!motion.startPath()) error = "START_FAILED";
        } else if (!strcmp(op, "home")) {
            if (motion.mode != Mode::Active) error = "NOT_ENABLED";
            else if (otaBusy) error = "OTA_BUSY";
            else {
                motion.path.clear();
                motion.path.add({0, 0, 0, 0});
                motion.path.add({1, 0, 0, 0});
                motion.path.speed = std::fmax(0.05f, cover.speed);
                motion.path.accel = std::fmin(0.25f, motion.lim.accel);
                motion.path.yawRate = 0.8f;
                motion.path.yawAccel = 2.0f;
                if (motion.source == Source::Manual) motion.source = Source::None;
                if (!motion.startPath()) error = "START_FAILED";
            }
        } else if (!strcmp(op, "radius")) {  // wheel contact radius, mm
            const float r = number(d["mm"], 0) / 1000.f;
            if (!wbb::validRadius(r)) error = "BAD_RADIUS";
            else if (motion.moving()) error = "MOVING";
            else { wbb::gRobotRadius = r; saveRadius = true; }
        } else if (!strcmp(op, "turn")) {  // rotate in place by deg (CCW +), measurement helper
            const float deg = number(d["deg"], 0);
            if (motion.mode != Mode::Active) error = "NOT_ENABLED";
            else if (otaBusy) error = "OTA_BUSY";
            else if (motion.source == Source::Path) error = "PATH_RUNNING";
            else if (std::fabs(deg) > 180) error = "TOO_FAR";
            else {
                motion.path.clear();
                motion.path.add({1, 0, 0, wbb::wrap(motion.ref.yaw + deg * wbb::kPi / 180.f)});
                motion.path.yawRate = wbb::clampf(number(d["rate"], 0.6f), 0.1f, 1.5f);
                if (motion.source == Source::Manual) motion.source = Source::None;
                if (!motion.startPath()) error = "START_FAILED";
            }
        } else if (!strcmp(op, "beacon")) {
            beacon = d["on"] | false;
            // A dimmer backlight keeps the camera from washing the colours out to white.
            const int level = d["level"] | (beacon ? 40 : 100);
            if (lcdReady) sigmaDeltaWrite(kBacklight, constrain(level, 5, 255));
        } else if (!strcmp(op, "front")) {
            if (!d["deg"].is<int>()) error = "BAD_FRONT";
            else if (motion.moving()) error = "MOVING";
            else {
                frontDeg = ((d["deg"].as<int>() % 360) + 360) % 360;
                motion.slip.mount = mountYaw();
                saveFront = true;
            }
        } else if (!strcmp(op, "slip")) {
            wbb::SlipComp c = motion.slip;
            if (d["reset"] | false) c = wbb::SlipComp{};
            c.up = number(d["up"], c.up);
            c.down = number(d["down"], c.down);
            c.side = number(d["side"], c.side);
            c.sagPerM = number(d["sag"], c.sagPerM);
            c.yawPerM = number(d["yaw"], c.yawPerM);
            c.upSide = number(d["up_side"], c.upSide);
            c.downSide = number(d["down_side"], c.downSide);
            c.sideLeft = number(d["side_left"], c.sideLeft);
            c.sagLeft = number(d["sag_left"], c.sagLeft);
            c.yawLeft = number(d["yaw_left"], c.yawLeft);
            c.yawUp = number(d["yaw_up"], c.yawUp);
            c.yawDown = number(d["yaw_down"], c.yawDown);
            c.mount = mountYaw();
            if (!wbb::validSlip(c)) error = "BAD_SLIP";
            else { motion.slip = c; saveSlipNow = true; }
        } else if (!strcmp(op, "move")) {  // relative straight move in the board frame (measurement helper)
            const float bx = number(d["x"], 0), by = number(d["y"], 0);
            if (motion.mode != Mode::Active) error = "NOT_ENABLED";
            else if (otaBusy) error = "OTA_BUSY";
            else if (motion.source == Source::Path) error = "PATH_RUNNING";
            else if (std::hypot(bx, by) > 1.5f) error = "TOO_FAR";
            else {
                float wx, wy;
                boardToWorld(bx, by, wx, wy);
                motion.path.clear();
                motion.path.add({0, motion.ref.x + wx, motion.ref.y + wy, 0});
                motion.path.speed = wbb::clampf(number(d["speed"], cover.speed), 0.02f, 0.3f);
                motion.path.accel = wbb::clampf(number(d["accel"], std::fmin(0.25f, motion.lim.accel)), 0.03f, 1.0f);
                if (motion.source == Source::Manual) motion.source = Source::None;
                if (!motion.startPath()) error = "START_FAILED";
            }
        } else if (!strcmp(op, "spin")) {  // board-frame move while turning by deg in total (spin-erase)
            const float bx = number(d["x"], 0), by = number(d["y"], 0), deg = number(d["deg"], 0);
            if (motion.mode != Mode::Active) error = "NOT_ENABLED";
            else if (otaBusy) error = "OTA_BUSY";
            else if (motion.source == Source::Path) error = "PATH_RUNNING";
            else if (std::hypot(bx, by) > 1.5f) error = "TOO_FAR";
            else if (std::fabs(deg) > 7200) error = "TOO_MANY_TURNS";
            else {
                float wx, wy;
                boardToWorld(bx, by, wx, wy);
                motion.path.clear();
                motion.path.add({2, motion.ref.x + wx, motion.ref.y + wy, deg * wbb::kPi / 180.f});
                motion.path.speed = wbb::clampf(number(d["speed"], cover.speed), 0.005f, 0.3f);
                motion.path.accel = wbb::clampf(number(d["accel"], std::fmin(0.25f, motion.lim.accel)), 0.01f, 1.0f);
                motion.path.yawRate = wbb::clampf(number(d["rate"], 1.5f), 0.1f, 4.0f);
                motion.path.yawAccel = wbb::clampf(number(d["yaw_accel"], 3.0f), 0.5f, 8.0f);
                if (motion.source == Source::Manual) motion.source = Source::None;
                if (!motion.startPath()) error = "START_FAILED";
            }
        } else if (!strcmp(op, "ff")) {  // feed-forward: {wheel: 0 = all or 1..3, kv: [fwd, rev] (1e-5/count/s), ks: [fwd, rev]}
            const int which = d["wheel"] | 0;
            for (int i = 0; i < 3 && !error; i++) {
                if (which && which != i + 1) continue;
                wbb::WheelFF f = motion.ff[i];
                for (int k = 0; k < 2; k++) {
                    f.kv[k] = number(d["kv"][k], f.kv[k] * 1e5f) * 1e-5f;
                    f.ks[k] = number(d["ks"][k], f.ks[k]);
                }
                if (!wbb::validFF(f)) error = "BAD_FF";
                else motion.ff[i] = f;
            }
            if (!error) save = d["save"] | false;
        } else if (!strcmp(op, "gains")) {
            wbb::ServoGains g = motion.gains;
            g.kp = number(d["kp"], g.kp);
            g.ki = number(d["ki"], g.ki);
            g.kd = number(d["kd"], g.kd);
            g.imax = number(d["imax"], g.imax);
            const float speed = number(d["speed"], motion.lim.speed), accel = number(d["accel"], motion.lim.accel);
            if (!wbb::validGains(g) || speed < 0.02f || speed > 0.4f || accel < 0.05f || accel > 2.f) error = "BAD_GAINS";
            else { motion.gains = g; motion.lim.speed = speed; motion.lim.accel = accel; save = d["save"] | false; }
        } else if (!strcmp(op, "save")) {
            save = true;
        } else if (!strcmp(op, "calibrate")) {
            if (!(d["suspended"] | false)) error = "CONFIRM_SUSPENDED";
            else if (motion.mode == Mode::Active || motion.mode == Mode::Calibrate) error = "RELEASE_FIRST";
            else { calibrator.begin(); calibrationResult = "RUNNING"; motion.mode = Mode::Calibrate; }
        } else if (!strcmp(op, "reboot")) {
            if (motion.moving()) error = "MOVING";
            else rebootAt = now + 200;
        } else if (!strcmp(op, "wifi")) {
            if (motion.moving()) error = "MOVING";
        } else if (strcmp(op, "status") && strcmp(op, "hello")) {
            error = "BAD_COMMAND";
        }
    }
    if (!error && !strcmp(op, "wifi")) {
        const char* ssid = d["ssid"] | "";
        const char* pass = d["password"] | "";
        if (!drive::validNetwork(ssid, strlen(ssid), pass, strlen(pass))) return "BAD_WIFI";
        drive::NetworkCredentials next{};
        strlcpy(next.ssid, ssid, sizeof(next.ssid));
        strlcpy(next.password, pass, sizeof(next.password));
        if (legacyPrefs.putBytes("home-wifi", &next, sizeof(next)) != sizeof(next)) return "SAVE_FAILED";
        *network = next;
        WiFi.disconnect(false, false);
        WiFi.begin(network->ssid, network->password);
    }
    if (save && !saveSettings()) error = "SAVE_FAILED";
    if (saveSlipNow && !saveSlip()) error = "SAVE_FAILED";
    if (saveFront && prefs.putInt("front", frontDeg) == 0) error = "SAVE_FAILED";
    if (saveRadius && prefs.putFloat("radius", wbb::gRobotRadius) == 0) error = "SAVE_FAILED";
    (void)usb;
    (void)reply;
    return error;
}

static String commandText(const char* text, size_t len, bool usb, bool* quiet = nullptr) {
    JsonDocument d, r;
    if (deserializeJson(d, text, len)) return "{\"t\":\"ack\",\"ok\":false,\"error\":\"BAD_JSON\"}";
    const char* error = runCommand(d.as<JsonVariantConst>(), r, usb);
    const bool drive = !strcmp(d["op"] | "", "drive");
    if (quiet) *quiet = drive && !error;
    r["t"] = "ack";
    r["op"] = d["op"] | "";
    r["id"] = d["id"] | 0;
    r["ok"] = error == nullptr;
    r["error"] = error ? error : "";
    String out;
    serializeJson(r, out);
    return out;
}

// ---- HTTP handlers (run in the httpd task)
static bool uptimeOk() { return millis() > 4000; }
static esp_err_t pageHandler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, reinterpret_cast<const char*>(WEB_PAGE_GZ), WEB_PAGE_GZ_LEN);
}
static esp_err_t statusHandler(httpd_req_t* req) {
    if (uptimeOk()) confirmOta("status request");
    const String s = telemetry();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, s.c_str(), s.length());
}
static esp_err_t cmdHandler(httpd_req_t* req) {
    char buf[1024];
    if (req->content_len >= sizeof(buf)) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "too long");
    int got = 0;
    while (got < int(req->content_len)) {
        const int n = httpd_req_recv(req, buf + got, req->content_len - got);
        if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (n <= 0) return ESP_FAIL;
        got += n;
    }
    const String r = commandText(buf, got, false);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, r.c_str(), r.length());
}
static esp_err_t wsHandler(httpd_req_t* req) {
    if (req->method == HTTP_GET) return ESP_OK;  // handshake done
    httpd_ws_frame_t f = {};
    f.type = HTTPD_WS_TYPE_TEXT;
    if (httpd_ws_recv_frame(req, &f, 0) != ESP_OK) return ESP_FAIL;
    if (f.len == 0 || f.len > 1000) return ESP_OK;
    uint8_t buf[1001];
    f.payload = buf;
    if (httpd_ws_recv_frame(req, &f, f.len) != ESP_OK) return ESP_FAIL;
    if (f.type != HTTPD_WS_TYPE_TEXT) return ESP_OK;
    if (uptimeOk()) confirmOta("websocket");
    bool quiet = false;
    const String r = commandText(reinterpret_cast<char*>(buf), f.len, false, &quiet);
    if (quiet) return ESP_OK;
    httpd_ws_frame_t out = {};
    out.type = HTTPD_WS_TYPE_TEXT;
    out.payload = (uint8_t*)r.c_str();
    out.len = r.length();
    return httpd_ws_send_frame(req, &out);
}
static esp_err_t otaHandler(httpd_req_t* req) {
    char token[48] = {};
    if (httpd_req_get_hdr_value_str(req, "X-OTA-Token", token, sizeof(token)) != ESP_OK || strcmp(token, WBB_OTA_TOKEN))
        return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "bad token");
    {
        Locked l;
        if (motion.moving() || motion.mode == Mode::Calibrate) {
            httpd_resp_set_status(req, "409 Conflict");
            return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"MOVING\"}");
        }
        motion.hold();
        otaBusy = true;
    }
    const esp_partition_t* part = esp_ota_get_next_update_partition(nullptr);
    esp_ota_handle_t ota = 0;
    const uint32_t started = millis();
    esp_err_t err = part && req->content_len > 0 && req->content_len <= part->size
                        ? esp_ota_begin(part, OTA_WITH_SEQUENTIAL_WRITES, &ota)
                        : ESP_ERR_INVALID_SIZE;
    static char buf[4096];
    size_t remaining = req->content_len;
    int timeouts = 0;
    while (err == ESP_OK && remaining > 0) {
        const int n = httpd_req_recv(req, buf, remaining < sizeof(buf) ? remaining : sizeof(buf));
        if (n == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts < 5) continue;
        if (n <= 0) { err = ESP_FAIL; break; }
        err = esp_ota_write(ota, buf, n);
        remaining -= n;
    }
    if (ota) {
        if (err == ESP_OK) err = esp_ota_end(ota);  // verifies the image
        else esp_ota_abort(ota);
    }
    if (err == ESP_OK) err = esp_ota_set_boot_partition(part);
    char reply[128];
    snprintf(reply, sizeof(reply), "{\"ok\":%s,\"error\":\"%s\",\"ms\":%lu}", err == ESP_OK ? "true" : "false",
             err == ESP_OK ? "" : esp_err_to_name(err), (unsigned long)(millis() - started));
    if (err == ESP_OK) rebootAt = millis() + 300;
    else otaBusy = false;
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, reply);
}

static volatile bool wsBusy = false;
struct WsJob { char* text; size_t len; };
static void wsBroadcast(void* arg) {
    auto* job = static_cast<WsJob*>(arg);
    size_t n = 8;
    int fds[8];
    if (httpd_get_client_list(http, &n, fds) == ESP_OK)
        for (size_t i = 0; i < n; i++)
            if (httpd_ws_get_fd_info(http, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
                httpd_ws_frame_t f = {};
                f.type = HTTPD_WS_TYPE_TEXT;
                f.payload = reinterpret_cast<uint8_t*>(job->text);
                f.len = job->len;
                f.final = true;
                httpd_ws_send_frame_async(http, fds[i], &f);
            }
    free(job->text);
    free(job);
    wsBusy = false;
}
static void pushTelemetry() {
    if (!http || wsBusy) return;
    const String s = telemetry();
    auto* job = static_cast<WsJob*>(malloc(sizeof(WsJob)));
    if (!job) return;
    job->text = static_cast<char*>(malloc(s.length() + 1));
    if (!job->text) { free(job); return; }
    memcpy(job->text, s.c_str(), s.length() + 1);
    job->len = s.length();
    wsBusy = true;
    if (httpd_queue_work(http, wsBroadcast, job) != ESP_OK) { free(job->text); free(job); wsBusy = false; }
}

static void startHttp() {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 8;
    cfg.lru_purge_enable = true;
    cfg.stack_size = 10240;
    cfg.send_wait_timeout = 2;
    cfg.recv_wait_timeout = 10;
    cfg.task_priority = 6;
    if (httpd_start(&http, &cfg) != ESP_OK) { http = nullptr; return; }
    httpd_uri_t uris[] = {
        {.uri = "/", .method = HTTP_GET, .handler = pageHandler, .user_ctx = nullptr},
        {.uri = "/api/status", .method = HTTP_GET, .handler = statusHandler, .user_ctx = nullptr},
        {.uri = "/api/cmd", .method = HTTP_POST, .handler = cmdHandler, .user_ctx = nullptr},
        {.uri = "/ota", .method = HTTP_POST, .handler = otaHandler, .user_ctx = nullptr},
    };
    for (auto& u : uris) httpd_register_uri_handler(http, &u);
    httpd_uri_t ws = {};
    ws.uri = "/ws";
    ws.method = HTTP_GET;
    ws.handler = wsHandler;
    ws.is_websocket = true;
    httpd_register_uri_handler(http, &ws);
}

static void startWifi() {
    const uint16_t id = unsigned(ESP.getEfuseMac() >> 32) & 0xffff;
    snprintf(apName, sizeof(apName), "WBB-C6-%04X", id);
    snprintf(hostName, sizeof(hostName), "wbb-c6-%04x", id);
    String pass = legacyPrefs.getString("ap-pass", "");
    if (pass.length() != 12) {
        snprintf(apPass, sizeof(apPass), "%08lX%04lX", (unsigned long)esp_random(), (unsigned long)(esp_random() & 0xffff));
        legacyPrefs.putString("ap-pass", apPass);
    } else {
        strlcpy(apPass, pass.c_str(), sizeof(apPass));
    }
    network = new drive::NetworkCredentials();
    if (!(legacyPrefs.getBytesLength("home-wifi") == sizeof(*network) &&
          legacyPrefs.getBytes("home-wifi", network, sizeof(*network)) == sizeof(*network) &&
          drive::validNetwork(*network) && network->auth == drive::NetworkCredentials::Personal))
        *network = {};
    WiFi.persistent(false);
    WiFi.setHostname(hostName);
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(apName, apPass, 6, 0, 3);
    WiFi.setSleep(false);  // low latency control
    WiFi.setAutoReconnect(true);
    if (network->ssid[0]) WiFi.begin(network->ssid, network->password);
    if (MDNS.begin(hostName)) MDNS.addService("http", "tcp", 80);
}

// ------------------------------------------------------------------ LCD
static Adafruit_ST7789 lcd(&SPI, 14, 15, 21);
static String cached[8];
static uint16_t colors[8];
static void row(unsigned index, const String& text, uint16_t color) {
    if (cached[index] == text && colors[index] == color) return;
    cached[index] = text;
    colors[index] = color;
    char padded[25];
    snprintf(padded, sizeof(padded), "%-24.24s", text.c_str());
    lcd.setTextWrap(false);
    lcd.setTextSize(2);
    lcd.setTextColor(color, 0x0841);
    lcd.setCursor(12, 6 + 20 * index);
    lcd.print(padded);
}
static void displayTask(void*) {
    bool beaconShown = false;
    for (;;) {
        if (beacon != beaconShown) {
            beaconShown = beacon;
            if (beaconShown) {
                lcd.fillRect(0, 0, 160, 172, ST77XX_CYAN);
                lcd.fillRect(160, 0, 160, 172, ST77XX_MAGENTA);
            } else {
                lcd.fillScreen(0x0841);
                for (auto& c : cached) c = "";
            }
        }
        if (beaconShown) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        Mode mode; Source source; const char* fault; int fm; wbb::Pose p; float lag; bool stuck, paused;
        int pi, pn; float err[3];
        {
            Locked l;
            mode = motion.mode; source = motion.source; fault = motion.fault; fm = motion.faultMotor;
            p = motion.odom; lag = motion.lagMetric(); stuck = motion.stuck;
            paused = motion.path.paused(); pi = motion.path.index; pn = motion.path.count;
            for (int i = 0; i < 3; i++) err[i] = motion.wheel[i].err;
        }
        char line[48];
        if (mode == Mode::Fault) snprintf(line, sizeof(line), "FAULT %s M%d", fault, fm);
        else if (mode == Mode::Active) snprintf(line, sizeof(line), "HOLD %s%s", source == Source::Path ? (paused ? "PAUSED" : "ERASING") : source == Source::Manual ? "DRIVE" : "", stuck ? " STUCK" : "");
        else snprintf(line, sizeof(line), mode == Mode::Calibrate ? "CALIBRATING" : "OFF (BRAKE)");
        row(0, line, mode == Mode::Fault ? ST77XX_RED : mode == Mode::Active ? ST77XX_GREEN : ST77XX_YELLOW);
        if (WiFi.status() == WL_CONNECTED) row(1, "IP " + WiFi.localIP().toString(), ST77XX_GREEN);
        else row(1, String("AP ") + apName, ST77XX_WHITE);
        row(2, WiFi.status() == WL_CONNECTED ? String(hostName) + ".local" : String("PW ") + apPass, ST77XX_CYAN);
        row(3, String("FW ") + WBB_BUILD_ID + (otaPendingVerify && !otaConfirmed ? " NEW" : ""), 0x9cf3);
        snprintf(line, sizeof(line), "X%5.0f Y%5.0f %4.0fd", p.x * 1000, p.y * 1000, p.yaw * 57.2958f);
        row(4, line, ST77XX_WHITE);
        snprintf(line, sizeof(line), "ERR %4.0f %4.0f %4.0f", err[0], err[1], err[2]);
        row(5, line, lag > 8 ? ST77XX_ORANGE : ST77XX_WHITE);
        if (source == Source::Path) snprintf(line, sizeof(line), "PATH %d/%d", pi, pn);
        else snprintf(line, sizeof(line), "%s", pinConflict ? "REMOVE TF CARD!" : "");
        row(6, line, pinConflict ? ST77XX_RED : ST77XX_WHITE);
        row(7, otaBusy ? "UPDATING..." : "", ST77XX_YELLOW);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// ------------------------------------------------------------------ setup / loop
void setup() {
    // 1. Motor pins first: brake, then release the reboot hold without a glitch.
    brakePinsGpio();
    releasePinHold();
    for (auto& p : kPins) { pinMode(p.a, INPUT_PULLUP); pinMode(p.b, INPUT_PULLUP); }
    pinMode(kButton, INPUT_PULLUP);
    lock = xSemaphoreCreateMutex();
    for (int i = 0; i < 3; i++) {
        const uint32_t in = REG_READ(GPIO_IN_REG);
        encPrev[i] = (((in >> kPins[i].a) & 1) << 1) | ((in >> kPins[i].b) & 1);
        attachInterruptArg(kPins[i].a, encoderIsr, reinterpret_cast<void*>(i), CHANGE);
        attachInterruptArg(kPins[i].b, encoderIsr, reinterpret_cast<void*>(i), CHANGE);
    }
    prefs.begin("c6-wall", false);
    legacyPrefs.begin("c6-drive", false);
    loadSettings();
    loadFront();
    loadSlip();
    restoreResume();
    pwmReady = attachPwm();
    if (!pwmReady) motion.setFault("PWM_ATTACH");
    xTaskCreate(controlTask, "control", 6144, nullptr, 10, nullptr);

    Serial.setTxBufferSize(4096);
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);

    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    otaPendingVerify = esp_ota_get_state_partition(running, &state) == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY;

    SPI.begin(7, -1, 6, 14);
    lcd.init(172, 320);
    lcd.setRotation(1);
    lcd.setSPISpeed(8000000);
    lcd.fillScreen(0x0841);
    lcdReady = sigmaDeltaAttach(kBacklight, 1000000) && sigmaDeltaWrite(kBacklight, 100);

    startWifi();
    startHttp();
    xTaskCreate(displayTask, "display", 4096, nullptr, 1, nullptr);
#ifdef WBB_TEST_CRASH
    delay(3000);
    abort();  // rollback test image: must never be confirmed
#endif
    Serial.printf("\nWBB c6_wall %s  AP %s / %s  host %s.local\n", buildId().c_str(), apName, apPass, hostName);
}

void loop() {
    static String input;
    static uint32_t lastPush = 0, lastSerial = 0, lastReconnect = 0;
    while (Serial.available()) {
        const int c = Serial.read();
        if (c == '\n') {
            if (input.length()) Serial.println(commandText(input.c_str(), input.length(), true));
            input = "";
        } else if (c != '\r' && input.length() < 1000) {
            input += char(c);
        }
    }
    const uint32_t now = millis();
    if (now - lastPush >= 100) { lastPush = now; pushTelemetry(); }
    if (now - lastSerial >= 250 && Serial && Serial.availableForWrite() > 1500) {
        lastSerial = now;
        Serial.println(telemetry());
    }
    // Wi-Fi watchdog and OTA confirmation.
    const bool sta = WiFi.status() == WL_CONNECTED;
    if (sta && !staConnectedSince) staConnectedSince = now;
    if (!sta) staConnectedSince = 0;
    if (sta && staConnectedSince && now - staConnectedSince > 45000) confirmOta("wifi up 45 s");
    if (otaPendingVerify && !otaConfirmed && now > 150000) {
        Serial.println("OTA image never came back online: rolling back");
        Locked l;
        saveResume();
        holdPinsForReboot();
        saveResumeNvs();
        esp_ota_mark_app_invalid_rollback_and_reboot();
    }
    if (network && network->ssid[0] && !sta && now - lastReconnect > 20000) {
        lastReconnect = now;
        WiFi.reconnect();
    }
    if (rebootAt && int32_t(now - rebootAt) >= 0) {
        Serial.println("Rebooting");
        Serial.flush();
        {
            Locked l;  // the control task cannot change the outputs from here on
            saveResume();
            for (int i = 0; i < 3; i++) writeMotor(i, 0);
            holdPinsForReboot();
            saveResumeNvs();  // flash write while the outputs are already held braked
        }
        esp_restart();
    }
    delay(5);
}
