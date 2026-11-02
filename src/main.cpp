/**
 * WhiteBoardBot main firmware — ESP32 autonomous control (no ROS)
 *
 * The ESP32 does it all: path planning + path following + FF/PID. The host PC (scripts/whiteboard_app.py)
 * only sends target areas / position corrections over WiFi UDP and receives status reports.
 *
 * Two tasks: controlTask (50Hz control) + commTask (WiFi send/receive). On the C6 both run on Core0.
 * loop() runs a serial tuning console — change PID/speed/lookahead at runtime without reflashing.
 *
 * Serial commands (115200, sent on line end):
 *   s              print status JSON
 *   p              print all tunable parameters
 *   t              toggle telemetry stream (per wheel target/actual/PWM, 10Hz)
 *   m <0|1|2|a>    select motor to tune (a=all, default all)
 *   kp/ki/kd/kf/il <v>   set P/I/D/feedforward/integral limit (applies to selected motor)
 *   v <v>          max linear speed m/s
 *   la <v>         lookahead distance m
 *   gt <v>         arrival tolerance m
 *   kl <v>         path-following proportional gain
 *   area <x y w h spacing>   plan a zigzag path locally (bench test)
 *   pos <x y th>   correct odometry pose
 *   go             start running the current path
 *   x              stop
 *   h / ?          help
 */

#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO)
#include <Arduino.h>
#include <type_traits>
// Check the installed core's actual alias for factory UART bridge profiles.
static_assert(std::is_same<decltype(Serial), HardwareSerial>::value,
              "Factory bridge Serial must be HardwareSerial, not native USB CDC");
static_assert(&Serial == &Serial0, "Factory bridge console must use hardware UART0");
#elif defined(WBB_TOP_BOARD_KISS_C6)
#include <Arduino.h>
#include <type_traits>
// KISS C6 has no USB-UART bridge since 2026-10-29: the console is the C6's native USB Serial/JTAG CDC.
static_assert(std::is_same<decltype(Serial), HWCDC>::value, "KISS C6 Serial must be the native USB CDC");
#endif

#if defined(WBB_R3_DIAGNOSTIC)
#include "../c6_selftest/src/main.cpp"
#elif !defined(WBB_TOP_BOARD_V2)
#include <Arduino.h>
#include <ArduinoJson.h>
#include "robot_controller.h"
#include "wifi_comm.h"
#include "firmware_config.h"

static RobotController robot;
static WifiComm wifi;

// Serial tuning state
static int  tune_motor = -1;       // -1=all three wheels
static bool telemetry_on = false;
static bool json_stream  = false;  // j: stream pose JSON over serial (USB navigation visualization)

// FreeRTOS control task (high priority, 50Hz)
static void controlTask(void* param) {
    (void)param;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        float dt = (float)CONTROL_PERIOD_MS / 1000.0f;
        robot.update(dt);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

// FreeRTOS communication task (Core 0, WiFi + status reports)
static void commTask(void* param) {
    (void)param;
    unsigned long last_status = 0;

    for (;;) {
        // Receive commands
        JsonDocument doc;
        if (wifi.receive(doc)) {
            robot.handleCommand(doc);

            // Report status right away
            JsonDocument resp;
            robot.fillStatus(resp);
            wifi.send(resp);
        }

        // Periodic status report
        if (millis() - last_status > (1000 / STATUS_RATE_HZ)) {
            JsonDocument status;
            robot.fillStatus(status);
            wifi.send(status);
            last_status = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(5));  // 200Hz polling
    }
}

static void printHelp() {
    Serial.println(F("Commands: s=status p=params t=telemetry g=machine-readable gains  m<0/1/2/a>=select motor"));
    Serial.println(F("  kp/ki/kd/kf/il <v>=PID  v=speed la=lookahead gt=tolerance kl=path-following gain"));
    Serial.println(F("  w <i> <v>=fixed speed on one wheel (tuning)  w x=stop  area/pos/goto/go/x/h"));
}

// Handle one line of serial command
static void handleSerialLine(char* line) {
    // Split out the command word and the argument string
    char* cmd = strtok(line, " \t");
    if (!cmd) return;
    char* args = strtok(nullptr, "");   // everything left is the arguments

    auto arg1 = [&](float def) -> float { return args ? atof(args) : def; };

    // --- Commands reused from handleCommand (same logic as UDP) ---
    if (strcmp(cmd, "go") == 0) {
        JsonDocument d; d["cmd"] = "start"; robot.handleCommand(d);
    } else if (strcmp(cmd, "x") == 0 || strcmp(cmd, "stop") == 0) {
        JsonDocument d; d["cmd"] = "stop"; robot.handleCommand(d);
    } else if (strcmp(cmd, "v") == 0) {
        JsonDocument d; d["cmd"] = "set_speed"; d["v"] = arg1(MAX_LINEAR_SPEED);
        robot.handleCommand(d);
    } else if (strcmp(cmd, "area") == 0) {
        float x = 0, y = 0, w = 0.1f, h = 0.1f, sp = 0.02f;
        if (args) sscanf(args, "%f %f %f %f %f", &x, &y, &w, &h, &sp);
        JsonDocument d;
        d["cmd"] = "area"; d["x"] = x; d["y"] = y; d["w"] = w; d["h"] = h; d["spacing"] = sp;
        robot.handleCommand(d);
    } else if (strcmp(cmd, "goto") == 0) {
        float x = 0, y = 0;
        if (args) sscanf(args, "%f %f", &x, &y);
        JsonDocument d; d["cmd"] = "goto"; d["x"] = x; d["y"] = y;
        robot.handleCommand(d);
    } else if (strcmp(cmd, "pos") == 0) {
        float x = 0, y = 0, th = 0;
        if (args) sscanf(args, "%f %f %f", &x, &y, &th);
        JsonDocument d;
        d["cmd"] = "pos"; d["x"] = x; d["y"] = y; d["theta"] = th;
        robot.handleCommand(d);
    }
    // --- Serial only: tuning / diagnostics ---
    else if (strcmp(cmd, "s") == 0) {
        JsonDocument status; robot.fillStatus(status);
        serializeJsonPretty(status, Serial); Serial.println();
    } else if (strcmp(cmd, "p") == 0) {
        robot.printParams(Serial);
    } else if (strcmp(cmd, "t") == 0) {
        telemetry_on = !telemetry_on;
        Serial.printf("Telemetry %s\n", telemetry_on ? "ON" : "OFF");
    } else if (strcmp(cmd, "j") == 0) {
        json_stream = !json_stream;
        Serial.printf("Pose JSON stream %s\n", json_stream ? "ON" : "OFF");
    } else if (strcmp(cmd, "m") == 0) {
        if (args && (args[0] == 'a' || args[0] == 'A')) tune_motor = -1;
        else tune_motor = constrain((int)arg1(-1), -1, 2);
        Serial.printf("Selected motor: %s\n", tune_motor < 0 ? "all" : (String("M") + tune_motor).c_str());
    } else if (strcmp(cmd, "kp") == 0) { robot.tunePid(tune_motor, 'p', arg1(0));
    } else if (strcmp(cmd, "ki") == 0) { robot.tunePid(tune_motor, 'i', arg1(0));
    } else if (strcmp(cmd, "kd") == 0) { robot.tunePid(tune_motor, 'd', arg1(0));
    } else if (strcmp(cmd, "kf") == 0) { robot.tunePid(tune_motor, 'f', arg1(0));
    } else if (strcmp(cmd, "il") == 0) { robot.tunePid(tune_motor, 'l', arg1(0));
    } else if (strcmp(cmd, "la") == 0) { float vv = arg1(LOOK_AHEAD_DIST); robot.setLookAhead(vv);
        Serial.printf("lookahead=%.3f\n", vv);
    } else if (strcmp(cmd, "gt") == 0) { float vv = arg1(GOAL_TOLERANCE); robot.setGoalTolerance(vv);
        Serial.printf("goal_tol=%.3f\n", vv);
    } else if (strcmp(cmd, "kl") == 0) { float vv = arg1(KP_LINEAR); robot.setLinearGain(vv);
        Serial.printf("kp_lin=%.2f\n", vv);
    } else if (strcmp(cmd, "g") == 0) {          // machine-readable gains (tuner)
        robot.printGains(Serial);
    } else if (strcmp(cmd, "w") == 0) {          // fixed speed on one wheel: w <i> <v>  /  w x=stop
        if (!args || args[0] == 'x' || args[0] == 'a' || args[0] == 'A') {
            robot.setWheelTest(-1, 0.0f);
        } else {
            int wi = -1; float wv = 0.0f;
            sscanf(args, "%d %f", &wi, &wv);
            robot.setWheelTest(wi, wv);
        }
    } else if (strcmp(cmd, "h") == 0 || strcmp(cmd, "?") == 0) {
        printHelp();
    } else {
        Serial.printf("Unknown command '%s' (h=help)\n", cmd);
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println(F("\n============================"));
    Serial.println(F("  WhiteBoardBot autonomous firmware"));
    Serial.println(F("============================"));

    robot.begin();
    wifi.begin(WIFI_SSID, WIFI_PASS);

    Serial.printf("UDP listen: %d, report: %d\n", UDP_LISTEN_PORT, UDP_REPORT_PORT);
    printHelp();

    // The ESP32-C6 only has Core0; only the dual-core ESP32 uses Core1.
    constexpr BaseType_t controlCore = portNUM_PROCESSORS > 1 ? 1 : 0;
    TaskHandle_t controlHandle = nullptr;
    const BaseType_t controlResult = xTaskCreatePinnedToCore(
        controlTask, "control", 4096, NULL, 19, &controlHandle, controlCore);
    const BaseType_t commResult = controlResult == pdPASS
        ? xTaskCreatePinnedToCore(commTask, "comm", 8192, NULL, 5, NULL, 0)
        : pdFAIL;
    if (controlResult != pdPASS || commResult != pdPASS) {
        if (controlHandle) vTaskDelete(controlHandle);
        robot.stopMotors();
        Serial.println(F("FAULT TASK_INIT: control/communication task could not start"));
        // Do not enter loop(), which can accept motion commands; reinitialize after reset.
        for (;;) delay(1000);
    }
}

void loop() {
    // Line-buffered serial commands
    static char buf[64];
    static int len = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (len > 0) { buf[len] = '\0'; handleSerialLine(buf); len = 0; }
        } else if (len < (int)sizeof(buf) - 1) {
            buf[len++] = c;
        }
    }

    // Telemetry stream (50Hz, for plotting step responses during live tuning)
    static unsigned long last_tele = 0;
    if (telemetry_on && millis() - last_tele >= 20) {
        robot.printTelemetry(Serial);
        last_tele = millis();
    }

    // Pose JSON stream (10Hz, for USB serial navigation visualization)
    static unsigned long last_json = 0;
    if (json_stream && millis() - last_json >= 100) {
        JsonDocument st; robot.fillStatus(st);
        serializeJson(st, Serial); Serial.println();
        last_json = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(20));
}
#endif // legacy ESP32 / C6 adapter entry point
