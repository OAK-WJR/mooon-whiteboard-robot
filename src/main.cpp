/**
 * WhiteBoardBot 主固件 — ESP32 自主控制 (无 ROS)
 *
 * ESP32 自己做: 路径规划 + 循线 + FF/PID。上位机 (scripts/whiteboard_app.py)
 * 只通过 WiFi UDP 发目标区域 / 位置校正, 并接收状态回传。
 *
 * 双任务: controlTask(Core1, 50Hz 控制) + commTask(Core0, WiFi 收发)。
 * loop() 跑一个串口调参控制台 —— 运行时改 PID/速度/前瞻, 无需重新烧录。
 *
 * 串口命令 (115200, 行结束发送):
 *   s              打印状态 JSON
 *   p              打印所有可调参数
 *   t              切换遥测流 (每轮 目标/实际/PWM, 10Hz)
 *   m <0|1|2|a>    选择调参电机 (a=全部, 默认全部)
 *   kp/ki/kd/kf/il <v>   设 P/I/D/前馈/积分限幅 (作用于选中电机)
 *   v <v>          最大线速度 m/s
 *   la <v>         前瞻距离 m
 *   gt <v>         到达容差 m
 *   kl <v>         循线比例增益
 *   area <x y w h spacing>   本地规划一条 zigzag 路径 (台架测试)
 *   pos <x y th>   校正里程计位姿
 *   go             开始执行当前路径
 *   x              停止
 *   h / ?          帮助
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include "robot_controller.h"
#include "wifi_comm.h"
#include "config.h"

static RobotController robot;
static WifiComm wifi;

// 串口调参状态
static int  tune_motor = -1;       // -1=全部三轮
static bool telemetry_on = false;
static bool json_stream  = false;  // j: 串口流式输出位姿 JSON (USB 导航可视化)

// FreeRTOS 控制任务 (Core 1, 高优先级, 50Hz)
static void controlTask(void* param) {
    (void)param;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        float dt = (float)CONTROL_PERIOD_MS / 1000.0f;
        robot.update(dt);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

// FreeRTOS 通信任务 (Core 0, WiFi + 状态回传)
static void commTask(void* param) {
    (void)param;
    unsigned long last_status = 0;

    for (;;) {
        // 接收命令
        JsonDocument doc;
        if (wifi.receive(doc)) {
            robot.handleCommand(doc);

            // 立即回传状态
            JsonDocument resp;
            robot.fillStatus(resp);
            wifi.send(resp);
        }

        // 定期状态回传
        if (millis() - last_status > (1000 / STATUS_RATE_HZ)) {
            JsonDocument status;
            robot.fillStatus(status);
            wifi.send(status);
            last_status = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(5));  // 200Hz 轮询
    }
}

static void printHelp() {
    Serial.println(F("命令: s=状态 p=参数 t=遥测  m<0/1/2/a>=选电机"));
    Serial.println(F("  kp/ki/kd/kf/il <v>=PID  v=速度 la=前瞻 gt=容差 kl=循线增益"));
    Serial.println(F("  area <x y w h sp>  pos <x y th>  go=开始 x=停止 h=帮助"));
}

// 处理一行串口命令
static void handleSerialLine(char* line) {
    // 拆出命令字与参数串
    char* cmd = strtok(line, " \t");
    if (!cmd) return;
    char* args = strtok(nullptr, "");   // 余下全部作为参数

    auto arg1 = [&](float def) -> float { return args ? atof(args) : def; };

    // --- 复用 handleCommand 的命令 (与 UDP 同一套逻辑) ---
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
    // --- 串口专属: 调参 / 诊断 ---
    else if (strcmp(cmd, "s") == 0) {
        JsonDocument status; robot.fillStatus(status);
        serializeJsonPretty(status, Serial); Serial.println();
    } else if (strcmp(cmd, "p") == 0) {
        robot.printParams(Serial);
    } else if (strcmp(cmd, "t") == 0) {
        telemetry_on = !telemetry_on;
        Serial.printf("遥测 %s\n", telemetry_on ? "ON" : "OFF");
    } else if (strcmp(cmd, "j") == 0) {
        json_stream = !json_stream;
        Serial.printf("位姿JSON流 %s\n", json_stream ? "ON" : "OFF");
    } else if (strcmp(cmd, "m") == 0) {
        if (args && (args[0] == 'a' || args[0] == 'A')) tune_motor = -1;
        else tune_motor = constrain((int)arg1(-1), -1, 2);
        Serial.printf("选中电机: %s\n", tune_motor < 0 ? "全部" : (String("M") + tune_motor).c_str());
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
    } else if (strcmp(cmd, "h") == 0 || strcmp(cmd, "?") == 0) {
        printHelp();
    } else {
        Serial.printf("未知命令 '%s' (h=帮助)\n", cmd);
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println(F("\n============================"));
    Serial.println(F("  WhiteBoardBot 自主固件"));
    Serial.println(F("============================"));

    robot.begin();
    wifi.begin(WIFI_SSID, WIFI_PASS);

    Serial.printf("UDP listen: %d, report: %d\n", UDP_LISTEN_PORT, UDP_REPORT_PORT);
    printHelp();

    // 控制循环固定在 Core 1
    xTaskCreatePinnedToCore(controlTask, "control", 4096, NULL, 19, NULL, 1);
    // 通信任务在 Core 0
    xTaskCreatePinnedToCore(commTask, "comm", 8192, NULL, 5, NULL, 0);
}

void loop() {
    // 行缓冲串口命令
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

    // 遥测流 (10Hz)
    static unsigned long last_tele = 0;
    if (telemetry_on && millis() - last_tele >= 100) {
        robot.printTelemetry(Serial);
        last_tele = millis();
    }

    // 位姿 JSON 流 (10Hz, 给 USB 串口导航可视化)
    static unsigned long last_json = 0;
    if (json_stream && millis() - last_json >= 100) {
        JsonDocument st; robot.fillStatus(st);
        serializeJson(st, Serial); Serial.println();
        last_json = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(20));
}
