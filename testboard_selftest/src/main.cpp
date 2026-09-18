// test board 交互自检: 板载 ESP32-C6-MINI-1, MT3608 升压 (EN=GPIO21), LED=GPIO15
// 串口(USB CDC)命令逐点排查; 编码器做真正的 AB 相四倍频计数 (带方向 + 非法跳变统计)。
#include <Arduino.h>
#include <esp_system.h>
#include "soc/gpio_reg.h"

struct M { const char* label; uint8_t fi, bi, a, b; };
static const M MOT[3] = {
    {"M0 -> 底板H2 -> 物理电机1", 0, 1, 16, 17},
    {"M1 -> 底板H1 -> 物理电机3", 2, 3, 18, 19},
    {"M2 -> 底板H3 -> 物理电机2", 4, 5, 20, 23},
};
static const int PIN_MOTOR_EN = 21, PIN_LED = 15;
static const int PWM_FREQ = 20000, PWM_RES = 8;
static int duty = 100;
static bool boostOn = true;

volatile int32_t  cnt[3]     = {0, 0, 0};
volatile uint32_t edges[3]   = {0, 0, 0};
volatile uint32_t invalid[3] = {0, 0, 0};
volatile uint8_t  prevAB[3]  = {0, 0, 0};
// 索引 = (prev<<2)|now, 位序 A<<1|B。合法跳变 +1/-1, 同态 0, 两位同时变 = 非法。
static const int8_t QT[16] = {0, 1, -1, 0,  -1, 0, 0, 1,  1, 0, 0, -1,  0, -1, 1, 0};

static inline __attribute__((always_inline)) uint8_t readAB(int i) {
    uint32_t v = REG_READ(GPIO_IN_REG);
    return (uint8_t)((((v >> MOT[i].a) & 1u) << 1) | ((v >> MOT[i].b) & 1u));
}
static void IRAM_ATTR isr(void* arg) {
    int i = (int)(intptr_t)arg;
    uint8_t now = readAB(i), pv = prevAB[i];
    if (now == pv) return;
    int8_t d = QT[(pv << 2) | now];
    if (d) cnt[i] += d; else invalid[i]++;
    edges[i]++; prevAB[i] = now;
}

static void drive(int i, int dir) {
    ledcWrite(MOT[i].fi, dir > 0 ? duty : 0);
    ledcWrite(MOT[i].bi, dir < 0 ? duty : 0);
}
static void stopAll() { for (int i = 0; i < 3; i++) drive(i, 0); }
static void zero() { for (int i = 0; i < 3; i++) { cnt[i] = 0; edges[i] = 0; invalid[i] = 0; } }

static void printJson() {
    Serial.printf("{\"type\":\"status\",\"boost_en\":%d,\"duty\":%d,\"uptime_ms\":%lu,\"motors\":[",
                  digitalRead(PIN_MOTOR_EN), duty, (unsigned long)millis());
    for (int i = 0; i < 3; i++) {
        uint8_t ab = readAB(i);
        Serial.printf("%s{\"i\":%d,\"fi\":%d,\"bi\":%d,\"enc_a\":%d,\"enc_b\":%d,\"a\":%d,\"b\":%d,"
                      "\"count\":%ld,\"edges\":%lu,\"invalid\":%lu}",
                      i ? "," : "", i, MOT[i].fi, MOT[i].bi, MOT[i].a, MOT[i].b, (ab >> 1) & 1, ab & 1,
                      (long)cnt[i], (unsigned long)edges[i], (unsigned long)invalid[i]);
    }
    Serial.println("]}");
}
static void printPins() {
    Serial.printf("\n--- 状态 ---  升压EN(GPIO21)=%d  占空比=%d/255\n", digitalRead(PIN_MOTOR_EN), duty);
    for (int i = 0; i < 3; i++) {
        uint8_t ab = readAB(i);
        Serial.printf("  %s\n     FI=GPIO%-2d BI=GPIO%-2d | A=GPIO%-2d(%d) B=GPIO%-2d(%d) | 计数=%ld 边沿=%lu 非法=%lu\n",
                      MOT[i].label, MOT[i].fi, MOT[i].bi, MOT[i].a, (ab >> 1) & 1, MOT[i].b, ab & 1,
                      (long)cnt[i], (unsigned long)edges[i], (unsigned long)invalid[i]);
    }
    Serial.println("(编码器脚内部上拉: 悬空/无供电=1, 被霍尔拉低=0)");
}
static void help() {
    Serial.println("\n======== 命令 ========");
    Serial.println("  0f 0r 0s   电机0 正转/反转/停 (1f.. 2f.. 同理)   x 全部停");
    Serial.println("  a          自动: 每个电机 正转0.8s -> 反转0.8s, 报计数/方向");
    Serial.println("  p / j      打印状态 (人读 / JSON 一行)      e 清零计数");
    Serial.println("  d <0-255>  设占空比      b1 / b0  升压 EN 高/低 (GPIO21)");
    Serial.println("  h          帮助");
    Serial.println("======================");
}
static void autoTest() {
    Serial.printf("\n### 自动测试 duty=%d, 升压EN=%d ###\n", duty, digitalRead(PIN_MOTOR_EN));
    for (int i = 0; i < 3; i++) {
        for (int dir = 1; dir >= -1; dir -= 2) {
            zero(); drive(i, dir); delay(800); drive(i, 0); delay(300);
            Serial.printf("[%d] %s %s: 计数=%ld 边沿=%lu 非法=%lu %s\n", i, MOT[i].label,
                          dir > 0 ? "正转" : "反转", (long)cnt[i], (unsigned long)edges[i],
                          (unsigned long)invalid[i], edges[i] > 10 ? "<-- 转了" : "<-- 没动");
        }
    }
    Serial.println("### 结束 ###");
}

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("\n[1] serial ok");
    pinMode(PIN_MOTOR_EN, OUTPUT); digitalWrite(PIN_MOTOR_EN, HIGH);   // 升压默认开
    pinMode(PIN_LED, OUTPUT);      digitalWrite(PIN_LED, HIGH);
    Serial.println("[2] gpio ok");
    for (int i = 0; i < 3; i++) {
        ledcAttach(MOT[i].fi, PWM_FREQ, PWM_RES); ledcAttach(MOT[i].bi, PWM_FREQ, PWM_RES); drive(i, 0);
        pinMode(MOT[i].a, INPUT_PULLUP); pinMode(MOT[i].b, INPUT_PULLUP);
    }
    Serial.println("[3] ledc ok");
    delay(5);
    for (int i = 0; i < 3; i++) {
        prevAB[i] = readAB(i);
        attachInterruptArg(digitalPinToInterrupt(MOT[i].a), isr, (void*)(intptr_t)i, CHANGE);
        attachInterruptArg(digitalPinToInterrupt(MOT[i].b), isr, (void*)(intptr_t)i, CHANGE);
    }
    Serial.println("[4] isr ok");
    Serial.println("\n==========================================");
    Serial.println("  test board 自检 (板载 ESP32-C6-MINI-1)");
    Serial.println("==========================================");
    Serial.printf("芯片 %s rev%d / Flash %luMB / 复位原因 %d\n", ESP.getChipModel(), ESP.getChipRevision(),
                  (unsigned long)(ESP.getFlashChipSize() / 1048576), (int)esp_reset_reason());
    printPins(); help(); Serial.print("\n> ");
}

void loop() {
    static uint32_t lastBlink = 0;
    if (millis() - lastBlink >= 500) { lastBlink = millis(); digitalWrite(PIN_LED, !digitalRead(PIN_LED)); }
    static char buf[32]; static int len = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c != '\n' && c != '\r') { if (len < 30) buf[len++] = c; continue; }
        if (len == 0) { Serial.print("> "); continue; }
        buf[len] = 0; String cmd = String(buf); cmd.trim(); len = 0;
        if      (cmd == "x") { stopAll(); Serial.println("全部停止"); }
        else if (cmd == "p") printPins();
        else if (cmd == "j") printJson();
        else if (cmd == "e") { zero(); Serial.println("计数已清零"); }
        else if (cmd == "a") autoTest();
        else if (cmd == "h" || cmd == "?") help();
        else if (cmd == "b1" || cmd == "b0") { boostOn = cmd == "b1"; digitalWrite(PIN_MOTOR_EN, boostOn); Serial.printf("升压EN -> %d\n", boostOn); }
        else if (cmd.startsWith("d")) { int v = cmd.substring(1).toInt(); if (v >= 0 && v <= 255) { duty = v; Serial.printf("占空比 = %d/255\n", duty); } }
        else if (cmd.length() == 2 && cmd[0] >= '0' && cmd[0] <= '2') {
            int i = cmd[0] - '0'; char d = cmd[1]; zero();
            drive(i, d == 'f' ? 1 : (d == 'r' ? -1 : 0));
            Serial.printf("电机%d %s (duty=%d)\n", i, d == 'f' ? "正转" : (d == 'r' ? "反转" : "停"), duty);
        }
        else Serial.printf("未知命令 '%s' (h=帮助)\n", cmd.c_str());
        Serial.print("> ");
    }
}
