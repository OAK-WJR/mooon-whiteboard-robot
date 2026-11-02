// test board interactive self-test: onboard ESP32-C6-MINI-1, MT3608 boost (EN=GPIO21), LED=GPIO15
// Check point by point with serial (USB CDC) commands; the encoders get true A/B quadrature x4 counting (with direction + invalid transition stats).
#include <Arduino.h>
#include <esp_system.h>
#include "soc/gpio_reg.h"

struct M { const char* label; uint8_t fi, bi, a, b; };
static const M MOT[3] = {
    {"M0 -> base H2 -> physical motor 1", 0, 1, 16, 17},
    {"M1 -> base H1 -> physical motor 3", 2, 3, 18, 19},
    {"M2 -> base H3 -> physical motor 2", 4, 5, 20, 23},
};
static const int PIN_MOTOR_EN = 21, PIN_LED = 15;
static const int PWM_FREQ = 20000, PWM_RES = 8;
static int duty = 100;
static bool boostOn = true;

volatile int32_t  cnt[3]     = {0, 0, 0};
volatile uint32_t edges[3]   = {0, 0, 0};
volatile uint32_t invalid[3] = {0, 0, 0};
volatile uint8_t  prevAB[3]  = {0, 0, 0};
// Index = (prev<<2)|now, bit order A<<1|B. Valid transition +1/-1, same state 0, both bits changing = invalid.
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
    Serial.printf("\n--- Status ---  boost EN(GPIO21)=%d  duty=%d/255\n", digitalRead(PIN_MOTOR_EN), duty);
    for (int i = 0; i < 3; i++) {
        uint8_t ab = readAB(i);
        Serial.printf("  %s\n     FI=GPIO%-2d BI=GPIO%-2d | A=GPIO%-2d(%d) B=GPIO%-2d(%d) | count=%ld edges=%lu invalid=%lu\n",
                      MOT[i].label, MOT[i].fi, MOT[i].bi, MOT[i].a, (ab >> 1) & 1, MOT[i].b, ab & 1,
                      (long)cnt[i], (unsigned long)edges[i], (unsigned long)invalid[i]);
    }
    Serial.println("(Encoder pins internal pull-up: floating/no power=1, pulled low by Hall=0)");
}
static void help() {
    Serial.println("\n======== Commands ========");
    Serial.println("  0f 0r 0s   motor 0 forward/reverse/stop (1f.. 2f.. likewise)   x stop all");
    Serial.println("  a          auto: each motor forward 0.8s -> reverse 0.8s, report counts/direction");
    Serial.println("  p / j      print status (human-readable / one JSON line)      e zero counts");
    Serial.println("  d <0-255>  set duty cycle      b1 / b0  boost EN high/low (GPIO21)");
    Serial.println("  h          help");
    Serial.println("======================");
}
static void autoTest() {
    Serial.printf("\n### Auto test duty=%d, boost EN=%d ###\n", duty, digitalRead(PIN_MOTOR_EN));
    for (int i = 0; i < 3; i++) {
        for (int dir = 1; dir >= -1; dir -= 2) {
            zero(); drive(i, dir); delay(800); drive(i, 0); delay(300);
            Serial.printf("[%d] %s %s: count=%ld edges=%lu invalid=%lu %s\n", i, MOT[i].label,
                          dir > 0 ? "forward" : "reverse", (long)cnt[i], (unsigned long)edges[i],
                          (unsigned long)invalid[i], edges[i] > 10 ? "<-- turned" : "<-- did not move");
        }
    }
    Serial.println("### Done ###");
}

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("\n[1] serial ok");
    pinMode(PIN_MOTOR_EN, OUTPUT); digitalWrite(PIN_MOTOR_EN, HIGH);   // boost on by default
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
    Serial.println("  test board self-test (onboard ESP32-C6-MINI-1)");
    Serial.println("==========================================");
    Serial.printf("Chip %s rev%d / Flash %luMB / reset reason %d\n", ESP.getChipModel(), ESP.getChipRevision(),
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
        if      (cmd == "x") { stopAll(); Serial.println("All stopped"); }
        else if (cmd == "p") printPins();
        else if (cmd == "j") printJson();
        else if (cmd == "e") { zero(); Serial.println("Counts zeroed"); }
        else if (cmd == "a") autoTest();
        else if (cmd == "h" || cmd == "?") help();
        else if (cmd == "b1" || cmd == "b0") { boostOn = cmd == "b1"; digitalWrite(PIN_MOTOR_EN, boostOn); Serial.printf("boost EN -> %d\n", boostOn); }
        else if (cmd.startsWith("d")) { int v = cmd.substring(1).toInt(); if (v >= 0 && v <= 255) { duty = v; Serial.printf("Duty = %d/255\n", duty); } }
        else if (cmd.length() == 2 && cmd[0] >= '0' && cmd[0] <= '2') {
            int i = cmd[0] - '0'; char d = cmd[1]; zero();
            drive(i, d == 'f' ? 1 : (d == 'r' ? -1 : 0));
            Serial.printf("Motor %d %s (duty=%d)\n", i, d == 'f' ? "forward" : (d == 'r' ? "reverse" : "stop"), duty);
        }
        else Serial.printf("Unknown command '%s' (h=help)\n", cmd.c_str());
        Serial.print("> ");
    }
}
