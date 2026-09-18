// WhiteBoardBot 交互式硬件诊断 (ESP32-C6 + 转接板 + 驱动底板)
// 串口命令逐点排查, 不用反复烧录。
//
// 引脚 (2026-07 实测配对):
//   M0 头 -> 底板H2 -> 物理电机1 | M1 头 -> 底板H1 -> 物理电机3 | M2 头 -> 底板H3 -> 物理电机2
#include <Arduino.h>
#include "soc/gpio_reg.h"

#ifdef WBB_TOP_BOARD_R3
#include <esp_system.h>
#include "board_profile_r3.h"
#include "r3_gpio_safe.h"
static r3::MotorArmSafeLow motorArmLow;
static r3::DiagnosticMotorSafeLow motorOutputsLow;

// R3 is a read-only diagnostic, not the legacy motor bench below. No LEDC,
// charger/I2C, motor supply selection, or encoder-output test is initialized.
static char input[64];
static size_t inputLength = 0;
static bool discardLine = false, rebootPending = false;
static uint32_t rebootAt = 0, session = 0, txDropped = 0;
static const char* lastError = "";
static bool encodersInitialized = false;

static const char* gpioFault() {
    if (motorArmLow.error() != ESP_OK) return "GPIO21_SAFE_LOW_FAILED";
    if (motorOutputsLow.error() != ESP_OK) return "MOTOR_GPIO_SAFE_LOW_FAILED";
    return nullptr;
}

static bool allLow() {
    if (!motorArmLow.holdLow()) { lastError = "GPIO21_SAFE_LOW_FAILED"; return false; }
    if (!motorOutputsLow.holdLow()) { lastError = "MOTOR_GPIO_SAFE_LOW_FAILED"; return false; }
    return true;
}
static void status() {
    const char* fault = gpioFault();
    const bool encoderAvailable = encodersInitialized && !fault;
    char out[1400];
    int n = snprintf(out, sizeof(out),
        "{\"firmware\":\"wbb-r3-review-diag\",\"profile\":\"%s\","
        "\"session\":%lu,\"uptime_ms\":%lu,\"reset_reason\":%d,"
        "\"reason\":\"%s\",\"last_error\":\"%s\","
        "\"gpio_fault_latched\":%s,\"arm_gpio_error\":%d,\"arm_gpio_release_error\":%d,"
        "\"motor_gpio_error\":%d,\"motor_gpio_release_error\":%d,\"encoder_inputs_available\":%s,"
        "\"motion_enabled\":false,\"charge_enabled\":false,\"power_disconnect\":false,"
        "\"motor_voltage_mv\":null,\"motor_current_ma\":null,\"rail_3v3_mv\":null,"
        "\"serial_link\":\"cp2102n_uart0\",\"usb_disconnect_detect\":false,"
        "\"reboot_pending\":%s,\"tx_dropped\":%lu,\"motors\":[",
        r3::profile, (unsigned long)session, (unsigned long)millis(), int(esp_reset_reason()),
        fault ? fault : "BOARD_POWER_UNREVIEWED", lastError, fault ? "true" : "false",
        int(motorArmLow.error()), int(motorArmLow.releaseError()),
        int(motorOutputsLow.error()), int(motorOutputsLow.releaseError()), encoderAvailable ? "true" : "false",
        rebootPending ? "true" : "false", (unsigned long)txDropped);
    for (unsigned i = 0; i < 3 && n > 0 && n < int(sizeof(out)); ++i) {
        const auto& p = r3::headers[i];
        n += snprintf(out + n, sizeof(out) - n,
            "%s{\"header\":\"%s\",\"physical\":%u,\"fi\":%u,\"bi\":%u,"
            "\"enc_a\":%u,\"enc_b\":%u,\"a\":%s,\"b\":%s}",
            i ? "," : "", p.header, p.physical, p.fi, p.bi, p.encA, p.encB,
            encoderAvailable ? (digitalRead(p.encA) ? "1" : "0") : "null",
            encoderAvailable ? (digitalRead(p.encB) ? "1" : "0") : "null");
    }
    if (n <= 0 || n + 4 > int(sizeof(out))) { ++txDropped; return; }
    memcpy(out + n, "]}\n", 4); n += 3;
    // UART writes have no CDC timeout. Do not wait for a host or flush().
    if (!Serial || Serial.availableForWrite() < n ||
        Serial.write(reinterpret_cast<const uint8_t*>(out), n) != size_t(n)) ++txDropped;
}
static void command(const char* op) {
    allLow();
    lastError = gpioFault() ? gpioFault() : "";
    if (!strcmp(op, "hello") || !strcmp(op, "status") || !strcmp(op, "p") ||
        !strcmp(op, "s") || !strcmp(op, "stop") || !strcmp(op, "x") ||
        !strcmp(op, "h") || !strcmp(op, "?")) {
        status(); return;
    }
    if (rebootPending) lastError = "REBOOT_PENDING";
    else if (!strcmp(op, "reboot")) { rebootAt = millis(); rebootPending = true; }
    else lastError = gpioFault() ? gpioFault() : "BOARD_POWER_UNREVIEWED"; // Includes all legacy motor commands.
    status();
}
void setup() {
    // Default-low precedes serial initialization and any possible allocation.
    if (motorArmLow.begin() && motorOutputsLow.begin()) {
        for (const auto& p : r3::headers) {
            pinMode(p.encA, INPUT_PULLUP); pinMode(p.encB, INPUT_PULLUP);
        }
        encodersInitialized = true;
    }
    Serial.setTxBufferSize(4096); Serial.setRxBufferSize(512);
    Serial.begin(115200, SERIAL_8N1, r3::uartRx, r3::uartTx);
    session = esp_random(); inputLength = 0; discardLine = rebootPending = false;
    lastError = gpioFault() ? gpioFault() : ""; status();
}
void loop() {
    allLow(); // Failure remains latched; UART status/reboot stays available.
    if (rebootPending && uint32_t(millis() - rebootAt) >= 100) { esp_restart(); return; }
    for (int budget = 0; budget < 64 && Serial.available(); ++budget) {
        int c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (discardLine) { lastError = "RX_OVERFLOW"; status(); }
            else if (inputLength) { input[inputLength] = 0; command(input); }
            inputLength = 0; discardLine = false;
        } else if (c < 32 || c > 126 || inputLength >= sizeof(input) - 1) {
            discardLine = true;
        } else if (!discardLine) input[inputLength++] = char(c);
    }
    delay(1);
}

#else // Preserve the adapter self-test and its existing bench commands.

struct MotorPins { const char* label; uint8_t fi, bi, encA, encB; };
static const MotorPins MOT[3] = {
    {"M0 -> 底板H2 -> 物理电机1", 0, 1, 16, 17},
    {"M1 -> 底板H1 -> 物理电机3", 2, 3, 18, 19},
    {"M2 -> 底板H3 -> 物理电机2", 4, 5, 20, 23},
};

static const int PWM_FREQ = 20000, PWM_RES = 8;
volatile long encCount[3] = {0, 0, 0};          // AB 相四倍频带方向计数
volatile unsigned long encEdges[3] = {0, 0, 0}; // 所有边沿
volatile unsigned long encBad[3] = {0, 0, 0};   // 非法跳变 (A/B 同时变)
volatile uint8_t encPrev[3] = {0, 0, 0};
static bool pwmMode = true;          // true=LEDC PWM, false=纯DC digitalWrite
static int  duty = 64;               // 从低占空比开始，使用限流电源逐步测试

// 索引 = (prev<<2)|now, 位序 A<<1|B。
static const int8_t QT[16] = {0, 1, -1, 0,  -1, 0, 0, 1,  1, 0, 0, -1,  0, -1, 1, 0};
static inline __attribute__((always_inline)) uint8_t readAB(int i) {
    uint32_t v = REG_READ(GPIO_IN_REG);
    return (uint8_t)((((v >> MOT[i].encA) & 1u) << 1) | ((v >> MOT[i].encB) & 1u));
}
static void IRAM_ATTR encIsr(void* arg) {
    int i = (int)(intptr_t)arg;
    uint8_t now = readAB(i), pv = encPrev[i];
    if (now == pv) return;
    int8_t d = QT[(pv << 2) | now];
    if (d) encCount[i] += d; else encBad[i]++;
    encEdges[i]++; encPrev[i] = now;
}
static void encZero() { for (int i = 0; i < 3; i++) { encCount[i] = 0; encEdges[i] = 0; encBad[i] = 0; } }
static void stopAll();

// 悬空检测: 只切换内部上拉/下拉并读回, 引脚始终是输入。
//   下拉读0 且 上拉读1 -> 悬空 (编码器没供电或没接)
//   两次都1 -> 被外部驱动为高      两次都0 -> 被外部拉低
static void floatTest() {
    stopAll();
    Serial.println("\n--- 编码器线悬空检测 (仅输入, 切换内部上/下拉) ---");
    for (int i = 0; i < 3; i++) {
        Serial.printf("  %s\n", MOT[i].label);
        const uint8_t pins[2] = {MOT[i].encA, MOT[i].encB};
        for (int k = 0; k < 2; k++) {
            pinMode(pins[k], INPUT_PULLDOWN); delay(3); int rd = digitalRead(pins[k]);
            pinMode(pins[k], INPUT_PULLUP);   delay(3); int ru = digitalRead(pins[k]);
            const char* v = (rd == 0 && ru == 1) ? "悬空 (无供电/未接)" : (rd == 1 && ru == 1) ? "外部驱动高" : (rd == 0 && ru == 0) ? "外部拉低" : "?";
            Serial.printf("     ENC_%c=GPIO%-2d  下拉->%d 上拉->%d  %s\n", k ? 'B' : 'A', pins[k], rd, ru, v);
        }
        encPrev[i] = readAB(i);
    }
}

static void useLedc(int i) {
    ledcAttach(MOT[i].fi, PWM_FREQ, PWM_RES);
    ledcAttach(MOT[i].bi, PWM_FREQ, PWM_RES);
}
static void useGpio(int i) {
    ledcDetach(MOT[i].fi); ledcDetach(MOT[i].bi);
    pinMode(MOT[i].fi, OUTPUT); pinMode(MOT[i].bi, OUTPUT);
    digitalWrite(MOT[i].fi, LOW); digitalWrite(MOT[i].bi, LOW);
}

static void drive(int i, int dir) {          // dir: +1 正 / -1 反 / 0 停
    if (pwmMode) {
        ledcWrite(MOT[i].fi, dir > 0 ? duty : 0);
        ledcWrite(MOT[i].bi, dir < 0 ? duty : 0);
    } else {
        digitalWrite(MOT[i].fi, dir > 0 ? HIGH : LOW);
        digitalWrite(MOT[i].bi, dir < 0 ? HIGH : LOW);
    }
}
static void stopAll() { for (int i = 0; i < 3; i++) drive(i, 0); }

static void printPins() {
    Serial.println("\n--- 引脚实时电平 ---");
    Serial.printf("模式: %s   占空比: %d/255\n", pwmMode ? "PWM 20kHz" : "纯DC(digitalWrite)", duty);
    for (int i = 0; i < 3; i++) {
        Serial.printf("  %s\n", MOT[i].label);
        Serial.printf("     FI=GPIO%-2d  BI=GPIO%-2d  |  ENC_A=GPIO%-2d(%d)  ENC_B=GPIO%-2d(%d)  计数=%ld 边沿=%lu 非法=%lu\n",
                      MOT[i].fi, MOT[i].bi, MOT[i].encA, digitalRead(MOT[i].encA),
                      MOT[i].encB, digitalRead(MOT[i].encB), encCount[i], encEdges[i], encBad[i]);
    }
    Serial.println("(编码器脚开了内部上拉: 悬空/无供电=1, 被霍尔拉低=0)");
}

// 只读采样：静止霍尔输出为低是正常情况，不能据此判断短路。
// 不得把连接着编码器输出的 GPIO 改成推挽输出。
static void shortTest() {
    stopAll();
    Serial.println("\n--- 编码器只读检查（不能单独判定短路）---");
    printPins();
    Serial.println("请在电机停止时手转各轮，观察 A/B 电平与计数变化。");
    Serial.println("若始终不变，需断电后测通断，并区分供电、线束、传感器与 GPIO 故障。");
}

static void help() {
    Serial.println("\n======== 命令 ========");
    Serial.println("  0f 0r 0s   电机0(物理1) 正转/反转/停     1f 1r 1s / 2f 2r 2s 同理");
    Serial.println("  x          全部停");
    Serial.println("  a          自动依次测3个电机(每向2秒)");
    Serial.println("  p          打印引脚电平+编码器计数");
    Serial.println("  e          清零编码器计数");
    Serial.println("  w          切换 PWM / 纯DC 驱动模式");
    Serial.println("  d <0-255>  设占空比 (如 d 200)");
    Serial.println("  s          编码器只读检查（低电平不等于短路）");
    Serial.println("  f          编码器线悬空检测 (切内部上/下拉, 仅输入)");
    Serial.println("  h          帮助");
    Serial.println("======================");
}

static void autoTest() {
    Serial.printf("\n### 自动测试: 每个电机 正转2秒 -> 停 -> 反转2秒 (duty=%d) ###\n", duty);
    for (int i = 0; i < 3; i++) {
        Serial.printf("\n[%d] %s  (FI=GPIO%d BI=GPIO%d)\n", i, MOT[i].label, MOT[i].fi, MOT[i].bi);
        for (int dir = 1; dir >= -1; dir -= 2) {
            encCount[i] = 0; encEdges[i] = 0; encBad[i] = 0;
            drive(i, dir);
            Serial.printf("    %s 驱动中", dir > 0 ? "正转" : "反转");
            for (int t = 0; t < 4; t++) { delay(500); Serial.print("."); }
            drive(i, 0);
            delay(300);
            Serial.printf(" 计数=%ld 边沿=%lu 非法=%lu %s\n", encCount[i], encEdges[i], encBad[i],
                          encEdges[i] > 10 ? "<-- 转了!" : "<-- 没动");
        }
    }
    Serial.println("\n### 结束 ###");
    printPins();
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n\n==========================================");
    Serial.println("  WhiteBoardBot 交互诊断 (ESP32-C6)");
    Serial.println("==========================================");
    Serial.printf("芯片 %s / Flash %luMB\n", ESP.getChipModel(),
                  (unsigned long)(ESP.getFlashChipSize() / 1048576));

    for (int i = 0; i < 3; i++) {
        useLedc(i); drive(i, 0);
        pinMode(MOT[i].encA, INPUT_PULLUP);
        pinMode(MOT[i].encB, INPUT_PULLUP);
    }
    delay(5);
    for (int i = 0; i < 3; i++) {
        encPrev[i] = readAB(i);
        attachInterruptArg(digitalPinToInterrupt(MOT[i].encA), encIsr, (void*)(intptr_t)i, CHANGE);
        attachInterruptArg(digitalPinToInterrupt(MOT[i].encB), encIsr, (void*)(intptr_t)i, CHANGE);
    }

    printPins();
    help();
    Serial.println("\n提示: 若三个电机都不动, 先确认 XT30 电池已插且有电 —");
    Serial.println("      编码器靠 3V3(USB就有电), 电机靠 BAT+(必须电池)。");
    Serial.print("\n> ");
}

void loop() {
    static char buf[32]; static int len = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (len == 0) { Serial.print("> "); continue; }
            buf[len] = 0;
            String cmd = String(buf); cmd.trim(); len = 0;

            if (cmd == "x") { stopAll(); Serial.println("全部停止"); }
            else if (cmd == "p") printPins();
            else if (cmd == "h" || cmd == "?") help();
            else if (cmd == "a") autoTest();
            else if (cmd == "s") shortTest();
            else if (cmd == "f") floatTest();
            else if (cmd == "e") { encZero(); Serial.println("编码器已清零"); }
            else if (cmd == "w") {
                stopAll(); pwmMode = !pwmMode;
                for (int i = 0; i < 3; i++) pwmMode ? useLedc(i) : useGpio(i);
                Serial.printf("驱动模式 -> %s\n", pwmMode ? "PWM 20kHz" : "纯DC");
            }
            else if (cmd.startsWith("d")) {
                int v = cmd.substring(1).toInt();
                if (v >= 0 && v <= 255) { duty = v; Serial.printf("占空比 = %d/255\n", duty); }
            }
            else if (cmd.length() == 2 && cmd[0] >= '0' && cmd[0] <= '2') {
                int i = cmd[0] - '0';
                char d = cmd[1];
                encCount[i] = 0; encEdges[i] = 0; encBad[i] = 0;
                drive(i, d == 'f' ? 1 : (d == 'r' ? -1 : 0));
                Serial.printf("电机%d %s (FI=GPIO%d BI=GPIO%d, %s duty=%d)\n", i,
                              d == 'f' ? "正转" : (d == 'r' ? "反转" : "停"),
                              MOT[i].fi, MOT[i].bi, pwmMode ? "PWM" : "DC", duty);
            }
            else Serial.printf("未知命令 '%s' (h=帮助)\n", cmd.c_str());
            Serial.print("> ");
        } else if (len < 30) buf[len++] = c;
    }
}
#endif
