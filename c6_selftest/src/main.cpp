// WhiteBoardBot 交互式硬件诊断 (ESP32-C6 + 转接板 + 驱动底板)
// 串口命令逐点排查, 不用反复烧录。
//
// 引脚 (2026-07 实测配对):
//   M0 头 -> 底板H2 -> 物理电机1 | M1 头 -> 底板H1 -> 物理电机3 | M2 头 -> 底板H3 -> 物理电机2
#include <Arduino.h>

struct MotorPins { const char* label; uint8_t fi, bi, encA, encB; };
static const MotorPins MOT[3] = {
    {"M0 -> 底板H2 -> 物理电机1", 0, 1, 16, 17},
    {"M1 -> 底板H1 -> 物理电机3", 2, 3, 18, 19},
    {"M2 -> 底板H3 -> 物理电机2", 4, 5, 20, 23},
};

static const int PWM_FREQ = 20000, PWM_RES = 8;
volatile long encCount[3] = {0, 0, 0};
static bool pwmMode = true;          // true=LEDC PWM, false=纯DC digitalWrite
static int  duty = 255;              // 默认全速, 排查阶段力矩最大

static void IRAM_ATTR isr0() { encCount[0]++; }
static void IRAM_ATTR isr1() { encCount[1]++; }
static void IRAM_ATTR isr2() { encCount[2]++; }

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
        Serial.printf("     FI=GPIO%-2d  BI=GPIO%-2d  |  ENC_A=GPIO%-2d(%d)  ENC_B=GPIO%-2d(%d)  计数=%ld\n",
                      MOT[i].fi, MOT[i].bi, MOT[i].encA, digitalRead(MOT[i].encA),
                      MOT[i].encB, digitalRead(MOT[i].encB), encCount[i]);
    }
    Serial.println("(编码器脚开了内部上拉: 悬空/无供电=1, 被霍尔拉低=0)");
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
    Serial.println("  h          帮助");
    Serial.println("======================");
}

static void autoTest() {
    Serial.println("\n### 自动测试: 每个电机 正转2秒 -> 停 -> 反转2秒 ###");
    for (int i = 0; i < 3; i++) {
        Serial.printf("\n[%d] %s  (FI=GPIO%d BI=GPIO%d)\n", i, MOT[i].label, MOT[i].fi, MOT[i].bi);
        for (int dir = 1; dir >= -1; dir -= 2) {
            encCount[i] = 0;
            drive(i, dir);
            Serial.printf("    %s 驱动中", dir > 0 ? "正转" : "反转");
            for (int t = 0; t < 4; t++) { delay(500); Serial.print("."); }
            drive(i, 0);
            delay(300);
            Serial.printf(" 编码器 %ld 脉冲 %s\n", encCount[i],
                          encCount[i] > 10 ? "<-- 转了!" : "<-- 没动");
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
    attachInterrupt(digitalPinToInterrupt(MOT[0].encA), isr0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOT[1].encA), isr1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOT[2].encA), isr2, CHANGE);

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
            else if (cmd == "e") { for (int i = 0; i < 3; i++) encCount[i] = 0; Serial.println("编码器已清零"); }
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
                encCount[i] = 0;
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
