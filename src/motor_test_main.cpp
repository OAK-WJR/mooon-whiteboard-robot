/**
 * WhiteBoardBot 三电机测试 (开环 / PID / 运动学)
 *
 * 串口命令:
 *   '0'~'2'  - 选择电机        'c' - 切换 开环/PID
 *   '+'/'-'  - 调整 PWM/速度    'k' - 切换 运动学模式
 *   'f'/'r'  - 正/反转          'w'/'x' - 前进/后退 (运动学)
 *   's'      - 停当前            'q'/'e' - 左移/右移 (运动学)
 *   'a'      - 停全部            'j'/'l' - 左转/右转 (运动学)
 *   'p'      - 状态  'z' - 清零
 */

#ifdef MOTOR_TEST_MODE

#include <Arduino.h>
#include <driver/adc.h>
#include "config.h"
#include "motor_driver.h"
#include "pid_controller.h"
#include "kinematics.h"

static MotorDriver motor[3];
static PIDController pid[3];
static Kinematics kin(WHEEL_RADIUS, ROBOT_RADIUS);

static bool pid_mode = false;
static bool kin_mode = false;
static float target_speed[3] = {0, 0, 0};
static BodyVelocity body_cmd = {0, 0, 0};

// 编码器计数 (volatile, ISR 里更新)
static volatile long enc_count[3] = {0, 0, 0};
static long prev_enc[3] = {0, 0, 0};
static constexpr int8_t ENC_DIR_SIGN[3] = {
    ENCODER_DIR_M0, ENCODER_DIR_M1, ENCODER_DIR_M2
};

static void IRAM_ATTR enc0_a_isr() {
    enc_count[0] += (digitalRead(ENC0_A_PIN) == digitalRead(ENC0_B_PIN)) ? -1 : 1;
}
static void IRAM_ATTR enc0_b_isr() {
    enc_count[0] += (digitalRead(ENC0_A_PIN) == digitalRead(ENC0_B_PIN)) ? 1 : -1;
}
static void IRAM_ATTR enc1_a_isr() {
    enc_count[1] += (digitalRead(ENC1_A_PIN) == digitalRead(ENC1_B_PIN)) ? -1 : 1;
}
static void IRAM_ATTR enc1_b_isr() {
    enc_count[1] += (digitalRead(ENC1_A_PIN) == digitalRead(ENC1_B_PIN)) ? 1 : -1;
}
static void IRAM_ATTR enc2_a_isr() {
    enc_count[2] += (digitalRead(ENC2_A_PIN) == digitalRead(ENC2_B_PIN)) ? -1 : 1;
}
static void IRAM_ATTR enc2_b_isr() {
    enc_count[2] += (digitalRead(ENC2_A_PIN) == digitalRead(ENC2_B_PIN)) ? 1 : -1;
}

static long read_enc(int i) {
    noInterrupts();
    long c = enc_count[i];
    interrupts();
    return c * ENC_DIR_SIGN[i];
}

static int selected = 0;       // 当前选中的电机
static int16_t pwm[3] = {0, 0, 0};

// 行缓冲 (支持多字符 PID 命令)
static char cmd_buf[32];
static int cmd_len = 0;

// PID 调参命令处理
static void handle_pid_command(const char* buf, int len) {
    char cmd = buf[0];
    float val = (len > 1) ? atof(buf + 1) : 0.0f;

    switch (cmd) {
    case 'P':
        for (int i = 0; i < 3; i++) pid[i].setKp(val);
        Serial.printf("OK P=%.1f\n", val);
        break;
    case 'I':
        for (int i = 0; i < 3; i++) pid[i].setKi(val);
        Serial.printf("OK I=%.1f\n", val);
        break;
    case 'D':
        for (int i = 0; i < 3; i++) pid[i].setKd(val);
        Serial.printf("OK D=%.1f\n", val);
        break;
    case 'F':
        for (int i = 0; i < 3; i++) pid[i].setKf(val);
        Serial.printf("OK F=%.1f\n", val);
        break;
    case 'L':
        for (int i = 0; i < 3; i++) pid[i].setIntegralLimit(val);
        Serial.printf("OK L=%.1f\n", val);
        break;
    case 'G':
        Serial.printf("GAINS:P=%.1f,I=%.1f,D=%.1f,F=%.1f,L=%.1f\n",
                      pid[0].getKp(), pid[0].getKi(), pid[0].getKd(),
                      pid[0].getKf(), pid[0].getIntegralLimit());
        break;
    case 'R':
        for (int i = 0; i < 3; i++) pid[i].reset();
        Serial.println("OK RESET");
        break;
    case 'S':
        // 设置选中电机的绝对目标速度 (m/s), 自动启用 PID 模式
        if (!pid_mode) {
            pid_mode = true;
            for (int i = 0; i < 3; i++) {
                pid[i].reset();
                prev_enc[i] = read_enc(i);
            }
        }
        target_speed[selected] = val;
        Serial.printf("OK M%d S=%.3f\n", selected, val);
        break;
    case 'V':
        // 增量调速 (m/s), 自动启用 PID 模式
        if (!pid_mode) {
            pid_mode = true;
            for (int i = 0; i < 3; i++) {
                pid[i].reset();
                prev_enc[i] = read_enc(i);
            }
        }
        target_speed[selected] += val;
        Serial.printf("OK M%d V=%.3f (tgt=%.3f)\n", selected, val, target_speed[selected]);
        break;
    case 'A':
        // 全部停止
        body_cmd = {0, 0, 0};
        for (int i = 0; i < 3; i++) {
            target_speed[i] = 0; pwm[i] = 0;
            motor[i].stop(); pid[i].reset();
        }
        Serial.println("OK STOP");
        break;
    default:
        Serial.printf("ERR unknown '%c'\n", cmd);
        break;
    }
}

// 检查串口并处理 PID 命令 (在阻塞循环内调用)
static void check_serial_pid() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmd_len > 0) {
                cmd_buf[cmd_len] = '\0';
                if (cmd_len > 1 && cmd_buf[0] >= 'A' && cmd_buf[0] <= 'Z') {
                    handle_pid_command(cmd_buf, cmd_len);
                }
                cmd_len = 0;
            }
        } else if (cmd_len < (int)sizeof(cmd_buf) - 1) {
            cmd_buf[cmd_len++] = c;
        }
    }
}

static float enc_to_speed(long delta_enc, float dt) {
    return (float)delta_enc / (ENCODER_CPR * 4) * (2.0f * PI * WHEEL_RADIUS) / dt;
}

static int sign_i(int v) { return (v > 0) - (v < 0); }
static int sign_f(float v, float eps = 1e-3f) { return (v > eps) - (v < -eps); }

static void print_status() {
    Serial.println("──────────────────────────────────");
    for (int i = 0; i < 3; i++) {
        long c = read_enc(i);
        float rev = (float)c / (ENCODER_CPR * 4);
        if (pid_mode) {
            float act = enc_to_speed(c - prev_enc[i], 0.02f);
            bool sign_match = (sign_i(pwm[i]) == 0) || (sign_f(act) == 0) || (sign_i(pwm[i]) == sign_f(act));
            Serial.printf("  M%d: tgt=%.3f  act=%.3f  PWM=%4d  enc=%ld (%.1f圈)  [FF=%.0f P=%.0f I=%.0f D=%.0f]\n",
                          i, target_speed[i], act,
                          pwm[i], c, rev,
                          pid[i].getFFTerm(), pid[i].getPTerm(),
                          pid[i].getITerm(), pid[i].getDTerm());
            Serial.printf("      sign(PWM/act): %d/%d  %s\n",
                          sign_i(pwm[i]), sign_f(act), sign_match ? "MATCH" : "MISMATCH");
        } else {
            Serial.printf("  M%d: PWM=%4d  enc=%7ld  (%.2f圈)\n", i, pwm[i], c, rev);
        }
    }
    const char* mode = kin_mode ? "运动学" : (pid_mode ? "PID" : "开环");
    if (kin_mode)
        Serial.printf("  模式: %s  Body: vx=%.2f vy=%.2f ω=%.1f\n",
                      mode, body_cmd.vx, body_cmd.vy, body_cmd.omega);
    else
        Serial.printf("  模式: %s  选中: M%d\n", mode, selected);
    Serial.println("──────────────────────────────────");
}

void setup() {
    Serial.begin(115200);
    delay(500);

    // 修复 ESP32 Errata 3.11: ADC 电源常开, 消除 GPIO36/39 噪声
    adc_power_acquire();

    Serial.println("\n============================");
    Serial.println("  三电机独立测试");
    Serial.println("============================");
    Serial.printf("  M0: FI=%d BI=%d  ENC=%d/%d\n",
                  MOTOR0_FI_PIN, MOTOR0_BI_PIN, ENC0_A_PIN, ENC0_B_PIN);
    Serial.printf("  M1: FI=%d BI=%d  ENC=%d/%d\n",
                  MOTOR1_FI_PIN, MOTOR1_BI_PIN, ENC1_A_PIN, ENC1_B_PIN);
    Serial.printf("  M2: FI=%d BI=%d  ENC=%d/%d\n",
                  MOTOR2_FI_PIN, MOTOR2_BI_PIN, ENC2_A_PIN, ENC2_B_PIN);

    motor[0].begin(MOTOR0_FI_PIN, MOTOR0_BI_PIN, 0, 1, PWM_FREQUENCY, PWM_RESOLUTION);
    motor[1].begin(MOTOR1_FI_PIN, MOTOR1_BI_PIN, 2, 3, PWM_FREQUENCY, PWM_RESOLUTION);
    motor[2].begin(MOTOR2_FI_PIN, MOTOR2_BI_PIN, 4, 5, PWM_FREQUENCY, PWM_RESOLUTION);

    // 编码器引脚 — 实测确认映射
    pinMode(ENC0_A_PIN, INPUT);        // GPIO34
    pinMode(ENC0_B_PIN, INPUT);        // GPIO35
    pinMode(ENC1_A_PIN, INPUT);        // GPIO36
    pinMode(ENC1_B_PIN, INPUT_PULLUP); // GPIO2 (strapping pin)
    pinMode(ENC2_A_PIN, INPUT_PULLUP); // GPIO19
    pinMode(ENC2_B_PIN, INPUT_PULLUP); // GPIO18

    attachInterrupt(digitalPinToInterrupt(ENC0_A_PIN), enc0_a_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC0_B_PIN), enc0_b_isr, CHANGE);
    // Enc1 (GPIO36/39): adc_power_acquire() 消除噪声后可用中断
    attachInterrupt(digitalPinToInterrupt(ENC1_A_PIN), enc1_a_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC1_B_PIN), enc1_b_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_A_PIN), enc2_a_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_B_PIN), enc2_b_isr, CHANGE);

    const float pid_kp[3]   = {PID_KP_M0, PID_KP_M1, PID_KP_M2};
    const float pid_ki[3]   = {PID_KI_M0, PID_KI_M1, PID_KI_M2};
    const float pid_kd[3]   = {PID_KD_M0, PID_KD_M1, PID_KD_M2};
    const float pid_kf[3]   = {PID_KF_M0, PID_KF_M1, PID_KF_M2};
    const float pid_ilim[3] = {PID_ILIM_M0, PID_ILIM_M1, PID_ILIM_M2};

    for (int i = 0; i < 3; i++) {
        pid[i].setGains(pid_kp[i], pid_ki[i], pid_kd[i], pid_kf[i]);
        pid[i].setOutputLimits(-PWM_MAX, PWM_MAX);
        pid[i].setIntegralLimit(pid_ilim[i]);
    }

    Serial.println("\n命令: 0/1/2=选电机 +/-=调整 f/r=正反转 s/a=停 p=状态 z=清零 c=切换PID");
    Serial.println("PID调参: P<v> I<v> D<v> F<v> L<v> G=查询 R=重置");
    Serial.printf("当前: M%d 开环模式\n\n", selected);
}

void loop() {
    // 串口读取: 大写字母开头缓冲到换行 (PID 命令), 其他立即执行
    while (Serial.available()) {
        char ch = Serial.read();
        // 缓冲模式: 正在收集多字符 PID 命令
        if (cmd_len > 0) {
            if (ch == '\n' || ch == '\r') {
                cmd_buf[cmd_len] = '\0';
                handle_pid_command(cmd_buf, cmd_len);
                cmd_len = 0;
            } else if (cmd_len < (int)sizeof(cmd_buf) - 1) {
                cmd_buf[cmd_len++] = ch;
            }
            continue;
        }
        // 大写字母: 开始缓冲 PID 命令
        if (ch >= 'A' && ch <= 'Z') {
            cmd_buf[0] = ch;
            cmd_len = 1;
            continue;
        }
        // 忽略换行
        if (ch == '\n' || ch == '\r') continue;
        // 其余: 立即执行单字符命令
        char c = ch;
        switch (c) {
        case '0': case '1': case '2':
            selected = c - '0';
            Serial.printf(">>> 选中 M%d (PWM=%d)\n", selected, pwm[selected]);
            break;

        case '+':
            if (pid_mode) {
                target_speed[selected] += 0.05f;
                Serial.printf("M%d 目标=%.3f m/s\n", selected, target_speed[selected]);
            } else {
                pwm[selected] = constrain(pwm[selected] + 20, -255, 255);
                motor[selected].setSpeed(pwm[selected]);
                Serial.printf("M%d PWM=%d\n", selected, pwm[selected]);
            }
            break;

        case '-':
            if (pid_mode) {
                target_speed[selected] -= 0.05f;
                Serial.printf("M%d 目标=%.3f m/s\n", selected, target_speed[selected]);
            } else {
                pwm[selected] = constrain(pwm[selected] - 20, -255, 255);
                motor[selected].setSpeed(pwm[selected]);
                Serial.printf("M%d PWM=%d\n", selected, pwm[selected]);
            }
            break;

        case 'f':
            if (pid_mode) {
                target_speed[selected] = 0.20f;
                Serial.printf("M%d 目标=0.200 m/s\n", selected);
            } else {
                pwm[selected] = 200;
                motor[selected].setSpeed(200);
                Serial.printf("M%d 正转 PWM=200\n", selected);
            }
            break;

        case 'r':
            if (pid_mode) {
                target_speed[selected] = -0.20f;
                Serial.printf("M%d 目标=-0.200 m/s\n", selected);
            } else {
                pwm[selected] = -200;
                motor[selected].setSpeed(-200);
                Serial.printf("M%d 反转 PWM=-200\n", selected);
            }
            break;

        case 's':
            if (pid_mode) {
                target_speed[selected] = 0;
                pid[selected].reset();
            }
            pwm[selected] = 0;
            motor[selected].stop();
            Serial.printf("M%d 停止\n", selected);
            break;

        case 'a':
            body_cmd = {0, 0, 0};
            for (int i = 0; i < 3; i++) {
                target_speed[i] = 0; pwm[i] = 0;
                motor[i].stop(); pid[i].reset();
            }
            Serial.println("全部停止");
            break;

        case 'c':
            pid_mode = !pid_mode;
            kin_mode = false;
            body_cmd = {0, 0, 0};
            for (int i = 0; i < 3; i++) {
                target_speed[i] = 0; pwm[i] = 0;
                motor[i].stop(); pid[i].reset();
                prev_enc[i] = read_enc(i);
            }
            Serial.printf(">>> 切换到 %s 模式\n", pid_mode ? "PID闭环" : "开环");
            break;

        case 'k':
            kin_mode = !kin_mode;
            pid_mode = kin_mode;
            body_cmd = {0, 0, 0};
            for (int i = 0; i < 3; i++) {
                target_speed[i] = 0; pwm[i] = 0;
                motor[i].stop(); pid[i].reset();
                prev_enc[i] = read_enc(i);
            }
            if (kin_mode)
                Serial.println(">>> 运动学模式  w/x=前后 q/e=左右 j/l=旋转 a=停");
            else
                Serial.println(">>> 退出运动学模式");
            break;

        case 'w': case 'x': case 'q': case 'e': case 'j': case 'l':
            if (kin_mode) {
                if (c == 'w') body_cmd.vx += 0.05f;
                if (c == 'x') body_cmd.vx -= 0.05f;
                if (c == 'q') body_cmd.vy += 0.05f;
                if (c == 'e') body_cmd.vy -= 0.05f;
                if (c == 'j') body_cmd.omega += 0.5f;
                if (c == 'l') body_cmd.omega -= 0.5f;
                WheelVelocity wv = kin.inverse(body_cmd);
                for (int i = 0; i < 3; i++) target_speed[i] = wv.v[i];
                Serial.printf("Body: vx=%.2f vy=%.2f ω=%.1f → W: %.3f %.3f %.3f\n",
                              body_cmd.vx, body_cmd.vy, body_cmd.omega,
                              target_speed[0], target_speed[1], target_speed[2]);
            }
            break;

        case 'p':
            print_status();
            break;

        case 'z':
            noInterrupts();
            enc_count[0] = enc_count[1] = enc_count[2] = 0;
            interrupts();
            Serial.println("编码器已清零 - 手转一圈后按 p 查看计数");
            break;

        case 'g':
            Serial.println("GPIO 轮询测试 (3秒, 转动电机轴)...");
            {
                int changes[6] = {0,0,0,0,0,0};
                uint8_t pins[6] = {ENC0_A_PIN, ENC0_B_PIN, ENC1_A_PIN, ENC1_B_PIN, ENC2_A_PIN, ENC2_B_PIN};
                uint8_t prev[6];
                for (int i = 0; i < 6; i++) prev[i] = digitalRead(pins[i]);
                unsigned long start = millis();
                while (millis() - start < 3000) {
                    for (int i = 0; i < 6; i++) {
                        uint8_t val = digitalRead(pins[i]);
                        if (val != prev[i]) { changes[i]++; prev[i] = val; }
                    }
                }
                Serial.printf("  Enc0 A(GPIO%d): %d 次变化\n", ENC0_A_PIN, changes[0]);
                Serial.printf("  Enc0 B(GPIO%d): %d 次变化\n", ENC0_B_PIN, changes[1]);
                Serial.printf("  Enc1 A(GPIO%d): %d 次变化\n", ENC1_A_PIN, changes[2]);
                Serial.printf("  Enc1 B(GPIO%d): %d 次变化\n", ENC1_B_PIN, changes[3]);
                Serial.printf("  Enc2 A(GPIO%d): %d 次变化\n", ENC2_A_PIN, changes[4]);
                Serial.printf("  Enc2 B(GPIO%d): %d 次变化\n", ENC2_B_PIN, changes[5]);
            }
            break;

        case 't':
            Serial.println("全 GPIO 扫描 (5秒, 转动电机轴)...");
            {
                const uint8_t scan_pins[] = {
                    36, 39, 34, 35, 32, 33, 25, 26, 27, 14, 12, 13,  // 左排
                    23, 22, 21, 19, 18, 5, 17, 16, 4, 2, 15           // 右排
                };
                const int N = sizeof(scan_pins) / sizeof(scan_pins[0]);
                int changes[25] = {0};
                uint8_t prev[25];
                for (int i = 0; i < N; i++) {
                    pinMode(scan_pins[i], INPUT);
                    prev[i] = digitalRead(scan_pins[i]);
                }
                unsigned long start = millis();
                while (millis() - start < 5000) {
                    for (int i = 0; i < N; i++) {
                        uint8_t val = digitalRead(scan_pins[i]);
                        if (val != prev[i]) { changes[i]++; prev[i] = val; }
                    }
                }
                Serial.println("  --- 有信号的 GPIO ---");
                for (int i = 0; i < N; i++) {
                    if (changes[i] > 5) {
                        Serial.printf("  GPIO%d: %d 次变化\n", scan_pins[i], changes[i]);
                    }
                }
                Serial.println("  --- 完成 ---");
            }
            break;

        case 'd':
            // 全调试数据输出 (10秒) — GPIO + 编码器 + PID + 速度
            Serial.println("T,E0A,E0B,E1A,E1B,E2A,E2B,enc0,enc1,enc2,rev0,rev1,rev2,PWM0,PWM1,PWM2,FF0,P0,I0,D0,FF1,P1,I1,D1,FF2,P2,I2,D2,tgt0,tgt1,tgt2,act0,act1,act2");
            {
                unsigned long start = millis();
                long dbg_prev_enc[3] = {read_enc(0), read_enc(1), read_enc(2)};
                float dbg_prev_t = millis() / 1000.0f;
                while (millis() - start < 10000) {
                    long c0 = read_enc(0), c1 = read_enc(1), c2 = read_enc(2);
                    float cpr4 = ENCODER_CPR * 4.0f;
                    float circ = 2.0f * PI * WHEEL_RADIUS;
                    float now_t = millis() / 1000.0f;
                    float dt = now_t - dbg_prev_t;
                    if (dt < 0.001f) dt = 0.02f;
                    float act[3];
                    long encs[3] = {c0, c1, c2};
                    for (int i = 0; i < 3; i++) {
                        long delta = encs[i] - (long)dbg_prev_enc[i];
                        act[i] = ((float)delta / cpr4) * circ / dt;
                        dbg_prev_enc[i] = encs[i];
                    }
                    dbg_prev_t = now_t;

                    Serial.printf("%.3f,%d,%d,%d,%d,%d,%d,"
                                  "%ld,%ld,%ld,%.2f,%.2f,%.2f,"
                                  "%d,%d,%d,"
                                  "%.1f,%.1f,%.1f,%.1f,"
                                  "%.1f,%.1f,%.1f,%.1f,"
                                  "%.1f,%.1f,%.1f,%.1f,"
                                  "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                                  (millis() - start) / 1000.0f,
                                  digitalRead(ENC0_A_PIN), digitalRead(ENC0_B_PIN),
                                  digitalRead(ENC1_A_PIN), digitalRead(ENC1_B_PIN),
                                  digitalRead(ENC2_A_PIN), digitalRead(ENC2_B_PIN),
                                  c0, c1, c2,
                                  c0 / cpr4, c1 / cpr4, c2 / cpr4,
                                  pwm[0], pwm[1], pwm[2],
                                  pid[0].getFFTerm(), pid[0].getPTerm(), pid[0].getITerm(), pid[0].getDTerm(),
                                  pid[1].getFFTerm(), pid[1].getPTerm(), pid[1].getITerm(), pid[1].getDTerm(),
                                  pid[2].getFFTerm(), pid[2].getPTerm(), pid[2].getITerm(), pid[2].getDTerm(),
                                  target_speed[0], target_speed[1], target_speed[2],
                                  act[0], act[1], act[2]);
                    check_serial_pid();  // 允许调试流期间调参
                    delayMicroseconds(500);
                }
                Serial.println("--- done ---");
            }
            break;

        case 'v':
            // 持续 GPIO 波形输出 (Serial Studio 格式, 按任意键停止)
            {
                const uint8_t vp[] = {
                    36, 39, 34, 35, 32, 33, 25, 26, 27, 14, 12, 13,
                    23, 22, 21, 19, 18, 5, 17, 16, 4, 2, 15
                };
                const int NV = sizeof(vp) / sizeof(vp[0]);
                const uint8_t input_pins[] = {36, 39, 34, 35, 12, 13, 23, 22, 21, 19, 18, 5, 17, 16, 4, 2, 15};
                for (int i = 0; i < (int)(sizeof(input_pins)/sizeof(input_pins[0])); i++)
                    pinMode(input_pins[i], INPUT);
                Serial.println("GPIO 波形输出中... 发送任意字符停止");
                while (!Serial.available()) {
                    Serial.print('$');
                    for (int i = 0; i < NV; i++) {
                        Serial.print(digitalRead(vp[i]));
                        if (i < NV - 1) Serial.print(',');
                    }
                    Serial.println();
                    delayMicroseconds(500);
                }
                Serial.read();  // 消耗停止字符
                Serial.println("GPIO 波形已停止");
            }
            break;

        default:
            Serial.printf("未知: '%c'  (0/1/2 +/- f/r s a p)\n", c);
            break;
        }
    }

    // PID 控制循环 (20ms = 50Hz)
    static unsigned long pid_last = 0;
    static float act_speed[3] = {0, 0, 0};
    if (pid_mode && millis() - pid_last >= CONTROL_PERIOD_MS) {
        float dt = (float)(millis() - pid_last) / 1000.0f;
        pid_last = millis();

        for (int i = 0; i < 3; i++) {
            long cur = read_enc(i);
            float raw_speed = enc_to_speed(cur - prev_enc[i], dt);
            act_speed[i] = 0.6f * act_speed[i] + 0.4f * raw_speed;  // EMA 滤波
            prev_enc[i] = cur;

            if (target_speed[i] != 0.0f) {
                float out = pid[i].compute(target_speed[i], act_speed[i], dt);
                pwm[i] = constrain((int)out, -255, 255);
                motor[i].setSpeed(pwm[i]);
            }
        }

        // pid_tuner.py / Serial Studio 帧: $tgt,act,pwm,FF,P,I,D (选中电机)
        Serial.printf("$%.4f,%.4f,%d,%.0f,%.0f,%.0f,%.0f\n",
                      target_speed[selected], act_speed[selected], pwm[selected],
                      pid[selected].getFFTerm(), pid[selected].getPTerm(),
                      pid[selected].getITerm(), pid[selected].getDTerm());
    }

    // 开环模式: 每 500ms 打印状态
    if (!pid_mode) {
        static unsigned long print_last = 0;
        bool any_active = false;
        for (int i = 0; i < 3; i++) {
            if (pwm[i] != 0) { any_active = true; break; }
        }
        if (any_active && millis() - print_last > 500) {
            for (int i = 0; i < 3; i++) {
                if (pwm[i] != 0) {
                    long c = read_enc(i);
                    float rev = (float)c / (ENCODER_CPR * 4);
                    Serial.printf("  M%d: PWM=%4d  enc=%ld (%.2f rev)\n", i, pwm[i], c, rev);
                }
            }
            print_last = millis();
        }
    }
}

#endif // MOTOR_TEST_MODE
