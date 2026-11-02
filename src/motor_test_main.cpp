/**
 * WhiteBoardBot three-motor test (open-loop / PID / kinematics)
 *
 * Serial commands:
 *   '0'~'2'  - select motor     'c' - toggle open-loop/PID
 *   '+'/'-'  - adjust PWM/speed 'k' - toggle kinematics mode
 *   'f'/'r'  - forward/reverse  'w'/'x' - forward/back (kinematics)
 *   's'      - stop current     'q'/'e' - move left/right (kinematics)
 *   'a'      - stop all         'j'/'l' - turn left/right (kinematics)
 *   'p'      - status  'z' - zero
 */

#ifdef MOTOR_TEST_MODE

#include <Arduino.h>
#include "firmware_config.h"
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

// Encoder counts (volatile, updated in the ISR)
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

static int selected = 0;       // currently selected motor
static int16_t pwm[3] = {0, 0, 0};

// Line buffer (supports multi-character PID commands)
static char cmd_buf[32];
static int cmd_len = 0;

// PID tuning command handling
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
        // Set the absolute target speed of the selected motor (m/s), turns on PID mode automatically
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
        // Incremental speed change (m/s), turns on PID mode automatically
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
        // Stop all
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

// Check serial and handle PID commands (called inside blocking loops)
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
            Serial.printf("  M%d: tgt=%.3f  act=%.3f  PWM=%4d  enc=%ld (%.1f rev)  [FF=%.0f P=%.0f I=%.0f D=%.0f]\n",
                          i, target_speed[i], act,
                          pwm[i], c, rev,
                          pid[i].getFFTerm(), pid[i].getPTerm(),
                          pid[i].getITerm(), pid[i].getDTerm());
            Serial.printf("      sign(PWM/act): %d/%d  %s\n",
                          sign_i(pwm[i]), sign_f(act), sign_match ? "MATCH" : "MISMATCH");
        } else {
            Serial.printf("  M%d: PWM=%4d  enc=%7ld  (%.2f rev)\n", i, pwm[i], c, rev);
        }
    }
    const char* mode = kin_mode ? "kinematics" : (pid_mode ? "PID" : "open-loop");
    if (kin_mode)
        Serial.printf("  Mode: %s  Body: vx=%.2f vy=%.2f ω=%.1f\n",
                      mode, body_cmd.vx, body_cmd.vy, body_cmd.omega);
    else
        Serial.printf("  Mode: %s  Selected: M%d\n", mode, selected);
    Serial.println("──────────────────────────────────");
}

void setup() {
    Serial.begin(115200);
    delay(500);

    // (adc_power_acquire() used to be called here to suppress Errata 3.11 noise on v1 board GPIO36/39;
    //  the v2 board moved ENC1 to GPIO22/23, and this API is gone in the new core)

    Serial.println("\n============================");
    Serial.println("  Three-motor independent test");
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

    // Encoder pins — mapping confirmed by measurement
    pinMode(ENC0_A_PIN, INPUT);        // GPIO34
    pinMode(ENC0_B_PIN, INPUT);        // GPIO35
    pinMode(ENC1_A_PIN, INPUT);        // GPIO36
    pinMode(ENC1_B_PIN, INPUT_PULLUP); // GPIO2 (strapping pin)
    pinMode(ENC2_A_PIN, INPUT_PULLUP); // GPIO19
    pinMode(ENC2_B_PIN, INPUT_PULLUP); // GPIO18

    attachInterrupt(digitalPinToInterrupt(ENC0_A_PIN), enc0_a_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC0_B_PIN), enc0_b_isr, CHANGE);
    // Enc1 (GPIO36/39): interrupts usable once adc_power_acquire() removes the noise
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

    Serial.println("\nCommands: 0/1/2=select motor +/-=adjust f/r=fwd/rev s/a=stop p=status z=zero c=toggle PID");
    Serial.println("PID tuning: P<v> I<v> D<v> F<v> L<v> G=query R=reset");
    Serial.printf("Current: M%d open-loop mode\n\n", selected);
}

void loop() {
    // Serial read: input starting with an uppercase letter is buffered until newline (PID command); others run right away
    while (Serial.available()) {
        char ch = Serial.read();
        // Buffer mode: collecting a multi-character PID command
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
        // Uppercase letter: start buffering a PID command
        if (ch >= 'A' && ch <= 'Z') {
            cmd_buf[0] = ch;
            cmd_len = 1;
            continue;
        }
        // Ignore newlines
        if (ch == '\n' || ch == '\r') continue;
        // Others: run single-character commands right away
        char c = ch;
        switch (c) {
        case '0': case '1': case '2':
            selected = c - '0';
            Serial.printf(">>> Selected M%d (PWM=%d)\n", selected, pwm[selected]);
            break;

        case '+':
            if (pid_mode) {
                target_speed[selected] += 0.05f;
                Serial.printf("M%d target=%.3f m/s\n", selected, target_speed[selected]);
            } else {
                pwm[selected] = constrain(pwm[selected] + 20, -255, 255);
                motor[selected].setSpeed(pwm[selected]);
                Serial.printf("M%d PWM=%d\n", selected, pwm[selected]);
            }
            break;

        case '-':
            if (pid_mode) {
                target_speed[selected] -= 0.05f;
                Serial.printf("M%d target=%.3f m/s\n", selected, target_speed[selected]);
            } else {
                pwm[selected] = constrain(pwm[selected] - 20, -255, 255);
                motor[selected].setSpeed(pwm[selected]);
                Serial.printf("M%d PWM=%d\n", selected, pwm[selected]);
            }
            break;

        case 'f':
            if (pid_mode) {
                target_speed[selected] = 0.20f;
                Serial.printf("M%d target=0.200 m/s\n", selected);
            } else {
                pwm[selected] = 200;
                motor[selected].setSpeed(200);
                Serial.printf("M%d forward PWM=200\n", selected);
            }
            break;

        case 'r':
            if (pid_mode) {
                target_speed[selected] = -0.20f;
                Serial.printf("M%d target=-0.200 m/s\n", selected);
            } else {
                pwm[selected] = -200;
                motor[selected].setSpeed(-200);
                Serial.printf("M%d reverse PWM=-200\n", selected);
            }
            break;

        case 's':
            if (pid_mode) {
                target_speed[selected] = 0;
                pid[selected].reset();
            }
            pwm[selected] = 0;
            motor[selected].stop();
            Serial.printf("M%d stopped\n", selected);
            break;

        case 'a':
            body_cmd = {0, 0, 0};
            for (int i = 0; i < 3; i++) {
                target_speed[i] = 0; pwm[i] = 0;
                motor[i].stop(); pid[i].reset();
            }
            Serial.println("All stopped");
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
            Serial.printf(">>> Switched to %s mode\n", pid_mode ? "PID closed-loop" : "open-loop");
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
                Serial.println(">>> Kinematics mode  w/x=fwd/back q/e=left/right j/l=rotate a=stop");
            else
                Serial.println(">>> Left kinematics mode");
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
            Serial.println("Encoders zeroed - turn one revolution by hand, then press p to see counts");
            break;

        case 'g':
            Serial.println("GPIO polling test (3 s, turn the motor shaft)...");
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
                Serial.printf("  Enc0 A(GPIO%d): %d changes\n", ENC0_A_PIN, changes[0]);
                Serial.printf("  Enc0 B(GPIO%d): %d changes\n", ENC0_B_PIN, changes[1]);
                Serial.printf("  Enc1 A(GPIO%d): %d changes\n", ENC1_A_PIN, changes[2]);
                Serial.printf("  Enc1 B(GPIO%d): %d changes\n", ENC1_B_PIN, changes[3]);
                Serial.printf("  Enc2 A(GPIO%d): %d changes\n", ENC2_A_PIN, changes[4]);
                Serial.printf("  Enc2 B(GPIO%d): %d changes\n", ENC2_B_PIN, changes[5]);
            }
            break;

        case 't':
            Serial.println("Full GPIO scan (5 s, turn the motor shaft)...");
            {
                const uint8_t scan_pins[] = {
                    36, 39, 34, 35, 32, 33, 25, 26, 27, 14, 12, 13,  // left row
                    23, 22, 21, 19, 18, 5, 17, 16, 4, 2, 15           // right row
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
                Serial.println("  --- GPIOs with a signal ---");
                for (int i = 0; i < N; i++) {
                    if (changes[i] > 5) {
                        Serial.printf("  GPIO%d: %d changes\n", scan_pins[i], changes[i]);
                    }
                }
                Serial.println("  --- Done ---");
            }
            break;

        case 'd':
            // Full debug data output (10 s) — GPIO + encoder + PID + speed
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
                    check_serial_pid();  // allow tuning during the debug stream
                    delayMicroseconds(500);
                }
                Serial.println("--- done ---");
            }
            break;

        case 'v':
            // Continuous GPIO waveform output (Serial Studio format, press any key to stop)
            {
                const uint8_t vp[] = {
                    36, 39, 34, 35, 32, 33, 25, 26, 27, 14, 12, 13,
                    23, 22, 21, 19, 18, 5, 17, 16, 4, 2, 15
                };
                const int NV = sizeof(vp) / sizeof(vp[0]);
                const uint8_t input_pins[] = {36, 39, 34, 35, 12, 13, 23, 22, 21, 19, 18, 5, 17, 16, 4, 2, 15};
                for (int i = 0; i < (int)(sizeof(input_pins)/sizeof(input_pins[0])); i++)
                    pinMode(input_pins[i], INPUT);
                Serial.println("GPIO waveform output running... send any character to stop");
                while (!Serial.available()) {
                    Serial.print('$');
                    for (int i = 0; i < NV; i++) {
                        Serial.print(digitalRead(vp[i]));
                        if (i < NV - 1) Serial.print(',');
                    }
                    Serial.println();
                    delayMicroseconds(500);
                }
                Serial.read();  // consume the stop character
                Serial.println("GPIO waveform stopped");
            }
            break;

        default:
            Serial.printf("Unknown: '%c'  (0/1/2 +/- f/r s a p)\n", c);
            break;
        }
    }

    // PID control loop (20ms = 50Hz)
    static unsigned long pid_last = 0;
    static float act_speed[3] = {0, 0, 0};
    if (pid_mode && millis() - pid_last >= CONTROL_PERIOD_MS) {
        float dt = (float)(millis() - pid_last) / 1000.0f;
        pid_last = millis();

        for (int i = 0; i < 3; i++) {
            long cur = read_enc(i);
            float raw_speed = enc_to_speed(cur - prev_enc[i], dt);
            act_speed[i] = 0.6f * act_speed[i] + 0.4f * raw_speed;  // EMA filter
            prev_enc[i] = cur;

            if (target_speed[i] != 0.0f) {
                float out = pid[i].compute(target_speed[i], act_speed[i], dt);
                pwm[i] = constrain((int)out, -255, 255);
                motor[i].setSpeed(pwm[i]);
            }
        }

        // pid_tuner.py / Serial Studio frame: $tgt,act,pwm,FF,P,I,D (selected motor)
        Serial.printf("$%.4f,%.4f,%d,%.0f,%.0f,%.0f,%.0f\n",
                      target_speed[selected], act_speed[selected], pwm[selected],
                      pid[selected].getFFTerm(), pid[selected].getPTerm(),
                      pid[selected].getITerm(), pid[selected].getDTerm());
    }

    // Open-loop mode: print status every 500ms
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
