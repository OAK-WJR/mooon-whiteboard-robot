// ESP32-C6 motor + encoder full-chain test (display board on adapter board, adapter board on driver board)
// Pin map and pairing: see memory note whiteboardbot-header-pairing (settled 2026-07-03)
//   firmware header M0 -> base H2 = physical motor 1 (ctrl1)
//   firmware header M1 -> base H1 = physical motor 3 (ctrl3)
//   firmware header M2 -> base H3 = physical motor 2 (ctrl2)
// Safety: every spin command has a dead-man switch; auto stop after 800ms by default, max 3000ms
#include <Arduino.h>

static const int PWM_FI[3] = {0, 2, 4};    // GP0/2/4
static const int PWM_BI[3] = {1, 3, 5};    // GP1/3/5
static const int ENC_A[3]  = {16, 18, 20}; // GP16(via 100Ω)/18/20
static const int ENC_B[3]  = {17, 19, 23}; // GP17/19/23
static const char* HEAD_DESC[3] = {
    "M0 header->base H2=motor 1", "M1 header->base H1=motor 3", "M2 header->base H3=motor 2"};

volatile long cnt[3] = {0, 0, 0};

void IRAM_ATTR isr0() { cnt[0] += (digitalRead(16) == digitalRead(17)) ? 1 : -1; }
void IRAM_ATTR isr1() { cnt[1] += (digitalRead(18) == digitalRead(19)) ? 1 : -1; }
void IRAM_ATTR isr2() { cnt[2] += (digitalRead(20) == digitalRead(23)) ? 1 : -1; }

static uint32_t stop_at[3] = {0, 0, 0};
static bool telem = true;

static void motorSet(int i, int duty) {
    duty = constrain(duty, -255, 255);
    if (duty >= 0) { ledcWrite(PWM_FI[i], duty); ledcWrite(PWM_BI[i], 0); }
    else           { ledcWrite(PWM_FI[i], 0);    ledcWrite(PWM_BI[i], -duty); }
}

static void stopAll() {
    for (int i = 0; i < 3; i++) { motorSet(i, 0); stop_at[i] = 0; }
}

static void printCounts() {
    Serial.printf("ENC 0=%ld 1=%ld 2=%ld\n", cnt[0], cnt[1], cnt[2]);
}

void setup() {
    Serial.begin(115200);
    delay(300);
    for (int i = 0; i < 3; i++) {
        ledcAttach(PWM_FI[i], 20000, 8);
        ledcAttach(PWM_BI[i], 20000, 8);
        motorSet(i, 0);
        pinMode(ENC_A[i], INPUT_PULLUP);
        pinMode(ENC_B[i], INPUT_PULLUP);
    }
    attachInterrupt(digitalPinToInterrupt(16), isr0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(18), isr1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(20), isr2, CHANGE);
    Serial.println("\n=== C6 MOTOR/ENCODER TEST READY ===");
    for (int i = 0; i < 3; i++) Serial.printf("  %s\n", HEAD_DESC[i]);
    Serial.println("Commands: w <i> <duty> [ms] | x=stop | s=counts | z=zero | t=telemetry on/off");
}

void loop() {
    static char buf[48];
    static int len = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (len > 0) {
                buf[len] = '\0'; len = 0;
                if (buf[0] == 'w') {
                    int i = -1, duty = 0, ms = 800;
                    sscanf(buf + 1, "%d %d %d", &i, &duty, &ms);
                    if (i >= 0 && i <= 2) {
                        ms = constrain(ms, 50, 3000);
                        motorSet(i, duty);
                        stop_at[i] = millis() + ms;
                        Serial.printf("RUN %s duty=%d ms=%d\n", HEAD_DESC[i], duty, ms);
                    }
                } else if (buf[0] == 'p') {
                    Serial.printf("PINS A0(16)=%d B0(17)=%d A1(18)=%d B1(19)=%d A2(20)=%d B2(23)=%d\n",
                        digitalRead(16),digitalRead(17),digitalRead(18),digitalRead(19),digitalRead(20),digitalRead(23));
                } else if (buf[0] == 'x') { stopAll(); Serial.println("STOP all");
                } else if (buf[0] == 's') { printCounts();
                } else if (buf[0] == 'z') { cnt[0]=cnt[1]=cnt[2]=0; Serial.println("counts zeroed");
                } else if (buf[0] == 't') { telem = !telem; Serial.printf("telem %d\n", telem);
                }
            }
        } else if (len < (int)sizeof(buf) - 1) buf[len++] = c;
    }
    for (int i = 0; i < 3; i++) {
        if (stop_at[i] && millis() > stop_at[i]) { motorSet(i, 0); stop_at[i] = 0; Serial.printf("AUTOSTOP M%d\n", i); }
    }
    static uint32_t last = 0;
    if (telem && millis() - last > 500) { printCounts(); last = millis(); }
}
