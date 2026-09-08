// USB diagnostic protocol v1. All motion is opt-in, bounded, and requires a
// current-limited bench supply. No encoder pin is ever configured as an output.
#include <Arduino.h>
#include <esp_system.h>
#include <stdlib.h>
#include <string.h>
#include "safety.h"

#ifndef DIAG_TOP_V2
#define DIAG_TOP_V2 0
#endif
struct Pins { uint8_t fi, bi, a, b, physical; const char* header; const char* bottom; };
static constexpr Pins pins[3] = {
    {0,1,DIAG_TOP_V2 ? 7 : 16,17,1,"M0","H2"},
    {2,3,18,19,3,"M1","H1"},
    {4,5,20,23,2,"M2","H3"}
};
static diag::Safety safety;
static bool pwm[3][2] = {};
static bool pwmReady = false;
static int commanded[3] = {};
static volatile uint32_t edgesA[3] = {}, edgesB[3] = {}, invalid[3] = {};
static volatile int32_t count[3] = {};
static volatile uint8_t previous[3] = {};
static portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t frame = 0, txDropped = 0, sessionNonce = 0;
static bool receivingOverflow = false;
static char input[128];
static size_t inputLength = 0;
static bool rebootPending = false;
static uint32_t rebootRequestedAt = 0;

static void IRAM_ATTR encoder(void* data) {
    const int i = int(reinterpret_cast<uintptr_t>(data));
    const uint8_t state = (digitalRead(pins[i].a) << 1) | digitalRead(pins[i].b);
    portENTER_CRITICAL_ISR(&encoderMux);
    const uint8_t old = previous[i], changed = old ^ state;
    if (changed & 2) edgesA[i] = edgesA[i] + 1;
    if (changed & 1) edgesB[i] = edgesB[i] + 1;
    if (changed == 3) invalid[i] = invalid[i] + 1;
    else count[i] = int32_t(uint32_t(count[i]) + diag::quadrature(old,state));
    previous[i] = state;
    portEXIT_CRITICAL_ISR(&encoderMux);
}

static void allLow() {
    for (int i=0;i<3;++i) {
        const uint8_t outputs[2] = {pins[i].fi,pins[i].bi};
        for (int k=0;k<2;++k) {
            if (pwm[i][k]) {
                if (!ledcWrite(outputs[k],0)) {
                    ledcDetach(outputs[k]); pwm[i][k]=false; pwmReady=false;
                    digitalWrite(outputs[k],LOW); pinMode(outputs[k],OUTPUT);
                }
            } else digitalWrite(outputs[k],LOW);
        }
        commanded[i] = 0;
    }
    // The current v2 draft's boost EN is not a true motor power disconnect.
    if (DIAG_TOP_V2) digitalWrite(21,LOW);
}
static void stop(const char* why) { allLow(); safety.stop(millis(),why); }

static void send(const char* data) {
    const size_t n = strlen(data);
    if (!Serial) return;
    if (Serial.availableForWrite() < int(n)) { ++txDropped; return; }
    if (Serial.write(reinterpret_cast<const uint8_t*>(data),n) != n) ++txDropped;
}
static void ack(uint32_t id, const char* error=nullptr) {
    char buf[200];
    snprintf(buf,sizeof(buf),"{\"type\":\"ack\",\"id\":%lu,\"ok\":%s,\"error\":\"%s\",\"session\":%lu}\n",
             (unsigned long)id,error?"false":"true",error?error:"",(unsigned long)sessionNonce);
    send(buf);
}
static void status() {
    uint32_t a[3],b[3],bad[3]; int32_t c[3];
    portENTER_CRITICAL(&encoderMux);
    for(int i=0;i<3;++i) { a[i]=edgesA[i]; b[i]=edgesB[i]; bad[i]=invalid[i]; c[i]=count[i]; }
    portEXIT_CRITICAL(&encoderMux);
    char buf[1900];
    int n=snprintf(buf,sizeof(buf),
      "{\"type\":\"status\",\"protocol\":1,\"firmware\":\"wbb-usb-diag-1\",\"profile\":\"%s\","
      "\"session\":%lu,\"seq\":%lu,\"uptime_ms\":%lu,\"reset_reason\":%d,\"pwm_ready\":%s,"
      "\"armed\":%s,\"running\":%s,\"reason\":\"%s\",\"tx_dropped\":%lu,"
      "\"motor_voltage_mv\":null,\"motor_current_ma\":null,\"rail_3v3_mv\":null,"
      "\"voltage_feedback\":false,\"current_feedback\":false,\"power_disconnect\":false,"
      "\"usb_reboot\":true,\"reboot_pending\":%s,\"motors\":[",
      DIAG_TOP_V2?"top_board_v2":"adapter",(unsigned long)sessionNonce,(unsigned long)frame++,
      (unsigned long)millis(),int(esp_reset_reason()),pwmReady?"true":"false",safety.armed?"true":"false",
      safety.running?"true":"false",safety.reason,(unsigned long)txDropped,rebootPending?"true":"false");
    for(int i=0;i<3;++i) n+=snprintf(buf+n,sizeof(buf)-n,
      "%s{\"index\":%d,\"physical\":%u,\"header\":\"%s\",\"bottom\":\"%s\",\"fi\":%u,\"bi\":%u,"
      "\"enc_a\":%u,\"enc_b\":%u,\"a\":%d,\"b\":%d,\"pwm_fi\":%s,\"pwm_bi\":%s,"
      "\"command\":%d,\"count\":%ld,\"edges_a\":%lu,\"edges_b\":%lu,\"invalid\":%lu}",
      i?",":"",i,pins[i].physical,pins[i].header,pins[i].bottom,pins[i].fi,pins[i].bi,pins[i].a,pins[i].b,
      digitalRead(pins[i].a),digitalRead(pins[i].b),pwm[i][0]?"true":"false",pwm[i][1]?"true":"false",
      commanded[i],(long)c[i],(unsigned long)a[i],(unsigned long)b[i],(unsigned long)bad[i]);
    snprintf(buf+n,sizeof(buf)-n,"]}\n"); send(buf);
}
static bool integer(const char* text, long& result) {
    if (!text || !*text || strlen(text)>10) return false;
    char* end=nullptr; result=strtol(text,&end,10);
    return end && *end=='\0' && result>=-2000000000L && result<=2000000000L;
}
static void command(char* line) {
    char* tokens[8]; size_t n=0; char* save=nullptr;
    for(char* t=strtok_r(line," \t",&save);t;t=strtok_r(nullptr," \t",&save)) {
        if(n==8) { stop("BAD_COMMAND"); ack(0,"TOO_MANY_FIELDS"); return; }
        tokens[n++]=t;
    }
    long id=0;
    if(n<2 || !integer(tokens[0],id) || id<1) { stop("BAD_COMMAND"); ack(0,"BAD_ID"); return; }
    const char* op=tokens[1];
    if(n==2 && !strcmp(op,"stop")) { stop("USER_STOP"); ack(id); status(); return; }
    if(n==2 && (!strcmp(op,"hello") || !strcmp(op,"status"))) { ack(id); status(); return; }
    if(rebootPending) { allLow(); ack(id,"REBOOT_PENDING"); return; }
    if(n==2 && !strcmp(op,"reboot")) {
        stop("REBOOT_REQUESTED");
        rebootRequestedAt=millis(); rebootPending=true;
        ack(id); status(); return;
    }
    if(n==2 && !strcmp(op,"ping")) { safety.lastPing=millis(); ack(id); return; }
    // A missing sensor is never reported as 0V / 0A or as a passing check.
    if(!strcmp(op,"arm")) { stop("NO_POWER_FEEDBACK"); ack(id,"NO_POWER_FEEDBACK"); return; }
    long arg[6]={};
    for(size_t i=2;i<n;++i) if(!integer(tokens[i],arg[i-2])) { stop("BAD_COMMAND"); ack(id,"BAD_NUMBER"); return; }
    if(n==5 && !strcmp(op,"armbench")) {
        if(DIAG_TOP_V2) { stop("BOARD_POWER_UNREVIEWED"); ack(id,"BOARD_POWER_UNREVIEWED"); return; }
        const char* err=safety.armBench(millis(),arg[0],arg[1],arg[2],pwmReady);
        if(err) stop(err);
        ack(id,err); return;
    }
    if(n==6 && !strcmp(op,"pulse")) {
        const char* err=safety.pulse(millis(),arg[0],arg[1],arg[2],arg[3]);
        if(err) { stop(err); ack(id,err); return; }
        allLow();
        // 3ms minimum after driver VCC is valid, per RZ7889-MS datasheet.
        // Boost EN stays low in bench mode: feed the motor rail externally.
        delayMicroseconds(3000);
        bool good=true;
        for(int i=0;i<3;++i) if(arg[0]==3 || arg[0]==i) {
            good=ledcWrite(arg[1]>0?pins[i].fi:pins[i].bi,arg[2]) && good;
            commanded[i]=arg[1]*arg[2];
        }
        if(!good) { stop("PWM_WRITE"); pwmReady=false; ack(id,"PWM_WRITE"); }
        else ack(id);
        return;
    }
    stop("BAD_COMMAND"); ack(id,"BAD_COMMAND");
}

void setup() {
    rebootPending=false;
    receivingOverflow=false; inputLength=0;
    // Safe levels precede USB startup, allocation and printing.
    if(DIAG_TOP_V2) { digitalWrite(21,LOW); pinMode(21,OUTPUT); }
    for(const auto& p:pins) {
        digitalWrite(p.fi,LOW); digitalWrite(p.bi,LOW);
        pinMode(p.fi,OUTPUT); pinMode(p.bi,OUTPUT);
        pinMode(p.a,INPUT_PULLUP); pinMode(p.b,INPUT_PULLUP);
    }
    Serial.setTxBufferSize(4096); Serial.setRxBufferSize(512);
    Serial.begin(115200); Serial.setTxTimeoutMs(2);
    sessionNonce=esp_random();
    pwmReady=true;
    for(int i=0;i<3;++i) {
        pwm[i][0]=ledcAttachChannel(pins[i].fi,20000,8,2*i);
        pwm[i][1]=ledcAttachChannel(pins[i].bi,20000,8,2*i+1);
        pwmReady=pwmReady && pwm[i][0] && pwm[i][1];
        previous[i]=(digitalRead(pins[i].a)<<1)|digitalRead(pins[i].b);
        attachInterruptArg(pins[i].a,encoder,reinterpret_cast<void*>(uintptr_t(i)),CHANGE);
        attachInterruptArg(pins[i].b,encoder,reinterpret_cast<void*>(uintptr_t(i)),CHANGE);
    }
    stop(pwmReady?"POWER_ON":"PWM_INIT");
}
void loop() {
    // Stop before acknowledging, then allow a bounded interval for USB TX.
    // No flush() or host wait can postpone the restart indefinitely.
    if(rebootPending && uint32_t(millis()-rebootRequestedAt)>=100) {
        allLow(); esp_restart(); return;
    }
    if(safety.tick(millis(),bool(Serial))) allLow();
    // Limit work per loop even if the host continuously floods the USB port.
    for(int budget=0;budget<64 && Serial.available();++budget) {
        const int c=Serial.read();
        if(c=='\n' || c=='\r') {
            if(receivingOverflow) { stop("RX_OVERFLOW"); ack(0,"RX_OVERFLOW"); }
            else if(inputLength) { input[inputLength]=0; command(input); }
            inputLength=0; receivingOverflow=false;
            if(safety.tick(millis(),bool(Serial))) allLow();
        } else if(c<32 || c>126 || inputLength>=sizeof(input)-1) {
            if(!receivingOverflow) stop("RX_OVERFLOW");
            receivingOverflow=true;
        }
        else if(!receivingOverflow) input[inputLength++]=char(c);
    }
    if(safety.tick(millis(),bool(Serial))) allLow();
    static uint32_t lastStatus=0;
    if(uint32_t(millis()-lastStatus)>=100) { lastStatus=millis(); status(); }
    delay(1);
}
