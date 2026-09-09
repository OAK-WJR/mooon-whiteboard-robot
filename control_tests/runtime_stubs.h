#pragma once
// macOS-native test adapter, not firmware code and not a radio/MCU emulator.
#include "mock/Arduino.h"
#include <ArduinoJson.h>
#include <CommonCrypto/CommonHMAC.h>
#include <atomic>
#include "control_session.h"
#ifndef CONTROL_PERIOD_MS
constexpr unsigned CONTROL_PERIOD_MS=20;
#endif
inline unsigned restartCalls=0,watchdogFeeds=0;
inline uint32_t randomState=1;
constexpr int ESP_RST_SW=3;
inline int fakeResetReason=1;
inline int esp_reset_reason(){return fakeResetReason;}
inline uint32_t esp_random(){return randomState=randomState*1664525u+1013904223u;}
inline void esp_restart(){++restartCalls;}
struct FakeESP {uint64_t getEfuseMac(){return 0x123456789abcULL;}};
inline FakeESP ESP;
using esp_err_t=int;
constexpr int ESP_OK=0,ESP_ERR_INVALID_STATE=-1;
struct esp_task_wdt_config_t{unsigned timeout_ms,idle_core_mask;bool trigger_panic;};
inline int watchdogResult=ESP_OK;
inline int watchdogFeedResult=ESP_OK;
inline unsigned watchdogAdds=0,nvsReadDelayMs=0;
inline std::vector<unsigned> watchdogTimeouts;
inline int esp_task_wdt_reconfigure(esp_task_wdt_config_t* c){watchdogTimeouts.push_back(c->timeout_ms);return watchdogResult;}
inline int esp_task_wdt_init(esp_task_wdt_config_t*){return watchdogResult;}
inline int esp_task_wdt_add(void*){++watchdogAdds;return watchdogResult;}
inline int esp_task_wdt_reset(){++watchdogFeeds;return watchdogFeedResult;}
inline std::string storedConfig="{}";
inline uint32_t hmacDelayMs=0;
inline bool nvsOpen=true,nvsWrite=true;
struct Preferences {
    bool begin(const char*,bool){return nvsOpen;}
    void end(){}
    size_t putString(const char*,const std::string& value){if(!nvsWrite)return 0;storedConfig=value;return value.size();}
    std::string getString(const char*,const char*){fakeNow+=nvsReadDelayMs;return storedConfig;}
};
#ifdef WBB_CONTROL_TRANSPORT_NATIVE_TEST
#include "../src/control_transport.h"
#else
struct FakeIP {std::string toString(){return "192.0.2.1";}};
struct FakeWiFi {FakeIP localIP(){return {};} int RSSI(){return -50;}};
inline FakeWiFi WiFi;
#endif
constexpr int MBEDTLS_MD_SHA256=1;
inline const int* mbedtls_md_info_from_type(int){static int type=1;return &type;}
inline int mbedtls_md_hmac(const int*,const uint8_t* key,size_t keyLen,const uint8_t* data,size_t n,uint8_t* out){
    fakeNow+=hmacDelayMs;
    CCHmac(kCCHmacAlgSHA256,key,keyLen,data,n,out);return 0;
}
class RobotController {
public:
    bool moving=false,pwmReady=true;
    unsigned stops=0,updates=0;
    float lastDt=0;
    std::string lastCommand,state="idle";
    void begin(){moving=false;state="idle";lastDt=0;updates=0;}
    void handleCommand(JsonDocument& d){
        lastCommand=d["cmd"]|"";
        if(lastCommand=="stop"){moving=false;++stops;state="idle";}
        else if(lastCommand=="area")state="ready";
    }
    void update(float dt){lastDt=dt;++updates;}
    void fillStatus(JsonDocument& d){
        d["state"]=state;d["x"]=0.0f;d["y"]=0.0f;d["theta"]=0.0f;d["progress"]=0;
        d["wp"]=0;d["wp_total"]=0;d["pwm_ready"]=pwmReady;
        auto ready=d["motor_pwm_ready"].to<JsonArray>();auto counts=d["encoder_counts"].to<JsonArray>();
        auto speed=d["wheel_speed_mps"].to<JsonArray>();auto pwm=d["pwm_command"].to<JsonArray>();
        auto physical=d["physical_motors"].to<JsonArray>();auto headers=d["header_order"].to<JsonArray>();
        auto faults=d["motor_pwm_fault"].to<JsonArray>();
        for(int i=0;i<3;++i){ready.add(pwmReady);counts.add(-2147483647);speed.add(-1.25f);pwm.add(moving?128:0);physical.add(i+1);}
        for(int i=0;i<3;++i)faults.add(7);
        for(const char* h:{"M0","M2","M1"})headers.add(h);
    }
};
#ifndef WBB_CONTROL_TRANSPORT_NATIVE_TEST
class ControlTransport {
public:
    static constexpr size_t maxLine=1536;
    using Receiver=void(*)(control::Link,uint64_t,const uint8_t*,size_t);
    struct Packet{control::Link link;uint64_t peer;std::string text;};
    uint32_t dropped=0,reconnects=0;
    bool wifiUp=false,bleReady=true,overflow=false,beginResult=true;
    std::atomic<bool> bleUp{false};
    std::atomic<uint32_t> notifyFailures{0};
    std::vector<Packet> sent;
    std::deque<Packet> incoming;
    bool begin(const char*,const char*,const char*){return beginResult;}
    bool alive(control::Link l,uint64_t)const{
        return l==control::Link::USB?bool(Serial):(l==control::Link::WiFi?wifiUp:bleUp.load());
    }
    bool takeOverflow(){bool x=overflow;overflow=false;return x;}
    bool txPending(control::Link,uint64_t)const{return false;}
    bool send(control::Link l,uint64_t p,const char* text){
        if(!alive(l,p)){++dropped;return false;}
        sent.push_back({l,p,text});return true;
    }
    void poll(Receiver f){
        while(!incoming.empty()){
            auto p=incoming.front();incoming.pop_front();
            f(p.link,p.peer,reinterpret_cast<const uint8_t*>(p.text.data()),p.text.size());
        }
    }
};
#endif
