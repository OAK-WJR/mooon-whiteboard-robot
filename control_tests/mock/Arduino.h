#pragma once
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#include <vector>
#define ESP_ARDUINO_VERSION_MAJOR 3
#define IRAM_ATTR
#define LOW 0
#define HIGH 1
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define CHANGE 3
#define F(value) value
#define SERIAL_8N1 0x800001c
using String=std::string;
using std::isfinite;
inline std::string operator+(std::string s,int n){return s+std::to_string(n);}
inline uint32_t fakeNow=0;
inline uint32_t fakeSubMicros=0, serialReadDelayMs=0;
inline std::array<int,64> levels{},modes{},duties{};
inline std::array<bool,64> attached{};
inline int failAttachPin=-1,failWritePin=-1,failDetachPin=-1;
struct PinOperation {std::string kind;int pin,value;};
inline std::vector<PinOperation> pinOperations;
inline uint32_t millis(){return fakeNow;}
inline void delay(unsigned ms){fakeNow+=ms;}
inline void delayMicroseconds(unsigned us){fakeSubMicros+=us;fakeNow+=fakeSubMicros/1000;fakeSubMicros%=1000;}
inline void digitalWrite(unsigned p,int value){levels.at(p)=value;pinOperations.push_back({"gpio",int(p),value});}
inline int digitalRead(unsigned p){return levels.at(p);}
inline void pinMode(unsigned p,int mode){modes.at(p)=mode;}
inline std::array<void(*)(),64> interruptHandlers{};
inline int digitalPinToInterrupt(int pin){return pin;}
inline void attachInterrupt(int pin,void(*handler)(),int){interruptHandlers.at(pin)=handler;}
inline void noInterrupts(){} inline void interrupts(){}
inline bool ledcAttach(unsigned p,unsigned,unsigned){
    pinOperations.push_back({"attach",int(p),0});return attached.at(p)=int(p)!=failAttachPin;
}
inline bool ledcWrite(unsigned p,unsigned duty){
    pinOperations.push_back({"write",int(p),int(duty)});
    if(!attached.at(p)||int(p)==failWritePin)return false;
    duties.at(p)=int(duty);return true;
}
inline bool ledcDetach(unsigned p){
    pinOperations.push_back({"detach",int(p),0});
    if(int(p)==failDetachPin)return false;
    attached.at(p)=false;duties.at(p)=0;return true;
}
template<class A,class B,class C> inline A constrain(A v,B lo,C hi){return std::max(A(lo),std::min(A(hi),v));}
struct FakeSerial {
    bool connected=true;
    bool uartMode=false;
    unsigned baud=0;
    int rxPin=-1,txPin=-1,writeCapacity=8192;
    std::deque<int> rx;
    std::string tx;
    void setTxBufferSize(unsigned){} void setRxBufferSize(unsigned){} void setTxTimeoutMs(unsigned){}
    void begin(unsigned value){baud=value;uartMode=false;}
    void begin(unsigned value,uint32_t config,int rx,int tx){
        assert(config==SERIAL_8N1);baud=value;rxPin=rx;txPin=tx;uartMode=true;
    }
    // HardwareSerial bool reports driver installation, not USB host presence.
    explicit operator bool()const{return uartMode||connected;}
    int availableForWrite(){return writeCapacity;} int available(){return int(rx.size());}
    int read(){fakeNow+=serialReadDelayMs;int c=rx.front();rx.pop_front();return c;}
    size_t write(const uint8_t* bytes,size_t n){tx.append(reinterpret_cast<const char*>(bytes),n);return n;}
    void feed(const std::string& text){for(unsigned char c:text)rx.push_back(c);}
    void println(const char* s){tx+=s;tx+='\n';}
    template<class... A> void printf(const char* format,A... args){char out[512];snprintf(out,sizeof(out),format,args...);tx+=out;}
};
inline FakeSerial Serial;
using Print=FakeSerial;
