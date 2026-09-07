#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#define IRAM_ATTR
#define LOW 0
#define HIGH 1
#define OUTPUT 1
#define INPUT_PULLUP 2
#define CHANGE 3
#define portMUX_INITIALIZER_UNLOCKED 0
using portMUX_TYPE = int;
#define portENTER_CRITICAL_ISR(x) ((void)(x))
#define portEXIT_CRITICAL_ISR(x) ((void)(x))
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
inline uint32_t fakeNow=0;
inline int levels[32]={}, modes[32]={}, duties[32]={};
inline bool attached[32]={};
inline int failAttach=-1;
inline bool failWrite=false;
inline unsigned millis(){return fakeNow;}
inline void delay(unsigned ms){fakeNow+=ms;}
inline void delayMicroseconds(unsigned us){fakeNow+=us/1000;}
inline void digitalWrite(unsigned pin,int value){levels[pin]=value;}
inline int digitalRead(unsigned pin){return levels[pin];}
inline void pinMode(unsigned pin,int mode){modes[pin]=mode;}
inline bool ledcAttachChannel(unsigned pin,unsigned,unsigned,unsigned channel){
    return attached[pin]=int(channel)!=failAttach;
}
inline bool ledcWrite(unsigned pin,unsigned duty){
    if(!attached[pin] || failWrite)return false;
    duties[pin]=duty;return true;
}
inline void ledcDetach(unsigned pin){attached[pin]=false;duties[pin]=0;}
inline void attachInterruptArg(unsigned,void(*)(void*),void*,int){}
struct FakeSerial {
    bool connected=true;
    std::deque<int> rx;
    std::string tx;
    void setTxBufferSize(unsigned){}
    void setRxBufferSize(unsigned){}
    void setTxTimeoutMs(unsigned){}
    void begin(unsigned){}
    explicit operator bool()const{return connected;}
    int availableForWrite(){return 4096;}
    int available(){return int(rx.size());}
    int read(){int c=rx.front();rx.pop_front();return c;}
    size_t write(const uint8_t* data,size_t n){tx.append(reinterpret_cast<const char*>(data),n);return n;}
    void feed(const std::string& text){for(unsigned char c:text)rx.push_back(c);}
};
inline FakeSerial Serial;
