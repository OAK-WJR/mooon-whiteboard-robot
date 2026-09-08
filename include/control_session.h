#pragma once
#include <cstdint>
#include <cstddef>

namespace control {
enum class Link : uint8_t { None, USB, WiFi, BLE };
// The lease is a command deadman, not a hardware power disconnect.
class Session {
public:
    static constexpr uint32_t timeoutMs=750;
    Link link=Link::None;
    uint64_t peer=0;
    uint32_t lease=0,sequence=0,lastSeen=0;
    void clear(){link=Link::None;peer=0;lease=0;sequence=0;}
    bool expired(uint32_t now)const{return link!=Link::None && uint32_t(now-lastSeen)>=timeoutMs;}
    bool owns(Link l,uint64_t p)const{return l==link && p==peer && link!=Link::None;}
    bool claim(Link l,uint64_t p,uint32_t token,uint32_t seq,uint32_t now){
        if(link!=Link::None || !token || !seq)return false;
        link=l;peer=p;lease=token;sequence=seq;lastSeen=now;return true;
    }
    const char* accept(Link l,uint64_t p,uint32_t token,uint32_t seq,uint32_t now){
        if(expired(now)){clear();return "LEASE_EXPIRED";}
        if(!owns(l,p)||token!=lease)return "NOT_OWNER";
        if(seq<=sequence)return "REPLAY_OR_REORDER";
        sequence=seq;lastSeen=now;return nullptr;
    }
};

template<size_t N> class Framer {
    char data_[N]{};
    size_t length_=0;
    bool discard_=false;
public:
    enum Result { More, Line, Overflow };
    const char* data()const{return data_;}
    void reset(){length_=0;discard_=false;data_[0]=0;}
    Result feed(unsigned char c){
        if(c=='\n'){
            if(discard_){reset();return More;}
            if(!length_)return More;
            data_[length_]=0;length_=0;return Line;
        }
        if(discard_)return More;
        if(c<32 || c>126 || length_>=N-1){discard_=true;length_=0;return Overflow;}
        data_[length_++]=char(c);return More;
    }
};
}
