#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cmath>

namespace drive {
struct NetworkCredentials {
    enum Auth:uint8_t {Personal,Peap,TtlsMschapv2,TtlsPap};
    uint8_t version=2,auth=Personal;
    char ssid[33]={},password[65]={};
    char username[65]={},identity[65]={},server[254]={},ca[4097]={};
};
// Networks explicitly selected by the owner for direct control-page access.
inline bool passwordlessLocalNetwork(const NetworkCredentials& c){
    return c.auth==NetworkCredentials::Personal&&c.password[0]!=0&&
           (std::strcmp(c.ssid,"Owner Hotspot")==0||std::strcmp(c.ssid,"Home Network")==0);
}
inline bool validNetwork(const char* ssid,size_t sn,const char* password,size_t pn){
    if(!ssid||!password||sn<1||sn>32||pn>64||std::memchr(ssid,0,sn)||std::memchr(password,0,pn))return false;
    if(pn!=0&&pn<8)return false;
    if(pn==64)for(size_t i=0;i<pn;i++)
        if(!((password[i]>='0'&&password[i]<='9')||(password[i]>='a'&&password[i]<='f')||(password[i]>='A'&&password[i]<='F')))return false;
    return true;
}
inline bool validNetwork(const NetworkCredentials& c){
    auto* s=static_cast<const char*>(std::memchr(c.ssid,0,sizeof(c.ssid)));
    auto* p=static_cast<const char*>(std::memchr(c.password,0,sizeof(c.password)));
    if(c.version!=2||!s||!p)return false;
    if(c.auth==NetworkCredentials::Personal)return validNetwork(c.ssid,s-c.ssid,c.password,p-c.password);
    if(c.auth>NetworkCredentials::TtlsPap||s==c.ssid||p==c.password)return false;
    auto* user=static_cast<const char*>(std::memchr(c.username,0,sizeof(c.username)));
    auto* identity=static_cast<const char*>(std::memchr(c.identity,0,sizeof(c.identity)));
    auto* domain=static_cast<const char*>(std::memchr(c.server,0,sizeof(c.server)));
    auto* ca=static_cast<const char*>(std::memchr(c.ca,0,sizeof(c.ca)));
    if(!user||user==c.username||!identity||!domain||domain-c.server<3||!ca)return false;
    bool dot=false;size_t label=0;
    for(const char* q=c.server;q<domain;q++){
        char ch=*q;
        if(ch=='.'){
            if(!label||q[-1]=='-')return false;
            dot=true;label=0;continue;
        }
        if(!((ch>='a'&&ch<='z')||(ch>='A'&&ch<='Z')||(ch>='0'&&ch<='9')||ch=='-'))return false;
        if(!label&&ch=='-')return false;
        if(++label>63)return false;
    }
    return dot&&label&&domain[-1]!='-'&&std::strstr(c.ca,"-----BEGIN CERTIFICATE-----")&&
           std::strstr(c.ca,"-----END CERTIFICATE-----");
}
inline bool canConfigureNetwork(bool armed,bool ready,uint32_t controlAge,const float speed[3]){
    if(armed||!ready||controlAge>40)return false;
    for(int i=0;i<3;i++)if(!std::isfinite(speed[i])||std::fabs(speed[i])>=30)return false;
    return true;
}
// All policy calls run in the main loop, never in asynchronous Wi-Fi callbacks.
// No network retry can re-arm the robot or restart a command lease.
struct NetworkPolicy {
    enum Action {None,Connect,Disconnect};
    bool configured=false,pending=false,connecting=false,connected=false,linkLost=false;
    uint32_t changed=0,attempt=0,connectTimeout=15000;
    unsigned attempts=0,attemptLimit=0;
    void configure(uint32_t now,bool enabled,bool enterprise=false){
        configured=enabled;pending=true;connecting=false;changed=now;
        attempts=0;attemptLimit=enterprise?3:0;connectTimeout=enterprise?30000:15000;
    }
    bool blocksArm()const{return pending||connecting;}
    bool exhausted()const{return attemptLimit&&attempts>=attemptLimit&&!connecting&&!connected;}
    Action tick(uint32_t now,bool link,bool armed){
        linkLost=connected&&!link;connected=link;
        if(pending){
            if(armed||uint32_t(now-changed)<250)return None;
            pending=false;connecting=configured;attempt=now;if(configured)++attempts;
            return configured?Connect:Disconnect;
        }
        if(link){connecting=false;attempt=now;attempts=0;return None;}
        if(connecting){
            if(uint32_t(now-attempt)<connectTimeout)return None;
            connecting=false;attempt=now;return Disconnect;
        }
        if(configured&&!armed&&!exhausted()&&uint32_t(now-attempt)>=30000){connecting=true;attempt=now;++attempts;return Connect;}
        return None;
    }
};
}
