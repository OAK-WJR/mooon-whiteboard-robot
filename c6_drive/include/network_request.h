#pragma once
#include <ArduinoJson.h>
#include "network_config.h"

namespace drive {
template<size_t N> inline bool networkString(JsonVariantConst value,char (&out)[N],bool optional=false){
    if(value.isNull()&&optional)return true;
    if(!value.is<const char*>())return false;
    JsonString s=value.as<JsonString>();
    if(s.size()>=N||std::memchr(s.c_str(),0,s.size()))return false;
    std::memcpy(out,s.c_str(),s.size());return true;
}

// Both transports use the same validation. USB's bounded 512-byte protocol
// supports personal Wi-Fi only; enterprise certificates use the web form.
inline const char* parseNetworkRequest(JsonVariantConst d,bool usb,
                                      NetworkCredentials& next,bool& forget,
                                      const NetworkCredentials* saved=nullptr){
    next={};forget=false;
    if(!d.is<JsonObjectConst>())return "BAD_JSON";
    if(!d["forget"].isNull()&&!d["forget"].is<bool>())return "INVALID_WIFI";
    forget=d["forget"].is<bool>()&&d["forget"].as<bool>();
    if(forget)return nullptr;
    if(!d["auth"].isNull()&&!d["auth"].is<const char*>())return "INVALID_AUTH";
    const char* auth=d["auth"]|"personal";
    if(!std::strcmp(auth,"personal"))next.auth=NetworkCredentials::Personal;
    else if(!std::strcmp(auth,"peap"))next.auth=NetworkCredentials::Peap;
    else if(!std::strcmp(auth,"ttls_mschapv2"))next.auth=NetworkCredentials::TtlsMschapv2;
    else if(!std::strcmp(auth,"ttls_pap"))next.auth=NetworkCredentials::TtlsPap;
    else return "INVALID_AUTH";
    if(usb&&next.auth!=NetworkCredentials::Personal)return "USB_PERSONAL_WIFI_ONLY";
    if(!d["reuse_saved_password"].isNull()&&!d["reuse_saved_password"].is<bool>())return "INVALID_WIFI";
    bool reuse=d["reuse_saved_password"]|false;
    if(!networkString(d["ssid"],next.ssid)||!networkString(d["password"],next.password,reuse)||
       !networkString(d["username"],next.username,true)||!networkString(d["identity"],next.identity,true)||
       !networkString(d["server_name"],next.server,true)||!networkString(d["ca_pem"],next.ca,true))return "INVALID_WIFI";
    if(reuse){
        if(next.auth!=NetworkCredentials::Personal||!saved||!validNetwork(*saved)||
           saved->auth!=next.auth||std::strcmp(saved->ssid,next.ssid)||next.password[0])return "SAVED_NETWORK_MISMATCH";
        std::memcpy(next.password,saved->password,sizeof(next.password));
    }
    if(!validNetwork(next))return "INVALID_WIFI";
    return nullptr;
}
inline bool sameNetwork(const NetworkCredentials& a,const NetworkCredentials& b){
    if(!validNetwork(a)||!validNetwork(b)||a.auth!=b.auth||std::strcmp(a.ssid,b.ssid)||std::strcmp(a.password,b.password))return false;
    if(a.auth==NetworkCredentials::Personal)return true;
    return !std::strcmp(a.username,b.username)&&!std::strcmp(a.identity,b.identity)&&
        !std::strcmp(a.server,b.server)&&!std::strcmp(a.ca,b.ca);
}
}
