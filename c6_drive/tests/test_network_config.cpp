#include "network_config.h"
#include <cassert>
#include <cstdio>
#include <limits>
#include <initializer_list>
int main(){
    drive::NetworkCredentials hotspot;
    std::strcpy(hotspot.ssid,"Owner Hotspot");
    assert(!drive::passwordlessLocalNetwork(hotspot)); // An open SSID must not qualify.
    std::strcpy(hotspot.password,"test-only-hotspot-password");
    assert(drive::passwordlessLocalNetwork(hotspot));
    std::strcpy(hotspot.ssid,"Home Network");
    assert(drive::passwordlessLocalNetwork(hotspot));
    for(auto auth:{hotspot.Peap,hotspot.TtlsMschapv2,hotspot.TtlsPap}){
        hotspot.auth=auth;assert(!drive::passwordlessLocalNetwork(hotspot));
    }
    hotspot.auth=hotspot.Personal;std::strcpy(hotspot.ssid,"School-WiFi");
    assert(!drive::passwordlessLocalNetwork(hotspot));
    std::strcpy(hotspot.ssid,"Another shared network");
    assert(!drive::passwordlessLocalNetwork(hotspot));
    std::strcpy(hotspot.ssid,"homenetwork");
    assert(!drive::passwordlessLocalNetwork(hotspot));
    drive::NetworkCredentials c;
    assert(!drive::validNetwork(c));
    std::strcpy(c.ssid,"Home 2.4G");std::strcpy(c.password,"test-only-password");
    assert(drive::validNetwork(c));
    c.password[0]=0;assert(drive::validNetwork(c));
    assert(!drive::validNetwork("a\0b",3,"12345678",8));
    assert(!drive::validNetwork("home",4,"short",5));
    std::memset(c.password,'g',64);c.password[64]=0;assert(!drive::validNetwork(c));
    std::memset(c.password,'a',64);assert(drive::validNetwork(c));
    c.ssid[32]='x';std::memset(c.ssid,'x',32);assert(!drive::validNetwork(c));
    c={};c.auth=c.Peap;std::strcpy(c.ssid,"TestSchool");std::strcpy(c.username,"test-user");std::strcpy(c.password,"short");
    assert(!drive::validNetwork(c)); // Account/password alone cannot disable server verification.
    std::strcpy(c.server,"radius.school.example");
    std::strcpy(c.ca,"-----BEGIN CERTIFICATE-----\ntest-only-format-fixture\n-----END CERTIFICATE-----");
    assert(drive::validNetwork(c)); // PEM structure only; runtime also parses X.509 before saving.
    for(auto auth:{c.Peap,c.TtlsMschapv2,c.TtlsPap}){c.auth=auth;assert(drive::validNetwork(c));}
    c.auth=99;assert(!drive::validNetwork(c));c.auth=c.Peap;
    std::strcpy(c.server,"radius.school.example/anything");assert(!drive::validNetwork(c));
    std::strcpy(c.server,"*.school.example");assert(!drive::validNetwork(c));
    std::strcpy(c.server,"radius..example");assert(!drive::validNetwork(c));
    std::strcpy(c.server,"radius.school.example");c.username[0]=0;assert(!drive::validNetwork(c));
    float speed[3]={};assert(drive::canConfigureNetwork(false,true,10,speed));
    assert(!drive::canConfigureNetwork(true,true,10,speed));
    assert(!drive::canConfigureNetwork(false,true,50,speed));
    speed[0]=std::numeric_limits<float>::quiet_NaN();assert(!drive::canConfigureNetwork(false,true,10,speed));
    drive::NetworkPolicy p;
    assert(p.tick(50000,false,false)==p.None);
    p.configure(50000,true);assert(p.blocksArm());
    assert(p.tick(50249,false,false)==p.None);
    assert(p.tick(50250,false,true)==p.None); // Never reconfigure a running robot.
    assert(p.tick(50251,false,false)==p.Connect);assert(p.blocksArm());
    assert(p.tick(51000,true,false)==p.None);assert(!p.blocksArm());
    assert(p.tick(51500,false,true)==p.None);assert(p.linkLost);
    assert(p.tick(90000,false,true)==p.None); // No reconnect/channel scan during motion.
    assert(p.tick(90001,false,false)==p.Connect);
    assert(p.tick(105000,false,false)==p.None);
    assert(p.tick(105001,false,false)==p.Disconnect);assert(!p.blocksArm());
    assert(p.tick(134999,false,false)==p.None);
    assert(p.tick(135001,false,false)==p.Connect);
    p.configure(135100,false);
    assert(p.tick(135350,false,false)==p.Disconnect);
    assert(p.tick(200000,false,false)==p.None);
    p.configure(0xfffffff0u,true);
    assert(p.tick(0x00000100u,false,false)==p.Connect); // Millis wrap is harmless.
    p.configure(0,true,true);
    for(unsigned i=0;i<3;i++){
        assert(p.tick(250+60000*i,false,false)==p.Connect);
        assert(p.tick(30250+60000*i,false,false)==p.Disconnect);
    }
    assert(p.exhausted());assert(p.tick(999999,false,false)==p.None);
    p.configure(1000000,true,true);assert(!p.exhausted());
    assert(p.tick(1000250,false,false)==p.Connect);
    assert(p.tick(1001000,true,false)==p.None);assert(p.attempts==0);
    std::puts("PASS: credential validation, stopped-only changes, delayed apply, retry/timeout, link-loss and rollover");
}
