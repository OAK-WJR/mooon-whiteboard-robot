#include "safety.h"
#include <cassert>
#include <cstring>
#include <iostream>
int main() {
    diag::Safety s;
    assert(s.pulse(1000,0,1,64,500));
    assert(s.armBench(1000,6000,12000,64,true));
    assert(s.armBench(1000,12000,6000,64,true));
    assert(s.armBench(1000,6000,6000,64,false));
    assert(!s.armBench(1000,6000,6000,64,true));
    assert(s.pulse(1000,0,1,65,500));
    assert(s.pulse(1000,4,1,64,500));
    assert(s.pulse(1000,0,0,64,500));
    assert(s.pulse(1000,0,1,64,1001));
    assert(!s.running);
    assert(!s.pulse(1000,3,-1,64,1000));
    assert(s.pulse(1001,0,1,64,500));
    assert(!s.tick(1349,true));
    assert(s.tick(1350,true));
    assert(!s.running && !s.armed && !strcmp(s.reason,"HEARTBEAT_TIMEOUT"));
    assert(!s.armBench(2000,12000,12000,255,true));
    assert(!s.pulse(2000,0,1,255,100));
    assert(s.tick(2100,true));
    assert(!s.armed && !strcmp(s.reason,"PULSE_DONE"));
    assert(s.pulse(2101,0,-1,64,100));
    assert(!s.armBench(2110,6000,6000,64,true));
    assert(s.pulse(2110,0,-1,64,100)); // minimum coast time
    s.stop(2200,"USER_STOP");
    assert(!s.armBench(3000,6000,6000,64,true));
    assert(!s.pulse(3000,0,1,64,500));
    assert(s.tick(3001,false));
    assert(!s.armed && !s.running && !strcmp(s.reason,"USB_LOST"));
    s.stop(0xfffff000u,"RESET");
    assert(!s.armBench(0xffffff80u,6000,6000,64,true));
    assert(!s.pulse(0xffffff80u,0,1,64,1000));
    assert(!s.tick(0x00000000u,true));
    assert(s.tick(0x000000deu,true)); // 350ms across millis() wrap
    const unsigned cycle[]={0,2,3,1,0};
    int count=0;
    for(int i=1;i<5;++i) count+=diag::quadrature(cycle[i-1],cycle[i]);
    assert(count==4);
    for(int i=4;i>0;--i) count+=diag::quadrature(cycle[i],cycle[i-1]);
    assert(count==0);
    assert(diag::quadrature(0,3)==0 && diag::quadrature(2,1)==0);
    std::cout << "Safety: invalid commands, supply mismatch, pulse expiry, heartbeat, disconnect, wrap and quadrature passed\n";
}
