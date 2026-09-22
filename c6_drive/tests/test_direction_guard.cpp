#include "control.h"
#include <cassert>
#include <cstring>
#include <cstdio>

int main(){
    // Actual captured M1 fault: +600 cps reference, -731.7 cps coast, +48 PWM.
    // A single sample during a fresh direction change is not persistent reversal.
    {
        drive::Control c;c.arm(0,42);
        float target[3]={2110.897f,-4221.794f,2110.897f};
        assert(c.accept(10,42,1,target,false));
        c.wheel[0].ramp=480;c.wheel[0].output=48;
        c.wheel[0].measured=-731.7469f;c.applied[0]=48;
        c.tick(10,.01f);
        assert(c.armed&&"Captured coast sample must not immediately trip direction guard");
    }
    // Coast through a reversal, then recover. Both signs and each physical wheel.
    for(int motor=0;motor<3;motor++)for(int sign:{-1,1}){
        drive::Control c;c.arm(0,42);float target[3]={};target[motor]=sign*1600;
        for(uint32_t t=10;t<=600;t+=10){
            assert(c.accept(t,42,t,target,false));
            c.applied[motor]=c.wheel[motor].output;
            c.wheel[motor].measured=sign*(t<=180?-700.f:1000.f);
            c.tick(t,.01f);assert(c.armed);
        }
        assert(c.oppositeTime[motor]==0);
    }
    // A genuinely persistent opposite sign at modest *actual* drive still stops.
    for(int motor=0;motor<3;motor++)for(int sign:{-1,1}){
        drive::Control c;c.arm(0,42);float target[3]={};target[motor]=sign*1600;
        uint32_t stoppedAt=0;
        for(uint32_t t=10;t<=600;t+=10){
            assert(c.accept(t,42,t,target,false));
            c.applied[motor]=sign*80;c.wheel[motor].measured=-sign*900;
            c.tick(t,.01f);
            if(!c.armed){stoppedAt=t;break;}
        }
        assert(stoppedAt>=250&&stoppedAt<=400);
        assert(!std::strcmp(c.reason,"ENCODER_DIRECTION")&&c.lastFault.motor==motor+1);
        assert(c.lastFault.applied[motor]==sign*80);
        for(auto& w:c.wheel)assert(w.output==0);
    }
    // Requested direction cannot indict the encoder while actual torque is old.
    {
        drive::Control c;c.arm(0,42);float target[3]={1600,0,0};
        for(uint32_t t=10;t<=400;t+=10){
            assert(c.accept(t,42,t,target,false));
            c.applied[0]=-120;c.wheel[0].measured=-900;
            c.tick(t,.01f);assert(c.armed&&c.oppositeTime[0]==0);
        }
        c.stop("USER_STOP");assert(c.oppositeTime[0]==0);
    }
    // Separate short coast episodes must not accumulate into a false fault.
    {
        drive::Control c;c.arm(0,42);float target[3]={1600,0,0};
        for(uint32_t t=10;t<=1200;t+=10){
            assert(c.accept(t,42,t,target,false));
            c.applied[0]=80;c.wheel[0].measured=t%200==0?900:-900;
            c.tick(t,.01f);assert(c.armed);
            if(t%200==0)assert(c.oppositeTime[0]==0);
        }
        c.stop("USER_STOP");c.arm(1210,43);
        assert(c.oppositeTime[0]==0);
    }
    // Low-duty coast does not start the timer; qualifying sustained drive does.
    {
        drive::Control c;c.arm(0,42);float target[3]={1600,0,0};
        for(uint32_t t=10;t<=600;t+=10){
            assert(c.accept(t,42,t,target,false));
            c.applied[0]=48;c.wheel[0].measured=-900;
            c.tick(t,.01f);assert(c.armed&&c.oppositeTime[0]==0);
        }
        for(uint32_t t=610;t<=900;t+=10){
            assert(c.accept(t,42,t,target,false));
            c.applied[0]=80;c.wheel[0].measured=-900;c.tick(t,.01f);
            if(!c.armed){assert(t>=850&&t<=870);break;}
        }
        assert(!c.armed&&!std::strcmp(c.reason,"ENCODER_DIRECTION"));
    }
    std::puts("PASS: captured coast, six transient reversals, six true polarity faults, actual-duty gating");
}
