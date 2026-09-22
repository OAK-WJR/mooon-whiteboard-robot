#include "control.h"
#include <algorithm>
#include <cassert>
#include <cstring>
#include <limits>
#include <cstdio>

static void code(const char* actual,const char* expected){assert(actual&&!std::strcmp(actual,expected));}
static void stopped(const drive::Control& c){
    assert(!c.armed&&c.lease==0);
    for(int i=0;i<3;i++)assert(c.wheel[i].output==0&&c.applied[i]==0&&c.raw[i]==0);
}
int main(){
    const float nan=std::numeric_limits<float>::quiet_NaN();
    // Every rejected input identifies the fault and leaves all returned outputs zero.
    for(int input=0;input<3;input++)for(int motor=0;motor<3;motor++)for(int invalid=0;invalid<2;invalid++){
        drive::RobustWheels bank;float target[3]={1500,1500,1500},speed[3]={},applied[3]={};int out[3]={123,123,123};
        float* values=input==0?target:input==1?speed:applied;
        values[motor]=invalid?nan:input==0?10001:input==1?30001:256;
        assert(!bank.update(target,speed,applied,.01f,out));
        code(bank.fault,input==0?"ROBUST_BAD_TARGET":input==1?"ROBUST_BAD_SPEED":"ROBUST_BAD_DUTY");
        assert(bank.faultMotor==motor);for(int x:out)assert(x==0);
    }
    for(float dt:{.004f,.026f,nan}){
        drive::RobustWheels bank;float v[3]={};int out[3]={1,2,3};
        assert(!bank.update(v,v,v,dt,out));code(bank.fault,"ROBUST_BAD_DT");
        assert(bank.faultMotor==-1);for(int x:out)assert(x==0);
    }
    // Reproduce an unresponsive wheel during both holding and intentional drive.
    // This is a synthetic no-motion fixture, not evidence of the wall fault's cause.
    for(int motor=0;motor<3;motor++)for(bool hold:{false,true}){
        drive::RobustWheels bank;float target[3]={},speed[3]={},applied[3]={};int out[3];target[motor]=1000;
        bool failed=false;int peak=0;
        for(int n=0;n<50;n++){
            if(!bank.update(target,speed,applied,.01f,out,hold,hold?nullptr:target)){
                failed=true;code(bank.fault,"START_NO_MOTION");assert(bank.faultMotor==motor);
                for(int x:out)assert(x==0);break;
            }
            for(int i=0;i<3;i++)applied[i]=out[i];peak=std::max(peak,out[motor]);
        }
        assert(failed&&peak==(motor==0?160:140));
    }
    // Capture before Control::stop resets the bank; preserve through later STOPs,
    // rejected commands and mode changes. A new run may replace it on a new fault.
    for(int motor=0;motor<3;motor++){
        drive::Control c;assert(c.selectHold());float delta[3]={},body[3]={0,0,.6f};
        c.observe(0,delta,.01f);c.arm(0,42);
        for(uint32_t t=10;t<=500;t+=10){
            c.observe(t,delta,.01f);
            for(int i=0;i<3;i++){c.wheel[i].measured=i==motor?0:250;c.applied[i]=c.wheel[i].output;}
            assert(c.acceptBody(t,42,t,body));c.tick(t,.01f);
            if(!c.armed)break;
        }
        stopped(c);code(c.reason,"START_NO_MOTION");code(c.lastFault.code,"START_NO_MOTION");
        assert(c.lastFault.motor==motor+1&&c.lastFault.ms>0&&c.lastFault.ms<=350);
        assert(c.lastFault.target[motor]>400&&c.lastFault.speed[motor]==0);
        assert(c.lastFault.applied[motor]==(motor==0?160:140));
        assert(c.lastFault.reference[motor]>0&&c.lastFault.run==1);
        code(c.lastFault.mode,"encoder_hold");
        auto snapshot=c.lastFault;
        c.stop("USER_STOP");c.reject("EXPIRED_OR_INVALID");c.faultStop("SECONDARY",600,.01f,0);
        assert(c.selectRobust(true));
        code(c.lastFault.code,snapshot.code);assert(c.lastFault.ms==snapshot.ms&&c.lastFault.motor==snapshot.motor);
        c.arm(700,43);assert(c.lastFault.run==1&&c.run==2); // explicitly a historical record
        c.tick(710,.04f);stopped(c);code(c.lastFault.code,"ROBUST_BAD_DT");
        assert(c.lastFault.run==2&&c.lastFault.motor==0&&c.lastFault.ms==710);
        c.stop("USER_STOP");assert(c.lastFault.ms==710);
    }
    std::puts("PASS: typed faults, six no-motion cases, zero outputs, pre-stop snapshots and fault retention");
}
