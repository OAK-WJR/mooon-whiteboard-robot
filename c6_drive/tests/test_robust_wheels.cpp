#include "control.h"
#include <cassert>
#include <cstring>
#include <limits>
#include <cstdio>

int main(){
    drive::Control c;
    assert(c.selectRobust(true));
    c.arm(0,42);
    assert(!c.selectRobust(false));
    float target[3]={3000,-1500,-1500};
    assert(c.accept(0,42,1,target,false));
    c.tick(10,.01f);
    assert(c.robustMode&&c.armed);
    assert(std::fabs(c.wheel[0].ramp+2*c.wheel[1].ramp)<.001f);
    assert(c.wheel[1].ramp==c.wheel[2].ramp);
    // Actuator readback is authoritative, even if previous output was requested.
    c.tick(20,.01f);
    for(auto& w:c.wheel)assert(std::fabs(w.output)<=12);
    c.tick(301,.01f);assert(!c.armed&&!std::strcmp(c.reason,"LINK_TIMEOUT"));
    for(auto& w:c.wheel)assert(w.output==0);
    assert(c.selectRobust(false));assert(!c.robustMode);
    assert(c.selectRobust(true));c.arm(400,43);
    assert(c.accept(400,43,1,target,false));
    c.wheel[2].measured=std::numeric_limits<float>::quiet_NaN();c.tick(410,.01f);
    assert(!c.armed&&!std::strcmp(c.reason,"ROBUST_BAD_SPEED"));
    assert(c.lastFault.motor==3&&std::isnan(c.lastFault.speed[2]));
    for(auto& w:c.wheel)assert(w.output==0);
    c.wheel[2].measured=0;c.arm(500,44);c.tick(510,.04f);assert(!c.armed);
    // The bounded static-friction kick does not wind up the dynamic controller.
    {drive::RobustWheels bank;bank.loadedPi=false;float set[3]={1500,0,0},v[3]={},u[3]={};int out[3];
     for(int n=0;n<20;n++){
      assert(bank.update(set,v,u,.01f,out));u[0]=out[0];
      assert(bank.wheel[0].integral==0&&bank.wheel[0].load==0);
      assert(out[1]==0&&out[2]==0&&out[0]<=180);
     }
     assert(out[0]>=160);v[0]=200;assert(bank.update(set,v,u,.01f,out));assert(bank.startup[0]==0);
     set[0]=0;assert(bank.update(set,v,u,.01f,out));assert(out[0]==0);
    }
    // A jam cannot cause indefinite repeated startup pulses in loaded mode.
    for(int motor=0;motor<3;motor++){
        drive::RobustWheels bank;float set[3]={},v[3]={},u[3]={};int out[3];set[motor]=-500;
        bool ok=true;int ticks=0;
        for(;ticks<100&&ok;ticks++){
            ok=bank.update(set,v,u,.01f,out);
            for(int i=0;i<3;i++){
                assert(std::abs(out[i])<=255);
                if(ok)assert(std::fabs(out[i]-u[i])<=6.001f);
                assert(bank.startup[i]==0);u[i]=out[i];
            }
        }
        assert(!ok&&ticks<100&&!std::strcmp(bank.fault,"START_NO_MOTION"));
        for(int i=0;i<3;i++)assert(out[i]==0);
    }
    // Release a detected breakaway promptly, while only reducing same-sign
    // drive. This is not permission to bypass the slew on increases/reversals.
    for(int motor=0;motor<3;motor++)for(int sign:{-1,1}){
        drive::RobustWheels bank;float set[3]={},v[3]={},u[3]={};int out[3];set[motor]=sign*500;
        for(int n=0;n<15;n++){
            assert(bank.update(set,v,u,.01f,out));
            for(int i=0;i<3;i++)u[i]=out[i];
        }
        float before=u[motor];v[motor]=sign*100;
        assert(bank.update(set,v,u,.01f,out));
        assert(bank.startup[motor]==0);
        assert(out[motor]*sign>=0&&std::fabs(out[motor]-before)<=6.001f);
        set[motor]=0;assert(bank.update(set,v,u,.01f,out));assert(out[motor]==0);
        wbb_robust::Wheel a,b;wbb_robust::Config cfg;
        int normal=a.update(motor,sign*2000,0,sign*20,.01f,true,false,cfg,9,false);
        int release=b.update(motor,sign*2000,0,sign*20,.01f,true,false,cfg,9,true);
        assert(normal==release&&std::abs(release)<=32);
    }
    // Analytic identified wheel plants; each direction, isolation and release.
    for(int motor=0;motor<3;motor++)for(int sign=-1;sign<=1;sign+=2){
        drive::RobustWheels bank;
        float set[3]={},speed[3]={},applied[3]={};int pwm[3];set[motor]=sign*3000;
        const auto& model=wbb_robust::measuredModels[motor][sign<0];
        float a=std::exp(-.01f/model.tau);
        for(int n=0;n<500;n++){
            assert(bank.update(set,speed,applied,.01f,pwm));
            for(int i=0;i<3;i++){
                assert(std::abs(pwm[i])<=255);
                assert(std::fabs(pwm[i]-applied[i])<=12.001f);
                if(i!=motor)assert(pwm[i]==0);
            }
            speed[motor]=a*speed[motor]+(1-a)*model.K*wbb_robust::Wheel::effective(pwm[motor],model);
            for(int i=0;i<3;i++)applied[i]=pwm[i];
        }
        assert(std::fabs(speed[motor]-set[motor])<120);
        set[motor]=0;assert(bank.update(set,speed,applied,.01f,pwm));
        for(int i=0;i<3;i++)assert(pwm[i]==0&&bank.wheel[i].integral==0&&bank.compensation[i]==0);
    }
    std::puts("PASS: hardware wheel adapter, actual-duty tracking, shared ramp, faults, isolation, six model directions and release");
}
