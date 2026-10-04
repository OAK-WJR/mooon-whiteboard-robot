#include "control.h"
#include <cassert>
#include <cstring>
#include <limits>
using namespace drive;
int main(){
    Control c;
    assert(!c.floorTrial);
    assert(c.selectFloorTrial()&&c.robustMode&&c.headingAssistEnabled);
    assert(!strcmp(c.mode(),"floor_trial"));
    float w[3];uint32_t seq=0;
    c.arm(0,42);
    // Test the body envelope in every direction, with combined turns.
    for(int i=0;i<72;i++)for(float yaw:{-.4f,0.f,.4f}){
        float a=i*6.283185307f/72;
        EncoderHold::wheels({.08f*std::cos(a),.08f*std::sin(a),yaw},w);
        assert(c.accept(1,42,++seq,w,false));
    }
    for(Pose v:{Pose{.081f,0,0},Pose{0,-.081f,0},Pose{.08f,.08f,0},Pose{0,0,.401f},Pose{0,0,-.401f}}){
        EncoderHold::wheels(v,w);assert(!c.accept(2,42,++seq,w,false));
    }
    w[0]=w[1]=w[2]=0;assert(!c.accept(2,42,++seq,w,true));
    w[0]=std::numeric_limits<float>::quiet_NaN();assert(!c.accept(2,42,++seq,w,false));
    assert(!c.selectRobust(false)&&c.floorTrial);
    c.tick(302,.01f);assert(!c.armed&&!strcmp(c.reason,"LINK_TIMEOUT"));
    for(auto& wheel:c.wheel)assert(wheel.output==0);
    assert(c.floorTrial); // STOP retains the selected trial, not an armed lease.
    assert(c.selectHold()&&!c.floorTrial&&!c.headingAssistEnabled);
    assert(c.selectFloorTrial());c.stop("USER_STOP");
    assert(c.selectRobust(false)&&!c.floorTrial&&!c.headingAssistEnabled);
    // Trial access must not change the existing coordinated controller output.
    Control reference,trial;reference.selectRobust(true);reference.headingAssistEnabled=true;
    trial.selectFloorTrial();reference.arm(0,1);trial.arm(0,1);
    for(uint32_t t=10;t<=1400;t+=10){
        float delta[3]={};
        for(int i=0;i<3;i++){
            float measured=reference.wheel[i].ramp*.85f;
            reference.wheel[i].measured=trial.wheel[i].measured=measured;
            delta[i]=measured*.01f;
            reference.applied[i]=trial.applied[i]=reference.wheel[i].output;
        }
        reference.observe(t,delta,.01f);trial.observe(t,delta,.01f);
        EncoderHold::wheels({0,t<1000?.08f:0,0},w);
        assert(reference.accept(t,1,t,w,false));assert(trial.accept(t,1,t,w,false));
        reference.tick(t,.01f);trial.tick(t,.01f);
        assert(reference.armed==trial.armed);
        assert(reference.coordinatedBraking==trial.coordinatedBraking);
        for(int i=0;i<3;i++)assert(reference.wheel[i].output==trial.wheel[i].output);
    }
    trial.stop("USER_STOP");for(auto& wheel:trial.wheel)assert(wheel.output==0);
}
