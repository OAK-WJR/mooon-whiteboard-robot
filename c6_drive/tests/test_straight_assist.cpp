#include "control.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>

struct Result{float heading=0,peak=0,travel=0;};
// Assumed scalar motor/contact screening, not an identified floor model.
// Encoder quantization and unequal breakaway/dead-time are external to Control.
static Result trial(bool enabled,int direction,int variant){
    drive::Control c;c.selectRobust(true);c.headingAssistEnabled=enabled;c.arm(0,42);
    drive::Pose demand;float speed=.08f;
    if(direction==0)demand.x=speed;if(direction==1)demand.x=-speed;
    if(direction==2)demand.y=speed;if(direction==3)demand.y=-speed;
    float target[3];drive::EncoderHold::wheels(demand,target);
    float v[3]={},pos[3]={},history[3][32]={};int previous[3]={};
    Result r;
    for(int n=1;n<=160;n++){
        float delta[3];
        for(int i=0;i<3;i++){
            const float gain=30.f+(i*7+variant*3)%19,tau=.045f+.015f*((i+variant)%5);
            const int lag=2+(i*3+variant)%7;
            float u=history[i][(n+32-lag)%32];
            float friction=70.f+((i*5+variant*3)%17),breakaway=128.f+(i*9+variant)%20;
            float acceleration=(gain*(u-std::copysign(friction,std::fabs(v[i])>5?v[i]:u))-v[i])/tau;
            if(std::fabs(v[i])<5&&std::fabs(u)<breakaway)v[i]=0;
            else{
                float next=v[i]+acceleration*.01f;
                v[i]=next*v[i]<0&&std::fabs(u)<breakaway?0:next;
            }
            pos[i]+=v[i]*.01f;int count=int(std::lround(pos[i]));delta[i]=float(count-previous[i]);previous[i]=count;
            c.wheel[i].measured+=.4f*(delta[i]*100-c.wheel[i].measured);
            c.applied[i]=c.wheel[i].output;
        }
        c.observe(n*10,delta,.01f);
        assert(c.accept(n*10,42,n,target,false));c.tick(n*10,.01f);
        if(!c.armed){std::printf("fault enabled=%d dir=%d variant=%d %s\n",enabled,direction,variant,c.reason);assert(c.armed);}
        for(int i=0;i<3;i++){
            assert(std::fabs(c.wheel[i].output)<=255);
            assert(std::fabs(c.wheel[i].output-c.applied[i])<=6.01f);
            history[i][n%32]=c.wheel[i].output;
        }
        r.peak=std::fmax(r.peak,std::fabs(c.odometry.pose.yaw));
    }
    r.heading=std::fabs(c.odometry.pose.yaw);r.travel=std::hypot(c.odometry.pose.x,c.odometry.pose.y);
    c.stop("USER_STOP");assert(!c.headingAssist.active&&c.headingAssist.correctionCps==0);
    for(auto& w:c.wheel)assert(w.output==0);
    return r;
}

int main(){
    setvbuf(stdout,nullptr,_IONBF,0);
    drive::EncoderHold od;float zero[3]={};od.observe(10,zero,.01f);
    drive::StraightAssist a;float target[3]={469.088f,-938.176f,469.088f},out[3];
    assert(a.update(target,od,10,.01f,out)&&a.active);
    od.pose.yaw=.08f;od.velocity.yaw=.2f;
    float before=0;
    for(int n=0;n<30;n++){
        assert(a.update(target,od,10,.01f,out));
        assert(a.yawRate<0&&std::fabs(a.yawRate)<=.35001f);
        assert(std::fabs(a.yawRate-before)<=.02001f);before=a.yawRate;
        auto corrected=drive::EncoderHold::body(out);auto original=drive::EncoderHold::body(target);
        assert(std::fabs(corrected.x-original.x)<1e-6f&&std::fabs(corrected.y-original.y)<1e-6f);
        assert(corrected.yaw<0);
    }
    assert(a.update(zero,od,10,.01f,out)&&!a.active);for(float x:out)assert(x==0);
    float yaw[3]={400,400,400};assert(a.update(yaw,od,10,.01f,out)&&!a.active);
    float single[3]={1000,0,0};assert(a.update(single,od,10,.01f,out)&&!a.active);
    assert(!a.update(target,od,50,.01f,out));
    od.pose.yaw=std::numeric_limits<float>::quiet_NaN();assert(!a.update(target,od,10,.01f,out));
    // Stale odometry must fault through the real command/stop wrapper.
    drive::Control c;c.selectRobust(true);c.headingAssistEnabled=true;c.arm(0,42);
    assert(c.accept(10,42,1,target,false));c.tick(10,.01f);
    assert(!c.armed&&!std::strcmp(c.reason,"HEADING_ODOMETRY"));
    for(auto& w:c.wheel)assert(w.output==0);
    // Normal zero is a bounded deceleration; explicit stop remains immediate.
    for(bool emergency:{false,true}){
        drive::Control b;b.selectRobust(true);b.headingAssistEnabled=true;
        b.observe(0,zero,.01f);b.arm(0,43);
        assert(b.accept(10,43,1,target,false));b.observe(10,zero,.01f);b.tick(10,.01f);
        assert(b.headingAssist.active);
        assert(b.accept(20,43,2,zero,false));assert(b.coordinatedBraking);
        if(emergency){
            b.stop("USER_STOP");assert(!b.coordinatedBraking&&!b.armed);
            for(auto& w:b.wheel)assert(w.output==0);
        }else{
            for(unsigned t=20;t<=100;t+=10){
                b.observe(t,zero,.01f);assert(b.accept(t,43,t,zero,false));b.tick(t,.01f);
            }
            assert(b.armed&&!b.coordinatedBraking&&!b.headingAssist.active);
            for(auto& w:b.wheel)assert(w.output==0);
        }
    }
    // A wheel that never slows cannot extend the release forever.
    {
        drive::Control b;b.selectRobust(true);b.headingAssistEnabled=true;b.observe(0,zero,.01f);b.arm(0,44);
        assert(b.accept(10,44,1,target,false));b.observe(10,zero,.01f);b.tick(10,.01f);
        assert(b.accept(20,44,2,zero,false));
        for(unsigned t=20;t<=600;t+=10){
            b.observe(t,zero,.01f);for(auto& w:b.wheel)w.measured=200;
            assert(b.accept(t,44,t,zero,false));b.tick(t,.01f);if(!b.armed)break;
        }
        assert(!b.armed&&!std::strcmp(b.reason,"COORDINATED_STOP_TIMEOUT"));
        for(auto& w:b.wheel)assert(w.output==0);
    }
    float oldSum=0,newSum=0,oldPeak=0,newPeak=0;int improved=0;
    for(int variant=0;variant<6;variant++)for(int d=0;d<4;d++){
        auto old=trial(false,d,variant),next=trial(true,d,variant);
        oldSum+=old.heading;newSum+=next.heading;oldPeak+=old.peak;newPeak+=next.peak;
        improved+=next.heading<old.heading;
        std::printf("case,%d,%d,%.6f,%.6f,%.6f,%.6f,%.5f,%.5f\n",variant,d,
            old.heading,next.heading,old.peak,next.peak,old.travel,next.travel);
    }
    std::printf("screen mean final yaw %.5f -> %.5f rad; mean peak %.5f -> %.5f; improved %d/24\n",
        oldSum/24,newSum/24,oldPeak/24,newPeak/24,improved);
    assert(newSum<oldSum*.8f&&newPeak<=oldPeak*1.1f);
    std::puts("PASS: bounds, translation-preserving correction, release/turn bypass, stale fault, assumed unequal-motor screening");
}
