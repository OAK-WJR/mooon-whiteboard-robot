#include "control.h"
#include <cassert>
#include <cstring>
#include <cstdio>
#include <limits>
#include <initializer_list>

static void close(float a,float b,float tolerance=1e-5f){assert(std::fabs(a-b)<tolerance);}
struct Rig {
    drive::Control c;
    float speed[3]={},fraction[3]={};uint32_t now=0,seq=0;
    float efficiency=1,load=0;
    bool staticFriction=true;
    Rig(){c.selectHold();float d[3]={};c.observe(0,d,.01f);c.arm(0,42);}
    void step(const float command[3]){
        now+=10;
        float delta[3];
        for(int i=0;i<3;i++){
            const auto& model=wbb_robust::measuredModels[i][c.wheel[i].output<0];
            float a=std::exp(-.01f/model.tau);
            if(staticFriction&&std::fabs(speed[i])<30&&std::fabs(c.wheel[i].output)<(i==0?155:135))speed[i]=0;
            else speed[i]=a*speed[i]+(1-a)*model.K*(efficiency*wbb_robust::Wheel::effective(c.wheel[i].output,model)-load);
            fraction[i]+=speed[i]*.01f;delta[i]=std::trunc(fraction[i]);fraction[i]-=delta[i];
            c.wheel[i].measured+=.4f*(delta[i]/.01f-c.wheel[i].measured);
            c.applied[i]=c.wheel[i].output;
        }
        c.observe(now,delta,.01f);
        if(!c.acceptBody(now,42,++seq,command)){std::printf("accept fault %s at %u\n",c.reason,now);assert(false);}
        c.tick(now,.01f);
        if(!c.armed){std::printf("control fault %s at %u\n",c.reason,now);assert(false);}
        for(auto& w:c.wheel)assert(std::isfinite(w.output)&&std::fabs(w.output)<=255);
    }
};
int main(){
    using drive::EncoderHold;
    // Forward/inverse kinematics for all axes and exact curved-path integration.
    for(drive::Pose v: {drive::Pose{.13f,0,0},drive::Pose{0,.07f,0},drive::Pose{0,0,.8f},drive::Pose{.1f,-.1f,.5f}}){
        float wheel[3];EncoderHold::wheels(v,wheel);auto back=EncoderHold::body(wheel);
        close(back.x,v.x);close(back.y,v.y);close(back.yaw,v.yaw);
    }
    EncoderHold od;float d[3];EncoderHold::wheels({.1f,0,.5f},d);
    for(float& n:d)n*=.01f;
    for(int k=1;k<=100;k++)assert(od.observe(k*10,d,.01f));
    close(od.pose.x,.2f*std::sin(.5f));close(od.pose.y,.2f*(1-std::cos(.5f)));close(od.pose.yaw,.5f);
    d[0]=10000;auto before=od.pose;assert(!od.observe(1010,d,.01f));close(od.pose.x,before.x);
    // Explicit lease, invalid input and stale-odometry boundaries.
    drive::Control c;c.selectHold();float z[3]={};c.observe(10,z,.01f);c.arm(10,7);
    assert(!c.accept(11,7,1,z,false));assert(!c.acceptBody(11,8,1,z));
    assert(c.acceptBody(11,7,1,z));assert(!c.acceptBody(12,7,1,z));
    c.tick(40,.01f);assert(!c.armed&&!std::strcmp(c.reason,"ODOMETRY_HOLD_FAULT"));
    c.observe(100,z,.01f);c.arm(100,8);float bad[3]={.41f,0,0};assert(!c.acceptBody(101,8,1,bad));
    bad[0]=.081f;assert(!c.acceptBody(101,8,1,bad));
    bad[0]=0;bad[2]=1.2f;assert(!c.acceptBody(101,8,1,bad));bad[2]=0;
    bad[0]=std::numeric_limits<float>::quiet_NaN();assert(!c.acceptBody(101,8,1,bad));
    assert(c.acceptBody(101,8,1,z));c.observe(410,z,.01f);c.tick(410,.01f);assert(!c.armed);
    for(auto& w:c.wheel)assert(w.output==0);
    c.observe(500,z,.01f);c.arm(500,9);assert(c.acceptBody(501,9,1,z));
    c.odometry.pose.x+=.09f;c.tick(510,.01f);assert(!c.armed);
    c.observe(600,z,.01f);c.arm(600,10);assert(c.acceptBody(601,10,1,z));
    float corrupt[3]={100000,0,0};c.observe(610,corrupt,.01f);assert(!c.armed);
    // Native identified-plant integration (not real-body or contact validation).
    for(float efficiency:{.85f,1.f,1.15f})for(int axis=0;axis<3;axis++)for(int sign:{-1,1}){
        Rig rig;rig.efficiency=efficiency;float command[3]={};command[axis]=axis==2?.6f:.08f;
        command[axis]*=sign;
        for(int k=0;k<150;k++)rig.step(command);
        for(float& n:command)n=0;
        rig.step(command);auto goal=rig.c.odometry.goal;
        float maxError=0;
        for(int k=0;k<700;k++){rig.step(command);maxError=std::fmax(maxError,rig.c.odometry.error);}
        float pos=std::hypot(goal.x-rig.c.odometry.pose.x,goal.y-rig.c.odometry.pose.y);
        float yaw=std::fabs(EncoderHold::wrap(goal.yaw-rig.c.odometry.pose.yaw));
        std::printf("hold efficiency %.2f axis %d: error %.3f mm %.3f deg, peak %.3f mm\n",efficiency,axis,pos*1000,yaw*180/3.14159265f,maxError*1000);
        assert(pos<.004f&&yaw<.05236f);
        for(int k=0;k<150;k++){
            rig.step(command);
            assert(rig.c.odometry.error<.004f&&std::fabs(rig.c.odometry.headingError)<.05236f);
            for(auto& w:rig.c.wheel)assert(std::fabs(w.measured)<150);
        }
        assert(rig.c.odometry.quiet);for(auto& w:rig.c.wheel)assert(w.output==0);
        // An actual count displacement must produce correction toward the fixed goal.
        float impulse[3];EncoderHold::wheels({.006f,0,0},impulse);
        rig.c.observe(rig.now,impulse,.01f);
        for(int k=0;k<700;k++)rig.step(command);
        assert(std::hypot(goal.x-rig.c.odometry.pose.x,goal.y-rig.c.odometry.pose.y)<.004f);
        rig.c.stop("TEST_STOP");for(auto& w:rig.c.wheel)assert(w.output==0);
    }
    // No independent pose means an unmeasured slide cannot be detected or corrected.
    {
        Rig rotating;float cmd[3]={0,0,-.6f};
        for(int i=0;i<100;i++)rotating.step(cmd);
        rotating.speed[0]=0; // Force one modeled low-speed wheel back into static friction.
        for(int i=0;i<80;i++)rotating.step(cmd);
        assert(rotating.speed[0]<-100&&rotating.c.armed);
        rotating.c.stop("TEST_STOP");
    }
    Rig invisible;for(int i=0;i<200;i++)invisible.step(z);
    close(invisible.c.odometry.pose.x,0);for(auto& w:invisible.c.wheel)assert(w.output==0);
    std::puts("PASS: encoder hold kinematics, model cases, faults, release, stop and unobservable-slip boundary");
}
