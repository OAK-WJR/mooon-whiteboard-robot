#include "control.h"
#include <cassert>
#include <cstring>
#include <limits>
#include <cstdio>

static void zero(const drive::Control& c){
    assert(!c.armed&&c.lease==0);
    for(auto& w:c.wheel)assert(w.output==0&&w.integral==0);
}
int main(){
    drive::Control c;float delta[3]={},cmd[3]={};
    assert(c.selectServo());assert(!std::strcmp(c.mode(),"encoder_servo"));
    c.observe(0,delta,.01f);c.arm(0,42);
    assert(!c.selectServo());assert(!c.accept(1,42,1,cmd,true));
    assert(!c.accept(1,42,1,cmd,false));
    assert(c.acceptBody(1,42,1,cmd));c.observe(10,delta,.01f);c.tick(10,.01f);
    for(auto& w:c.wheel)assert(w.output==0);
    // Retained bias at zero position error: quiet no longer means force off.
    c.servo.integral[0]=60;c.applied[0]=60;
    c.observe(20,delta,.01f);c.tick(20,.01f);
    assert(c.armed&&c.wheel[0].output==60&&c.servo.integral[0]==60);
    c.stop("USER_STOP");zero(c);assert(c.servo.integral[0]==0&&!c.servo.initialized);
    c.arm(30,43);c.observe(40,delta,.01f);c.tick(40,.01f);
    for(auto& w:c.wheel)assert(w.output==0); // no command, no inherited torque
    assert(c.acceptBody(40,43,1,cmd));c.observe(341,delta,.01f);c.tick(341,.01f);
    zero(c);assert(!std::strcmp(c.lastFault.code,"LINK_TIMEOUT"));
    // A moving command with all encoders frozen must stop; no retry/re-arm.
    c.observe(400,delta,.01f);c.arm(400,44);cmd[0]=.08f;
    for(uint32_t t=410;t<3400&&c.armed;t+=10){
        for(int i=0;i<3;i++)c.applied[i]=c.wheel[i].output;
        c.observe(t,delta,.01f);assert(c.acceptBody(t,44,t,cmd));c.tick(t,.01f);
    }
    zero(c);assert(c.lastFault.motor>=1&&c.lastFault.motor<=3);
    assert(!std::strcmp(c.lastFault.code,"SERVO_START_NO_MOTION")||!std::strcmp(c.lastFault.code,"SERVO_NO_MOTION"));
    c.observe(3500,delta,.01f);assert(!c.acceptBody(3500,44,9999,cmd));c.tick(3500,.01f);zero(c);
    // Controller-wide invalid sample, per-wheel bad readback, tracking envelope.
    for(int kind=0;kind<3;kind++){
        for(auto& w:c.wheel)w.measured=0;
        c.observe(4000,delta,.01f);c.arm(4000,45+kind);cmd[0]=0;
        assert(c.acceptBody(4000,45+kind,1,cmd));c.tick(4000,.01f);
        if(kind==0)c.tick(4010,.04f);
        if(kind==1){c.wheel[2].measured=std::numeric_limits<float>::quiet_NaN();c.tick(4010,.01f);}
        if(kind==2){c.wheel[2].measured=0;c.odometry.pose.y+=.1f;c.tick(4010,.01f);}
        zero(c);
        assert(!std::strcmp(c.lastFault.code,kind==0?"SERVO_BAD_DT":kind==1?"SERVO_BAD_SPEED":"SERVO_TRACKING_ERROR"));
    }
    // Frozen sensors cannot reveal chassis slip. No fictitious slip flag/pose.
    drive::EncoderServo servo;drive::Pose p;float speed[3]={},u[3]={};int out[3];
    for(int n=0;n<100;n++)assert(servo.update(p,p,speed,u,.01f,out));
    for(int x:out)assert(x==0);
    // Holding output and integral are bounded, with arbitrary retained load bias.
    servo.integral[0]=180;servo.integral[1]=-180;u[0]=250;u[1]=-250;
    assert(servo.update(p,p,speed,u,.01f,out));
    assert(out[0]==180&&out[1]==-180);
    for(float i:servo.integral)assert(std::fabs(i)<=180);
    // Static compensation may use bounded torque, but a large held position
    // error at the holding ceiling with no motion must still latch a fault.
    drive::EncoderServo overloaded;overloaded.initialized=true;
    drive::Pose displaced{.02f,0,0};for(float& x:u)x=0;
    bool failed=false;
    for(int n=0;n<1000;n++){
        if(!overloaded.update(displaced,p,speed,u,.01f,out)){failed=true;break;}
        for(int i=0;i<3;i++){assert(std::abs(out[i])<=180);u[i]=out[i];}
    }
    assert(failed&&!std::strcmp(overloaded.fault,"SERVO_HOLD_LIMIT"));
    for(int x:out)assert(x==0);
    std::puts("PASS: continuous retained torque, explicit stop/reset, leases, no-motion latch, invalid sensors and unobservable slip");
}
