#include "encoder_load_servo.h"
#include <cassert>
#include <cstdio>
#include <limits>

int main(){
    // Independent input/output fixture: the identified moving motor is supplied
    // exactly enough duty for 500 cps plus an unknown 35-PWM equivalent load.
    // The estimator must recover the load without being given its value.
    drive::EncoderLoadServo servo;
    float speed[3]={500,500,500},duty[3];int out[3];
    for(int i=0;i<3;i++){
        const auto& model=wbb_robust::measuredModels[i][0];
        duty[i]=model.ks+500/model.K+35;
    }
    drive::Pose spin{0,0,.4f};
    for(int n=0;n<500;n++){
        const auto pose=servo.goal;
        assert(servo.update(pose,spin,speed,duty,.01f,out));
        for(int i=0;i<3;i++){
            assert(std::isfinite(servo.loadEstimate[i]));
            assert(std::fabs(servo.integral[i])<=servo.holdPwm);
            assert(std::abs(out[i])<=255);
        }
    }
    for(float load:servo.loadEstimate)assert(std::fabs(load-35)<.5f);
    // Stop/reset cannot leak learned support duty into a later arm/run.
    servo.reset();
    for(int i=0;i<3;i++)assert(servo.loadEstimate[i]==0&&servo.loadCompensation[i]==0&&servo.integral[i]==0);
    drive::Pose zero;float stopped[3]={};
    assert(servo.update(zero,zero,stopped,stopped,.01f,out));
    for(int x:out)assert(x==0);
    // At rest, absence of movement must not itself cause estimated load to rise.
    servo.integral[0]=60;servo.loadEstimate[0]=servo.loadCompensation[0]=60;duty[0]=60;duty[1]=duty[2]=0;
    for(int n=0;n<200;n++)assert(servo.update(zero,zero,stopped,duty,.01f,out));
    assert(servo.integral[0]==60&&out[0]==60&&servo.loadEstimate[0]==60);
    // Validation runs before any observer update; failed runs stay latched.
    const auto before=servo.loadEstimate[0];speed[2]=std::numeric_limits<float>::quiet_NaN();
    assert(!servo.update(zero,zero,speed,duty,.01f,out));
    assert(servo.loadEstimate[0]==before&&servo.faultMotor==2);
    for(int x:out)assert(x==0);
    assert(!servo.update(zero,zero,stopped,stopped,.01f,out));
    std::puts("PASS: unknown moving load estimation, bounded compensation, reset, stationary retention and invalid feedback latch");
}
