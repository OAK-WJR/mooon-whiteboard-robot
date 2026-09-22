#pragma once
#include "encoder_hold.h"
#include "robust_control.h"
#include <initializer_list>

namespace drive {
// Encoder-only candidate: profiled body reference, coupled position/velocity
// feedback and a bounded retained torque bias. No external-pose or load input.
// Development/simulation only until separately validated on supported hardware.
struct EncoderServo {
    Pose goal,profile;
    float integral[3]={},reference[3]={},previousReference[3]={};
    float positionError[3]={},stallTime[3]={};
    float recoveryTime[3]={},restTime[3]={},cooldown[3]={};
    int recoverySign[3]={};
    float scale=1,error=0,headingError=0;
    bool initialized=false,holding=true,limited=false;
    const char* fault=nullptr;int faultMotor=-1;
    static constexpr float maxPwm=255,holdPwm=180,pwmRate=1200;
    static float clip(float v,float lo,float hi){return EncoderHold::clip(v,lo,hi);}
    void reset(){*this=EncoderServo{};}
    bool fail(const char* why,int motor,int out[3]){
        fault=why;faultMotor=motor;for(int i=0;i<3;i++)out[i]=0;return false;
    }
    bool update(const Pose& pose,const Pose& command,const float measured[3],
                const float applied[3],float dt,int out[3]){
        for(int i=0;i<3;i++)out[i]=0;
        if(fault)return false;
        if(!std::isfinite(dt)||dt<.005f||dt>.025f)return fail("SERVO_BAD_DT",-1,out);
        for(float x:{pose.x,pose.y,pose.yaw,command.x,command.y,command.yaw})
            if(!std::isfinite(x))return fail("SERVO_BAD_INPUT",-1,out);
        if(std::hypot(command.x,command.y)>EncoderHold::maxSpeed+.000001f||
           std::fabs(command.yaw)>EncoderHold::maxYaw+.000001f)return fail("SERVO_BAD_INPUT",-1,out);
        for(int i=0;i<3;i++){
            if(!std::isfinite(measured[i])||std::fabs(measured[i])>30000)return fail("SERVO_BAD_SPEED",i,out);
            if(!std::isfinite(applied[i])||std::fabs(applied[i])>255)return fail("SERVO_BAD_DUTY",i,out);
        }
        if(!initialized){goal=pose;initialized=true;}
        const float ex=goal.x-pose.x,ey=goal.y-pose.y;
        error=std::hypot(ex,ey);headingError=EncoderHold::wrap(goal.yaw-pose.yaw);
        if(error>.08f||std::fabs(headingError)>.65f)return fail("SERVO_TRACKING_ERROR",-1,out);
        float c=std::cos(pose.yaw),s=std::sin(pose.yaw);
        EncoderHold::wheels({c*ex+s*ey,-s*ex+c*ey,headingError},positionError);
        // Slow the entire reference when any wheel accumulates tracking debt.
        // Never move the goal to hide error or change wheel ratios separately.
        float debt=0;for(float e:positionError)debt=std::fmax(debt,std::fabs(e)*EncoderHold::mpc);
        const float wantedScale=clip((.020f-debt)/.012f,0,1);
        scale+=clip(wantedScale-scale,-3*dt,.5f*dt);
        float dx=command.x*scale-profile.x,dy=command.y*scale-profile.y;
        float dw=command.yaw*scale-profile.yaw;
        float fraction=std::fmin(1.f,.25f*dt/std::fmax(.000001f,std::hypot(dx,dy)));
        fraction=std::fmin(fraction,2.f*dt/std::fmax(.000001f,std::fabs(dw)));
        profile.x+=fraction*dx;profile.y+=fraction*dy;profile.yaw+=fraction*dw;
        holding=std::hypot(command.x,command.y)<.0001f&&std::fabs(command.yaw)<.0001f&&
                std::hypot(profile.x,profile.y)<.0001f&&std::fabs(profile.yaw)<.0001f;
        float gc=std::cos(goal.yaw),gs=std::sin(goal.yaw);
        float vx=gc*profile.x-gs*profile.y,vy=gs*profile.x+gc*profile.y;
        EncoderHold::wheels({c*vx+s*vy,-s*vx+c*vy,profile.yaw},reference);
        goal.x+=vx*dt;goal.y+=vy*dt;goal.yaw=EncoderHold::wrap(goal.yaw+profile.yaw*dt);
        limited=scale<.98f;
        float intent[3];EncoderHold::wheels(command,intent);
        for(int i=0;i<3;i++){
            const auto& m=wbb_robust::measuredModels[i][reference[i]<0];
            // Pole-inspired PD coefficients for the identified moving-range
            // first-order wheel + position integrator; I rejects steady load.
            constexpr float omega=9.f,zeta=1.05f;
            float kp=omega*omega*m.tau/m.K;
            float kd=(2*zeta*omega*m.tau-1)/m.K;
            float ki=.8f*kp;
            float acceleration=(reference[i]-previousReference[i])/dt;
            previousReference[i]=reference[i];
            float ff=(reference[i]+.5f*m.tau*acceleration)/m.K;
            float friction=m.ks*reference[i]/(std::fabs(reference[i])+150.f);
            float raw=ff+friction+kp*positionError[i]+kd*(reference[i]-measured[i])+integral[i];
            float ceiling=holding?holdPwm:maxPwm;
            cooldown[i]=std::fmax(0.f,cooldown[i]-dt);
            int sign=positionError[i]>0?1:positionError[i]<0?-1:0;
            // Near the held goal, let retained PI torque settle slowly. Repeated
            // breakaway pulses here confuse stationary load balance with a jam.
            const float breakaway=i==0?160.f:140.f;
            bool needsMotion=std::fabs(positionError[i])>55&&(!holding||error>.008f||std::fabs(headingError)>.07f)&&std::fabs(raw)<breakaway;
            restTime[i]=needsMotion&&std::fabs(measured[i])<30?restTime[i]+dt:0;
            bool motionExit=recoveryTime[i]>0&&measured[i]*recoverySign[i]>80;
            if(!needsMotion||sign!=recoverySign[i]||motionExit)recoveryTime[i]=0;
            if(restTime[i]>.08f&&cooldown[i]==0&&recoveryTime[i]==0){
                recoveryTime[i]=.25f;cooldown[i]=.5f;recoverySign[i]=sign;restTime[i]=0;
            }
            bool recovering=recoveryTime[i]>0;
            float request=raw;
            if(recovering){
                request=sign>0?std::fmax(raw,breakaway):std::fmin(raw,-breakaway);
                recoveryTime[i]=std::fmax(0.f,recoveryTime[i]-dt);
                // Under static load, a sub-limit breakaway attempt can balance
                // torque without motion. Return to bounded PI rather than call
                // that invalid feedback. Demanded travel still has a deadline.
                if(!holding&&recoveryTime[i]==0&&measured[i]*sign<30)return fail("SERVO_START_NO_MOTION",i,out);
            }
            float desired=clip(request,-ceiling,ceiling);
            float output=clip(desired,applied[i]-pwmRate*dt,applied[i]+pwmRate*dt);
            // End a breakaway pulse promptly; rate-limiting this decrease adds
            // impulse after motion is detected. Only reduce same-sign drive.
            if(motionExit&&desired*applied[i]>=0&&std::fabs(desired)<std::fabs(applied[i]))output=desired;
            // Hard holding limit wins over the slew limit at the state boundary.
            out[i]=int(std::lround(clip(output,-ceiling,ceiling)));
            float tracking=std::fabs(request-output)>.6f?output-raw:0;
            float grow=ki*positionError[i];
            // Stop winding up on a sub-resolution held error while preserving
            // the existing torque bias. PD remains live if the wheel moves.
            if(holding&&std::fabs(positionError[i])<55&&std::fabs(measured[i])<100)grow=0;
            if((raw>out[i]+.6f&&grow>0)||(raw<out[i]-.6f&&grow<0))grow=0;
            if(!recovering)integral[i]=clip(integral[i]+(grow+tracking/.20f)*dt,-holdPwm,holdPwm);
            limited=limited||std::fabs(tracking)>.6f;
            // Zero speed at a held position is expected. A significant position
            // debt with sustained drive and no motion is still a latched fault.
            bool holdLimited=holding&&std::fabs(positionError[i])>100&&std::fabs(applied[i])>=holdPwm-1;
            // Demand derating must not hide a user's still-active motion demand.
            bool motionRequested=std::fabs(reference[i])>400||std::fabs(intent[i])>400;
            bool stuck=(motionRequested||holdLimited)&&std::fabs(positionError[i])>100&&std::fabs(measured[i])<30&&std::fabs(applied[i])>100;
            stallTime[i]=stuck?stallTime[i]+dt:0;
            if(stallTime[i]>.35f)return fail(holdLimited?"SERVO_HOLD_LIMIT":"SERVO_NO_MOTION",i,out);
        }
        return true;
    }
};
}
