#pragma once
#include "robust_control.h"

// Wheel-only hardware adapter. No pose estimator, slip detector or wall hold.
// Reuses the simulated Wheel law unchanged. Command leases and hardware faults
// remain owned by drive::Control; actual LEDC duty is supplied by the caller.
namespace drive {
struct RobustWheels {
    wbb_robust::Wheel wheel[3];
    wbb_robust::Config config;
    float filtered[3]={},reference[3]={},compensation[3]={};
    float scale=1,saturated=0;
    bool limited=false;
    bool loadedPi=true; // Development comparison can disable the loaded PI.
    float startup[3]={};int direction[3]={};
    float holdRest[3]={},holdCooldown[3]={};
    float loadedNoMotion[3]={};
    const char* fault=nullptr;
    int faultMotor=-1; // zero-based; -1 denotes a controller-wide fault

    bool fail(const char* why,int motor,int pwm[3]){
        fault=why;faultMotor=motor;
        for(int i=0;i<3;i++)pwm[i]=0;
        return false;
    }

    void reset(){*this=RobustWheels{};}
    bool update(const float target[3],const float measured[3],const float applied[3],
                float dt,int pwm[3],bool hold=false,const float* intent=nullptr,bool coordinated=false){
        for(int i=0;i<3;i++)pwm[i]=0;
        fault=nullptr;faultMotor=-1;
        if(!std::isfinite(dt)||dt<.005f||dt>.025f)return fail("ROBUST_BAD_DT",-1,pwm);
        bool moving=hold||coordinated;
        float peak=0,error=0;
        for(int i=0;i<3;i++){
            if(!std::isfinite(target[i])||std::fabs(target[i])>10000)return fail("ROBUST_BAD_TARGET",i,pwm);
            if(!std::isfinite(measured[i])||std::fabs(measured[i])>30000)return fail("ROBUST_BAD_SPEED",i,pwm);
            if(!std::isfinite(applied[i])||std::fabs(applied[i])>255)return fail("ROBUST_BAD_DUTY",i,pwm);
            moving=moving||std::fabs(target[i])>=1;
            peak=std::fmax(peak,std::fabs(applied[i]));
            error=std::fmax(error,std::fabs(reference[i]-measured[i]));
        }
        if(!moving){reset();return true;}
        saturated=peak>=240&&error>400?saturated+dt:std::fmax(0.f,saturated-2*dt);
        bool constrained=saturated>.12f;
        if(constrained)scale=std::fmax(0.f,scale-1.2f*dt);
        else if(peak<215&&error<300)scale=std::fmin(1.f,scale+.12f*dt);
        limited=constrained||scale<.98f;
        const float alpha=1-std::exp(-dt/.06f);
        const bool loadedSpeed=loadedPi&&!hold&&!intent;
        // Loaded speed-loop trial uses the error integrator as the sole load
        // compensator. The suspended fitted model is retained for nominal PI/FF;
        // the separate observer is not qualified for this contact load.
        auto speedConfig=config;
        if(loadedSpeed){speedConfig.useObserver=false;speedConfig.pwmRate=600.f;}
        float delta=0;bool cutPulse[3]={},releaseStartup[3]={};
        for(int i=0;i<3;i++){
            holdCooldown[i]=std::fmax(0.f,holdCooldown[i]-dt);
            int sign=target[i]>0?1:target[i]<0?-1:0;
            if(sign!=direction[i]){
                cutPulse[i]=hold&&startup[i]>0;
                // Measured breakaway duty is 160/255 for intermittent M1
                // starts, 140/255 for M2/M3. The identified
                // moving-range model does not cover low-speed static friction.
                // Apply a short kick only from rest at low targets, never hold
                // it indefinitely or trigger it on an already moving reversal.
                bool intentional=!intent||(std::fabs(intent[i])>=400&&intent[i]*sign>0);
                // Loaded velocity mode uses one continuous PI law from rest.
                // The old separate kick/handover caused visible chassis jerk.
                startup[i]=!loadedSpeed&&!hold&&intentional&&sign&&std::fabs(target[i])<2200&&std::fabs(measured[i])<100?.25f:0;
                if(loadedSpeed&&startup[i]>0)holdCooldown[i]=.30f;
                direction[i]=sign;
            }
            // Zero-target wheels must not spin during a single-wheel bench test.
            if(hold||intent||loadedSpeed){
                // Measured hold residual: the moving-range model cannot start a
                // stopped wheel at a few hundred cps. Permit one bounded pulse
                // after confirmed rest, with a cooldown and immediate motion exit.
                bool needsMotion=std::fabs(target[i])>80&&
                    (hold||(loadedSpeed&&std::fabs(target[i])<2200)||
                     (intent&&std::fabs(intent[i])>=400&&intent[i]*target[i]>0));
                holdRest[i]=needsMotion&&std::fabs(measured[i])<30?holdRest[i]+dt:0;
                if(!needsMotion)startup[i]=0;
                if(!loadedSpeed&&holdRest[i]>.08f&&holdCooldown[i]==0&&startup[i]==0){
                    startup[i]=.25f;holdCooldown[i]=loadedSpeed?.30f:.50f;holdRest[i]=0;
                }
            }else holdRest[i]=0;
            // A moving heading loop can ask the normally idle wheel to pass
            // smoothly through zero. Preserve its PI state at that crossing;
            // Control ends this exception when deceleration settles. Ordinary
            // uncoordinated all-zero commands still reset immediately.
            if(!hold&&!coordinated&&std::fabs(target[i])<1){filtered[i]=reference[i]=0;wheel[i].reset();}
            else filtered[i]+=alpha*(target[i]*scale-filtered[i]);
            delta=std::fmax(delta,std::fabs(filtered[i]-reference[i]));
        }
        float progress=delta>0?std::fmin(1.f,config.acceleration*dt/delta):1;
        for(int i=0;i<3;i++){
            reference[i]+=progress*(filtered[i]-reference[i]);
            // Retain the same sustained-high-drive no-feedback deadline while
            // allowing the PI to build breakaway torque without pulse switching.
            loadedNoMotion[i]=loadedSpeed&&std::fabs(target[i])>80&&
                std::fabs(measured[i])<30&&std::fabs(applied[i])>100?
                loadedNoMotion[i]+dt:0;
            if(loadedNoMotion[i]>.35f)return fail("START_NO_MOTION",i,pwm);
            // M1's stronger breakaway pulse must finish after the first
            // sustained encoder motion; waiting to 500 cps caused overshoot.
            const float movingThreshold=hold?150.f:std::fmin(loadedSpeed?80.f:(i==0?150.f:500.f),std::fabs(target[i])*.35f);
            if(measured[i]*direction[i]>=movingThreshold){
                releaseStartup[i]=loadedSpeed&&startup[i]>0;
                cutPulse[i]=cutPulse[i]||(hold&&startup[i]>0);startup[i]=0;
            }
            if(cutPulse[i]){pwm[i]=0;wheel[i].reset();wheel[i].reference=reference[i];}
            else if(startup[i]>0){
                // Freeze PI/observer while using the measured breakaway rule;
                // seed the dynamic model at the actual sample for handover.
                // A loaded recovery must retain the PI/observer compensation
                // already learned. Do not wind up the PI during the pulse.
                if(!loadedSpeed)wheel[i].reset();wheel[i].reference=reference[i];
                wheel[i].estimate=measured[i];wheel[i].initialized=true;
                const float base=i==0?160.f:140.f;
                const float breakaway=loadedSpeed?base+(180.f-base)*(1.f-startup[i]/.25f):base;
                pwm[i]=int(std::lround(wbb_robust::clip(breakaway*direction[i],
                    applied[i]-config.pwmRate*dt,applied[i]+config.pwmRate*dt)));
                startup[i]=std::fmax(0.f,startup[i]-dt);
                // A pulse without encoder motion is a fault, not a reason to
                // repeat pulses forever. The owning Control clears all outputs.
                if((hold||intent||loadedSpeed)&&startup[i]==0&&measured[i]*direction[i]<30)return fail("START_NO_MOTION",i,pwm);
            }else pwm[i]=wheel[i].update(i,reference[i],measured[i],applied[i],dt,!constrained,hold,speedConfig,
                                        loadedPi&&!hold&&!intent?9.f:0.f,releaseStartup[i],coordinated);
            compensation[i]=wheel[i].comp;
        }
        return true;
    }
};
}
