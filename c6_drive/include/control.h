#pragma once
#include <stdint.h>
#include <math.h>
#include "robust_wheels.h"
#include "encoder_hold.h"
#include "encoder_servo.h"
#include "straight_assist.h"

namespace drive {
constexpr float countsPerTurn = 2800; // Existing 700 CPR, quadrature x4; verify mechanically.
constexpr float metresPerCount = 0.11938052f / countsPerTurn; // Existing 38mm wheel.
constexpr float maxWheelSpeed = 10000; // counts/s; command ceiling, not a loaded-speed guarantee.
constexpr float integralLimit = 100; // M2 needs ~95 PWM correction at 9500 counts/s.
inline float clamp(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }
struct Gains { float kp=.003f, ki=.01f, kv=.01f, ks=0; };
inline bool valid(const Gains& g) {
    return isfinite(g.kp)&&isfinite(g.ki)&&isfinite(g.kv)&&isfinite(g.ks)&&
        g.kp>=0&&g.kp<=.2f&&g.ki>=0&&g.ki<=1&&g.kv>=0&&g.kv<=.2f&&g.ks>=0&&g.ks<=180;
}
struct FeedbackWatch {
    uint32_t since=0, count=0, edges=0;
    int direction=0, suspect=0;
    // Two consecutive 100ms windows with many edges but almost no net travel
    // indicate unusable feedback (e.g. one missing quadrature channel).
    bool update(uint32_t now,uint32_t c,uint32_t e,float pwm) {
        int dir=pwm>100?1:(pwm<-100?-1:0);
        if(!dir||dir!=direction){since=now;count=c;edges=e;direction=dir;suspect=0;return true;}
        if(uint32_t(now-since)<100)return true;
        uint32_t de=e-edges;float dc=fabsf(float(int32_t(c-count)));
        suspect=(de>50&&dc<.25f*de)?suspect+1:0;
        since=now;count=c;edges=e;
        return suspect<2;
    }
};
struct Wheel {
    Gains gains[2]; // positive, negative
    float measured=0, target=0, ramp=0, integral=0, output=0;
    float startupTime=0;
    int direction=0;
    uint32_t stalled=0;
    void reset() { target=ramp=integral=output=startupTime=0;direction=0;stalled=0; }
    void update(float dt) {
        if (fabsf(target)<1) { reset(); return; }
        // Require passage through zero before reversing drive polarity.
        float goal=(target*ramp<0)?0:target;
        ramp += clamp(goal-ramp,-12000*dt,12000*dt);
        int dir=ramp>0?1:(ramp<0?-1:0);
        if(dir&&dir!=direction){direction=dir;startupTime=.25f;integral=0;}
        if(measured*dir>500)startupTime=0;
        bool starting=startupTime>0&&dir;
        startupTime=fmaxf(0,startupTime-dt);
        const auto& g=gains[ramp<0?1:0];
        const float error=ramp-measured;
        float ff=g.kv*ramp+(ramp>0?g.ks:(ramp<0?-g.ks:0));
        float pre=ff+g.kp*error+integral;
        if (!starting&&!((pre>=255&&error>0)||(pre<=-255&&error<0)))
            integral=clamp(integral+g.ki*error*dt,-integralLimit,integralLimit);
        float desired=clamp(ff+g.kp*error+integral,-255,255);
        // Coasting is preferable to active opposite torque near a low-speed setpoint.
        desired=ramp>=0?fmaxf(0,desired):fminf(0,desired);
        // Measured breakaway PWM: 140 starts the suspended wheels. End as soon
        // as feedback confirms motion; never extend the 250ms startup window.
        if(starting)desired=dir>0?fmaxf(140,desired):fminf(-140,desired);
        output+=clamp(desired-output,-1200*dt,1200*dt);
    }
};
// Latched before stop clears controller state. Retained across user stops,
// mode changes and re-arming; the first fault of a new armed run replaces it.
struct FaultSnapshot {
    const char* code=nullptr;
    const char* mode="legacy";
    uint32_t ms=0,run=0;
    int motor=0; // physical motor 1..3; 0 denotes a controller-wide fault
    float dt=0,target[3]={},reference[3]={},speed[3]={},applied[3]={};
    Pose pose;
    float positionError=0,headingError=0;
};
struct Control {
    Wheel wheel[3];
    RobustWheels robust;
    bool robustMode=false;
    // Explicit manual floor trial, distinct from full-range qualification.
    // Session-local; velocity limits are enforced here, not only by the page.
    bool floorTrial=false;
    static constexpr float floorSpeed=.08f, floorYaw=.4f;
    bool headingAssistEnabled=false; // USB development setting; not persisted/qualified.
    StraightAssist headingAssist;
    bool coordinatedBraking=false;
    float brakeStart[3]={},brakeTime=0,brakeQuiet=0;
    EncoderHold odometry;
    bool holdMode=false;
    EncoderServo servo;
    bool servoMode=false; // Simulation/development selector; no web/USB route yet.
    float applied[3]={}; // Actual signed actuator duty from the previous interval.
    float oppositeTime[3]={};
    uint32_t lease=0, last=0, sequence=0;
    bool armed=false, open=false;
    const char* reason="BOOT";
    int raw[3]={};
    FaultSnapshot lastFault;
    uint32_t run=0;
    bool recordedFault=false;
    void faultStop(const char* why,uint32_t now,float dt,int motor=-1){
        if(armed&&!recordedFault){
            lastFault={};lastFault.code=why;lastFault.mode=mode();
            lastFault.ms=now;lastFault.dt=dt;lastFault.motor=motor+1;lastFault.run=run;
            lastFault.pose=odometry.pose;lastFault.positionError=odometry.error;
            lastFault.headingError=odometry.headingError;
            for(int i=0;i<3;i++){
                lastFault.target[i]=wheel[i].target;
                lastFault.reference[i]=servoMode?servo.reference[i]:robustMode&&!open?robust.reference[i]:wheel[i].ramp;
                lastFault.speed[i]=wheel[i].measured;lastFault.applied[i]=applied[i];
            }
            recordedFault=true;
        }
        stop(why);
    }
    void stop(const char* why) {
        armed=false; lease=0; open=false; reason=why;
        robust.reset();
        headingAssist.reset();
        coordinatedBraking=false;brakeTime=brakeQuiet=0;
        for(float& v:brakeStart)v=0;
        servo.reset();
        odometry.resetGoal();
        for(int i=0;i<3;i++){ wheel[i].reset();raw[i]=0;applied[i]=0;oppositeTime[i]=0; }
    }
    // A late packet must not erase the fault that already stopped the wheels.
    void reject(const char* why) { if(armed)stop(why); }
    void arm(uint32_t now,uint32_t nonce) { stop("READY");++run;recordedFault=false;armed=true;last=now;lease=nonce;sequence=0; }
    bool selectRobust(bool enabled){
        if(armed)return false;
        if(floorTrial)headingAssistEnabled=false;
        floorTrial=false;
        stop("CONTROLLER_CHANGED");robustMode=enabled;holdMode=servoMode=false;return true;
    }
    bool selectFloorTrial(){
        if(!selectRobust(true))return false;
        floorTrial=true;headingAssistEnabled=true;return true;
    }
    bool selectHold(){if(!selectRobust(true))return false;holdMode=true;return true;}
    bool selectServo(){if(!selectHold())return false;servoMode=true;return true;}
    const char* mode()const{return floorTrial?"floor_trial":servoMode?"encoder_servo":holdMode?"encoder_hold":robustMode?"robust_wheels":"legacy";}
    void observe(uint32_t now,const float delta[3],float dt){
        if(!odometry.observe(now,delta,dt)&&holdMode&&armed)faultStop("ODOMETRY_INVALID",now,dt);
    }
    bool acceptBody(uint32_t now,uint32_t nonce,uint32_t seq,const float values[3]){
        if(!holdMode||!robustMode||!armed||nonce!=lease||seq<=sequence||uint32_t(now-last)>300)return false;
        for(int i=0;i<3;i++)if(!isfinite(values[i]))return false;
        if(std::hypot(values[0],values[1])>EncoderHold::maxSpeed+.000001f||fabsf(values[2])>EncoderHold::maxYaw||!odometry.valid)return false;
        last=now;sequence=seq;open=false;odometry.accept(values);
        reason=odometry.holding?"ENCODER_HOLD":"ODOM_DRIVE";return true;
    }
    bool accept(uint32_t now,uint32_t nonce,uint32_t seq,const float values[3],bool pwm) {
        if(holdMode)return false; // No raw or wheel command can bypass the body-mode contract.
        if(!armed||nonce!=lease||seq<=sequence||uint32_t(now-last)>300) return false;
        for(int i=0;i<3;i++) if(!isfinite(values[i])||fabsf(values[i])>(pwm?255:maxWheelSpeed)) return false;
        if(floorTrial){
            if(pwm)return false;
            const Pose v=EncoderHold::body(values);
            if(std::hypot(v.x,v.y)>floorSpeed+.000001f||std::fabs(v.yaw)>floorYaw+.00001f)return false;
        }
        bool zero=true;for(int i=0;i<3;i++)zero=zero&&fabsf(values[i])<1;
        // Only a normal zero-velocity command requests coordinated deceleration.
        // Explicit STOP, lease loss, faults and raw PWM retain immediate zeroing.
        if(!pwm&&zero&&headingAssistEnabled&&headingAssist.active&&!coordinatedBraking){
            coordinatedBraking=true;brakeTime=brakeQuiet=0;
            for(int i=0;i<3;i++)brakeStart[i]=wheel[i].target;
        }else if(pwm||!zero){coordinatedBraking=false;brakeTime=brakeQuiet=0;}
        last=now;sequence=seq;reason=pwm?"BENCH":"DRIVE";
        if(open!=pwm){for(auto& w:wheel)w.reset();robust.reset();headingAssist.reset();}
        open=pwm;
        for(int i=0;i<3;i++){raw[i]=pwm?int(values[i]):0;wheel[i].target=pwm?0:values[i];}
        return true;
    }
    void tick(uint32_t now,float dt) {
        if(!armed)return;
        if(uint32_t(now-last)>300){faultStop("LINK_TIMEOUT",now,dt);return;}
        if(!isfinite(dt)||dt<=0||dt>.06f){faultStop("CONTROL_LATE",now,dt);return;}
        if(holdMode&&!servoMode){
            float target[3];
            if(!odometry.update(now,dt,robust.scale,target)){faultStop("ODOMETRY_HOLD_FAULT",now,dt);return;}
            for(int i=0;i<3;i++)wheel[i].target=target[i];
        }
        if(servoMode){
            if(!odometry.valid||uint32_t(now-odometry.stamp)>25){faultStop("ODOMETRY_HOLD_FAULT",now,dt);return;}
            if(!odometry.active)return;
            float measured[3];int outputs[3];
            for(int i=0;i<3;i++)measured[i]=wheel[i].measured;
            if(!servo.update(odometry.pose,odometry.command,measured,applied,dt,outputs)){
                faultStop(servo.fault,now,dt,servo.faultMotor);return;
            }
            odometry.goal=servo.goal;odometry.error=servo.error;odometry.headingError=servo.headingError;
            odometry.holding=servo.holding;
            odometry.quiet=servo.holding&&servo.error<.003f&&std::fabs(servo.headingError)<.043f;
            for(int i=0;i<3;i++){
                wheel[i].target=wheel[i].ramp=servo.reference[i];wheel[i].output=outputs[i];wheel[i].integral=servo.integral[i];
            }
        }else if(robustMode&&!open){
            float targets[3],measured[3],intent[3];int outputs[3];
            for(int i=0;i<3;i++){targets[i]=wheel[i].target;measured[i]=wheel[i].measured;}
            if(headingAssistEnabled&&!holdMode){
                if(coordinatedBraking){
                    brakeTime+=dt;bool still=true;
                    for(int i=0;i<3;i++)still=still&&std::fabs(measured[i])<50;
                    brakeQuiet=still?brakeQuiet+dt:0;
                    if(brakeQuiet>=.04f){
                        coordinatedBraking=false;headingAssist.reset();robust.reset();
                    }else if(brakeTime>.55f){faultStop("COORDINATED_STOP_TIMEOUT",now,dt);return;}
                    if(coordinatedBraking){
                        float fraction=std::fmax(0.f,1.f-brakeTime/.20f);
                        for(int i=0;i<3;i++)targets[i]=brakeStart[i]*fraction;
                    }
                }
                float corrected[3];
                if(!headingAssist.update(targets,odometry,now,dt,corrected,coordinatedBraking)){
                    faultStop("HEADING_ODOMETRY",now,dt);return;
                }
                for(int i=0;i<3;i++)targets[i]=corrected[i];
            }
            if(holdMode)EncoderHold::wheels(odometry.command,intent);
            if(!robust.update(targets,measured,applied,dt,outputs,holdMode&&odometry.active&&odometry.holding&&!odometry.quiet,holdMode?intent:nullptr,headingAssistEnabled&&!holdMode&&headingAssist.active)){
                faultStop(robust.fault,now,dt,robust.faultMotor);return;
            }
            for(int i=0;i<3;i++){
                wheel[i].output=outputs[i];wheel[i].ramp=robust.reference[i];
                wheel[i].integral=robust.wheel[i].integral;
            }
        }
        for(int i=0;i<3;i++) {
            auto& w=wheel[i];
            if(open)w.output=raw[i];else if(!robustMode)w.update(dt);
            // Stationary hold can require torque. Still fault if commanded motion
            // has no feedback, or high stationary duty cannot be justified safely.
            bool holding=holdMode&&odometry.active&&odometry.holding;
            if(!servoMode&&fabsf(w.output)>100&&fabsf(w.measured)<30&&
               (!holding||fabsf(w.ramp)>400||fabsf(w.output)>180)){
                if(!w.stalled)w.stalled=now;
                if(uint32_t(now-w.stalled)>350){faultStop("NO_ENCODER",now,dt,i);return;}
            }else w.stalled=0;
            // Reference reversal precedes physical reversal. Evaluate sustained
            // disagreement only after actual drive has the requested sign and
            // meaningful duty; coasting and the PWM slew are not polarity faults.
            bool opposite=!open&&fabsf(w.ramp)>500&&w.measured*w.ramp<0&&fabsf(w.measured)>400&&
                applied[i]*w.ramp>0&&fabsf(applied[i])>(holdMode?100.f:50.f);
            oppositeTime[i]=opposite?oppositeTime[i]+dt:0;
            // Apply the same continuous dwell in ordinary driving and holding.
            // Persistent opposite travel still stops and retains its snapshot.
            if(opposite&&oppositeTime[i]>.25f){
                faultStop("ENCODER_DIRECTION",now,dt,i);return;
            }
        }
    }
};
}
