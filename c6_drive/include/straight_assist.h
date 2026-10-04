#pragma once
#include "encoder_hold.h"

namespace drive {
// Experimental moving-only heading loop around the existing wheel PI.
// Uses wheel odometry, never claims independent chassis heading or wall hold.
struct StraightAssist {
    bool active=false;
    float goal=0,error=0,yawRate=0,correctionCps=0;
    void reset(){*this=StraightAssist{};}
    bool update(const float requested[3],const EncoderHold& od,uint32_t now,float dt,float out[3],bool braking=false){
        for(int i=0;i<3;i++){
            out[i]=requested[i];
            if(!std::isfinite(requested[i]))return false;
        }
        if(!std::isfinite(dt)||dt<.005f||dt>.025f)return false;
        Pose command=EncoderHold::body(requested);
        float speed=std::hypot(command.x,command.y);
        // Explicit turns, individual-wheel tests, zero/release and speeds above
        // the tested development region keep their original wheel commands.
        if((!braking&&speed<.005f)||speed>.08001f||std::fabs(command.yaw)>.0005f){reset();return true;}
        if(!od.valid||uint32_t(now-od.stamp)>25||!std::isfinite(od.pose.yaw)||
           !std::isfinite(od.velocity.yaw))return false;
        if(!active){active=true;goal=od.pose.yaw;}
        error=EncoderHold::wrap(goal-od.pose.yaw);
        // Bound the outer loop rather than storing an ever-growing correction.
        // Rate damping uses encoder velocity already filtered by EncoderHold.
        float wanted=EncoderHold::clip(4.f*error-.6f*od.velocity.yaw,-.35f,.35f);
        yawRate+=EncoderHold::clip(wanted-yawRate,-2.f*dt,2.f*dt);
        correctionCps=yawRate*EncoderHold::radius/EncoderHold::mpc;
        // A common wheel increment changes angular velocity without changing
        // the nominal body translation in the three-wheel kinematic mapping.
        for(int i=0;i<3;i++)out[i]+=correctionCps;
        return true;
    }
};
}
