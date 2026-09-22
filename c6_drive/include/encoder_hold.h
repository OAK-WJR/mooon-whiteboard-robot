#pragma once
#include <cmath>
#include <cstdint>

// Relative odometry only. Wheel slip and a sliding chassis are unobservable.
// Same wheel order and body-frame axes as the established WASD interface.
namespace drive {
struct Pose { float x=0,y=0,yaw=0; };
struct EncoderHold {
    static constexpr float mpc=.11938052f/2800, radius=.033738f;
    static constexpr float maxSpeed=.08f,maxYaw=.6f; // Measured suspended hold envelope.
    Pose pose,goal,velocity,command;
    bool valid=false,active=false,holding=true,quiet=true;
    uint32_t stamp=0;
    float error=0,headingError=0;
    static float wrap(float a){return std::atan2(std::sin(a),std::cos(a));}
    static float clip(float v,float lo,float hi){return std::fmin(hi,std::fmax(lo,v));}
    static Pose body(const float wheel[3]){
        return {(wheel[2]-wheel[0])*mpc/1.732050808f,
            (wheel[0]-2*wheel[1]+wheel[2])*mpc/3,
            (wheel[0]+wheel[1]+wheel[2])*mpc/(3*radius)};
    }
    static void wheels(const Pose& v,float out[3]){
        out[0]=(-.866025404f*v.x+.5f*v.y+radius*v.yaw)/mpc;
        out[1]=(-v.y+radius*v.yaw)/mpc;
        out[2]=(.866025404f*v.x+.5f*v.y+radius*v.yaw)/mpc;
    }
    bool observe(uint32_t now,const float delta[3],float dt){
        valid=false;
        if(!std::isfinite(dt)||dt<.005f||dt>.025f)return false;
        for(int i=0;i<3;i++)if(!std::isfinite(delta[i])||std::fabs(delta[i])>30000*dt+2)return false;
        Pose step=body(delta);
        // SE(2) exponential: exact for a constant body twist over this sample.
        float a=std::fabs(step.yaw)<.0001f?1-step.yaw*step.yaw/6:std::sin(step.yaw)/step.yaw;
        float b=std::fabs(step.yaw)<.0001f?step.yaw/2:(1-std::cos(step.yaw))/step.yaw;
        float dx=a*step.x-b*step.y,dy=b*step.x+a*step.y;
        float c=std::cos(pose.yaw),s=std::sin(pose.yaw);
        pose.x+=c*dx-s*dy;pose.y+=s*dx+c*dy;pose.yaw=wrap(pose.yaw+step.yaw);
        const float alpha=1-std::exp(-dt/.04f);
        velocity.x+=alpha*(step.x/dt-velocity.x);
        velocity.y+=alpha*(step.y/dt-velocity.y);
        velocity.yaw+=alpha*(step.yaw/dt-velocity.yaw);
        stamp=now;valid=true;return true;
    }
    void resetGoal(){goal=pose;command={};active=false;holding=true;quiet=true;error=headingError=0;}
    void accept(const float value[3]){
        bool wasMoving=std::hypot(command.x,command.y)>.0001f;
        bool moving=std::hypot(value[0],value[1])>.0001f;
        // Release locks the measured position now; don't chase accrued path debt.
        if(wasMoving&&!moving){goal.x=pose.x;goal.y=pose.y;}
        if(std::fabs(command.yaw)>.0001f&&std::fabs(value[2])<.0001f)goal.yaw=pose.yaw;
        command={value[0],value[1],value[2]};active=true;
        holding=!moving&&std::fabs(command.yaw)<.0001f;
        if(!holding)quiet=false;
    }
    bool update(uint32_t now,float dt,float scale,float out[3]){
        for(int i=0;i<3;i++)out[i]=0;
        if(!valid||uint32_t(now-stamp)>25)return false;
        if(!active)return true;
        float c=std::cos(goal.yaw),s=std::sin(goal.yaw);
        float vx=(c*command.x-s*command.y)*scale,vy=(s*command.x+c*command.y)*scale;
        float wz=command.yaw*scale;
        goal.x+=vx*dt;goal.y+=vy*dt;goal.yaw=wrap(goal.yaw+wz*dt);
        float ex=goal.x-pose.x,ey=goal.y-pose.y,n=std::hypot(ex,ey);
        // Restrict catch-up during commanded travel. A hold target never follows drift.
        if(!holding&&n>.025f){ex*=.025f/n;ey*=.025f/n;goal.x=pose.x+ex;goal.y=pose.y+ey;n=.025f;}
        error=n;headingError=wrap(goal.yaw-pose.yaw);
        if(n>.08f||std::fabs(headingError)>.65f)return false;
        if(holding){
            // Hysteresis prevents static-friction pulses from hunting around a
            // sub-count ideal. These are estimated tolerances, not body accuracy.
            if(n>.004f||std::fabs(headingError)>.052f)quiet=false;
            if(n<.003f&&std::fabs(headingError)<.043f&&
               std::hypot(velocity.x,velocity.y)<.006f&&std::fabs(velocity.yaw)<.08f)quiet=true;
            if(quiet)return true;
        }
        const float deadband=.0008f;
        float gain=n>deadband?2.5f*(n-deadband)/n:0;
        vx+=gain*ex;vy+=gain*ey;
        c=std::cos(pose.yaw);s=std::sin(pose.yaw);
        Pose desired{c*vx+s*vy,-s*vx+c*vy,wz};
        // Damping uses wheel-derived velocity, not an independent body measurement.
        if(holding){desired.x-=3.5f*velocity.x;desired.y-=3.5f*velocity.y;}
        float yawError=std::fabs(headingError)>.006f?headingError-std::copysign(.006f,headingError):0;
        desired.yaw+=3*yawError+(holding?3.5f:.35f)*(wz-velocity.yaw);
        float speed=std::hypot(desired.x,desired.y),limit=holding?.08f:.40f;
        if(speed>limit){desired.x*=limit/speed;desired.y*=limit/speed;}
        desired.yaw=clip(desired.yaw,holding?-.8f:-1.2f,holding?.8f:1.2f);
        wheels(desired,out);
        float peak=1;for(int i=0;i<3;i++)peak=std::fmax(peak,std::fabs(out[i]));
        if(peak>10000)for(int i=0;i<3;i++)out[i]*=10000/peak;
        return true;
    }
};
}
