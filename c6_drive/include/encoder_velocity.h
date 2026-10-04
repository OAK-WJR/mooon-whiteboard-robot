#pragma once
#include <cstdint>
#include <cmath>

namespace drive {
// Access is protected by encoderMux on hardware. ISR recording uses integer
// operations only; division/filtering happens in the 100 Hz control task.
struct EncoderEdgeWindow {
    uint32_t times[16]={};
    uint8_t head=0,used=0;
    int8_t direction=0;
    __attribute__((always_inline)) inline void clear(){head=used=0;direction=0;}
    __attribute__((always_inline)) inline void edge(uint32_t us,int step){
        if(step!=1&&step!=-1){clear();return;}
        if(direction!=step||(used&&uint32_t(us-times[(head+15)&15])>60000))clear();
        direction=step;times[head]=us;head=(head+1)&15;if(used<16)++used;
    }
};
struct EncoderVelocitySample {float cps=0,alpha=.4f;bool period=false;};

inline EncoderVelocitySample encoderVelocity(const EncoderEdgeWindow& h,uint32_t now,
                                             int32_t delta,float dt){
    EncoderVelocitySample out;
    if(!std::isfinite(dt)||dt<=0)return out;
    out.cps=float(delta)/dt;
    if(h.used<5||!h.direction)return out;
    // Average one or two full quadrature cycles, avoiding A/B duty asymmetry.
    unsigned intervals=h.used>=9?8:4;
    uint32_t last=h.times[(h.head+15)&15];
    uint32_t span=last-h.times[(h.head+15-intervals)&15];
    uint32_t age=now-last;
    if(span==0||span>60000)return out;
    float edgeUs=float(span)/intervals;
    float expiry=std::fmin(60000.f,std::fmax(20000.f,6.f*edgeUs));
    if(age>expiry)return out; // Never retain an old nonzero speed after stopping.
    if(delta&&((delta>0)!=(h.direction>0)))return out;
    float rate=1e6f/edgeUs;
    // If an expected next edge is late, decay the reciprocal estimate. No
    // extrapolation of motion beyond the finite no-edge expiry above.
    if(age>2*edgeUs)rate=std::fmin(rate,2e6f/float(age));
    float weight=std::fmax(0.f,std::fmin(1.f,(2000.f-rate)/1000.f));
    if(weight==0)return out; // Count windows remain appropriate at high speed.
    out.cps=weight*h.direction*rate+(1-weight)*out.cps;
    out.alpha=.4f+.25f*weight;out.period=true;
    return out;
}
}
