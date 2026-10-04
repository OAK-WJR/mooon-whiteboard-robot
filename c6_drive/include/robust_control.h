#pragma once
// Portable candidate controller. No GPIO, transport, dynamic allocation or Arduino.
// Empirical robustness is qualified by the accompanying simulation campaign;
// this is not a proof of stability or a qualified wall-holding safety controller.
#include <cmath>
#include <cstdint>

namespace wbb_robust {
constexpr float pi=3.14159265358979323846f;
constexpr float metresPerCount=.11938052f/2800, radius=.033738f;
inline float clip(float x,float lo,float hi){return std::fmin(hi,std::fmax(lo,x));}
inline float sign(float x){return x>0?1.f:x<0?-1.f:0.f;}
inline float wrap(float x){return std::atan2(std::sin(x),std::cos(x));}
inline bool finite(float x){return std::isfinite(x);}
struct Vec {float x=0,y=0,w=0;};
struct Model {float K,tau,ks;};
// sysid-model-03, positive / negative. Effective dynamics include sensor filtering.
constexpr Model measuredModels[3][2]={
 {{64.1858795f,.27849533f,54.6500725f},{62.3843104f,.27849533f,49.5755004f}},
 {{47.3364415f,.32756895f,2.69055701f},{48.6703189f,.32756895f,3.72966934f}},
 {{60.8947392f,.30203712f,43.1660876f},{58.5763912f,.27849533f,37.0985182f}}};
struct Config {
 float lambda=.20f,observerBandwidth=6.f,observerLimit=65.f;
 float accelerationFF=.7f,acceleration=11000.f,pwmRate=1200.f;
 float positionGain=3.f,headingGain=3.f,maxSpeed=.30f,maxYaw=1.2f;
 float bodyTimeout=.30f,commandTimeout=.30f;
 bool useObserver=true,requireBody=true;
};
enum class State : int {DISARMED,WAITING_FOR_BODY,TRACKING,HOLD,COAST,SLIP_DERATE,OUTPUT_LIMITED,
 COMMAND_STALE,ENCODER_INVALID,BODY_LOST,BAD_INPUT,CONTROL_LATE,STALL,FEEDBACK_INCONSISTENT,EMERGENCY_STOP};
inline const char* name(State s){
 switch(s){
 case State::DISARMED:return "DISARMED";case State::WAITING_FOR_BODY:return "WAITING_FOR_BODY";
 case State::TRACKING:return "TRACKING";case State::HOLD:return "HOLD";
 case State::COAST:return "COAST";
 case State::SLIP_DERATE:return "SLIP_DERATE";case State::OUTPUT_LIMITED:return "OUTPUT_LIMITED";
 case State::COMMAND_STALE:return "COMMAND_STALE";case State::ENCODER_INVALID:return "ENCODER_INVALID";
 case State::BODY_LOST:return "BODY_LOST";case State::BAD_INPUT:return "BAD_INPUT";
 case State::CONTROL_LATE:return "CONTROL_LATE";case State::STALL:return "STALL";
 case State::FEEDBACK_INCONSISTENT:return "FEEDBACK_INCONSISTENT";
 case State::EMERGENCY_STOP:return "EMERGENCY_STOP";}
 return "BAD_INPUT";
}
struct Input {
 float now=0,dt=.01f,commandStamp=0;
 Vec command; // Board-frame metres/s, radians/s. Fresh command required even in HOLD.
 float speed[3]={},applied[3]={}; // Encoder counts/s; duty actually applied last interval.
 bool encoderValid=true,bodyValid=false,estop=false;
 float bodyStamp=0;Vec bodyPose; // Capture timestamp, not arrival timestamp.
};
struct Output {
 int pwm[3]={};float reference[3]={},compensation[3]={},integral[3]={};
 Vec estimatedPose,goal;State state=State::DISARMED;
 bool fault=false,slip=false;float demandScale=1;
};
inline void toWheels(const Vec& v,float heading,float result[3]){
 constexpr float phi[3]={pi/3,pi,-pi/3};
 for(int i=0;i<3;i++){float a=phi[i]+heading;result[i]=(-std::sin(a)*v.x+std::cos(a)*v.y+radius*v.w)/metresPerCount;}
}

struct Wheel {
 float integral=0,load=0,estimate=0,comp=0,reference=0,stalled=0;
 bool initialized=false;
 void reset(){*this=Wheel{};}
 static float effective(float duty,const Model& m){return sign(duty)*std::fmax(0.f,std::fabs(duty)-m.ks);}
 int update(int index,float target,float measured,float applied,float dt,bool learn,bool hold,const Config& c,float velocityOmega=0,bool releaseStartup=false,bool coordinated=false){
  if(!hold&&!coordinated&&std::fabs(target)<1){reset();return 0;}
  const Model& obs=measuredModels[index][(applied!=0?applied:measured)<0];
  if(!initialized){estimate=measured;initialized=true;}
  const float a=std::exp(-dt/obs.tau),b=obs.K*(1-a),p=std::exp(-c.observerBandwidth*dt);
  float prediction=a*estimate+b*(effective(applied,obs)-load);
  float innovation=clip(measured-prediction,-2500,2500);
  estimate=prediction+(1-p*p/a)*innovation;
  // Low-speed friction is not identified: retain, rather than re-identify, load there.
  bool observing=c.useObserver&&learn&&std::fabs(applied)<245&&std::fabs(measured)>250;
  if(observing)load=clip(load-((1-p)*(1-p)/b)*innovation,-c.observerLimit,c.observerLimit);
  const float wanted=c.useObserver?load:0;
  comp+=clip(wanted-comp,-100*dt,100*dt);
  float acceleration=(target-reference)/dt;reference=target;
  const Model& m=measuredModels[index][(target!=0?target:measured)<0];
  float error=target-measured;
  if(std::fabs(error)<20)error=0;
  float kp=m.tau/(m.K*c.lambda),ki=.65f/(m.K*c.lambda);
  // Optional loaded wheel-speed PI, pole placement for K/(tau*s+1):
  // tau*s^2 + (1+K*kp)*s + K*ki, damping ratio 1.5. This is a nominal
  // moving-range design; static friction and delay require separate testing.
  if(velocityOmega>0){
   kp=std::fmax(0.f,(3.f*velocityOmega*m.tau-1.f)/m.K);
   ki=velocityOmega*velocityOmega*m.tau/m.K;
  }
  float ff=(target+c.accelerationFF*m.tau*acceleration)/m.K;
  float raw=ff+kp*error+integral+comp;
  // A continuous dead-zone inverse avoids a full ks jump at each zero crossing.
  const Model& drive=measuredModels[index][raw<0];
  float requested=raw+drive.ks*raw/(std::fabs(raw)+2.f);
  float bounded=clip(requested,-255,255);
  float limited=clip(bounded,applied-c.pwmRate*dt,applied+c.pwmRate*dt);
  // A completed breakaway pulse must not keep adding impulse while a downward
  // slew catches up. At this one transition, permit only a reduction toward
  // zero; increasing drive or reversing polarity still uses the normal slew.
  if(releaseStartup&&std::fabs(bounded)<std::fabs(applied)&&bounded*applied>=0)
   limited=bounded;
  int output=int(std::lround(clip(limited,-255,255)));
  // Track final saturated/rate-limited actuator contribution, including feedforward.
  float tracking=effective(float(output),drive)-raw;
  bool constrained=std::fabs(requested-limited)>.6f;
  if(learn)integral+=ki*error*dt;
  if(constrained)integral+=tracking*dt/.15f;
  integral=clip(integral,-120,120);
  if(std::fabs(target)>1200&&std::fabs(measured)<30&&std::fabs(applied)>140)stalled+=dt;
  else stalled=0;
  return output;
 }
};

class Controller {
 struct Sample {float time=0;float distance[3]={};};
 Config cfg_;Wheel wheels_[3];Output out_;
 Sample history_[64];int count_=0,head_=0;
 float distance_[3]={},previousBodyDistance_[3]={};
 float inconsistent_[3]={};
 float filteredTargets_[3]={};
 Vec body_,velocity_,goal_;float stamp_=-1,armedAt_=0,lastNow_=-1;
 float slipTimer_=0,clearTimer_=0,saturationTimer_=0,scale_=1;
 bool armed_=false,latched_=false,haveBody_=false,slipping_=false,haveBodyDistance_=false;
 bool sampleAt(float time,float out[3])const{
  if(count_<2)return false;
  int oldest=(head_-count_+64)%64;
  for(int n=1;n<count_;n++){
   const Sample& a=history_[(oldest+n-1)%64];const Sample& b=history_[(oldest+n)%64];
   if(time>=a.time-.00001f&&time<=b.time+.00001f){
    float u=clip((time-a.time)/(b.time-a.time),0,1);
    for(int j=0;j<3;j++)out[j]=a.distance[j]+u*(b.distance[j]-a.distance[j]);
    return true;
   }
  }return false;
 }
 void pushHistory(const Input& in){
  Sample& s=history_[head_];s.time=in.now;
  for(int j=0;j<3;j++){distance_[j]+=in.speed[j]*metresPerCount*in.dt;s.distance[j]=distance_[j];}
  head_=(head_+1)%64;if(count_<64)count_++;
 }
 bool acceptBody(const Input& in){
  if(!in.bodyValid||!finite(in.bodyStamp)||in.bodyStamp>in.now+.001f||in.now-in.bodyStamp>cfg_.bodyTimeout||in.bodyStamp<=stamp_+.00001f)return false;
  Vec p=in.bodyPose;if(!finite(p.x)||!finite(p.y)||!finite(p.w)||std::fabs(p.x)>10||std::fabs(p.y)>10)return false;
  const float elapsed=in.bodyStamp-stamp_;
  if(haveBody_){
   float dx=p.x-body_.x,dy=p.y-body_.y,dw=wrap(p.w-body_.w);
   if(std::hypot(dx,dy)>.9f*elapsed+.015f||std::fabs(dw)>6.f*elapsed+.08f)return false;
   Vec v{dx/elapsed,dy/elapsed,dw/elapsed};
   float alpha=1-std::exp(-elapsed/.06f);
   velocity_.x+=alpha*(v.x-velocity_.x);velocity_.y+=alpha*(v.y-velocity_.y);velocity_.w+=alpha*(v.w-velocity_.w);
   float ds[3];bool found=sampleAt(in.bodyStamp,ds);
   if(found&&haveBodyDistance_){
    float expected[3];toWheels(v,body_.w+.5f*dw,expected);float residual=0;
    for(int i=0;i<3;i++){
     float observed=(ds[i]-previousBodyDistance_[i])/elapsed,projected=expected[i]*metresPerCount;
     residual=std::fmax(residual,std::fabs(observed-projected));
     bool disagreement=std::fabs(observed)<.004f&&std::fabs(projected)>.025f&&
      (std::fabs(out_.reference[i])>400||std::abs(out_.pwm[i])>120);
     inconsistent_[i]=disagreement?inconsistent_[i]+elapsed:std::fmax(0.f,inconsistent_[i]-2*elapsed);
    }
    if(residual>.085f){slipTimer_+=elapsed;clearTimer_=0;}else{slipTimer_=std::fmax(0.f,slipTimer_-2*elapsed);clearTimer_+=elapsed;}
    if(slipTimer_>.18f)slipping_=true;
    if(clearTimer_>.5f)slipping_=false;
   }
   if(found)for(int i=0;i<3;i++)previousBodyDistance_[i]=ds[i];
   haveBodyDistance_=found;
  }else{
   goal_=p;haveBodyDistance_=sampleAt(in.bodyStamp,previousBodyDistance_);
  }
  body_=p;stamp_=in.bodyStamp;haveBody_=true;return true;
 }
 void zero(State reason,bool fault){
  for(auto& w:wheels_)w.reset();
  for(float& x:filteredTargets_)x=0;
  out_=Output{};out_.state=reason;out_.fault=fault;
  if(fault){latched_=true;armed_=false;}
 }
 public:
 explicit Controller(const Config& c=Config{}):cfg_(c){}
 const Config& config()const{return cfg_;}
 const Output& output()const{return out_;}
 void arm(float now){
  Config c=cfg_;*this=Controller(c);armed_=true;armedAt_=now;
  out_.state=State::WAITING_FOR_BODY;
  const float parameters[]={c.lambda,c.observerBandwidth,c.observerLimit,c.accelerationFF,c.acceleration,
   c.pwmRate,c.positionGain,c.headingGain,c.maxSpeed,c.maxYaw,c.bodyTimeout,c.commandTimeout,now};
  bool valid=true;for(float x:parameters)valid=valid&&finite(x);
  if(!valid||c.lambda<.12f||c.lambda>.6f||c.observerBandwidth<1||c.observerBandwidth>12||
     c.observerLimit<0||c.observerLimit>90||c.accelerationFF<0||c.accelerationFF>1||
     c.acceleration<=0||c.acceleration>20000||c.pwmRate<=0||c.pwmRate>2000||
     c.positionGain<0||c.positionGain>8||c.headingGain<0||c.headingGain>8||
     c.maxSpeed<=0||c.maxSpeed>.4f||c.maxYaw<=0||c.maxYaw>2||
     c.bodyTimeout<.1f||c.bodyTimeout>.3f||c.commandTimeout<.1f||c.commandTimeout>.3f){
   zero(State::BAD_INPUT,true);
  }
 }
 void stop(){if(latched_)return;armed_=false;zero(State::EMERGENCY_STOP,true);}
 const Output& update(const Input& in){
  if(in.estop){stop();return out_;}
  if(latched_||!armed_)return out_;
  if(!finite(in.dt)||in.dt<.005f||in.dt>.025f||!finite(in.now)||
     (lastNow_>=0&&(in.now<=lastNow_||in.now-lastNow_>.04f))){
   zero(State::CONTROL_LATE,true);return out_;
  }lastNow_=in.now;
  if(!finite(in.commandStamp)||in.commandStamp>in.now+.001f||in.now-in.commandStamp>cfg_.commandTimeout){zero(State::COMMAND_STALE,true);return out_;}
  if(!finite(in.command.x)||!finite(in.command.y)||!finite(in.command.w)){zero(State::BAD_INPUT,true);return out_;}
  for(int i=0;i<3;i++)if(!finite(in.speed[i])||std::fabs(in.speed[i])>30000||!finite(in.applied[i])||std::fabs(in.applied[i])>255){zero(State::BAD_INPUT,true);return out_;}
  if(!in.encoderValid){zero(State::ENCODER_INVALID,true);return out_;}
  pushHistory(in);acceptBody(in);
  for(float duration:inconsistent_)if(duration>.35f){zero(State::FEEDBACK_INCONSISTENT,true);return out_;}
  if(cfg_.requireBody&&(!haveBody_||in.now-stamp_>cfg_.bodyTimeout)){
   if(in.now-armedAt_<=cfg_.bodyTimeout){zero(State::WAITING_FOR_BODY,false);return out_;}
   zero(State::BODY_LOST,true);return out_;
  }
  if(!cfg_.requireBody&&std::hypot(in.command.x,in.command.y)+std::fabs(in.command.w)<.0001f){
   zero(State::COAST,false);return out_;
  }
  Vec estimate=body_;float age=haveBody_?clip(in.now-stamp_,0,.20f):0;
  estimate.x+=velocity_.x*age;estimate.y+=velocity_.y*age;estimate.w=wrap(estimate.w+velocity_.w*age);
  float peak=0,trackingError=0;
  for(int i=0;i<3;i++){
   peak=std::fmax(peak,std::fabs(in.applied[i]));
   trackingError=std::fmax(trackingError,std::fabs(out_.reference[i]-in.speed[i]));
  }
  saturationTimer_=peak>=240&&trackingError>400?saturationTimer_+in.dt:std::fmax(0.f,saturationTimer_-2*in.dt);
  bool hardLimited=saturationTimer_>.12f;
  // Fixed "50% speed" is not sufficient for an unknown load. Keep reducing
  // the common demand, all the way to zero motion, until actual output has margin.
  if(hardLimited)scale_=std::fmax(0.f,scale_-1.2f*in.dt);
  else if(peak<215&&trackingError<300)scale_=std::fmin(1.f,scale_+.12f*in.dt);
  if(slipping_)scale_=std::fmax(.0f,std::fmin(scale_,.30f));
  bool limited=hardLimited||scale_<.98f;
  Vec command=in.command;float norm=std::hypot(command.x,command.y);
  float factor=norm>cfg_.maxSpeed?cfg_.maxSpeed/norm:1;
  command.x*=factor*scale_;command.y*=factor*scale_;command.w=clip(command.w,-cfg_.maxYaw,cfg_.maxYaw)*scale_;
  goal_.x+=command.x*in.dt;goal_.y+=command.y*in.dt;goal_.w=wrap(goal_.w+command.w*in.dt);
  // Bound accumulated tracking debt during infeasible motion; scoring remains external.
  float ex=goal_.x-estimate.x,ey=goal_.y-estimate.y,n=std::hypot(ex,ey);
  float debt=slipping_?.015f:limited?.025f:.08f;
  if(n>debt){goal_.x=estimate.x+ex*debt/n;goal_.y=estimate.y+ey*debt/n;}
  // Never move the heading goal to hide accumulated yaw error. Limit corrective
  // yaw rate instead; translation yields when a large orientation error develops.
  Vec desired=command;
  if(cfg_.requireBody){
   desired.x+=cfg_.positionGain*(goal_.x-estimate.x);desired.y+=cfg_.positionGain*(goal_.y-estimate.y);
   desired.w+=cfg_.headingGain*clip(wrap(goal_.w-estimate.w),-.5f,.5f)+.5f*(command.w-velocity_.w);
  }
  float headingError=std::fabs(wrap(goal_.w-estimate.w));
  float headingScale=cfg_.requireBody?clip(1.f-(headingError-.10f)/.4f,.15f,1.f):1.f;
  desired.x*=headingScale;desired.y*=headingScale;
  norm=std::hypot(desired.x,desired.y);factor=norm>cfg_.maxSpeed?cfg_.maxSpeed/norm:1;
  desired.x*=factor;desired.y*=factor;desired.w=clip(desired.w,-cfg_.maxYaw,cfg_.maxYaw);
  float request[3];toWheels(desired,haveBody_?estimate.w:pi/2,request);
  // One common low-pass coefficient keeps wheel ratios and suppresses 20 Hz
  // position-noise steps before acceleration feedforward differentiates them.
  float referenceAlpha=1-std::exp(-in.dt/.06f);
  for(int i=0;i<3;i++){
   filteredTargets_[i]+=referenceAlpha*(request[i]-filteredTargets_[i]);request[i]=filteredTargets_[i];
  }
  float maximum=0,delta=0;
  for(float x:request)maximum=std::fmax(maximum,std::fabs(x));
  factor=maximum>10000?10000/maximum:1;
  for(int i=0;i<3;i++){request[i]*=factor;delta=std::fmax(delta,std::fabs(request[i]-out_.reference[i]));}
  float progress=delta>0?std::fmin(1.f,cfg_.acceleration*(slipping_?.3f:1.f)*in.dt/delta):1;
  bool hold=cfg_.requireBody;
  out_.fault=false;out_.slip=slipping_;out_.demandScale=scale_;out_.estimatedPose=estimate;out_.goal=goal_;
  out_.state=slipping_?State::SLIP_DERATE:limited?State::OUTPUT_LIMITED:
   (std::hypot(command.x,command.y)+std::fabs(command.w)<.0001f?State::HOLD:State::TRACKING);
  for(int i=0;i<3;i++){
   out_.reference[i]+=progress*(request[i]-out_.reference[i]);
   out_.pwm[i]=wheels_[i].update(i,out_.reference[i],in.speed[i],in.applied[i],in.dt,!slipping_&&!hardLimited,hold,cfg_);
   out_.compensation[i]=wheels_[i].comp;out_.integral[i]=wheels_[i].integral;
   if(wheels_[i].stalled>.5f){zero(State::STALL,true);break;}
  }
  return out_;
 }
};
}
