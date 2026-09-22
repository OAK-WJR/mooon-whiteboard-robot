#include "robust_control.h"
#include <cassert>
#include <cstdio>
#include <limits>
#include <initializer_list>
using namespace wbb_robust;

static Input input(float time){
 Input i;i.now=i.commandStamp=i.bodyStamp=time;i.bodyValid=true;i.bodyPose.w=pi/2;return i;
}
static void zero(const Output& o){for(int p:o.pwm)assert(p==0);}
static void fault(const Output& o,State s){assert(o.fault&&o.state==s);zero(o);}
int main(){
 static_assert(sizeof(Controller)<4096,"Keep the embedded controller bounded");
 {Controller c;zero(c.update(input(0)));c.arm(0);auto i=input(0);i.command.x=.14f;
  const auto& o=c.update(i);assert(!o.fault);assert(std::fabs(o.reference[0]+o.reference[1]+o.reference[2])<.002f);
  c.stop();fault(c.output(),State::EMERGENCY_STOP);i.now=i.commandStamp=.01f;fault(c.update(i),State::EMERGENCY_STOP);
  c.arm(.02f);assert(!c.update(input(.02f)).fault);}
 {Controller c;c.arm(0);auto i=input(0);i.command.x=std::numeric_limits<float>::quiet_NaN();fault(c.update(i),State::BAD_INPUT);}
 {Controller c;c.arm(0);auto i=input(0);i.speed[1]=std::numeric_limits<float>::infinity();fault(c.update(i),State::BAD_INPUT);}
 {Controller c;c.arm(0);auto i=input(0);i.applied[1]=256;fault(c.update(i),State::BAD_INPUT);}
 {Controller c;c.arm(0);auto i=input(0);i.encoderValid=false;fault(c.update(i),State::ENCODER_INVALID);
  i=input(.01f);fault(c.update(i),State::ENCODER_INVALID);c.stop();fault(c.output(),State::ENCODER_INVALID);}
 {Controller c;c.arm(0);for(int n=0;n<40;n++){auto i=input(n*.01f);i.commandStamp=0;c.update(i);}
  fault(c.output(),State::COMMAND_STALE);}
 {Controller c;c.arm(0);auto i=input(0);i.commandStamp=.1f;fault(c.update(i),State::COMMAND_STALE);}
 {Controller c;c.arm(0);c.update(input(0));fault(c.update(input(.08f)),State::CONTROL_LATE);}
 {Controller c;c.arm(0);auto i=input(0);i.dt=.03f;fault(c.update(i),State::CONTROL_LATE);}
 {Controller c;c.arm(0);c.update(input(0));fault(c.update(input(0)),State::CONTROL_LATE);}
 {Controller c;c.arm(0);for(int n=0;n<40;n++){auto i=input(n*.01f);i.bodyStamp=0;c.update(i);}
  fault(c.output(),State::BODY_LOST);}
 {Controller c;c.arm(0);for(int n=0;n<40;n++){auto i=input(n*.01f);i.bodyStamp=i.now+.1f;c.update(i);}
  fault(c.output(),State::BODY_LOST);}
 {Config cfg;cfg.lambda=0;Controller c(cfg);c.arm(0);fault(c.output(),State::BAD_INPUT);}
 {Config cfg;cfg.pwmRate=std::numeric_limits<float>::infinity();Controller c(cfg);c.arm(0);fault(c.output(),State::BAD_INPUT);}
 // Reject an isolated position jump; do not move the hold goal to that outlier.
 {Controller c;c.arm(0);for(int n=0;n<50;n++){auto i=input(n*.01f);if(n>=20&&n<25)i.bodyPose.x=.3f;
   const auto& o=c.update(i);assert(!o.fault);assert(std::fabs(o.estimatedPose.x)<.001f);assert(std::fabs(o.goal.x)<.001f);}}
 // A full turn can cross +-pi without being treated as a position outlier.
 {Controller c;c.arm(0);for(int n=0;n<50;n++){auto i=input(n*.01f);i.bodyPose.w=wrap(3.05f+n*.004f);
   const auto& o=c.update(i);assert(!o.fault);}}
 // Reference limiting must not silently erase the user's heading target.
 {Controller c;c.arm(0);float original=pi/2;for(int n=0;n<150;n++){auto i=input(n*.01f);i.bodyPose.w=original-n*.006f;
   const auto& o=c.update(i);assert(std::fabs(wrap(o.goal.w-original))<.001f);}}
 // No-body mode is explicitly wheel-only and never claims a wall hold.
 {Config cfg;cfg.requireBody=false;Controller c(cfg);c.arm(0);Input i;i.command.x=.14f;
  c.update(i);i.now=i.commandStamp=.01f;i.command.x=0;auto& o=c.update(i);zero(o);assert(o.state==State::COAST);}
 // Applied-output tracking: limit against the actual drive, not yesterday's request.
 {Wheel w;Config cfg;float applied=0;for(int n=0;n<200;n++){
   int duty=w.update(0,8000,0,applied,.01f,true,true,cfg);
   assert(std::abs(duty)<=255);assert(std::fabs(duty-applied)<=13);assert(std::fabs(w.integral)<=120);
   applied=0; // actuator unavailable; all updates must respect that actual output
  }}
 // A frozen estimator cannot learn the injected bad data; emergency stop is separate.
 {Wheel w;Config cfg;w.load=25;w.comp=25;w.initialized=true;
  for(int n=0;n<100;n++)w.update(0,4000,1000,180,.01f,false,true,cfg);
  assert(w.load==25);assert(std::fabs(w.comp-25)<.01f);}
 // A measured-model wheel responds in both directions without exceeding output bounds.
 for(int motor=0;motor<3;motor++)for(int direction:{1,-1}){
  Wheel wheel;Config cfg;float v=0,applied=0,reference=0;
  const Model& m=measuredModels[motor][direction<0];
  for(int n=0;n<800;n++){
   float target=direction*5000.f;reference+=clip(target-reference,-110,110);
   int u=wheel.update(motor,reference,v,applied,.01f,true,true,cfg);
   assert(std::abs(u)<=255);assert(std::fabs(u-applied)<=13);
   const float a=std::exp(-.01f/m.tau);
   v=a*v+(1-a)*m.K*(Wheel::effective(float(u),m)-(n>=300?direction*25.f:0));applied=u;
  }
  assert(std::fabs(v-direction*5000)<150);
 }
 std::printf("PASS: robust control faults, latches, timestamps, outliers, angle wrap, shared ramp, actuator tracking and 6 wheel models; size=%zu bytes\n",sizeof(Controller));
}
