// Assumed wheel plants for loaded wheel PI screening, not measured floor physics.
// Only encoder-like filtered speed and prior actuator output reach the controller.
#include "robust_wheels.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>

int main(){
 for(int motor=0;motor<3;motor++)for(int sign:{-1,1})
 for(float target:{500.f,900.f,1800.f})for(float friction:{45.f,80.f,110.f})
 for(bool candidate:{false,true}){
  drive::RobustWheels bank;bank.loadedPi=candidate;
  float set[3]={},v[3]={},applied[3]={};int out[3];set[motor]=sign*target;
  float speed=0,queue[4]={},tail=0,peak=0,absError=0;int stoppedTicks=0;
  const float gain=motor==1?50.f:63.f,tau=.30f;
  for(int n=0;n<160;n++){
   const bool starting=bank.startup[motor]>0;
   assert(bank.update(set,v,applied,.01f,out));
   for(int i=0;i<3;i++){
    assert(std::abs(out[i])<=255);
    const bool release=candidate&&i==motor&&starting&&bank.startup[i]==0&&
       out[i]*applied[i]>=0&&std::abs(out[i])<std::fabs(applied[i]);
    assert(release||std::fabs(out[i]-applied[i])<=12.001f);
    if(i!=motor)assert(out[i]==0);
    applied[i]=out[i];
   }
   queue[n%4]=out[motor];float duty=queue[(n+4-motor)%4];
   // Static threshold exceeds kinetic drag; it and inertia are hypotheses.
   float driveSign=std::fabs(speed)>1?wbb_robust::sign(speed):wbb_robust::sign(duty);
   float next=speed+.01f*(gain*(duty-driveSign*friction)-speed)/tau;
   if((std::fabs(speed)<1||next*speed<0)&&std::fabs(duty)<friction+20)next=0;
   speed=next;v[motor]+=.4f*(speed-v[motor]);
   peak=std::fmax(peak,sign*v[motor]);
   if(n>=40&&n<60)tail+=sign*v[motor]/20;
   if(n>=30&&n<60&&std::fabs(v[motor])<30)stoppedTicks++;
   if(n>=140)absError+=std::fabs(v[motor]-set[motor])/target/20;
  }
  set[motor]=0;assert(bank.update(set,v,applied,.01f,out));
  for(int i=0;i<3;i++)assert(out[i]==0&&bank.wheel[i].integral==0);
  std::printf("{\"motor\":%d,\"sign\":%d,\"target_cps\":%.0f,\"assumed_friction_pwm\":%.0f,\"candidate\":%s,\"tail_600ms_percent\":%.3f,\"peak_percent\":%.3f,\"tail_1600ms_error\":%.4f,\"low_speed_ticks\":%d}\n",
   motor+1,sign,target,friction,candidate?"true":"false",tail/target*100,peak/target*100,absError,stoppedTicks);
 }
}
