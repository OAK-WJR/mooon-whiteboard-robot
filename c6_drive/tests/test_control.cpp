#include "control.h"
#include <cassert>
#include <limits>
#include <cstring>
int main(){
 drive::Control c;float v[3]={1000,0,0};
 assert(!c.accept(0,1,1,v,false));
 c.arm(10,42);assert(!c.accept(20,41,1,v,false));
 assert(c.accept(20,42,1,v,false));assert(!c.accept(21,42,1,v,false));
 c.tick(40,.02f);assert(c.wheel[0].output>0);
 c.tick(321,.02f);assert(!c.armed&&c.wheel[0].output==0);
 assert(!c.accept(322,42,2,v,false));
 c.reject("EXPIRED_OR_INVALID");assert(!strcmp(c.reason,"LINK_TIMEOUT"));
 c.arm(400,43);v[1]=std::numeric_limits<float>::quiet_NaN();assert(!c.accept(401,43,1,v,false));v[1]=0;
 c.stop("STOP");assert(!c.accept(402,43,2,v,false));
 c.arm(1000,44);v[0]=200;
 for(unsigned t=1000;t<=1400;t+=20){assert(c.accept(t,44,t,v,true));c.tick(t,.02f);if(!c.armed)break;}
 assert(!c.armed);assert(c.wheel[0].output==0);
 c.reject("EXPIRED_OR_INVALID");assert(!strcmp(c.reason,"NO_ENCODER"));
 c.arm(0xfffffff0,45);assert(c.accept(10,45,1,v,true));c.tick(311,.02f);assert(!c.armed);
 c.arm(2000,46);c.tick(2020,.1f);assert(!c.armed);
 drive::Wheel w;w.target=2000;for(int i=0;i<200;i++)w.update(.02f);
 assert(w.output>=0&&w.output<=255&&w.integral<=drive::integralLimit);
 w.target=0;w.update(.02f);assert(w.output==0&&w.integral==0);
 assert(!drive::valid({0,0,-1,0}));
 drive::Wheel correction;correction.gains[0]={.008f,.06f,.0055f,53.4f};
 correction.target=9500;correction.measured=8000;
 for(int i=0;i<300;i++)correction.update(.01f);
 assert(correction.integral>95&&correction.integral<=100&&correction.output<=255);
 correction.target=0;correction.update(.01f);assert(correction.integral==0&&correction.output==0);
 drive::Control limits;float fast[3]={10000,-10000,0};limits.arm(0,99);
 assert(limits.accept(1,99,1,fast,false));
 fast[0]=10001;assert(!limits.accept(2,99,2,fast,false));
 fast[0]=256;assert(!limits.accept(2,99,2,fast,true));
 limits.stop("STOP");assert(limits.wheel[0].output==0);
 drive::Wheel start;start.target=800;
 for(int i=0;i<15;i++)start.update(.01f);
 assert(start.output>=130&&start.startupTime>0);
 start.measured=700;start.update(.01f);assert(start.startupTime==0);
 start.target=0;start.update(.01f);assert(start.output==0);
 drive::FeedbackWatch f;
 assert(f.update(0,0,0,140));assert(f.update(100,1,300,140));
 assert(!f.update(200,0,600,140)); // one channel oscillates, net position cancels
 assert(f.update(210,0,600,0)); // stop resets the observation window
 assert(f.update(300,0,600,-140));assert(f.update(400,uint32_t(-500),1100,-140));
 assert(f.update(500,uint32_t(-1000),1600,-140)); // valid reverse through wrap
 assert(f.update(600,uint32_t(-1000),1800,140)); // reversal starts a fresh window
}
