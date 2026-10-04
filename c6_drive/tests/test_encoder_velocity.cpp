#include "encoder_velocity.h"
#include <cassert>
#include <cstdio>
#include <algorithm>
#include <limits>

int main(){
    // Constant physical speed with count-window phase beating: period timing
    // removes the staircase without changing encoder positions or their sign.
    for(int sign:{-1,1})for(uint32_t start:{0u,0xffff0000u}){
        drive::EncoderEdgeWindow h;uint32_t edge=2131;int32_t count=0,last=0;
        float old=0,modern=0,oldErr=0,newErr=0;int n=0;
        for(uint32_t t=10000;t<=1000000;t+=10000){
            while(edge<=t){h.edge(start+edge,sign);count+=sign;edge+=2131;}
            auto v=drive::encoderVelocity(h,start+t,count-last,.01f);
            old+=.4f*(float(count-last)/.01f-old);
            modern+=v.alpha*(v.cps-modern);last=count;
            if(t>200000){float truth=sign*1e6f/2131;oldErr+=(old-truth)*(old-truth);newErr+=(modern-truth)*(modern-truth);++n;}
        }
        assert(newErr<oldErr*.01f);
        auto stale=drive::encoderVelocity(h,start+1100000,0,.01f);
        assert(!stale.period&&stale.cps==0);
        h.edge(start+1101000,-sign);
        assert(h.used==1&&h.direction==-sign);
        auto reverse=drive::encoderVelocity(h,start+1102000,-sign,.01f);
        assert(!reverse.period&&reverse.cps*sign<0);
        std::printf("constant-speed RMS: count %.3f, edge %.3f cps\n",std::sqrt(oldErr/n),std::sqrt(newErr/n));
    }
    // Unequal quadrature phase lengths cancel over complete four-edge cycles.
    {drive::EncoderEdgeWindow h;uint32_t t=0;
     for(int k=0;k<20;k++){t+=(k%2?3000:1000);h.edge(t,1);}
     auto v=drive::encoderVelocity(h,t,5,.01f);
     assert(v.period&&std::fabs(v.cps-500)<.001f);
     h.edge(t+5000,0);assert(h.used==0);
    }
    // High speed uses the original count estimator. Invalid dt cannot divide.
    {drive::EncoderEdgeWindow h;for(uint32_t t=100;t<=2000;t+=100)h.edge(t,1);
     auto v=drive::encoderVelocity(h,2000,100,.01f);assert(!v.period&&v.cps==10000);
     assert(drive::encoderVelocity(h,2000,1,0).cps==0);
     assert(drive::encoderVelocity(h,2000,1,std::numeric_limits<float>::quiet_NaN()).cps==0);
    }
    // After a true speed step, timestamps must follow new motion promptly.
    {drive::EncoderEdgeWindow h;uint32_t t=0;
     for(int k=0;k<20;k++){t+=2000;h.edge(t,1);}
     for(int k=0;k<8;k++){t+=1000;h.edge(t,1);}
     assert(std::fabs(drive::encoderVelocity(h,t,10,.01f).cps-1000)<.001f);
    }
    std::puts("PASS: quantization, quadrature asymmetry, reversal, no-edge expiry, high-speed fallback, clock wrap and speed step");
}
