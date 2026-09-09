#define WBB_CONTROL_NATIVE_TEST 1
#include "runtime_stubs.h"
#include "kiss_c6_radio_power.h"
#include <cassert>
#include <iostream>
using kiss_c6::RadioPower;
using Action=RadioPower::Action;
static kiss_c6::VoltageMonitor::Reading sample(uint32_t at,int mv=7800){
    kiss_c6::VoltageMonitor::Reading v{};v.valid=true;v.sequence=1;v.at=at;v.railMv[0]=mv;return v;
}
static void start(RadioPower& gate,uint32_t base=0){
    for(unsigned i=0;i<=10;++i){
        const auto t=uint32_t(base+i*100);
        assert(gate.observe(sample(t),t,true)==(i==10?Action::Start:Action::None));
    }
    gate.started(true);assert(gate.hasStarted());
}
int main(){
    {
        RadioPower g;auto v=sample(0);
        for(unsigned now=0;now<2000;++now)assert(g.observe(v,now,true)==Action::None);
        assert(!g.hasStarted());start(g,3000);
        for(unsigned i=1;i<20;++i)assert(g.observe(sample(4000+i*100),4000+i*100,true)==Action::None);
    }std::cout<<"fresh consecutive acquisition timestamps; one startup per boot PASS\n";
    {
        RadioPower g;
        for(unsigned i=0;i<=10;++i)assert(g.observe(sample(i*100,6499),i*100,true)==Action::None);
        for(unsigned i=11;i<15;++i)assert(g.observe(sample(i*100,6500),i*100,true)==Action::None);
        assert(g.observe(sample(1500,6499),1500,true)==Action::None);
        start(g,1600);
        assert(g.observe(sample(2700,6000),2700,false)==Action::None);
        assert(g.observe(sample(2800,5999),2800,false)==Action::Restart);
    }std::cout<<"startup dwell resets on voltage dip; hysteresis boundary PASS\n";
    {
        RadioPower g;
        for(unsigned i=0;i<6;++i)assert(g.observe(sample(i*100),i*100,true)==Action::None);
        assert(g.observe(sample(900),900,true)==Action::None);
        for(unsigned i=10;i<19;++i)assert(g.observe(sample(i*100),i*100,true)==Action::None);
        assert(g.observe(sample(1900),1900,true)==Action::Start);
        assert(g.observe(sample(1900),2151,true)==Action::Restart);
    }std::cout<<"missing acquisitions reset dwell; stale running evidence requests restart PASS\n";
    for(int mv:{0,5999,8801,13000}){
        RadioPower g;start(g);assert(g.observe(sample(1100,mv),1100,true)==Action::Restart);
    }
    {
        RadioPower g;start(g);auto v=sample(1100);v.valid=false;
        assert(g.observe(v,1100,true)==Action::Restart);
    }std::cout<<"absent/out-of-window battery and invalid ADC cannot retain radios PASS\n";
    {
        RadioPower g;
        for(unsigned i=0;i<20;++i)assert(g.observe(sample(i*100),i*100,false)==Action::None);
        assert(!g.hasStarted());start(g,2000);
        g.started(false);assert(std::string(g.state())=="init_failed");
        assert(g.observe(sample(3100),3100,true)==Action::None);
        assert(g.observe(sample(3200,0),3200,true)==Action::Restart);
    }std::cout<<"inhibited startup and failed-init state still monitor supply PASS\n";
    {RadioPower g;start(g,UINT32_MAX-499);}
    std::cout<<"millisecond wrap preserves qualification PASS\n";
}
