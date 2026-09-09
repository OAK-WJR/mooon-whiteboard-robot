#pragma once
#include "kiss_c6_voltage.h"
#ifdef WBB_TOP_BOARD_KISS_C6
namespace kiss_c6 {
// A software source-selection policy, not cell protection or a USB power grant.
// Start each radio stack only once per boot. Loss of supply evidence requests a
// clean chip restart; the next boot remains USB-only until battery qualification.
class RadioPower {
public:
    enum class Action { None, Start, Restart };
    static constexpr int startMv=6500, holdMv=6000, maximumMv=8800;
    static constexpr uint32_t qualifyMs=1000, maximumGapMs=150;
    Action observe(const VoltageMonitor::Reading& v,uint32_t now,bool canStart){
        const bool fresh=v.valid&&v.sequence&&uint32_t(now-v.at)<=VoltageMonitor::maxAgeMs;
        if(attempted){
            if(!fresh||v.railMv[0]<holdMv||v.railMv[0]>maximumMv){
                phase="restart";return Action::Restart;
            }
            return Action::None;
        }
        if(!canStart||!fresh||v.railMv[0]<startMv||v.railMv[0]>maximumMv){
            qualifying=false;phase=canStart?"waiting_voltage":"inhibited";return Action::None;
        }
        if(!qualifying||uint32_t(v.at-lastSample)>maximumGapMs){
            qualifying=true;since=v.at;phase="qualifying";
        }
        // Re-reading one sample does not accumulate qualification time.
        lastSample=v.at;
        if(uint32_t(v.at-since)<qualifyMs)return Action::None;
        attempted=true;phase="starting";return Action::Start;
    }
    void started(bool ready){phase=ready?"online":"init_failed";}
    bool hasStarted()const{return attempted;}
    const char* state()const{return phase;}
private:
    bool qualifying=false,attempted=false;
    uint32_t since=0,lastSample=0;
    const char* phase="waiting_voltage";
};
}
#endif
