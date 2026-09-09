// Host schema fixture from the production command/status serializer.
// Reuse its native SDK boundary; never access a device.
#define main unused_pico_runtime_test_main
#include "test_pico_profile.cpp"
#undef main

int main(int argc, char** argv) {
    const std::string mode=argc>1?argv[1]:"normal";
    levels.fill(HIGH);modes.fill(-99);
    if(mode=="large" || mode=="rails_large")fakeNow=0xfffff000;
    const bool rails=mode.rfind("rails_",0)==0;
    if(rails)pico_rail_mock::bus.present=true;
    if(mode=="init_failed")watchdogResult=-42;
    if(mode=="fault") {
        pico_gpio_mock::failOp="level";pico_gpio_mock::failPin=16;
    }
    setup();
    if(rails) {
        for(unsigned i=0;i<100 && !pico_rails::diagnostics().status().sample;++i)loop();
        require(pico_rails::diagnostics().status().sample.has_value(),"rail fixture did not complete acquisition");
        if(mode=="rails_expired")fakeNow+=(50000-pico_rails::diagnostics().status().ageUs)/1000;
        if(mode=="rails_sampling") {
            for(unsigned i=0;i<60 && pico_rails::diagnostics().status().sample;++i)loop();
        }
        if(mode=="rails_fault") {
            pico_rail_mock::bus.nackNext=true;
            for(unsigned i=0;i<100 && strcmp(pico_rails::diagnostics().status().state,"fault");++i)loop();
        }
        if(mode=="rails_cleanup") {
            pico_rail_mock::bus.cleanupFails=true;pico_rails::diagnostics().stop();
        }
    } else if(mode!="acquiring")for(unsigned i=0;i<600;++i)loop();
    if(mode=="stale"){fakeNow+=101;loop();}
    if(mode=="large" || mode=="rails_large") {
        controlGapMs=maxControlGapMs=controlOverruns=transport.dropped=transport.reconnects=UINT32_MAX;
        transport.notifyFailures=UINT32_MAX;
        enc_counts_[0]=enc_counts_[1]=enc_counts_[2]=0x80000000;
        reason="SOURCE_CYCLE_UNAVAILABLE";
    }
    Serial.tx.clear();
    command(Link::USB,0,"hello");
    std::cout<<Serial.tx;
    return Serial.tx.empty()?1:0;
}
