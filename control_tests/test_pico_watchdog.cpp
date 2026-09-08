// Execute the production setup/loop, controller and transports. SDK boundaries
// supply time/failure injection; no timer callback or substitute controller.
#define main unused_pico_profile_test_main
#include "test_pico_profile.cpp"
#undef main

static void runFor(uint32_t duration) {
    const uint32_t start=millis();
    while(uint32_t(millis()-start)<duration && !restartCalls)loop();
}
static std::vector<pico_gpio_mock::Event> fallingEdges() {
    std::vector<pico_gpio_mock::Event> result;
    for(const auto& e:pico_gpio_mock::events)
        if(e.pin==17 && std::string(e.op)=="level" && e.value==LOW && e.error==0)result.push_back(e);
    return result;
}
static void checkPulses() {
    bool low=false;uint32_t lowAt=0,lastFall=0;bool hadFall=false;
    for(const auto& e:pico_gpio_mock::events) {
        if(e.pin!=17 || std::string(e.op)!="level" || e.error)continue;
        if(e.value==LOW) {
            require(!low,"WDI did not release between falling edges");
            if(hadFall)require(uint32_t(e.atMs-lastFall)>=40 && uint32_t(e.atMs-lastFall)<=120,"WDI falling interval outside service budget");
            low=true;lowAt=e.atUs;lastFall=e.atMs;hadFall=true;
        } else if(low) {
            require(uint32_t(e.atUs-lowAt)>=2,"WDI low pulse shorter than requested 2us");
            low=false;
        }
    }
    require(hadFall && !low,"missing complete OD heartbeat");
    checkSafety();
}

// Independent TI timing model: RESET ignores WDI for tRST; a late falling edge
// asserts WDO for tRST, during which WDI is also ignored. It does not reset RUN.
// This validates emitted events against min/typ/max parts, not physical timing.
struct TPS3851 {
    enum Phase { Reset, Counting, WdoLow } phase=Reset;
    uint32_t wd,rst,faults=0;
    uint64_t deadline;
    TPS3851(uint32_t watchdog,uint32_t reset):wd(watchdog),rst(reset),deadline(reset){}
    void advance(uint64_t now) {
        while(deadline<=now) {
            if(phase==Counting){phase=WdoLow;deadline+=rst;++faults;}
            else{phase=Counting;deadline+=wd;}
        }
    }
    void falling(uint64_t now) {
        advance(now);
        if(phase==Counting)deadline=now+wd;
    }
};
static void checkAcquisitionAndTimeout(uint32_t setupEnd) {
    const auto edges=fallingEdges();require(!edges.empty(),"no production heartbeat events");
    for(uint32_t wd:{170U,200U,230U})for(uint32_t rst:{170U,200U,230U}) {
        TPS3851 device(wd,rst);uint32_t faultsAtAcquisition=0;bool acquired=false;
        for(const auto& e:edges) {
            device.falling(e.atMs);
            if(!acquired && e.atMs>=setupEnd+600){faultsAtAcquisition=device.faults;acquired=true;}
            if(acquired)require(device.faults==faultsAtAcquisition,"healthy progress lost external watchdog after acquisition");
        }
        require(acquired && device.phase==TPS3851::Counting,"did not acquire a previously timed-out supervisor");
        require(faultsAtAcquisition>0,"slow startup fixture did not exercise ignored WDI/WDO recovery");
        const auto last=edges.back().atMs;
        device.advance(uint64_t(last)+wd-1);require(device.phase==TPS3851::Counting,"early timeout in model");
        device.advance(uint64_t(last)+wd);require(device.phase==TPS3851::WdoLow,"lost progress did not expire at selected tWD");
    }
}

int main(int argc,char** argv) {
    const std::string mode=argc>1?argv[1]:"healthy";
    levels.fill(HIGH);modes.fill(-99);
    storedConfig=R"({"ssid":"public-native-fixture","password":"test-only-password","key":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"})";
    if(mode=="healthy")nvsReadDelayMs=1500;
    if(mode=="startup_timeout")nvsReadDelayMs=5000;
    if(mode=="startup_wdt_failure")watchdogResult=-42;
    if(mode=="boot_rom_margin")fakeNow=60000; // ROM need not service GPIO17.
    if(mode=="wrap")fakeNow=UINT32_MAX-200;
    setup();const uint32_t setupEnd=millis();
    require(fallingEdges().empty(),"startup emitted an unearned heartbeat");
    require(watchdogTimeouts.front()==5000,"startup was not bounded before NVS/radio work");
    if(mode=="startup_timeout") {
        require(restartCalls==1 && watchdogFeeds==0,"late initialization renewed watchdog instead of restarting");
        require(watchdogTimeouts.size()==1,"late startup changed to runtime watchdog budget");
        require(!externalWatchdog.ready(millis()),"late startup qualified heartbeat");
        checkSafety();std::cout<<mode<<" PASS\n";return 0;
    }
    if(mode=="startup_wdt_failure") {
        runFor(700);
        require(fallingEdges().empty() && !watchdogReady,"failed task watchdog allowed external heartbeat");
        require(std::string(externalWatchdog.stateName(millis()))=="init_failed","init failure not reported");
        statusChecks(exchange(Link::USB,request("status")));
        checkSafety();std::cout<<mode<<" PASS\n";return 0;
    }
    require(watchdogTimeouts.back()==3000 && watchdogAdds==1,"runtime watchdog double-subscribed or kept startup timeout");
    require(std::string(externalWatchdog.stateName(millis()))=="acquiring","startup claimed watchdog already acquired");
    runFor(400);require(!externalWatchdog.ready(millis()),"watchdog qualified before acquisition interval");
    runFor(800);require(externalWatchdog.ready(millis()),"healthy idle did not acquire external watchdog");
    checkPulses();
    if(mode=="healthy") {
        checkAcquisitionAndTimeout(setupEnd);
        const auto count=fallingEdges().size();
        fakeNow+=230; // No loop, ISR or timer is invoked; there must be no pulses.
        require(fallingEdges().size()==count && !externalWatchdog.ready(millis()),"absent loop was kept alive by autonomous pulses");
    } else if(mode=="stale")fakeNow+=101;
    else if(mode=="lost_control") {
        const auto feeds=watchdogFeeds;
        const auto edges=fallingEdges().size();
        for(unsigned i=0;i<300;++i) {
            lastControlAt=millis(); // Loop and hello traffic run, but no control update completes.
            if(i%30==0)Serial.feed("hello\n");
            loop();
        }
        require(watchdogFeeds>feeds && fallingEdges().size()==edges,"loop/RX activity masked missing control progress");
    } else if(mode=="slow_control") {
        fakeNow+=20;criticalSectionHook=[] { fakeNow+=101;criticalSectionHook=nullptr; };
    } else if(mode=="slow_transport") {
        Serial.feed("hello\n");serialReadDelayMs=25;
    } else if(mode=="task_feed_failure")watchdogFeedResult=-42;
    else if(mode.rfind("pulse_",0)==0) {
        pico_gpio_mock::failOp="level";pico_gpio_mock::failPin=17;
        pico_gpio_mock::failValue=mode=="pulse_release_failure"?HIGH:LOW;
        if(mode=="pulse_cleanup_failure")pico_gpio_mock::releaseFailPin=17;
        fakeNow+=40;
    } else if(mode=="reboot_progress") {
        BLEDevice::servers.back()->connect(12);
        require(exchange(Link::BLE,request("claim"))["ok"]==true,"BLE claim failed");
        for(int i=0;i<3;++i)command(Link::BLE,peer(Link::BLE),"hello");
        command(Link::BLE,peer(Link::BLE),signedLine(request("reboot")).c_str());
        const auto edges=fallingEdges().size();runFor(2600);
        require(restartCalls==1 && fallingEdges().size()>edges+5,"ACK drain lost progress heartbeat or reboot");
        checkPulses();std::cout<<mode<<" PASS\n";return 0;
    } else if(mode=="boot_rom_margin" || mode=="wrap") {
        require(!restartCalls,"ROM margin/wrap prevented a healthy application");
        statusChecks(exchange(Link::USB,request("status")));
        checkPulses();std::cout<<mode<<" PASS\n";return 0;
    } else throw std::runtime_error("unknown watchdog test mode");

    const auto beforeLoop=fallingEdges().size();loop();serialReadDelayMs=0;
    if(mode.rfind("pulse_",0)!=0)
        require(fallingEdges().size()==beforeLoop,"late/failed work emitted a catch-up heartbeat");
    require(!externalWatchdog.ready(millis()) && externalWatchdog.failed(),"failed service remained ready");
    const auto stopped=fallingEdges().size();runFor(600);
    require(fallingEdges().size()==stopped,"lost-progress/SDK fault self-cleared");
    externalWatchdog.begin(millis());externalWatchdog.finishStartup(millis(),true);
    require(!externalWatchdog.ready(millis()),"reinitialization cleared a latched watchdog fault");
    if(mode.rfind("pulse_",0)==0) {
        require(!boardSafe.ready() && boardSafe.error()==-42,"WDI SDK failure not latched");
        if(mode=="pulse_cleanup_failure")require(boardSafe.releaseError()==-42,"WDI cleanup failure hidden");
        statusChecks(exchange(Link::USB,request("status")),true);
    } else {
        require(boardSafe.ready(),"timing failure destroyed diagnostic GPIO ownership");
        statusChecks(exchange(Link::USB,request("status")));
    }
    require(exchange(Link::USB,request("reboot"))["ok"]==true,"watchdog fault lost software reboot");
    runFor(200);require(restartCalls==1,"watchdog fault prevented reboot");
    checkSafety();std::cout<<mode<<" PASS\n";
}
