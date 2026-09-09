// Actual setup/loop, INA state machine, service and authenticated serialization;
// only the worker/device/SDK boundaries are fake. Each mode is a fresh process.
#define main unused_pico_profile_main
#include "test_pico_profile.cpp"
#undef main

static void step() {
    const auto calls=pico_rail_mock::bus.starts+pico_rail_mock::bus.polls;
    loop();
    require(pico_rail_mock::bus.starts+pico_rail_mock::bus.polls-calls<=1,"rail work blocks/loops within a control iteration");
}
static JsonDocument sample() {
    JsonDocument d;fillStatus(d);return d;
}
static void noSample(const JsonDocument& d) {
    for(const char* field:{"motor_voltage_mv","raw_logic_5v_mv","usb_vbus_mv"})
        require(d[field].isNull(),"partial or stale voltage published");
    for(const char* field:{"seq","age_us","span_us"})require(d["rail_monitor"][field].isNull(),"stale sample metadata");
    require(d["rail_monitor"]["state"]!="ready","invalid sample reported ready");
}
static JsonDocument waitReady() {
    for(unsigned i=0;i<100;++i) {
        auto d=sample();if(d["rail_monitor"]["state"]=="ready")return d;
        step();
    }
    auto d=sample();std::cerr<<json(d)<<'\n';throw std::runtime_error("monitor never completed a real conversion sequence");
}
static void checkedValues(const JsonDocument& d, uint32_t raw, uint32_t usb, uint32_t vmot) {
    require(d["rail_monitor"]["state"]=="ready" && d["rail_monitor"]["bus"]=="ready" &&
            d["rail_monitor"]["error"]=="none","bad ready metadata");
    require(d["raw_logic_5v_mv"]==raw && d["usb_vbus_mv"]==usb && d["motor_voltage_mv"]==vmot,"wrong rail/channel mapping");
    require(d["rail_monitor"]["seq"].as<uint32_t>()>0,"missing conversion sequence");
    const auto age=d["rail_monitor"]["age_us"].as<uint32_t>(), span=d["rail_monitor"]["span_us"].as<uint32_t>();
    require(age<50000 && span<=age && span>=3630,"invalid acquisition age/window");
    require(d["motor_current_ma"].isNull() && d["rail_3v3_mv"].isNull(),"unwired measurement fabricated");
    for(const char* field:{"motion_enabled","charge_enabled","source_cycle_enabled","power_disconnect"})
        require(d[field]==false,"rail readout enabled an unqualified load/recovery path");
}
static void waitFault(const char* error) {
    for(unsigned i=0;i<220 && sample()["rail_monitor"]["state"]!="fault";++i)step();
    auto d=sample();noSample(d);
    require(d["rail_monitor"]["state"]=="fault" && d["rail_monitor"]["error"]==error,"wrong or lost rail fault");
    const auto calls=pico_rail_mock::bus.starts;
    for(unsigned i=0;i<200;++i)step();
    require(pico_rail_mock::bus.starts==calls && pico_rail_mock::bus.opens==1,"fault silently retried/reopened I2C");
    require(!pico_rails::diagnostics().begin(),"fault was cleared by another begin");
}
int main(int argc,char** argv) {
    const std::string mode=argc>1?argv[1]:"ready";
    auto& bus=pico_rail_mock::bus;
    levels.fill(HIGH);modes.fill(-99);bus.present=mode!="absent";
    storedConfig=R"({"ssid":"public-native-fixture","password":"test-only-password","key":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"})";
    if(mode=="start_failure")bus.openOk=false;
    if(mode=="setup_timeout")bus.setupPending=true;
    if(mode=="gpio_startup_fault"){pico_gpio_mock::failOp="level";pico_gpio_mock::failPin=16;}
    if(mode=="watchdog_startup_fault")watchdogResult=-42;
    if(mode=="wrap")fakeNow=UINT32_MAX-25;
    setup();
    noSample(sample());
    if(mode=="gpio_startup_fault" || mode=="watchdog_startup_fault") {
        require(bus.opens==0 && bus.starts==0,"unsafe startup opened rail worker");
        for(unsigned i=0;i<10;++i)step();noSample(sample());
    } else if(mode=="start_failure")waitFault("start_failed");
    else if(mode=="setup_timeout")waitFault("setup_timeout");
    else if(mode=="absent")waitFault("nack");
    else {
        auto first=waitReady();checkedValues(first,5000,5000,0);
        if(mode=="ready" || mode=="wrap") {
            WiFi.connected=true;BLEDevice::servers.back()->connect(12);
            for(Link link:{Link::USB,Link::WiFi,Link::BLE}) {
                waitReady();auto response=exchange(link,request("status"));checkedValues(response,5000,5000,0);
                // BLE may arrive after sample lifetime. Age above is captured
                // at serialization, never an arrival-time freshness assertion.
            }
            waitReady();
            require(exchange(Link::USB,request("claim"))["ok"]==true,"claim failed");
            require(exchange(Link::USB,request("source_cycle"))["ok"]==false,"real voltage enabled unqualified cycling");
        } else if(mode=="generation") {
            const auto seq=first["rail_monitor"]["seq"].as<uint32_t>();
            bus.voltages={4800,0,6000};
            for(unsigned i=0;i<50 && sample()["rail_monitor"]["state"]=="ready";++i)step();
            noSample(sample());auto second=waitReady();checkedValues(second,4800,0,6000);
            require(second["rail_monitor"]["seq"].as<uint32_t>()==seq+1,"generation did not advance atomically");
        } else if(mode=="expired") {
            fakeNow+=(50000-first["rail_monitor"]["age_us"].as<uint32_t>())/1000;
            auto d=sample();noSample(d);require(d["rail_monitor"]["state"]=="expired","exact sample expiry was not enforced at serialization");
        } else if(mode=="nack" || mode=="short" || mode=="pending" || mode=="por" || mode=="invalid_voltage") {
            if(mode=="nack")bus.nackNext=true;
            if(mode=="short")bus.shortNext=true;
            if(mode=="pending")bus.pending=true;
            if(mode=="por")bus.corruptConfig=true;
            if(mode=="invalid_voltage")bus.voltages[2]=26008;
            waitFault(mode=="nack"?"nack":mode=="short"?"short_read":mode=="pending"?"transfer_timeout":
                      mode=="por"?"configuration_changed":"invalid_voltage");
        } else if(mode=="gpio_runtime_fault" || mode=="watchdog_fault") {
            bus.pending=true;
            for(unsigned i=0;i<60 && !bus.active();++i)step();
            require(bus.active(),"no owned read to cancel");
            if(mode=="gpio_runtime_fault") {pico_gpio_mock::failOp="level";pico_gpio_mock::failPin=41;halt("TEST_GPIO");}
            else {fakeNow+=101;step();}
            noSample(sample());require(!bus.active() && bus.cancels==1,"fault failed to invalidate/cancel pending buffer");
            const auto calls=bus.starts;for(unsigned i=0;i<20;++i)step();require(bus.starts==calls,"fault resumed monitor");
        } else if(mode=="reboot") {
            require(exchange(Link::USB,request("claim"))["ok"]==true,"claim failed");
            require(exchange(Link::USB,request("reboot"))["ok"]==true,"reboot lost");
            noSample(sample());require(sample()["rail_monitor"]["state"]=="stopped","reboot retained measurement service");
            for(unsigned i=0;i<200 && !restartCalls;++i)step();require(restartCalls==1,"reboot blocked on rail cleanup");
        } else if(mode=="cleanup_fault") {
            bus.cleanupFails=true;pico_rails::diagnostics().stop();
            auto d=sample();noSample(d);require(d["rail_monitor"]["error"]=="cleanup_failed" &&
                d["rail_monitor"]["bus"]=="cleanup_fault","worker cleanup error hidden");
        } else throw std::runtime_error("unknown scenario");
    }
    for(uint8_t address:bus.addresses)require(address==0x40,"runtime scanned/woke another I2C device");
    require(bus.opens<=1,"worker reopened during runtime");
    checkSafety();std::cout<<mode<<" PASS\n";
}
