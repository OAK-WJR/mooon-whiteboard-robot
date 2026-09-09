// Full firmware cell/rail scheduling and GPIO sequencing with independent TI
// protocol model at the shared-worker boundary. No hardware is accessed.
#pragma push_macro("main")
#undef main
#define main unused_pico_profile_main
#include "test_pico_profile.cpp"
#undef main
#pragma pop_macro("main")

static JsonDocument cellReport() { JsonDocument d;fillCellReport(d);return d; }
static void stepCells() {
    auto& b=pico_rail_mock::bus;
    const auto n=b.starts+b.polls+b.cellStarts+b.cellPolls;
    loop();
    require(b.starts+b.polls+b.cellStarts+b.cellPolls-n<=1,"two I2C client operations in one loop");
    require(!b.active() || !b.cellDevice.active,"shared clients overlap");
}
static void waitCell(const char* state,unsigned limit=15000) {
    for(unsigned i=0;i<limit && cellReport()["cell_monitor"]!=state;++i)stepCells();
    if(cellReport()["cell_monitor"]!=state){std::cerr<<json(cellReport())<<'\n';throw std::runtime_error("cell state did not arrive");}
}
static void absentValues(const JsonDocument& d) {
    require(d["fresh"]==false,"invalid cell report fresh");
    for(int i=0;i<2;++i)require(d["cell_mv"][i].isNull() && d["safety_alert"][i].isNull() && d["safety_status"][i].isNull(),"stale cell/status array leaked");
    for(const char* field:{"stack_mv","ts_counts","ts_nominal_uv","seq","age_ms","read_span_ms",
                           "battery_status","alarm_status","alarm_raw","balance_mask","fets_off_observed"})
        require(d[field].isNull(),"stale cell metadata leaked");
}
static void inhibited(const JsonDocument& d) {
    for(const char* field:{"motion_enabled","charge_enabled","system_protection_qualified","voltage_profile_verified"})
        require(d[field]==false,"cell readout enabled unqualified battery protection/power");
    require(d["battery_current_ma"].isNull() && d["pack_temperature_mc"].isNull(),"unwired pack measurement fabricated");
    for(int pin:{16,42,41})require(levels[pin]==LOW,"load enabled during cell monitoring");
}
static void checkWake() {
    bool asserted=false,observed=false;uint32_t at=0;
    for(const auto& e:pico_gpio_mock::events) {
        if(e.pin!=39 || std::string(e.op)!="level" || e.error)continue;
        if(e.value==LOW) {
            require(!asserted && !observed && e.mode==GPIO_MODE_OUTPUT_OD,"wake repeated or not open drain");
            asserted=observed=true;at=e.atMs;
        }else if(asserted) {
            const auto width=uint32_t(e.atMs-at);
            require(width>=100 && width<150,"nominal wake pulse outside software budget");
            asserted=false;
        }
    }
    require(observed && !asserted && !boardSafe.wakeRequested(),"wake not completed/released");
}
int main(int argc,char** argv) {
    const std::string mode=argc>1?argv[1]:"normal";
    auto& bus=pico_rail_mock::bus;auto& chip=bus.cellDevice;
    levels.fill(HIGH);modes.fill(-99);bus.present=true;bus.cellPresent=mode!="absent";
    storedConfig=R"({"ssid":"public-native-fixture","password":"test-only-password","key":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"})";
    if(mode=="wrap")fakeNow=UINT32_MAX-120;
    if(mode=="long_frames")chip.delay_polls=7; // Explicit long in-flight transfers, still below25ms.
    setup();absentValues(cellReport());
    waitCell("waking",10);
    require(boardSafe.wakeRequested() && levels[39]==LOW,"runtime did not assert wake");
    if(mode=="wake_abort") {
        halt("USER_STOP");absentValues(cellReport());
        require(cellReport()["cell_error"]=="wake_aborted" && levels[39]==HIGH,"stop failed to abort wake");
    } else if(mode=="wake_assert_fault" || mode=="wake_release_fault") {
        // The assert-fault scenario explicitly exercises the low GPIO API;
        // release failure is exercised through the timed production service.
        pico_gpio_mock::failOp="level";pico_gpio_mock::failPin=39;
        pico_gpio_mock::failValue=mode=="wake_assert_fault"?LOW:HIGH;
        if(mode=="wake_assert_fault")require(!boardSafe.setWakeRequest(true),"injected assert succeeded");
        for(unsigned i=0;i<160 && boardSafe.ready();++i)stepCells();
        auto d=cellReport();absentValues(d);require(!boardSafe.ready() && !d["wake_requested"].as<bool>(),"GPIO failure kept wake active");
    } else if(mode=="wake_stall") {
        fakeNow+=151;stepCells();auto d=cellReport();absentValues(d);
        require(!boardSafe.wakeRequested() && levels[39]==HIGH,"late control loop retained wake");
    } else if(mode=="absent") {
        waitCell("fault");require(cellReport()["cell_error"]=="wake_timeout","absent target accepted");
        absentValues(cellReport());checkWake();
        const auto starts=bus.cellStarts;
        for(unsigned i=0;i<300;++i)stepCells();require(starts==bus.cellStarts,"failed target automatically reconfigured");
        require(bus.starts>10,"missing BQ starved rail diagnostics");
    } else {
        waitCell("monitoring");auto first=cellReport();checkWake();
        require(first["configuration_verified"]==true && first["device_number"]==0x7605,"configuration/identity not established");
        require(first["cell_mv"][0]==3700 && first["cell_mv"][1]==3800 && first["stack_mv"]==7500,"wrong2S mapping");
        require(first["cell_pair_valid"]==true && first["fets_off_observed"]==true,"missing sample evidence");
        if(mode=="normal" || mode=="wrap" || mode=="long_frames") {
            const auto seq=first["seq"].as<uint32_t>();
            for(unsigned i=0;i<6000 && cellReport()["seq"].as<uint32_t>()==seq;++i)stepCells();
            require(cellReport()["cell_monitor"]=="monitoring" && cellReport()["seq"].as<uint32_t>()==seq+1,"interleaving stalled next cell audit/frame");
            require(bus.starts>30,"BQ initialization or audits starved INA");
            WiFi.connected=true;BLEDevice::servers.back()->connect(12);
            for(Link link:{Link::USB,Link::WiFi,Link::BLE}) {
                auto reply=exchange(link,request("cell_status"));
                require(reply["kind"]=="cells" && reply["fresh"]==true,"cell report lost across control transport");
                inhibited(reply);
            }
        } else if(mode=="stale") {
            // Getter-only expiration check; no next loop to hide it with a new frame.
            fakeNow+=3000;auto c=pico_rails::diagnostics().cellStatus();
            require(!c.sample.fresh && std::string(c.state)=="stale","stopped scheduler leaked stale cells");
            absentValues(cellReport()); // Watchdog staleness also stops the public report.
        } else if(mode=="por" || mode=="configuration_drift" || mode=="bus_error") {
            if(mode=="por")chip.reset();
            if(mode=="configuration_drift")chip.ram[0x901b]=3;
            if(mode=="bus_error")chip.error_next=2;
            waitCell("fault");auto d=cellReport();absentValues(d);
            require(d["configuration_verified"]==false && d["generation"].as<uint32_t>()!=first["generation"].as<uint32_t>(),"fault retained configuration/sample generation");
        } else if(mode=="lower_cell_invalid") {
            chip.lower=1800;chip.stack=5600;const auto seq=first["seq"].as<uint32_t>();
            for(unsigned i=0;i<4000 && cellReport()["seq"].as<uint32_t>()==seq;++i)stepCells();
            auto d=cellReport();require(d["fresh"]==true && d["cell_mv"][0]==1800 && d["cell_mv"][1].isNull() &&
                d["cell_pair_valid"]==false,"invalid2S upper measurement accepted");
        } else if(mode=="reboot_in_transfer") {
            chip.stuck_pending=true;
            for(unsigned i=0;i<300 && !chip.active;++i)stepCells();
            require(chip.active,"missing pending read for cancellation");
            require(exchange(Link::USB,request("reboot"))["ok"]==true,"reboot rejected");
            absentValues(cellReport());require(!chip.active,"reboot retained caller buffer ownership");
            for(unsigned i=0;i<200 && !restartCalls;++i)stepCells();require(restartCalls==1,"reboot blocked on BQ");
        } else throw std::runtime_error("unknown cell scenario");
    }
    inhibited(cellReport());checkSafety();std::cout<<mode<<" PASS\n";
}
