// Runs the actual profile, controller, driver, ISRs, command loop and transport.
// Only SDK/Arduino/NVS/clocks/radio boundary calls are mocked; no device access.
#define WBB_CONTROL_NATIVE_TEST 1
#define WBB_CONTROL_TRANSPORT_NATIVE_TEST 1
#define RobotController UnusedRuntimeStubRobot
#include "runtime_stubs.h"
#undef RobotController
#include "../src/kinematics.cpp"
#include "../src/pid_controller.cpp"
#include "../src/path_planner.cpp"
#include "../src/trajectory_follower.cpp"
#include "../src/odometry.cpp"
#include "../src/motor_driver.cpp"
#include "../src/robot_controller.cpp"
#include "../src/top_v2_main.cpp"
#include "../src/control_transport.cpp"
#include <iostream>
#include <stdexcept>

using control::Link;
static void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
static std::string json(const JsonDocument& d) { std::string s; serializeJson(d, s); return s; }
static std::string signedLine(const JsonDocument& d) {
    const std::string payload = json(d);
    uint8_t digest[32]; require(mac(payload.c_str(), digest), "HMAC fixture unavailable");
    char signature[65];
    for (int i=0;i<32;++i) snprintf(signature+2*i,3,"%02x",digest[i]);
    JsonDocument envelope; envelope["payload"]=payload; envelope["mac"]=signature;
    return json(envelope);
}
static uint64_t peer(Link link) {
    return link == Link::WiFi ? (uint64_t(0xc0000201)<<16)|5555 : link == Link::BLE ? transport.blePeer.load() : 0;
}
static uint32_t nextId = 1;
static JsonDocument request(const char* op) {
    JsonDocument d; d["id"]=nextId++; d["cmd"]=op; d["boot"]=boot; d["lease"]=owner.lease;
    return d;
}
static void inject(Link link, const JsonDocument& d) {
    const auto wire = (link == Link::USB ? json(d) : signedLine(d)) + '\n';
    if (link == Link::USB) {
        Serial.feed(wire);
        while (Serial.available()) loop();
    } else if (link == Link::WiFi) {
        udpIncoming.push_back({IPAddress(0xc0000201),5555,wire}); loop();
    } else {
        auto* rx=BLEDevice::servers.back()->service.characteristics.at(1).get();
        for (size_t i=0;i<wire.size();i+=20) { rx->write(12,wire.substr(i,20)); loop(); }
    }
}
static JsonDocument lastResponse(Link link) {
    if (link == Link::BLE) {
        for (unsigned i=0; transport.txPending(link,peer(link)) && i<2400 && !restartCalls; ++i) loop();
    }
    std::string output;
    if (link == Link::USB) output=Serial.tx;
    else if (link == Link::WiFi) { require(!udpOutgoing.empty(),"missing UDP reply"); output=udpOutgoing.back().data; }
    else for (const auto& n:notifications) output+=n.data;
    JsonDocument found;
    for (size_t at=0,end; (end=output.find('\n',at))!=std::string::npos; at=end+1) {
        JsonDocument envelope,inner;
        if (deserializeJson(envelope,output.substr(at,end-at)) || !envelope["payload"].is<const char*>()) continue;
        require(!deserializeJson(inner,envelope["payload"].as<const char*>()),"invalid response payload");
        require(end-at < ControlTransport::maxLine,"oversize response frame");
        found=inner;
    }
    if (!found["id"].is<uint32_t>()) {
        JsonDocument d; fillStatus(d);
        std::cerr<<"status payload bytes="<<measureJson(d)<<" drops="<<transport.dropped<<'\n';
    }
    require(found["id"].is<uint32_t>(),"no complete protocol reply (check status frame budget)");
    return found;
}
static JsonDocument exchange(Link link, JsonDocument d) {
    const auto id=d["id"].as<uint32_t>(); inject(link,d); auto reply=lastResponse(link);
    require(reply["id"]==id,"response dropped or wrong id"); return reply;
}
static void checkSafety() {
    for (const auto& e:pico_gpio_mock::events) {
        require(!pico::reserved(e.pin) && e.pin!=40,"reserved/NC pin touched by GPIO service");
        if (std::string(e.op)=="config" && e.mode==GPIO_MODE_OUTPUT) {
            require(e.value==LOW,"retained HIGH latch enabled as output");
            if (pico::isMotorPin(e.pin)) require(e.armLevel==LOW && e.armMode==GPIO_MODE_OUTPUT,"FI/BI enabled before driven LOW ARM");
        }
        if (e.pin==17) {
            require(e.mode!=GPIO_MODE_OUTPUT,"WDI used in push-pull mode");
            if (std::string(e.op)=="level" && e.value==LOW)
                require(e.mode==GPIO_MODE_OUTPUT_OD && e.armLevel==LOW && e.armMode==GPIO_MODE_OUTPUT,
                        "watchdog pulse without OD and driven LOW ARM");
        } else if (e.pin==39) {
            require(e.mode!=GPIO_MODE_OUTPUT,"wake used in push-pull mode");
            if (std::string(e.op)=="level")
                require((e.value==HIGH && (e.mode==GPIO_MODE_INPUT || e.mode==GPIO_MODE_OUTPUT_OD)) ||
                        (e.value==LOW && e.mode==GPIO_MODE_OUTPUT_OD && e.armLevel==LOW && e.armMode==GPIO_MODE_OUTPUT),
                        "wake level/ownership or motor inhibit invalid");
            if (std::string(e.op)=="config")
                require(e.mode==GPIO_MODE_OUTPUT_OD && e.value==HIGH,"wake enabled before released latch");
        } else if (std::string(e.op)=="level") require(e.value==LOW,"ARM/cycle/charge/motor HIGH requested");
    }
    for (const auto& op:pinOperations) require(op.kind=="gpio","Pico attached/wrote a PWM peripheral");
    for (int pin=0;pin<64;++pin) if (pico::reserved(pin)) {
        require(modes[pin]==-99 && !interruptHandlers[pin],"reserved pin claimed by Arduino/encoder");
    }
    require(modes[40]==-99 && !interruptHandlers[40],"NC GPIO40 claimed");
}
static void statusChecks(const JsonDocument& d, bool gpioFault=false) {
    require(d["profile"]=="top_board_v2_pico_review","wrong selected profile");
    for (const char* field:{"motion_enabled","charge_enabled","pwm_ready","power_disconnect","motor_arm_requested","source_cycle_enabled","odometry_qualified"})
        require(d[field].is<bool>() && !d[field].as<bool>(),"unqualified capability reported ready");
    require(d["external_watchdog_ready"].is<bool>(),"missing watchdog service status");
    require(d["external_watchdog_ready"].as<bool>() == (d["external_watchdog_state"]=="active"),"inconsistent watchdog service status");
    if(gpioFault)require(d["external_watchdog_state"]=="gpio_fault","watchdog active after GPIO fault");
    for (const char* field:{"motor_voltage_mv","motor_current_ma","rail_3v3_mv","usb_vbus_mv","raw_logic_5v_mv","x","y","theta"})
        require(d[field].is<JsonVariantConst>() && d[field].isNull(),"fabricated measurement");
    for (int i=0;i<3;++i) {
        require(d["wheel_speed_mps"][i].isNull() && d["pwm_command"][i]==0,"unqualified speed/PWM");
        if (gpioFault) require(d["encoder_counts"][i].isNull(),"encoder telemetry in startup fault");
    }
}
int main(int argc, char** argv) {
    const std::string mode=argc>1?argv[1]:"startup";
    levels.fill(HIGH); modes.fill(-99); // Retained latches; no Arduino ownership.
    if (mode.rfind("fail_",0)==0 || mode.rfind("motor_",0)==0) {
        const auto first=mode.find('_'),last=mode.rfind('_');
        pico_gpio_mock::failOp=mode.substr(first+1,last-first-1);
        pico_gpio_mock::failPin=std::stoi(mode.substr(last+1));
    }
    if (mode=="release_failure") {
        pico_gpio_mock::failOp="level";pico_gpio_mock::failPin=16;pico_gpio_mock::releaseFailPin=16;
    }
    storedConfig=R"({"ssid":"public-native-fixture","password":"test-only-password","key":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"})";
    setup();
    require(Serial.uartMode && Serial.rxPin==44 && Serial.txPin==43 && Serial.baud==115200,"factory UART not selected");
    if (mode.rfind("hold_",0)==0) {
        pico_gpio_mock::failOp="level";pico_gpio_mock::failPin=std::stoi(mode.substr(5));
        halt("TEST_STOP");
    } else if (mode.rfind("release_failure_",0)==0) {
        // Arm cleanup injection only after wake's normal INPUT initialization,
        // so it fails the actual release rather than that earlier direction call.
        pico_gpio_mock::failOp="level";pico_gpio_mock::failPin=41;
        pico_gpio_mock::releaseFailPin=std::stoi(mode.substr(16));
        halt("TEST_STOP");
    }
    if (!boardSafe.ready()) {
        const bool hadRobot=robotInitialized;
        auto d=exchange(Link::USB,request("status")); statusChecks(d,true);
        require(d["reason"]=="PICO_SAFE_DEFAULTS_FAILED" && d["gpio_error"]==-42,"missing latched GPIO fault");
        if (mode.rfind("release_failure",0)==0) require(d["gpio_release_error"]==-42,"release failure hidden");
        require(!boardSafe.begin(),"reinitialization cleared latched fault");
        require(exchange(Link::USB,request("claim"))["ok"]==false,"fault allowed ownership");
        require(exchange(Link::USB,request("reboot"))["ok"]==true,"fault lost USB reboot");
        for (int i=0; i<120 && !restartCalls; ++i) loop();
        require(restartCalls==1 && robotInitialized==hadRobot,"fault recovery initialized robot or lost reboot");
        checkSafety(); std::cout<<mode<<" PASS\n"; return 0;
    }
    require(robotInitialized,"robot not initialized after safe defaults");
    require(pico_gpio_mock::driverModes[17]==GPIO_MODE_OUTPUT_OD && levels[17]==HIGH,"watchdog not released OD");
    require(pico_gpio_mock::driverModes[39]==GPIO_MODE_OUTPUT_OD && levels[39]==HIGH,"wake not released OD");
    require(pico_gpio_mock::driverModes[41]==GPIO_MODE_OUTPUT && levels[41]==LOW,"charge not driven LOW");
    require(levels[16]==LOW && levels[42]==LOW,"unsafe startup requests");
    WiFi.connected=true; loop(); BLEDevice::servers.back()->connect(12);
    if (mode=="status_budget") {
        fakeNow=0xffffff00;lastControlAt=fakeNow;
        controlGapMs=maxControlGapMs=controlOverruns=transport.dropped=transport.reconnects=UINT32_MAX;
        transport.notifyFailures=UINT32_MAX;nextId=UINT32_MAX-1;
        enc_counts_[0]=enc_counts_[1]=enc_counts_[2]=0x80000000;
        reason="SOURCE_CYCLE_UNAVAILABLE";
        auto d=exchange(Link::USB,request("status"));statusChecks(d);
        // Cover the longest dotted IPv4 field too (the transport mock uses a
        // shorter documentation address). Count escaping and the real HMAC.
        d["wifi_ip"]="255.255.255.255";
        d["reason"]="EXTERNAL_WATCHDOG_INIT_FAILED";
        d["external_watchdog_state"]="init_failed";d["external_watchdog_ready"]=false;
        require(measureJson(d)<1792,"status payload buffer too small at counter limits");
        require(signedLine(d).size()+2<ControlTransport::maxLine,"signed status buffer too small");
        // Serializer budget only: worst-width synthetic numeric fields. This
        // does not inject measurements into the monitor or qualify this state.
        for(const char* field:{"raw_logic_5v_mv","usb_vbus_mv","motor_voltage_mv"})d[field]=26000;
        d["rail_monitor"]["seq"]=UINT32_MAX;d["rail_monitor"]["age_us"]=49999;
        d["rail_monitor"]["span_us"]=29999;
        require(measureJson(d)<1792 && signedLine(d).size()+2<ControlTransport::maxLine,"rail counters overflow status envelope");
        checkSafety();std::cout<<mode<<" PASS\n";return 0;
    }
    if (mode.rfind("motor_",0)==0) {
        auto d=exchange(Link::USB,request("status")); statusChecks(d);
        int faultCount=0;for(int i=0;i<3;++i)faultCount+=d["motor_pwm_fault"][i].as<int>()!=0;
        require(faultCount==1,"motor GPIO failure not retained on its wheel");
        checkSafety();std::cout<<mode<<" PASS\n";return 0;
    }
    if (mode.rfind("reboot_",0)==0) {
        Link link=mode=="reboot_usb"?Link::USB:mode=="reboot_wifi"?Link::WiFi:Link::BLE;
        require(exchange(link,request("claim"))["ok"]==true,"transport claim failed");
        if(link==Link::BLE) for(int i=0;i<3;++i)command(link,peer(link),"hello");
        auto d=exchange(link,request("reboot")); require(d["ok"]==true,"reboot not acknowledged");
        for(int i=0;i<2600 && !restartCalls;++i)loop();
        require(restartCalls==1,"transport reboot lost");
        checkSafety();std::cout<<mode<<" PASS\n";return 0;
    }
    for(Link link:{Link::USB,Link::WiFi,Link::BLE}) {
        auto d=exchange(link,request("status"));statusChecks(d);
        for(const char* op:{"arm","start","goto","wheel","pulse","source_cycle","logic_cycle"}) {
            require(exchange(link,request("claim"))["ok"]==true,"claim failed");
            auto cmd=request(op); cmd["usb_vbus_mv"]=5000;cmd["raw_logic_5v_mv"]=0;cmd["motor_voltage_mv"]=0;cmd["logic_pgood"]=1;
            auto result=exchange(link,cmd);require(result["ok"]==false,"motion/source cycle accepted fabricated inputs");
            require(owner.link==Link::None,"rejected operation retained ownership");
        }
    }
    // UART's bool is driver readiness even with the host absent. It must not
    // enable source cycling or defeat lease expiry in battery-only conditions.
    Serial.connected=false;
    require(exchange(Link::USB,request("claim"))["ok"]==true,"UART fixture unavailable");
    fakeNow+=control::Session::timeoutMs;loop();require(owner.link==Link::None,"UART lease did not expire");
    levels[18]=0;levels[38]=0;
    auto low=exchange(Link::USB,request("status"));statusChecks(low);
    require(low["logic_flt_n"]==0 && low["logic_pgood"]==0,"raw LOW status not sampled");
    levels[18]=1;levels[38]=1;
    auto high=exchange(Link::USB,request("status"));statusChecks(high);
    require(high["logic_flt_n"]==1 && high["logic_pgood"]==1,"raw HIGH status not sampled");
    // Exercise actual low-level driver calls as well as the command gate.
    TriMotorDriver motors;motors.begin();
    for(int i=0;i<3;++i) {
        require(!motors.motor(i).setSpeed(255) && !motors.motor(i).setSpeed(-255),"driver allowed nonzero speed");
        require(!motors.motor(i).ready() && motors.motor(i).stop(),"inhibited driver lost LOW outputs");
    }
    const auto eventsBefore=pico_gpio_mock::events.size();
    MotorDriver invalid;invalid.begin(43,44,0,1,20000,8);
    require(invalid.fault()!=0 && pico_gpio_mock::events.size()==eventsBefore,"invalid motor pins touched UART");
    // Emit observed consumer behavior for independent JSON contract checking.
    JsonDocument observed;
    auto outputs=observed["outputs"].to<JsonArray>();auto inputs=observed["encoders"].to<JsonArray>();
    auto statusInputs=observed["status_inputs"].to<JsonArray>();
    for(int pin=0;pin<64;++pin) {
        if(modes[pin]==OUTPUT)outputs.add(pin);
        if(interruptHandlers[pin])inputs.add(pin);
        if(modes[pin]==INPUT && !interruptHandlers[pin])statusInputs.add(pin);
    }
    auto groups=observed["motor_config_groups"].to<JsonArray>();
    // begin() configures each physical driver's FI/BI consecutively.
    for(const auto& e:pico_gpio_mock::events)
        if(std::string(e.op)=="config" && pico::isMotorPin(e.pin))groups.add(e.pin);
    auto edges=observed["edge_responses"].to<JsonArray>();
    for(int pin=0;pin<64;++pin)if(interruptHandlers[pin]) {
        volatile long before[3],after[3];encoders_read(before);
        levels[pin]^=1;interruptHandlers[pin]();encoders_read(after);
        auto edge=edges.add<JsonArray>();edge.add(pin);
        for(int i=0;i<3;++i)edge.add(after[i]-before[i]);
    }
    auto uart=observed["uart"].to<JsonArray>();uart.add(Serial.txPin);uart.add(Serial.rxPin);
    observed["i2c_sda"]=pico::i2cSda;observed["i2c_scl"]=pico::i2cScl;
    observed["arduino_i2c_pins_claimed"]=modes[pico::i2cSda]!=-99 || modes[pico::i2cScl]!=-99;
    observed["rail_worker_open_requests"]=pico_rail_mock::bus.opens;
    observed["wake_request_n"]=pico::wakeRequestN;
    observed["charge_request"]=pico::chargeRequest;
    observed["profile"]=high["profile"];
    checkSafety();std::cout<<"PICO_OBSERVED "<<json(observed)<<"\nstartup/commands/status/encoders PASS\n";
    return 0;
}
