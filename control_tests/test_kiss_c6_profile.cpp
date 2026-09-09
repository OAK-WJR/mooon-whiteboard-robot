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
    for (const auto& e:gpio_mock::events) {
        require(e.pin==14 || kiss_c6::isMotorPin(e.pin), "GPIO service touched unrelated pin");
        if (std::string(e.operation)=="idf_set_level") require(e.level==LOW,"HIGH requested");
        if (std::string(e.operation)=="output_enable") {
            require(e.level==LOW,"retained HIGH latch enabled");
            if(e.pin!=14)require(e.requestLevel==LOW && e.requestDriven,"FI/BI enabled before ARM LOW");
        }
    }
    for(const auto& e:pinOperations)require(e.kind=="gpio","PWM peripheral activated in review build");
    for(int pin:{4,5,8,9,10,11,12,13,15,16,17})
        require(modes[pin]==-99 && !interruptHandlers[pin],"reserved GPIO used as motor/encoder");
}
static void statusChecks(const JsonDocument& d) {
    require(d["profile"]=="top_board_v2_kiss_c6_review","wrong profile");
    for(const char* f:{"motion_enabled","charge_enabled","pwm_ready","power_disconnect","odometry_qualified"})
        require(d[f].is<bool>() && !d[f].as<bool>(),"unqualified capability reported ready");
    for(const char* f:{"x","y","theta","motor_voltage_mv","motor_current_ma","rail_3v3_mv"})
        require(d[f].is<JsonVariantConst>() && d[f].isNull(),"unmeasured quantity reported");
    for(int i=0;i<3;++i)require(d["pwm_command"][i]==0 && d["wheel_speed_mps"][i].isNull(),"nonzero PWM or uncalibrated speed");
}
int main(int argc,char** argv) {
    const std::string mode=argc>1?argv[1]:"startup";
    levels.fill(HIGH);modes.fill(-99);gpio_mock::requestPin=14;
    if(mode=="arm_preload" || mode=="release_failure"){
        gpio_mock::levelFailurePin=14;gpio_mock::levelFailures=1;
    }
    if(mode=="arm_config"){gpio_mock::outputFailurePin=14;gpio_mock::outputFailures=1;}
    if(mode=="release_failure"){gpio_mock::releaseFailurePin=14;gpio_mock::releaseFailures=1;}
    const bool motorFailure=mode.rfind("motor_",0)==0;
    if(motorFailure){
        const int pin=std::stoi(mode.substr(mode.rfind('_')+1));
        if(mode.rfind("motor_level_",0)==0){gpio_mock::levelFailurePin=pin;gpio_mock::levelFailures=1;}
        else {gpio_mock::outputFailurePin=pin;gpio_mock::outputFailures=1;}
    }
    storedConfig=R"({"ssid":"public-native-fixture","password":"test-only-password","key":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"})";
    if(mode=="radio_usb_only"){adc_mock::mv[0]=0;adc_mock::raw[0]=0;}
    const bool cookieTest=mode=="radio_prior_reset"||mode=="radio_cold_cookie"||mode=="radio_bad_cookie";
    if(cookieTest){
        adc_mock::mv[0]=0;adc_mock::raw[0]=0;
        radioRestartCookie[0]=radioResetMagic;
        radioRestartCookie[1]=mode=="radio_bad_cookie"?0:~radioResetMagic;
        fakeResetReason=mode=="radio_cold_cookie"?1:ESP_RST_SW;
    }
    if(mode=="radio_adc_failure")adc_mock::calError=ESP_ERR_NOT_SUPPORTED;
    if(mode=="radio_no_watchdog")watchdogResult=-42;
    if(mode=="radio_init_fail")radioFailureStage=1;
    if(mode=="radio_service_fail")radioFailureStage=2;
    if(mode=="radio_advertise_fail")radioFailureStage=3;
    if(mode=="radio_wifi_fail")radioFailureStage=4;
    if(mode=="radio_init_loss"){
        BLEDevice::initDelayMs=300;
        BLEDevice::duringInit=[](){adc_mock::mv[0]=0;adc_mock::raw[0]=0;};
    }
    setup();
    require(Serial.uartMode&&Serial.rxPin==17&&Serial.txPin==16&&Serial.baud==115200,"wrong UART console");
    require(BLEDevice::initCalls==0&&WiFi.modeCalls==0&&WiFi.beginCalls==0,"startup enabled radio before battery qualification");
    if(mode=="arm_hold"){gpio_mock::levelFailurePin=14;gpio_mock::levelFailures=1;halt("TEST_STOP");}
    if(!boardSafe.ready()){
        const bool initialized=robotInitialized;
        auto d=exchange(Link::USB,request("status"));statusChecks(d);
        require(d["reason"]=="GPIO14_SAFE_LOW_FAILED" && d["gpio_error"]==-42,"GPIO failure hidden");
        require(d["encoder_counts"][0].isNull(),"fault read encoder counts");
        if(mode=="release_failure")require(d["gpio_release_error"]==-42,"release error hidden");
        if(mode!="arm_hold"){
            require(!robotInitialized,"startup failure initialized robot");
            for(auto handler:interruptHandlers)require(!handler,"startup failure attached encoder ISR");
        }
        require(exchange(Link::USB,request("claim"))["ok"]==false,"fault accepted ownership");
        require(exchange(Link::USB,request("reboot"))["ok"]==true,"fault lost USB reboot");
        for(int i=0;i<120 && !restartCalls;++i)loop();
        require(restartCalls==1 && robotInitialized==initialized,"fault recovery did not reboot cleanly");
        checkSafety();std::cout<<mode<<" PASS\n";return 0;
    }
    if(mode=="adc_lease_deadline"){
        require(exchange(Link::USB,request("claim"))["ok"]==true,"claim failed");
        fakeNow+=control::Session::timeoutMs-2;lastControlAt=fakeNow;
        adc_mock::readDelayMs=2;
        const unsigned reads=adc_mock::readCalls;loop();
        require(adc_mock::readCalls>reads,"ADC deadline fixture did not sample");
        require(owner.link==Link::None&&!strcmp(reason,"HEARTBEAT_TIMEOUT"),"ADC work retained expired owner");
        checkSafety();std::cout<<mode<<" PASS\n";return 0;
    }
    if(mode=="radio_usb_only"||mode=="radio_adc_failure"||mode=="radio_no_watchdog"||cookieTest){
        for(int i=0;i<1400;++i)loop();
        require(BLEDevice::initCalls==0&&WiFi.beginCalls==0&&WiFi.modeCalls==0,"USB-only/fault startup touched radios");
        auto d=exchange(Link::USB,request("status"));statusChecks(d);
        require(d["ble_ready"]==false&&d["wifi_connected"]==false,"inactive wireless reported ready");
        auto rail=exchange(Link::USB,request("rail_status"));
        require(rail["wireless_start_attempted"]==false,"unexpected wireless attempt");
        require(rail["previous_radio_power_reset"]==(mode=="radio_prior_reset"),"previous reset marker incorrectly consumed");
        require(radioRestartCookie[0]==0&&radioRestartCookie[1]==0,"old reset tag not cleared for next reboot");
        if(mode=="radio_adc_failure")require(rail["fresh"]==false&&rail["battery_voltage_mv"].isNull(),"ADC failure hidden");
        require(exchange(Link::USB,request("reboot"))["ok"]==true,"USB-only reboot unavailable");
        for(int i=0;i<200&&!restartCalls;++i)loop();
        require(restartCalls==1&&BLEDevice::initCalls==0,"USB-only recovery enabled radio or lost reboot");
        checkSafety();std::cout<<mode<<" PASS\n";return 0;
    }
    if(mode=="radio_qualify"){
        for(int i=0;i<750;++i)loop();
        require(BLEDevice::initCalls==0,"radio started before dwell");
        adc_mock::mv[0]=490; // 6.37V, below startup threshold
        for(int i=0;i<200;++i)loop();
        adc_mock::mv[0]=600;
        for(int i=0;i<900;++i)loop();
        require(BLEDevice::initCalls==0,"voltage dip did not reset dwell");
    }
    for(unsigned i=0;i<1400&&!BLEDevice::initCalls&&!restartCalls;++i)loop();
    if(radioFailureStage){
        require(!restartCalls&&BLEDevice::initCalls==1&&WiFi.beginCalls==0,"failed init triggered retry or Wi-Fi connect");
        auto rail=exchange(Link::USB,request("rail_status"));
        require(rail["wireless_power_state"]=="init_failed","SDK failure reported ready");
        for(int i=0;i<1500;++i)loop();
        require(BLEDevice::initCalls==1,"failed SDK initialization retried without cleanup");
        require(exchange(Link::USB,request("reboot"))["ok"]==true,"SDK failure lost USB recovery");
        for(int i=0;i<200&&!restartCalls;++i)loop();
        require(restartCalls==1,"SDK failure lost reboot");
        checkSafety();std::cout<<mode<<" PASS\n";return 0;
    }
    if(mode=="radio_init_loss"){
        require(restartCalls==1&&owner.link==Link::None&&!strcmp(reason,"RADIO_POWER_LOST"),"battery loss during blocking SDK init missed");
        checkSafety();std::cout<<mode<<" PASS\n";return 0;
    }
    require(BLEDevice::initCalls==1&&WiFi.beginCalls==1&&!restartCalls,"qualified battery failed to enable both radios exactly once");
    WiFi.connected=true;loop();BLEDevice::servers.back()->connect(12);
    if(mode=="radio_qualify"){
        for(int i=0;i<2000;++i)loop();
        require(BLEDevice::initCalls==1&&WiFi.beginCalls==1,"radios repeatedly initialized");
        statusChecks(exchange(Link::WiFi,request("status")));
        statusChecks(exchange(Link::BLE,request("status")));
        checkSafety();std::cout<<mode<<" PASS\n";return 0;
    }
    if(mode=="radio_battery_loss"||mode=="radio_adc_loss"||mode=="radio_stale"||mode=="radio_gpio_loss"){
        require(exchange(Link::WiFi,request("claim"))["ok"]==true,"wireless claim failed");
        auto queued=request("heartbeat");udpIncoming.push_back({IPAddress(0xc0000201),5555,signedLine(queued)+'\n'});
        if(mode=="radio_battery_loss"){adc_mock::mv[0]=0;adc_mock::raw[0]=0;fakeNow+=100;}
        if(mode=="radio_adc_loss"){adc_mock::readError=ESP_ERR_TIMEOUT;fakeNow+=100;}
        if(mode=="radio_stale")fakeNow+=251;
        if(mode=="radio_gpio_loss"){gpio_mock::levelFailurePin=14;gpio_mock::levelFailures=1;halt("TEST_GPIO_FAULT");}
        const auto reads=adc_mock::readCalls;loop();
        require(restartCalls==1&&owner.link==Link::None,"power loss retained radio control");
        require(radioRestartCookie[0]==radioResetMagic&&radioRestartCookie[1]==~radioResetMagic,"automatic reset reason not retained");
        require(udpIncoming.size()==1,"queued radio command processed after power loss");
        if(mode=="radio_stale")require(adc_mock::readCalls==reads,"fresh sample concealed expired evidence");
        checkSafety();std::cout<<mode<<" PASS\n";return 0;
    }
    if(mode=="rails"){
        fakeNow+=10;loop();
        for(Link link:{Link::USB,Link::WiFi,Link::BLE}){
            auto report=exchange(link,request("rail_status"));
            require(report["fresh"]==true&&report["kind"]=="rails","rail response missing");
            require(report["battery_voltage_mv"]==7800&&report["motor_voltage_mv"]==5980,"rail mapping/scaling mismatch");
            require(report["hardware_qualified"]==false&&report["divider_calibrated"]==false,"qualification overstated");
            if(link==Link::USB)std::cout<<"KISS_RAILS "<<json(report)<<'\n';
        }
        adc_mock::readError=ESP_ERR_TIMEOUT;fakeNow+=100;loop();
        require(!voltageMonitor.reading().valid&&voltageMonitor.error()==ESP_ERR_TIMEOUT,"ADC error hidden");
        require(restartCalls==1&&owner.link==Link::None,"ADC failure after RF startup did not reset to source qualification");
        checkSafety();std::cout<<mode<<" PASS\n";return 0;
    }
    if(mode=="status_budget"){
        fakeNow=UINT32_MAX;
        controlGapMs=maxControlGapMs=controlOverruns=transport.dropped=transport.reconnects=UINT32_MAX;
        transport.notifyFailures=UINT32_MAX;reason="TRANSPORT_INIT_FAILED";
        for(auto& count:enc_counts_)count=0x80000000U;
        require(owner.claim(Link::USB,0,UINT32_MAX,UINT32_MAX,millis()),"budget fixture claim failed");
        Serial.tx.clear();reply(Link::USB,0,UINT32_MAX,nullptr,true);
        auto d=lastResponse(Link::USB);statusChecks(d);
        std::cout<<"status payload="<<measureJson(d)<<" frame="<<Serial.tx.size()<<'\n';
        require(Serial.tx.size()<1472,"status exceeds conservative single UDP datagram budget");
        checkSafety();std::cout<<mode<<" PASS\n";return 0;
    }
    if(mode.rfind("reboot_",0)==0){
        Link link=mode=="reboot_usb"?Link::USB:mode=="reboot_wifi"?Link::WiFi:Link::BLE;
        require(exchange(link,request("claim"))["ok"]==true,"claim failed");
        require(exchange(link,request("reboot"))["ok"]==true,"reboot rejected");
        for(int i=0;i<2600 && !restartCalls;++i)loop();
        require(restartCalls==1,"reboot lost");checkSafety();std::cout<<mode<<" PASS\n";return 0;
    }
    auto d=exchange(Link::USB,request("status"));statusChecks(d);
    if(motorFailure){
        int n=0;for(int i=0;i<3;++i)n+=d["motor_pwm_fault"][i].as<int>()!=0;
        require(n==1,"motor fault assigned to wrong number of wheels");
        checkSafety();std::cout<<mode<<" PASS\n";return 0;
    }
    for(Link link:{Link::USB,Link::WiFi,Link::BLE}){
        statusChecks(exchange(link,request("status")));
        for(const char* op:{"start","goto","wheel","arm","pulse"}){
            require(exchange(link,request("claim"))["ok"]==true,"claim failed");
            auto result=exchange(link,request(op));
            require(result["error"]=="BOARD_POWER_UNREVIEWED" && !result["ok"].as<bool>(),"motion gate bypass");
            require(owner.link==Link::None,"motion rejection retained ownership");
        }
    }
    Serial.connected=false;
    require(exchange(Link::USB,request("claim"))["ok"]==true,"UART fixture lost");
    for(unsigned i=0;i<=control::Session::timeoutMs;++i)loop();
    require(owner.link==Link::None&&!restartCalls,"cable loss did not expire lease independently of fresh supply monitoring");
    JsonDocument observed;
    auto outputs=observed["outputs"].to<JsonArray>();auto inputs=observed["encoders"].to<JsonArray>();
    for(int pin=0;pin<64;++pin){if(modes[pin]==OUTPUT)outputs.add(pin);if(interruptHandlers[pin])inputs.add(pin);}
    auto groups=observed["motor_config_groups"].to<JsonArray>();
    for(const auto& e:gpio_mock::events)if(std::string(e.operation)=="output_enable" && e.pin!=14)groups.add(e.pin);
    auto edges=observed["edge_responses"].to<JsonArray>();
    for(int pin=0;pin<64;++pin)if(interruptHandlers[pin]){
        volatile long before[3],after[3];encoders_read(before);levels[pin]^=1;interruptHandlers[pin]();encoders_read(after);
        auto edge=edges.add<JsonArray>();edge.add(pin);for(int i=0;i<3;++i)edge.add(after[i]-before[i]);
    }
    auto uart=observed["uart"].to<JsonArray>();uart.add(Serial.txPin);uart.add(Serial.rxPin);
    observed["arm"]=kiss_c6::motorArmRequest;
    auto adc=observed["adc_gpio"].to<JsonArray>();for(int pin:adc_mock::mapped)adc.add(pin);
    TriMotorDriver motors;motors.begin();
    for(int i=0;i<3;++i)require(!motors.motor(i).setSpeed(255)&&!motors.motor(i).setSpeed(-255)&&motors.motor(i).stop(),"review driver allowed modulation");
    const auto n=gpio_mock::events.size();MotorDriver invalid;invalid.begin(16,17,0,1,20000,8);
    require(invalid.fault()!=0 && gpio_mock::events.size()==n,"invalid pair touched UART");
    checkSafety();
    std::cout<<"KISS_OBSERVED "<<json(observed)<<'\n';
    std::cout<<"KISS_STATUS "<<json(d)<<"\nstartup/commands/encoders PASS\n";
}
