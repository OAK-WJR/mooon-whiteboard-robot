// Runs the actual KISS C6 profile, controller, driver, ISRs, command loop and transport against the
// 2026-10-29 EasyEDA board: native USB Serial/JTAG CDC console (no USB-UART bridge), ARM request (MOT_EN)
// on GPIO0, IO4 VBAT divider, IO5 INA180 motor current, IO8 USB-power sense, 15/16/17 sensor-head pins.
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
    if (!strcmp(op,"provision")) {
        d["ssid"]="replacement-ssid"; d["password"]="replacement-password"; d["key"]=std::string(64,'b');
    }
    return d;
}

constexpr int arm = kiss_c6::motorArmRequest;
// The latched-fault reason names the pin that actually failed (GPIO0 = MOT_EN), never a stale map.
static const std::string faultReason = "GPIO" + std::to_string(arm) + "_SAFE_LOW_FAILED";
// ADC inputs (owned by the ADC driver, never GPIO-configured), GPIO9 BOOT strap, GPIO10/11 (not bonded out on
// the MINI-1) and GPIO12/13 USB D-/D+ (any pinMode there drops the CDC console).
constexpr int reservedPins[] = {kiss_c6::batterySense, kiss_c6::motorSense, 9, 10, 11, 12, 13};
// Sensor head J1 (I2C SDA/SCL, IR_EN_N with its pull-up): application pins, never used by the motor/GPIO-safety service.
constexpr int applicationPins[] = {kiss_c6::i2cSda, kiss_c6::i2cScl, kiss_c6::irEnableN};
constexpr const char* headerOrder[] = {"M0", "M2", "M1"};   // status arrays: physical Motor1/2/3
static bool isEncoderPin(int pin) {
    for (const auto& h : kiss_c6::headers) if (pin == h.encA || pin == h.encB) return true;
    return false;
}
static int wheelOfPin(int pin) {
    for (const auto& h : kiss_c6::headers)
        if (pin == h.fi || pin == h.bi || pin == h.encA || pin == h.encB) return h.physical - 1;
    return -1;
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
static size_t lastPayloadBytes = 0, lastFrameBytes = 0;
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
        lastPayloadBytes=strlen(envelope["payload"].as<const char*>());lastFrameBytes=end-at+1;
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
        require(e.pin==arm || kiss_c6::isMotorPin(e.pin), "GPIO service touched a pin other than ARM/FI/BI");
        if (std::string(e.operation)=="idf_set_level") require(e.level==LOW,"HIGH requested");
        if (std::string(e.operation)=="output_enable") {
            require(e.level==LOW,"retained HIGH latch enabled");
            if(e.pin!=arm)require(e.requestLevel==LOW && e.requestDriven,"FI/BI enabled before ARM LOW");
        }
    }
    for(const auto& e:pinOperations){
        require(e.kind=="gpio","PWM peripheral activated in review build");
        require(e.pin==arm || kiss_c6::isMotorPin(e.pin),"level written to a pin other than ARM/FI/BI");
    }
    for(int pin=0;pin<64;++pin){
        if(modes[pin]==OUTPUT)require(pin==arm || kiss_c6::isMotorPin(pin),"output enabled on a pin other than ARM/FI/BI");
        if(interruptHandlers[pin])require(isEncoderPin(pin),"interrupt attached to a non-encoder pin");
    }
    for(int pin:reservedPins)
        require(modes[pin]==-99 && !interruptHandlers[pin] && !gpio_mock::outputEnabled[pin],"reserved GPIO configured (ADC/BOOT/USB)");
    for(int pin:applicationPins)
        require(modes[pin]==-99 && !interruptHandlers[pin] && !gpio_mock::outputEnabled[pin] && levels[pin]==HIGH,
                "sensor-head pin used by the motor/GPIO-safety service");
    // GPIO8 is the USB-power sense: a plain input (the divider/isolation sets the level), never driven or pulled.
    require(modes[kiss_c6::usbPowerSense]==INPUT && !interruptHandlers[kiss_c6::usbPowerSense] &&
            !gpio_mock::outputEnabled[kiss_c6::usbPowerSense],"IO8 USB-power sense is not a plain input");
}
static void statusChecks(const JsonDocument& d) {
    require(d["profile"]=="top_board_v2_kiss_c6_review","wrong profile");
    for(const char* f:{"motion_enabled","charge_enabled","pwm_ready","power_disconnect","odometry_qualified"})
        require(d[f].is<bool>() && !d[f].as<bool>(),"unqualified capability reported ready");
    for(const char* f:{"x","y","theta","motor_voltage_mv","motor_current_ma","rail_3v3_mv"})
        require(d[f].is<JsonVariantConst>() && d[f].isNull(),"unmeasured quantity reported");
    for(int i=0;i<3;++i)require(d["pwm_command"][i]==0 && d["wheel_speed_mps"][i].isNull(),"nonzero PWM or uncalibrated speed");
}
static void faultChecks(const JsonDocument& d) {
    statusChecks(d);
    require(faultReason==safeDefaultsFailure,"firmware fault reason does not name the ARM pin");
    require(d["state"]=="gpio_fault" && d["reason"]==faultReason && d["gpio_fault_latched"]==true,"GPIO fault hidden");
    require(d["gpio_error"]==gpio_mock::failure && d["read_only"]==true && d["wireless_control_enabled"]==false &&
            d["owner"]==int(Link::None) && d["usb_reboot"]==true,"fault state not reported read-only");
    for(int i=0;i<3;++i)require(d["encoder_counts"][i].isNull(),"fault read encoder counts");
    for(const char* f:{"wireless_status","wireless_power_state","wifi_connected","wifi_ip","wifi_rssi_dbm",
                       "ble_ready","ble_connected","authenticated_wireless","watchdog_ready"})
        require(d[f].is<JsonVariantConst>(),"fault status lacks a wireless/watchdog field");
}
static void railChecks(const JsonDocument& r) {
    require(r["kind"]=="rails" && r["profile"]=="top_board_v2_kiss_c6_review","rail response missing");
    require(r["hardware_qualified"]==false && r["divider_calibrated"]==false && r["divider_gain"]==13,"qualification overstated");
    require(r["motor_voltage_mv"].is<JsonVariantConst>() && r["motor_voltage_mv"].isNull(),"VMOT reported without a VMOT divider");
    require(r["usb_power"].is<bool>(),"USB power not reported");
    require(r["charge"]==(r["usb_power"].as<bool>()?"external_power":"battery"),"charge state not derived from IO8");
    if(r["fresh"]==true){
        require(r["motor_current_ma"]==r["adc_pin_mv"][1].as<int>()/2 && r["motor_current_saturated"].is<bool>(),
                "motor current is not the INA180 pin voltage at 2 V/A");
        require(r["battery_voltage_mv"].is<int>() && r["state"]=="ready","fresh rail sample incomplete");
    }else{
        require(r["battery_voltage_mv"].isNull() && r["motor_current_ma"].isNull() &&
                !r["motor_current_saturated"].is<JsonVariantConst>(),"stale rail values reported");
    }
}
static void done(const std::string& mode) { checkSafety(); std::cout<<mode<<" PASS\n"; }
static int digits(uint64_t v) { int n=1; while (v>=10) { v/=10; ++n; } return n; }

// Boot-time GPIO fault (ARM preload failed) with a healthy battery: the board stays read-only but
// the radio still qualifies the battery, starts once and answers read-only queries.
static int faultRadio(const std::string& mode) {
    require(!boardSafe.ready() && !robotInitialized,"boot GPIO fault fixture did not latch");
    for(auto handler:interruptHandlers)require(!handler,"startup failure attached encoder ISR");
    require(watchdogReady==(mode!="fault_no_watchdog") && watchdogAdds==(mode!="fault_no_watchdog"),
            "fault state did not enrol the task watchdog exactly once");
    if(mode=="fault_no_watchdog"){
        for(int i=0;i<2000;++i)loop();
        require(BLEDevice::initCalls==0 && WiFi.beginCalls==0 && !restartCalls,"radio started without the task watchdog");
        auto rail=exchange(Link::USB,request("rail_status"));railChecks(rail);
        require(rail["fresh"]==true && rail["wireless_power_state"]=="inhibited" && rail["wireless_start_attempted"]==false,
                "missing watchdog not reported as radio inhibit");
        auto d=exchange(Link::USB,request("status"));faultChecks(d);
        require(d["watchdog_ready"]==false && d["wireless_status"]==false,"watchdog/radio state overstated");
        require(exchange(Link::USB,request("reboot"))["ok"]==true,"USB recovery unavailable");
        for(int i=0;i<200 && !restartCalls;++i)loop();
        require(restartCalls==1,"USB recovery reboot lost");
        done(mode);return 0;
    }
    for(int i=0;i<1400 && !BLEDevice::initCalls && !restartCalls;++i)loop();
    if(mode=="fault_init_loss"){
        require(restartCalls==1 && owner.link==Link::None,"battery loss during the blocking SDK init (fault state) missed");
        require(radioRestartCookie[0]==radioResetMagic && radioRestartCookie[1]==~radioResetMagic,"radio power reset not tagged");
        done(mode);return 0;
    }
    require(BLEDevice::initCalls==1 && WiFi.beginCalls==1 && !restartCalls,
            "fault state did not start the radios once after battery qualification (or a 300 ms SDK init restarted)");
    for(int i=0;i<300;++i)loop();
    require(!restartCalls && BLEDevice::initCalls==1 && WiFi.beginCalls==1,"fault-state radios restarted or re-initialized");
    WiFi.connected=true;loop();BLEDevice::servers.back()->connect(12);loop();
    if(mode=="fault_battery_loss"||mode=="fault_adc_loss"||mode=="fault_stale"){
        if(mode=="fault_battery_loss"){adc_mock::mv[0]=0;adc_mock::raw[0]=0;fakeNow+=100;}
        if(mode=="fault_adc_loss"){adc_mock::readError=ESP_ERR_TIMEOUT;fakeNow+=100;}
        if(mode=="fault_stale")fakeNow+=251;
        const auto reads=adc_mock::readCalls;loop();
        require(restartCalls==1 && owner.link==Link::None,"supply loss in the GPIO-fault state kept the radios");
        require(radioRestartCookie[0]==radioResetMagic && radioRestartCookie[1]==~radioResetMagic,"radio power reset not tagged");
        if(mode=="fault_stale")require(adc_mock::readCalls==reads,"fresh sample concealed expired evidence");
        done(mode);return 0;
    }
    if(mode=="fault_budget"){
        // Worst case of the fault status sent over the radio: the rejection reply (error string + details),
        // longest IP/RSSI, maximal counters, a two-digit reset reason.
        fakeIpText="255.255.255.255";fakeRssi=-100;fakeResetReason=15;fakeNow=UINT32_MAX;
        transport.dropped=UINT32_MAX;udpOutgoing.clear();
        reply(Link::WiFi,peer(Link::WiFi),UINT32_MAX,safeDefaultsFailure,true);
        auto d=lastResponse(Link::WiFi);faultChecks(d);
        require(d["error"]==faultReason && d["wifi_ip"]=="255.255.255.255" && d["wifi_rssi_dbm"]==-100,"budget fixture not applied");
        // Unreached here: a longer radio phase ("waiting_voltage" vs "online").
        const size_t headroom=strlen("waiting_voltage")-strlen(d["wireless_power_state"].as<const char*>());
        std::cout<<"fault status payload="<<lastPayloadBytes<<" (+"<<headroom<<" headroom) frame="<<lastFrameBytes<<'\n';
        require(lastPayloadBytes+headroom<replyPayloadBytes,"fault status exceeds the firmware reply buffer");
        require(lastFrameBytes<1472,"fault status exceeds a single UDP datagram");
        done(mode);return 0;
    }
    require(mode=="fault_wireless","unknown fault mode");
    const std::string config=storedConfig;
    for(Link link:{Link::WiFi,Link::BLE}){
        auto st=exchange(link,request("status"));faultChecks(st);
        require(st["wireless_status"]==true && st["wireless_power_state"]=="online" && st["authenticated_wireless"]==true &&
                st["watchdog_ready"]==true,"fault status hides the read-only radio");
        require(st[link==Link::WiFi?"wifi_connected":"ble_connected"]==true,"radio link not reported");
        if(link==Link::WiFi)std::cout<<"KISS_FAULT_STATUS "<<json(st)<<'\n';
        auto rail=exchange(link,request("rail_status"));railChecks(rail);
        require(rail["ok"]==true && rail["fresh"]==true && rail["battery_voltage_mv"]==7800 &&
                rail["wireless_power_state"]=="online" && rail["wireless_start_attempted"]==true,"fault rails over radio");
        for(const char* op:{"claim","heartbeat","release","reboot","provision","start","goto","wheel","arm","pulse",
                            "area","pos","set_speed"}){
            auto r=exchange(link,request(op));
            require(r["ok"]==false && r["error"]==faultReason && r["state"]=="gpio_fault","wireless command accepted in the GPIO-fault state");
            require(owner.link==Link::None && !rebootPending && storedConfig==config && !restartCalls,"wireless command had a side effect in the fault state");
        }
        auto stop=exchange(link,request("stop"));
        require(stop["ok"]==true && stop["state"]=="gpio_fault" && stop["reason"]==faultReason,"stop not answered read-only");
    }
    // Unsigned discovery over Wi-Fi gets the same read-only status.
    udpIncoming.push_back({IPAddress(0xc0000201),5555,"hello\n"});loop();
    {
        JsonDocument envelope,hello;
        require(!deserializeJson(envelope,udpOutgoing.back().data) && !deserializeJson(hello,envelope["payload"].as<const char*>()),"hello reply");
        require(hello["id"]==0 && hello["state"]=="gpio_fault","hello over Wi-Fi not answered read-only");
    }
    require(exchange(Link::USB,request("claim"))["ok"]==false,"USB ownership accepted in the fault state");
    require(exchange(Link::USB,request("provision"))["ok"]==false && storedConfig==config,"USB provisioning accepted in the fault state");
    require(!restartCalls && BLEDevice::initCalls==1 && WiFi.beginCalls==1,"read-only traffic restarted the radios");
    require(exchange(Link::USB,request("reboot"))["ok"]==true,"USB recovery unavailable with the radio up");
    for(int i=0;i<200 && !restartCalls;++i)loop();
    require(restartCalls==1 && radioRestartCookie[0]==0 && radioRestartCookie[1]==0,"USB recovery reboot lost or tagged as a power loss");
    done(mode);return 0;
}

static bool startFaultInjected=false;
int main(int argc,char** argv) {
    const std::string mode=argc>1?argv[1]:"startup";
    levels.fill(HIGH);modes.fill(-99);gpio_mock::requestPin=arm;
    const bool faultMode=mode.rfind("fault_",0)==0;
    if(mode=="arm_preload" || mode=="release_failure" || faultMode){
        gpio_mock::levelFailurePin=arm;gpio_mock::levelFailures=1;
    }
    if(mode=="arm_config"){gpio_mock::outputFailurePin=arm;gpio_mock::outputFailures=1;}
    if(mode=="release_failure"){gpio_mock::releaseFailurePin=arm;gpio_mock::releaseFailures=1;}
    const bool motorFailure=mode.rfind("motor_",0)==0;
    if(motorFailure){
        const int pin=std::stoi(mode.substr(mode.rfind('_')+1));
        require(kiss_c6::isMotorPin(pin),"motor fault mode names a pin that is not FI/BI");
        if(mode.rfind("motor_level_",0)==0){gpio_mock::levelFailurePin=pin;gpio_mock::levelFailures=1;}
        else {gpio_mock::outputFailurePin=pin;gpio_mock::outputFailures=1;}
    }
    storedConfig=R"({"ssid":"public-native-fixture","password":"test-only-password","key":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"})";
    // BLEDevice::init blocks ~300 ms on the real SDK: every radio start here pays it, so a supply check that
    // judged the pre-init sample afterwards would restart (RADIO_POWER_LOST) in every scenario.
    BLEDevice::initDelayMs=300;
    if(mode=="radio_usb_only"){adc_mock::mv[0]=0;adc_mock::raw[0]=0;}
    const bool cookieTest=mode=="radio_prior_reset"||mode=="radio_cold_cookie"||mode=="radio_bad_cookie";
    if(cookieTest){
        adc_mock::mv[0]=0;adc_mock::raw[0]=0;
        radioRestartCookie[0]=radioResetMagic;
        radioRestartCookie[1]=mode=="radio_bad_cookie"?0:~radioResetMagic;
        fakeResetReason=mode=="radio_cold_cookie"?1:ESP_RST_SW;
    }
    if(mode=="radio_adc_failure")adc_mock::calError=ESP_ERR_NOT_SUPPORTED;
    if(mode=="radio_no_watchdog"||mode=="fault_no_watchdog")watchdogResult=-42;
    if(mode=="radio_init_fail")radioFailureStage=1;
    if(mode=="radio_service_fail")radioFailureStage=2;
    if(mode=="radio_advertise_fail")radioFailureStage=3;
    if(mode=="radio_wifi_fail")radioFailureStage=4;
    if(mode=="radio_init_loss"||mode=="fault_init_loss")
        BLEDevice::duringInit=[](){adc_mock::mv[0]=0;adc_mock::raw[0]=0;};
    if(mode=="radio_start_gpio_fault"){
        // Fail exactly the ARM write inside halt("RADIO_STARTING"), just before the blocking radio init.
        gpio_mock::beforeSetLevel=[](int pin){
            if(pin==arm && !startFaultInjected && !strcmp(radioPower.state(),"starting")){
                startFaultInjected=true;gpio_mock::levelFailurePin=arm;gpio_mock::levelFailures=1;
            }
        };
    }
    if(mode=="rails_budget")fakeNow=UINT32_MAX-4000;   // ten-digit uptime; sampling and qualification cross the wrap
    setup();
    require(Serial.cdcMode && !Serial.uartMode && Serial.rxPin==-1 && Serial.txPin==-1 && Serial.baud==115200 &&
            Serial.txTimeoutMs==2,"console is not the native USB Serial/JTAG CDC (HWCDC)");
    // HWCDC is not "connected" until the host first moves a USB packet, so no protocol frame sent from setup() (the
    // boot-time GPIO-fault reply included) reaches a host: ControlTransport drops it (tx_dropped) without queueing
    // it, and hosts read the fault with hello/status. (Plain println diagnostics bypass ControlTransport.)
    require(!Serial.connected && Serial.tx.find("\"payload\"")==std::string::npos && !Serial.hostAbsentWrites,
            "setup() protocol output reached a USB host that had not connected");
    require(BLEDevice::initCalls==0&&WiFi.modeCalls==0&&WiFi.beginCalls==0,"startup enabled radio before battery qualification");
    require(modes[kiss_c6::usbPowerSense]==INPUT,"IO8 USB-power sense not configured as an input");
    if(faultMode)return faultRadio(mode);
    if(mode=="arm_hold"){gpio_mock::levelFailurePin=arm;gpio_mock::levelFailures=1;halt("TEST_STOP");}
    if(!boardSafe.ready()){
        const bool initialized=robotInitialized;
        auto d=exchange(Link::USB,request("status"));faultChecks(d);
        if(mode=="release_failure")require(d["gpio_release_error"]==gpio_mock::failure,"release error hidden");
        if(mode!="arm_hold"){
            require(!robotInitialized,"startup failure initialized robot");
            for(auto handler:interruptHandlers)require(!handler,"startup failure attached encoder ISR");
        }
        for(int i=0;i<20;++i)loop();   // first ADC sample after the 10 ms settle
        auto rail=exchange(Link::USB,request("rail_status"));railChecks(rail);
        require(rail["fresh"]==true,"fault state stopped the rail monitor");
        const std::string config=storedConfig;
        require(exchange(Link::USB,request("claim"))["ok"]==false,"fault accepted ownership");
        require(exchange(Link::USB,request("provision"))["ok"]==false && storedConfig==config,"fault accepted provisioning");
        require(exchange(Link::USB,request("reboot"))["ok"]==true,"fault lost USB reboot");
        for(int i=0;i<120 && !restartCalls;++i)loop();
        require(restartCalls==1 && robotInitialized==initialized,"fault recovery did not reboot cleanly");
        require(radioRestartCookie[0]==0 && radioRestartCookie[1]==0,"USB recovery tagged as a radio power reset");
        done(mode);return 0;
    }
    if(mode=="adc_lease_deadline"){
        require(exchange(Link::USB,request("claim"))["ok"]==true,"claim failed");
        fakeNow+=control::Session::timeoutMs-2;lastControlAt=fakeNow;
        adc_mock::readDelayMs=2;
        const unsigned reads=adc_mock::readCalls;loop();
        require(adc_mock::readCalls>reads,"ADC deadline fixture did not sample");
        require(owner.link==Link::None&&!strcmp(reason,"HEARTBEAT_TIMEOUT"),"ADC work retained expired owner");
        done(mode);return 0;
    }
    if(mode=="radio_usb_only"||mode=="radio_adc_failure"||mode=="radio_no_watchdog"||cookieTest){
        for(int i=0;i<1400;++i)loop();
        require(BLEDevice::initCalls==0&&WiFi.beginCalls==0&&WiFi.modeCalls==0,"USB-only startup touched radios");
        auto d=exchange(Link::USB,request("status"));statusChecks(d);
        require(d["ble_ready"]==false&&d["wifi_connected"]==false,"inactive wireless reported ready");
        auto rail=exchange(Link::USB,request("rail_status"));railChecks(rail);
        require(rail["wireless_start_attempted"]==false,"unexpected wireless attempt");
        require(rail["previous_radio_power_reset"]==(mode=="radio_prior_reset"),"previous reset marker incorrectly consumed");
        require(radioRestartCookie[0]==0&&radioRestartCookie[1]==0,"old reset tag not cleared for next reboot");
        if(mode=="radio_adc_failure")require(rail["fresh"]==false&&rail["state"]=="adc_calibration","ADC failure hidden");
        if(mode=="radio_no_watchdog")require(rail["wireless_power_state"]=="inhibited","missing watchdog not reported");
        require(exchange(Link::USB,request("reboot"))["ok"]==true,"USB-only reboot unavailable");
        for(int i=0;i<200&&!restartCalls;++i)loop();
        require(restartCalls==1&&BLEDevice::initCalls==0,"USB-only recovery enabled radio or lost reboot");
        done(mode);return 0;
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
        require(!controlTimingFault&&!strcmp(reason,"TRANSPORT_INIT_FAILED"),"blocking SDK init overwrote TRANSPORT_INIT_FAILED with a control overrun");
        auto rail=exchange(Link::USB,request("rail_status"));
        require(rail["wireless_power_state"]=="init_failed","SDK failure reported ready");
        for(int i=0;i<1500;++i)loop();
        require(BLEDevice::initCalls==1,"failed SDK initialization retried without cleanup");
        require(exchange(Link::USB,request("reboot"))["ok"]==true,"SDK failure lost USB recovery");
        for(int i=0;i<200&&!restartCalls;++i)loop();
        require(restartCalls==1,"SDK failure lost reboot");
        done(mode);return 0;
    }
    if(mode=="radio_init_loss"){
        require(restartCalls==1&&owner.link==Link::None&&!strcmp(reason,"RADIO_POWER_LOST"),"battery loss during blocking SDK init missed");
        done(mode);return 0;
    }
    require(BLEDevice::initCalls==1&&WiFi.beginCalls==1&&!restartCalls,"qualified battery failed to enable both radios exactly once");
    require(!controlTimingFault&&controlOverruns==0&&strcmp(reason,"CONTROL_OVERRUN"),"blocking radio SDK init latched a control overrun");
    if(mode=="radio_start_gpio_fault"){
        // The GPIO fault latched by halt("RADIO_STARTING") is not a supply loss: the radio comes up read-only.
        require(startFaultInjected && !boardSafe.ready(),"fault not injected at RADIO_STARTING");
        for(int i=0;i<300;++i)loop();
        require(!restartCalls && BLEDevice::initCalls==1 && WiFi.beginCalls==1,"GPIO fault at radio start caused a radio-power reboot");
        require(radioRestartCookie[0]==0 && radioRestartCookie[1]==0,"GPIO fault tagged as a radio power reset");
        WiFi.connected=true;loop();
        auto st=exchange(Link::WiFi,request("status"));faultChecks(st);
        require(st["wireless_power_state"]=="online","radio not online after the start-time fault");
        auto claim=exchange(Link::WiFi,request("claim"));
        require(claim["ok"]==false && claim["error"]==faultReason && owner.link==Link::None,"wireless claim accepted in the fault state");
        require(exchange(Link::USB,request("reboot"))["ok"]==true,"USB recovery unavailable");
        for(int i=0;i<200&&!restartCalls;++i)loop();
        require(restartCalls==1,"USB recovery reboot lost");
        done(mode);return 0;
    }
    WiFi.connected=true;loop();BLEDevice::servers.back()->connect(12);
    if(mode=="radio_qualify"){
        for(int i=0;i<2000;++i)loop();
        require(BLEDevice::initCalls==1&&WiFi.beginCalls==1&&!restartCalls,"radios repeatedly initialized");
        statusChecks(exchange(Link::WiFi,request("status")));
        statusChecks(exchange(Link::BLE,request("status")));
        done(mode);return 0;
    }
    if(mode=="radio_gpio_loss"){
        // Runtime GPIO fault after the radio is online: no restart; the radio stays up read-only.
        require(exchange(Link::WiFi,request("claim"))["ok"]==true,"wireless claim failed");
        auto queued=request("heartbeat");udpIncoming.push_back({IPAddress(0xc0000201),5555,signedLine(queued)+'\n'});
        gpio_mock::levelFailurePin=arm;gpio_mock::levelFailures=1;halt("TEST_GPIO_FAULT");
        require(!boardSafe.ready(),"runtime GPIO fault fixture did not latch");
        const std::string config=storedConfig;
        loop();
        require(!restartCalls && owner.link==Link::None,"runtime GPIO fault restarted the radios or kept control");
        require(radioRestartCookie[0]==0 && radioRestartCookie[1]==0,"GPIO fault tagged as a radio power reset");
        require(udpIncoming.empty(),"queued radio command not answered");
        auto hb=lastResponse(Link::WiFi);
        require(hb["id"]==queued["id"] && hb["ok"]==false && hb["error"]==faultReason && hb["state"]=="gpio_fault",
                "queued heartbeat not answered with the fault");
        for(Link link:{Link::WiFi,Link::BLE}){
            auto st=exchange(link,request("status"));faultChecks(st);
            require(st["wireless_power_state"]=="online","fault status hides the online radio");
            auto rail=exchange(link,request("rail_status"));railChecks(rail);
            require(rail["fresh"]==true,"fault rails over radio");
            for(const char* op:{"claim","reboot","provision","heartbeat"}){
                auto r=exchange(link,request(op));
                require(r["ok"]==false && r["error"]==faultReason,"wireless command accepted after a runtime GPIO fault");
            }
            require(exchange(link,request("stop"))["ok"]==true,"stop not answered");
            require(owner.link==Link::None && !rebootPending && storedConfig==config,"wireless side effect in the fault state");
        }
        for(int i=0;i<300;++i)loop();
        require(!restartCalls && BLEDevice::initCalls==1 && WiFi.beginCalls==1,"fault restarted or re-initialized the radios");
        // A supply loss after the fault still restarts (and tags the reset as a radio power loss).
        adc_mock::mv[0]=0;adc_mock::raw[0]=0;fakeNow+=100;loop();
        require(restartCalls==1 && radioRestartCookie[0]==radioResetMagic && radioRestartCookie[1]==~radioResetMagic,
                "battery loss after a runtime GPIO fault kept the radios");
        done(mode);return 0;
    }
    if(mode=="radio_battery_loss"||mode=="radio_adc_loss"||mode=="radio_stale"){
        require(exchange(Link::WiFi,request("claim"))["ok"]==true,"wireless claim failed");
        auto queued=request("heartbeat");udpIncoming.push_back({IPAddress(0xc0000201),5555,signedLine(queued)+'\n'});
        if(mode=="radio_battery_loss"){adc_mock::mv[0]=0;adc_mock::raw[0]=0;fakeNow+=100;}
        if(mode=="radio_adc_loss"){adc_mock::readError=ESP_ERR_TIMEOUT;fakeNow+=100;}
        if(mode=="radio_stale")fakeNow+=251;
        const auto reads=adc_mock::readCalls;loop();
        require(restartCalls==1&&owner.link==Link::None,"power loss retained radio control");
        require(radioRestartCookie[0]==radioResetMagic&&radioRestartCookie[1]==~radioResetMagic,"automatic reset reason not retained");
        require(udpIncoming.size()==1,"queued radio command processed after power loss");
        if(mode=="radio_stale")require(adc_mock::readCalls==reads,"fresh sample concealed expired evidence");
        done(mode);return 0;
    }
    if(mode=="rails"){
        fakeNow+=10;loop();
        for(Link link:{Link::USB,Link::WiFi,Link::BLE}){
            auto report=exchange(link,request("rail_status"));railChecks(report);
            require(report["fresh"]==true,"rail response not fresh");
            require(report["battery_voltage_mv"]==7800 && report["adc_pin_mv"][0]==600 && report["adc_raw"][0]==2000,"VBAT mapping/scaling mismatch");
            require(report["motor_current_ma"]==230 && report["adc_pin_mv"][1]==460 && report["adc_raw"][1]==1500 &&
                    report["motor_current_saturated"]==false,"motor current mapping/scaling mismatch");
            require(report["usb_power"]==true && report["charge"]=="external_power","USB power (IO8 high) not reported");
            if(link==Link::USB)std::cout<<"KISS_RAILS "<<json(report)<<'\n';
        }
        levels[kiss_c6::usbPowerSense]=LOW;
        auto battery=exchange(Link::WiFi,request("rail_status"));railChecks(battery);
        require(battery["usb_power"]==false && battery["charge"]=="battery","battery-only power (IO8 low) not reported");
        levels[kiss_c6::usbPowerSense]=HIGH;
        adc_mock::mv[1]=3200;fakeNow+=100;loop();     // INA180 at its rail: 1.6 A or more
        auto rail=exchange(Link::USB,request("rail_status"));railChecks(rail);
        require(rail["motor_current_ma"]==1600 && rail["motor_current_saturated"]==true,"INA180 rail not reported");
        adc_mock::mv[1]=460;
        adc_mock::readError=ESP_ERR_TIMEOUT;fakeNow+=100;loop();
        require(!voltageMonitor.reading().valid&&voltageMonitor.error()==ESP_ERR_TIMEOUT,"ADC error hidden");
        require(restartCalls==1&&owner.link==Link::None,"ADC failure after RF startup did not reset to source qualification");
        done(mode);return 0;
    }
    if(mode=="rails_budget"){
        // ~800 bytes in practice; the margin covers counters this fixture leaves short (sequence, error codes).
        adc_mock::raw[0]=4094;adc_mock::raw[1]=4094;adc_mock::mv[0]=640;adc_mock::mv[1]=3350;fakeNow+=100;loop();
        fakeIpText="255.255.255.255";Serial.tx.clear();
        Serial.connected=true;   // a host has the port open and reads (no request sent in this fixture)
        reply(Link::USB,0,UINT32_MAX,nullptr,false,false,true);
        auto r=lastResponse(Link::USB);railChecks(r);
        require(r["fresh"]==true && r["motor_current_saturated"]==true && digits(r["uptime_ms"].as<uint32_t>())==10,"budget fixture not applied");
        const size_t headroom=2*(10-1)+2*6+(strlen("adc_calibration")-strlen("ready"))+(strlen("waiting_voltage")-strlen("online"));
        std::cout<<"rails payload="<<lastPayloadBytes<<" (+"<<headroom<<" headroom) frame="<<lastFrameBytes<<'\n';
        require(lastPayloadBytes+headroom<replyPayloadBytes,"rail report exceeds the firmware reply buffer");
        require(lastFrameBytes<1472,"rail report exceeds a single UDP datagram");
        done(mode);return 0;
    }
    if(mode=="status_budget"){
        // True worst case of the normal status: area plan (state "ready", 200 waypoints), longest reason,
        // longest IP/RSSI, two-digit reset reason, maximal counters and lease, extreme encoder counts.
        require(exchange(Link::USB,request("claim"))["ok"]==true,"claim failed");
        auto area=request("area");area["x"]=0;area["y"]=0;area["w"]=5;area["h"]=5;area["spacing"]=0.005;
        auto planned=exchange(Link::USB,area);
        require(planned["ok"]==true && planned["state"]=="ready" && planned["wp_total"]==MAX_WAYPOINTS,"area fixture did not plan");
        fakeIpText="255.255.255.255";fakeRssi=-100;fakeResetReason=15;transport.wifiUp=true;
        fakeNow=UINT32_MAX;
        controlGapMs=maxControlGapMs=controlOverruns=transport.dropped=transport.reconnects=UINT32_MAX;
        transport.notifyFailures=UINT32_MAX;reason="BOARD_POWER_UNREVIEWED";   // longest KISS reason (22)
        for(auto& count:enc_counts_)count=0x80000000U;
        owner.clear();
        require(owner.claim(Link::USB,0,UINT32_MAX,UINT32_MAX,millis()),"budget fixture claim failed");
        Serial.tx.clear();reply(Link::USB,0,UINT32_MAX,nullptr,true);
        auto d=lastResponse(Link::USB);statusChecks(d);
        require(d["wifi_ip"]=="255.255.255.255" && d["wifi_rssi_dbm"]==-100 && d["reason"]=="BOARD_POWER_UNREVIEWED","budget fixture not applied");
        // Unreached here: two-digit motor fault codes on all three wheels.
        size_t headroom=0;for(int i=0;i<3;++i)headroom+=2-digits(d["motor_pwm_fault"][i].as<unsigned>());
        std::cout<<"status payload="<<lastPayloadBytes<<" (+"<<headroom<<" headroom) frame="<<lastFrameBytes<<'\n';
        require(lastPayloadBytes+headroom<replyPayloadBytes,"status exceeds the firmware reply buffer");
        require(lastFrameBytes<1472,"status exceeds a single UDP datagram");
        done(mode);return 0;
    }
    if(mode.rfind("reboot_",0)==0){
        Link link=mode=="reboot_usb"?Link::USB:mode=="reboot_wifi"?Link::WiFi:Link::BLE;
        require(exchange(link,request("claim"))["ok"]==true,"claim failed");
        require(exchange(link,request("reboot"))["ok"]==true,"reboot rejected");
        for(int i=0;i<2600 && !restartCalls;++i)loop();
        require(restartCalls==1,"reboot lost");done(mode);return 0;
    }
    auto d=exchange(Link::USB,request("status"));statusChecks(d);
    if(motorFailure){
        const int pin=std::stoi(mode.substr(mode.rfind('_')+1));
        int n=0;for(int i=0;i<3;++i)n+=d["motor_pwm_fault"][i].as<int>()!=0;
        require(n==1,"motor fault assigned to wrong number of wheels");
        require(d["motor_pwm_fault"][wheelOfPin(pin)].as<int>()!=0,"motor fault reported on another header's wheel");
        done(mode);return 0;
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
    // HWCDC: bool(Serial) drops on an unplug/bus reset. That revokes a USB lease at once (no heartbeat wait),
    // and a request still queued from before the unplug is answered into nothing: dropped, not written.
    // (The robot's plain "Stopped." println bypasses ControlTransport and is not a protocol frame.)
    require(exchange(Link::USB,request("claim"))["ok"]==true,"USB CDC claim failed");
    {
        const auto before=Serial.tx.size();const auto dropped=transport.dropped;
        Serial.feed(json(request("status"))+"\n");Serial.connected=false;
        while(Serial.available())loop();
        require(owner.link==Link::None&&!strcmp(reason,"LINK_LOST")&&!restartCalls,"CDC host loss did not revoke the USB lease");
        require(Serial.tx.find("\"payload\"",before)==std::string::npos && !Serial.hostAbsentWrites &&
                transport.dropped==dropped+1,"reply written while no USB host was attached");
    }
    Serial.connected=true;
    statusChecks(exchange(Link::USB,request("status")));
    JsonDocument observed;
    auto outputs=observed["outputs"].to<JsonArray>();auto inputs=observed["encoders"].to<JsonArray>();
    for(int pin=0;pin<64;++pin){if(modes[pin]==OUTPUT)outputs.add(pin);if(interruptHandlers[pin])inputs.add(pin);}
    auto groups=observed["motor_config_groups"].to<JsonArray>();
    for(const auto& e:gpio_mock::events)if(std::string(e.operation)=="output_enable" && e.pin!=arm)groups.add(e.pin);
    auto edges=observed["edge_responses"].to<JsonArray>();
    for(int pin=0;pin<64;++pin)if(interruptHandlers[pin]){
        volatile long before[3],after[3];encoders_read(before);levels[pin]^=1;interruptHandlers[pin]();encoders_read(after);
        auto edge=edges.add<JsonArray>();edge.add(pin);for(int i=0;i<3;++i)edge.add(after[i]-before[i]);
    }
    observed["console"]=Serial.cdcMode?"usb_cdc":"uart";
    auto armLevels=observed["arm_output"].to<JsonArray>();
    armLevels.add(arm);armLevels.add(gpio_mock::outputEnabled[arm]);armLevels.add(levels[arm]);
    observed["arm"]=arm;
    auto adc=observed["adc_gpio"].to<JsonArray>();for(int pin:adc_mock::mapped)adc.add(pin);
    auto inputsOnly=observed["plain_inputs"].to<JsonArray>();
    for(int pin=0;pin<64;++pin)if(modes[pin]==INPUT && !interruptHandlers[pin] && !gpio_mock::outputEnabled[pin])inputsOnly.add(pin);
    auto untouched=observed["untouched"].to<JsonArray>();
    for(int pin=0;pin<31;++pin)if(modes[pin]==-99 && !interruptHandlers[pin])untouched.add(pin);
    TriMotorDriver motors;motors.begin();
    for(int i=0;i<3;++i)require(!motors.motor(i).setSpeed(255)&&!motors.motor(i).setSpeed(-255)&&motors.motor(i).stop(),"review driver allowed modulation");
    // Pairs the driver must refuse: USB D-/D+, sensor head, the straps, ARM + M1_FI, swapped FI/BI and
    // cross-header pairs. A refused pair may only force its genuine FI/BI legs LOW; it never addresses
    // any other pin (USB, I2C, IR, ARM, straps).
    for(const auto& pair:std::vector<std::pair<int,int>>{{12,13},{15,16},{16,17},{8,9},{0,1},{23,22},{7,14},{6,1},{22,7},{1,23}}){
        const auto n=gpio_mock::events.size();const auto ops=pinOperations.size();
        MotorDriver invalid;invalid.begin(pair.first,pair.second,0,1,20000,8);
        require(invalid.fault()!=0,"invalid FI/BI pair accepted");
        for(auto i=n;i<gpio_mock::events.size();++i){
            const auto& e=gpio_mock::events[i];
            require(kiss_c6::isMotorPin(e.pin) && (e.pin==pair.first || e.pin==pair.second),"invalid pair touched a non-motor GPIO");
            require(std::string(e.operation)!="output_enable" && (std::string(e.operation)!="idf_set_level" || e.level==LOW),
                    "invalid pair enabled or raised a leg");
        }
        for(auto i=ops;i<pinOperations.size();++i)require(kiss_c6::isMotorPin(pinOperations[i].pin) && pinOperations[i].value==LOW,
                                                         "invalid pair wrote a non-motor GPIO");
        if(!kiss_c6::isMotorPin(pair.first) && !kiss_c6::isMotorPin(pair.second))
            require(gpio_mock::events.size()==n && pinOperations.size()==ops,"invalid non-motor pair touched a GPIO");
    }
    checkSafety();
    JsonDocument profile;
    profile["profile"]=kiss_c6::profile;profile["arm"]=arm;profile["battery_sense"]=kiss_c6::batterySense;
    profile["motor_sense"]=kiss_c6::motorSense;profile["usb_power_sense"]=kiss_c6::usbPowerSense;
    profile["i2c_sda"]=kiss_c6::i2cSda;profile["i2c_scl"]=kiss_c6::i2cScl;profile["ir_enable_n"]=kiss_c6::irEnableN;
    profile["fault_reason"]=safeDefaultsFailure;
    auto headers=profile["headers"].to<JsonArray>();
    for(const auto& h:kiss_c6::headers){
        auto o=headers.add<JsonObject>();o["header"]=h.header;o["fi"]=h.fi;o["bi"]=h.bi;
        o["enc_a"]=h.encA;o["enc_b"]=h.encB;o["physical"]=h.physical;
    }
    auto order=profile["header_order"].to<JsonArray>();for(const char* h:headerOrder)order.add(h);
    std::cout<<"KISS_PROFILE "<<json(profile)<<'\n';
    std::cout<<"KISS_OBSERVED "<<json(observed)<<'\n';
    std::cout<<"KISS_STATUS "<<json(d)<<"\nstartup/commands/encoders PASS\n";
}
