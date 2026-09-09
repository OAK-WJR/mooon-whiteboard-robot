#define WBB_TOP_BOARD_V2 1
#define WBB_CONTROL_NATIVE_TEST 1
#include <cassert>
#include <iostream>
#include <random>
#include "../src/top_v2_main.cpp"
using control::Link;
static unsigned cases=0;
static void passed(const char* name){++cases;std::cout<<name<<" PASS\n";}
static void fresh(){
    owner.clear();usbFrame.reset();bleFrame.reset();framedBLEPeer=0;
    fakeNow=0;Serial.connected=true;restartCalls=0;watchdogResult=ESP_OK;
    transport.sent.clear();transport.incoming.clear();transport.wifiUp=true;transport.bleUp=true;
    transport.overflow=false;transport.beginResult=true;transport.dropped=0;
    rebootPending=false;keySet=false;controlTimingFault=false;controlOverruns=0;hmacDelayMs=0;
    maxControlGapMs=controlGapMs=0;nvsOpen=nvsWrite=true;storedConfig="{}";
    robot.pwmReady=true;setup();transport.sent.clear();
}
static std::string json(const JsonDocument& d){std::string s;serializeJson(d,s);return s;}
static JsonDocument request(const char* cmd,uint32_t id=1){
    JsonDocument d;d["id"]=id;d["cmd"]=cmd;d["boot"]=boot;d["lease"]=owner.lease;return d;
}
static std::string signedLine(const std::string& payload){
    uint8_t digest[32];assert(mac(payload.c_str(),digest));char hex[65];
    for(int i=0;i<32;++i)snprintf(hex+2*i,3,"%02x",digest[i]);
    JsonDocument d;d["payload"]=payload;d["mac"]=hex;return json(d);
}
static JsonDocument response(){
    assert(!transport.sent.empty());
    JsonDocument outer,inner;
    assert(!deserializeJson(outer,transport.sent.back().text));
    assert(!deserializeJson(inner,outer["payload"].as<const char*>()));return inner;
}
static void send(Link l,uint64_t p,const JsonDocument& d){
    transport.sent.clear();std::string line=json(d);
    if(l!=Link::USB)line=signedLine(line);
    command(l,p,line.c_str());
}
static void auth(){for(int i=0;i<32;++i)key[i]=uint8_t(i);keySet=true;}
int main(){
    fresh();command(Link::USB,0,"hello");
    auto d=response();assert(d["ok"]&&d["protocol"]==2&&!d["motion_enabled"]);
    assert(d["motor_voltage_mv"].isNull()&&!d["power_disconnect"]);
    assert(transport.sent.back().text.size()<1472);passed("discovery and status frame budget");
#if defined(WBB_TOP_BOARD_KISS_C6)
    assert(Serial.uartMode&&Serial.rxPin==17&&Serial.txPin==16);
    assert(d["profile"]=="top_board_v2_kiss_c6_review"&&d["charge_enabled"]==false);
#elif defined(WBB_TOP_BOARD_R3)
    assert(Serial.uartMode&&Serial.baud==115200&&Serial.rxPin==17&&Serial.txPin==16);
    assert(!strcmp(d["profile"],"top_board_v2_r3_review")&&d["charge_enabled"]==false);
    std::cout<<"R3_STATUS "<<json(d)<<'\n';
#else
    assert(!Serial.uartMode&&!strcmp(d["profile"],"top_board_v2"));
#endif

    fresh();send(Link::USB,0,request("claim"));assert(response()["ok"]&&owner.lease);
    robot.moving=true;fakeNow=750;loop();assert(!robot.moving&&owner.link==Link::None);
    assert(!strcmp(reason,"CONTROL_OVERRUN")||!strcmp(reason,"HEARTBEAT_TIMEOUT"));passed("owner expires without a request");

    fresh();auth();send(Link::WiFi,99,request("claim"));assert(response()["ok"]);
    auto lease=owner.lease;send(Link::BLE,1,request("claim",2));assert(!response()["ok"]&&owner.lease==lease);
    send(Link::WiFi,98,request("heartbeat",3));assert(!response()["ok"]&&owner.peer==99);
    send(Link::WiFi,99,request("heartbeat",4));assert(response()["ok"]);
    robot.moving=true;send(Link::WiFi,99,request("heartbeat",4));
    assert(!robot.moving&&owner.link==Link::None&&!response()["ok"]);passed("owner arbitration and replay stop");

    // The deadline can pass during real parsing/authentication work after the
    // handler's entry check. Clearing the lease must also stop the controller.
    for(Link link:{Link::WiFi,Link::BLE})for(uint32_t start:{0U,0xfffffff0U}){
        fresh();auth();fakeNow=start;send(link,9,request("claim"));
        const auto heartbeat=signedLine(json(request("heartbeat",2)));
        robot.moving=true;fakeNow=start+control::Session::timeoutMs-1;
        const auto previousStops=robot.stops;hmacDelayMs=1;
        command(link,9,heartbeat.c_str());
        assert(!response()["ok"]&&!strcmp(response()["error"],"LEASE_EXPIRED"));
        assert(owner.link==Link::None);
        assert(!robot.moving&&robot.stops>previousStops);
        assert(!strcmp(reason,"HEARTBEAT_TIMEOUT"));
    }passed("lease deadline crossed during authentication stops before reply, including clock wrap");

    fresh();auth();send(Link::WiFi,11,request("claim"));
    auto valid=signedLine(json(request("reboot",2)));valid.back()^=1;
    size_t count=transport.sent.size();command(Link::WiFi,22,valid.c_str());
    assert(transport.sent.size()==count&&!rebootPending&&owner.peer==11);
    auto envelope=JsonDocument();deserializeJson(envelope,signedLine(json(request("reboot",2))));
    envelope["mac"]=std::string(64,'0');command(Link::WiFi,11,json(envelope).c_str());
    assert(!rebootPending);passed("invalid signatures cannot reboot");

    fresh();auth();auto old=request("claim");old["boot"]="0000000000000000";
    send(Link::WiFi,1,old);assert(!response()["ok"]&&owner.link==Link::None);passed("old boot rejected");

    for(const char* op:{"start","goto","wheel","arm","pulse"}){
        for(int voltage:{6000,12000})for(Link link:{Link::USB,Link::WiFi,Link::BLE}){
            fresh();auth();send(link,1,request("claim"));
            auto motion=request(op,2);motion["motor_voltage_mv"]=voltage;
            send(link,1,motion);
            assert(!response()["ok"]&&!strcmp(response()["error"],"BOARD_POWER_UNREVIEWED"));
            assert(!robot.moving&&owner.link==Link::None&&levels[
#ifdef WBB_TOP_BOARD_KISS_C6
            10
#else
            21
#endif
            ]==LOW);
        }
    }passed("all five motion operations stay locked");

    fresh();send(Link::USB,0,request("claim"));robot.moving=true;
    std::string bytes(ControlTransport::maxLine+10,'9');
    receive(Link::USB,0,reinterpret_cast<const uint8_t*>(bytes.data()),bytes.size());
    assert(!robot.moving&&owner.link==Link::None&&!strcmp(reason,"RX_OVERFLOW"));
    auto reboot=json(request("reboot",2))+"\n";
    receive(Link::USB,0,reinterpret_cast<const uint8_t*>(reboot.data()),reboot.size());
    assert(!rebootPending);passed("overflow stops before newline and discards its remainder");

    fresh();auth();std::string partial=signedLine(json(request("reboot")));
    receive(Link::BLE,1,reinterpret_cast<const uint8_t*>(partial.data()),partial.size()/2);
    std::string ending=partial.substr(partial.size()/2)+"\n";
    receive(Link::BLE,2,reinterpret_cast<const uint8_t*>(ending.data()),ending.size());
    assert(!rebootPending);passed("BLE peers cannot combine fragments");

    fresh();send(Link::USB,0,request("claim"));robot.moving=true;Serial.connected=false;loop();
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_KISS_C6)
    assert(robot.moving&&owner.link==Link::USB); // No CP2102N cable-present feedback.
    fakeNow+=control::Session::timeoutMs;loop();
    assert(!robot.moving&&owner.link==Link::None);passed("UART cable loss expires through heartbeat");
#else
    assert(!robot.moving&&owner.link==Link::None);passed("USB disconnect stops owner");
#endif
    fresh();auth();send(Link::BLE,4,request("claim"));robot.moving=true;transport.bleUp=false;loop();
    assert(!robot.moving&&owner.link==Link::None);passed("BLE disconnect stops owner");
    fresh();auth();send(Link::WiFi,4,request("claim"));robot.moving=true;transport.wifiUp=false;loop();
    assert(!robot.moving&&owner.link==Link::None);passed("Wi-Fi disconnect stops owner");

    fresh();send(Link::USB,0,request("claim"));robot.moving=true;
    send(Link::USB,0,request("reboot",2));assert(response()["ok"]&&!robot.moving&&rebootPending);
    send(Link::USB,0,request("claim",3));assert(!response()["ok"]&&owner.link==Link::None);
    fakeNow=99;loop();assert(restartCalls==0);loop();assert(restartCalls==1);passed("reboot stops and rejects rearming");

    fresh();send(Link::USB,0,request("claim"));robot.moving=true;
    fakeNow=101;loop();assert(!robot.moving&&controlTimingFault&&controlOverruns==1);
    assert(std::abs(robot.lastDt-.101f)<.00001f);passed("control deadline miss is latched and stops");
    fresh();lastControlAt=0xfffffff0;fakeNow=0x18;loop();
    assert(std::abs(robot.lastDt-.040f)<.00001f&&!controlTimingFault);passed("control clock wrap");

    fresh();auth();auto config=request("provision");config["ssid"]="lab";config["password"]="password";config["key"]=std::string(64,'a');
    send(Link::WiFi,1,config);assert(!response()["ok"]&&storedConfig=="{}");
    nvsWrite=false;send(Link::USB,0,config);assert(!response()["ok"]&&storedConfig=="{}");
    nvsWrite=true;send(Link::USB,0,config);assert(response()["ok"]&&storedConfig!="{}");passed("provisioning USB-only and write failure");

    fresh();auto bad=request("reboot");std::string hidden("reboot\0ignored",14);bad["cmd"]=hidden;
    command(Link::USB,0,json(bad).c_str());assert(!rebootPending);passed("embedded NUL command rejected");

    for(const char* suffix:{"garbage","x","{}"," true","\t0","\v"}){
        fresh();std::string trailing=json(request("reboot"))+suffix;
        command(Link::USB,0,trailing.c_str());assert(!rebootPending);
        fresh();auth();send(Link::WiFi,1,request("claim"));
        trailing=signedLine(json(request("reboot",2))+suffix);
        command(Link::WiFi,1,trailing.c_str());assert(!rebootPending);
        fresh();auth();send(Link::BLE,1,request("claim"));
        trailing=signedLine(json(request("reboot",2)))+suffix;
        command(Link::BLE,1,trailing.c_str());assert(!rebootPending);
    }passed("trailing data rejected in USB, signed payload and envelope");
    fresh();std::string padded=" \t"+json(request("reboot"))+"\r\n \t";
    command(Link::USB,0,padded.c_str());assert(rebootPending);passed("complete object with whitespace accepted");
    for(const char* value:{"[]","true","123","null","\"reboot\""}){
        fresh();command(Link::USB,0,value);assert(!rebootPending&&transport.sent.empty());
    }passed("non-object roots rejected");

    fresh();std::mt19937 generator(7823);
    for(int i=0;i<10000;++i){
        usbFrame.reset();std::string input;int n=generator()%1600;
        for(int j=0;j<n;++j)input.push_back(char(generator()%256));input+='\n';
        receive(Link::USB,0,reinterpret_cast<const uint8_t*>(input.data()),input.size());
        assert(!rebootPending&&!robot.moving&&owner.link==Link::None);
    }passed("10000 deterministic malformed frames");
    std::cout<<cases<<" production command/loop scenarios PASS\n";
}
