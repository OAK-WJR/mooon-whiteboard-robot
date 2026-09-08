#define WBB_TOP_BOARD_V2 1
#define WBB_CONTROL_NATIVE_TEST 1
#define WBB_CONTROL_TRANSPORT_NATIVE_TEST 1
#include <cassert>
#include <iostream>
#include "../src/top_v2_main.cpp"
#include "../src/control_transport.cpp"
int main(int argc,char** argv){
    const std::string mode=argc>1?argv[1]:"backlog";
    if(mode=="wrap")fakeNow=0xffffff00;
    setup();for(int i=0;i<32;++i)key[i]=i;keySet=true;
    BLEDevice::servers.back()->connect(12);
    uint32_t peer=transport.blePeer;
    assert(owner.claim(control::Link::BLE,peer,77,1,millis()));
    const auto started=millis();
    if(mode=="owner-failure"){
        robot.moving=true;notifyFails=true;command(control::Link::BLE,peer,"hello");
        for(int i=0;i<20;++i)loop();
        assert(!robot.moving&&owner.link==control::Link::None&&!restartCalls&&transport.notifyFailures>0);
        std::cout<<"Actual notification failure reaches runtime owner stop PASS\n";return 0;
    }
    // Full-size status responses ahead of a reboot ACK must not be cut off.
    for(int i=0;i<(mode=="full"?4:3);++i)command(control::Link::BLE,peer,"hello");
    JsonDocument d;d["id"]=2;d["cmd"]="reboot";d["boot"]=boot;d["lease"]=owner.lease;
    std::string payload;serializeJson(d,payload);
    uint8_t digest[32];assert(mac(payload.c_str(),digest));char signature[65];
    for(int i=0;i<32;++i)snprintf(signature+2*i,3,"%02x",digest[i]);
    JsonDocument envelope;envelope["payload"]=payload;envelope["mac"]=signature;
    std::string wire;serializeJson(envelope,wire);wire+='\n';
    auto* rx=BLEDevice::servers.back()->service.characteristics.at(1).get();
    for(size_t i=0;i<wire.size();i+=20){
        // One status has begun sending while RX fragments arrive. Fill that
        // freed queue slot immediately before the final command fragment.
        if(mode=="full"&&i+20>=wire.size())command(control::Link::BLE,peer,"hello");
        rx->write(12,wire.substr(i,20));loop();
    }
    while(!restartCalls&&uint32_t(millis()-started)<3000){
        if(mode=="flood"&&millis()%10==0)command(control::Link::BLE,peer,"hello");
        if(mode=="tx-failure"&&uint32_t(millis()-started)>=20)notifyFails=true;
        loop();
    }
    assert(restartCalls==1);
    std::string output;for(const auto& n:notifications)output+=n.data;
    unsigned complete=0;bool ack=false;size_t at=0;
    for(size_t end;(end=output.find('\n',at))!=std::string::npos;at=end+1){
        JsonDocument outer,inner;assert(!deserializeJson(outer,output.substr(at,end-at)));
        assert(!deserializeJson(inner,outer["payload"].as<const char*>()));
        ++complete;if(inner["id"]==2&&inner["ok"])ack=true;
    }
    std::cout<<"mode="<<mode<<" restart_elapsed_ms="<<uint32_t(millis()-started)<<" complete_frames="<<complete<<" ack="<<ack<<std::endl;
    if(mode=="full")assert(!ack&&complete==5&&transport.dropped>0&&at==output.size());
    else if(mode=="flood")assert(ack&&uint32_t(millis()-rebootAt)==2500);
    else if(mode=="tx-failure")assert(!ack&&transport.notifyFailures>0&&!transport.bleUp);
    else assert(ack&&complete==4&&at==output.size());
    assert(uint32_t(millis()-rebootAt)<=2500);
    std::cout<<"Actual transport queue + runtime reboot PASS\n";
}
