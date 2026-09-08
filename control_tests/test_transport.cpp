#define WBB_TOP_BOARD_V2 1
#define WBB_CONTROL_TRANSPORT_NATIVE_TEST 1
#include <cassert>
#include <iostream>
#include "../src/control_transport.cpp"
struct Input {control::Link link;uint64_t peer;std::string data;};
static std::vector<Input> input;
static void receive(control::Link l,uint64_t p,const uint8_t* bytes,size_t n){
    input.push_back({l,p,std::string(reinterpret_cast<const char*>(bytes),n)});
}
static void fresh(){
    fakeNow=0;connectedHandle=0xffff;notifyFails=false;subscribed=true;
    notifications.clear();input.clear();udpIncoming.clear();udpOutgoing.clear();WiFi=FakeWiFi{};
    Serial.connected=true;Serial.rx.clear();Serial.tx.clear();
}
static void pump(ControlTransport& t,unsigned ms){for(unsigned i=0;i<ms;++i){++fakeNow;t.poll(receive);}}
int main(){
    {
        fresh();ControlTransport t;assert(t.begin("test","",""));
        BLEDevice::servers.back()->connect(1);auto peer=t.blePeer.load();
        std::string text(1499,'x');text+='\n';
        for(int i=0;i<4;++i)assert(t.send(control::Link::BLE,peer,text.c_str()));
        assert(!t.send(control::Link::BLE,peer,text.c_str())&&t.dropped==1);
        pump(t,1600);std::string joined;
        for(const auto& n:notifications){assert(n.handle==1&&n.data.size()<=20);joined+=n.data;}
        assert(joined==text+text+text+text);
        std::cout<<"Bounded TX queue and complete 20-byte fragmentation PASS\n";
    }
    {
        fresh();ControlTransport t;assert(t.begin("test","",""));auto* server=BLEDevice::servers.back().get();
        server->connect(1);auto old=t.blePeer.load();auto* rx=server->service.characteristics[1].get();
        assert(t.send(control::Link::BLE,old,"old-old-old-old-old-old-old-old-old\n"));pump(t,5);
        rx->write(1,"stale");server->disconnect(1);server->connect(2);auto current=t.blePeer.load();
        assert(old!=current);rx->write(1,"foreign");rx->write(2,"current\n");
        assert(t.send(control::Link::BLE,current,"new\n"));pump(t,30);
        assert(input.size()==1&&input[0].data=="current\n"&&input[0].peer==current);
        std::string data;for(const auto& n:notifications)if(n.handle==2)data+=n.data;
        assert(data=="new\n");server->connect(3);
        assert(t.blePeer==current&&t.bleHandle==2&&t.bleUp);
        std::cout<<"Old peer RX/TX discarded and second connection rejected PASS\n";
    }
    {
        fresh();ControlTransport t;assert(t.begin("test","",""));auto* server=BLEDevice::servers.back().get();
        server->connect(1);auto* rx=server->service.characteristics[1].get();
        rx->write(1,std::string(129,'x'));assert(t.takeOverflow());
        for(int i=0;i<17;++i)rx->write(1,"hello\n");
        assert(t.takeOverflow()&&!t.takeOverflow());pump(t,1);assert(input.size()==4);
        pump(t,3);assert(input.size()==16);
        std::cout<<"RX size/capacity limits and four-packet poll budget PASS\n";
    }
    {
        fresh();ControlTransport t;assert(t.begin("test","lab","password"));WiFi.connected=true;
        udpIncoming.push_back({IPAddress(1),11,"hello\n"});udpIncoming.push_back({IPAddress(2),22,"unfinished"});
        udpIncoming.push_back({IPAddress(3),33,std::string(ControlTransport::maxLine,'x')});
        pump(t,1);assert(t.wifiUp&&input.size()==3&&udpIncoming.size()==1);
        assert(input[0].peer==((uint64_t(1)<<16)|11)&&input[2].data==std::string(1,'\0'));
        pump(t,1);assert(input.size()==4&&input.back().data==std::string(1,'\0'));
        WiFi.connected=false;pump(t,1);assert(!t.wifiUp);
        fakeNow=15000;t.poll(receive);assert(t.reconnects==1&&WiFi.reconnectCalls==1);
        std::cout<<"UDP boundaries, size checks, budget and reconnect timer PASS\n";
    }
    for(int failure:{0,1,2}){
        fresh();ControlTransport t;assert(t.begin("test","",""));auto* server=BLEDevice::servers.back().get();
        server->connect(1);assert(t.send(control::Link::BLE,t.blePeer,"reply-reply-reply-reply-reply\n"));
        notifyFails=failure==0;subscribed=failure!=2;pump(t,5);
        if(failure==1)server->service.characteristics[0]->asynchronousFailure();
        pump(t,5);
        std::cout<<"notify_failure mode="<<failure<<" dropped="<<t.dropped<<" connected="<<t.bleUp<<std::endl;
        assert(t.dropped>0&&t.notifyFailures>0&&!t.bleUp);
        server->connect(2);notifyFails=false;subscribed=true;notifications.clear();
        assert(t.send(control::Link::BLE,t.blePeer,"fresh\n"));pump(t,50);
        std::string data;for(const auto& n:notifications)data+=n.data;assert(data=="fresh\n");
    }
    std::cout<<"Synchronous/asynchronous/unsubscribed TX failure discards partial stream PASS\n";
}
