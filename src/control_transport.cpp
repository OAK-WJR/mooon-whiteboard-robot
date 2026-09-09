#ifdef WBB_TOP_BOARD_V2
#include "control_transport.h"
#if !CONFIG_ESP_COEX_SW_COEXIST_ENABLE
#error "Wi-Fi/BLE coexistence must be enabled in the ESP-IDF build"
#endif
namespace {
class ServerCallbacks:public BLEServerCallbacks {
 ControlTransport& t;
public:
 explicit ServerCallbacks(ControlTransport& t):t(t){}
 void onConnect(BLEServer* server,ble_gap_conn_desc* desc)override{
  if(t.bleHandle.load()!=0xffff){server->disconnect(desc->conn_handle);return;}
  t.bleHandle=desc->conn_handle;t.blePeer.fetch_add(1);t.bleUp=true;
 }
 void onDisconnect(BLEServer*,ble_gap_conn_desc* desc)override{
  if(desc->conn_handle!=t.bleHandle.load())return;
  t.bleUp=false;t.bleHandle=0xffff;t.advertiseAgain=true;
 }
};
class RXCallbacks:public BLECharacteristicCallbacks {
 ControlTransport& t;
public:
 explicit RXCallbacks(ControlTransport& t):t(t){}
 void onWrite(BLECharacteristic* c,ble_gap_conn_desc* desc)override{
  if(!t.bleUp || desc->conn_handle!=t.bleHandle.load())return;
  String value=c->getValue();ControlTransport::RX packet{};
  if(value.length()>sizeof(packet.data)){t.overflow=true;return;}
  packet.peer=t.blePeer.load();packet.size=value.length();
  memcpy(packet.data,value.c_str(),packet.size);
  if(xQueueSend(t.rxQueue,&packet,0)!=pdTRUE)t.overflow=true;
 }
};
class TXCallbacks:public BLECharacteristicCallbacks {
 ControlTransport& t;
public:
 explicit TXCallbacks(ControlTransport& t):t(t){}
 void onStatus(BLECharacteristic*,Status status,uint32_t)override{
  // SUCCESS_NOTIFY also occurs when the SDK accepts a notification. It is not
  // proof that the application received it; the host checks the signed reply.
  if(status==SUCCESS_NOTIFY||status==SUCCESS_INDICATE)return;
  t.notifyFailures.fetch_add(1);t.notifyFailed=true;
 }
};
}
bool ControlTransport::begin(const char* name,const char* ssid,const char* password){
 rxQueue=xQueueCreate(16,sizeof(RX));txQueue=xQueueCreate(4,sizeof(TX));
 if(!rxQueue||!txQueue)return false;
 if(!BLEDevice::init(name)||BLEDevice::setMTU(185)!=0)return false;
 server=BLEDevice::createServer();if(!server)return false;
 server->setCallbacks(new ServerCallbacks(*this));
 auto* s=server->createService(service);if(!s)return false;
 tx=s->createCharacteristic(txUUID,BLECharacteristic::PROPERTY_NOTIFY);
 auto* rx=s->createCharacteristic(rxUUID,BLECharacteristic::PROPERTY_WRITE);
 if(!tx||!rx)return false;
 tx->setCallbacks(new TXCallbacks(*this));
 rx->setCallbacks(new RXCallbacks(*this));if(!s->start())return false;
 auto* a=server->getAdvertising();if(!a)return false;
 a->addServiceUUID(service);if(!a->start())return false;
 bleReady=true;
 configured=ssid&&*ssid;
 if(configured){
  if(!WiFi.mode(WIFI_STA))return false;
  WiFi.setAutoReconnect(true);WiFi.begin(ssid,password);
  retryAt=millis();
 }
 return true;
}
bool ControlTransport::alive(control::Link l,uint64_t peer)const{
 switch(l){
 case control::Link::USB:return bool(Serial);
 case control::Link::WiFi:return wifiUp&&WiFi.status()==WL_CONNECTED;
 case control::Link::BLE:return bleUp.load()&&peer==blePeer.load();
 default:return false;
 }
}
bool ControlTransport::txPending(control::Link l,uint64_t peer)const{
 if(l!=control::Link::BLE||!alive(l,peer))return false;
 return sent<sending.size||(txQueue&&uxQueueMessagesWaiting(txQueue)>0);
}
void ControlTransport::handleNotifyFailure(){
 if(!notifyFailed.exchange(false))return;
 ++dropped;
 if(txQueue)xQueueReset(txQueue);
 sent=sending.size;
 // A missing fragment corrupts a newline-framed response. End this connection
 // so the peer discards the partial frame before a new control session begins.
 bleUp=false;
 if(server&&bleHandle.load()!=0xffff)server->disconnect(bleHandle.load());
}
void ControlTransport::poll(Receiver receive){
 handleNotifyFailure();
 uint8_t bytes[maxLine];size_t n=0;
 while(n<64 && Serial.available())bytes[n++]=Serial.read();
 if(n)receive(control::Link::USB,0,bytes,n);
 const bool connected=configured&&WiFi.status()==WL_CONNECTED;
 if(connected&&!wifiUp)wifiUp=udp.begin(udpPort)==1;
 if(!connected&&wifiUp){udp.stop();wifiUp=false;}
 if(configured&&!connected&&uint32_t(millis()-retryAt)>=15000){
  retryAt=millis();++reconnects;WiFi.reconnect();
 }
 if(wifiUp){
  for(int budget=0;budget<2;++budget){
   int len=udp.parsePacket();if(len<=0)break;
   uint64_t peer=(uint64_t(uint32_t(udp.remoteIP()))<<16)|udp.remotePort();
   int got=udp.read(bytes,sizeof(bytes));
   if(len>=int(sizeof(bytes))||got!=len){
    const uint8_t invalid=0;receive(control::Link::WiFi,peer,&invalid,1);continue;
   }
   receive(control::Link::WiFi,peer,bytes,got);
   // Each UDP datagram is exactly one framed request, never joined to another sender.
   if(got && bytes[got-1]!='\n'){
    const uint8_t invalid=0;receive(control::Link::WiFi,peer,&invalid,1);
   }
  }
 }
 RX packet{};
 for(int budget=0;rxQueue&&budget<4&&xQueueReceive(rxQueue,&packet,0)==pdTRUE;++budget){
  if(alive(control::Link::BLE,packet.peer))receive(control::Link::BLE,packet.peer,packet.data,packet.size);
 }
 if(advertiseAgain.exchange(false)&&server)server->startAdvertising();
 if(txQueue&&tx&&uint32_t(millis()-lastNotify)>=5){
  lastNotify=millis();
  if(sent==sending.size){
   if(xQueueReceive(txQueue,&sending,0)!=pdTRUE)return;
   sent=0;
  }
  if(!alive(control::Link::BLE,sending.peer)){sent=sending.size;return;}
  size_t part=std::min<size_t>(20,sending.size-sent); // works even at default MTU 23
  tx->setValue(reinterpret_cast<const uint8_t*>(sending.data+sent),part);tx->notify();sent+=part;
  handleNotifyFailure();
 }
}
bool ControlTransport::send(control::Link l,uint64_t peer,const char* line){
 size_t n=strlen(line);bool ok=false;
 if(n>=maxLine||!alive(l,peer)){++dropped;return false;}
 if(l==control::Link::USB){
  ok=Serial.availableForWrite()>=int(n)&&Serial.write(reinterpret_cast<const uint8_t*>(line),n)==n;
 }else if(l==control::Link::WiFi){
  if(udp.beginPacket(IPAddress(uint32_t(peer>>16)),uint16_t(peer))) {
   ok=udp.write(reinterpret_cast<const uint8_t*>(line),n)==n;
   ok=udp.endPacket()==1&&ok;
  }
 }else if(l==control::Link::BLE){
  TX message{};message.peer=uint32_t(peer);message.size=n;memcpy(message.data,line,n);
  ok=xQueueSend(txQueue,&message,0)==pdTRUE;
 }
 if(!ok)++dropped;
 return ok;
}
#endif
