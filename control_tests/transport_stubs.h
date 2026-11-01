#pragma once
// SDK boundary for the actual ControlTransport source, not an RF/FreeRTOS emulator.
#include "mock/Arduino.h"
#include <memory>
#define CONFIG_ESP_COEX_SW_COEXIST_ENABLE 1
constexpr int pdTRUE=1,pdFALSE=0;
struct FakeQueue {size_t capacity,itemSize;std::deque<std::vector<uint8_t>> data;};
using QueueHandle_t=FakeQueue*;
inline std::vector<std::unique_ptr<FakeQueue>> queues;
inline QueueHandle_t xQueueCreate(size_t capacity,size_t size){
    queues.emplace_back(new FakeQueue{capacity,size,{}});return queues.back().get();
}
inline int xQueueSend(QueueHandle_t q,const void* value,unsigned){
    assert(q);if(q->data.size()==q->capacity)return pdFALSE;
    const auto* bytes=static_cast<const uint8_t*>(value);
    q->data.emplace_back(bytes,bytes+q->itemSize);return pdTRUE;
}
inline int xQueueReceive(QueueHandle_t q,void* value,unsigned){
    assert(q);if(q->data.empty())return pdFALSE;
    memcpy(value,q->data.front().data(),q->itemSize);q->data.pop_front();return pdTRUE;
}
inline size_t uxQueueMessagesWaiting(QueueHandle_t q){assert(q);return q->data.size();}
inline int xQueueReset(QueueHandle_t q){assert(q);q->data.clear();return pdTRUE;}

// Status-budget fixtures widen these to the longest values the SDK can report.
inline std::string fakeIpText="192.0.2.1";
inline int fakeRssi=-50;
struct IPAddress {
    uint32_t value=0xc0000201;
    IPAddress()=default;
    explicit IPAddress(uint32_t value):value(value){}
    explicit operator uint32_t()const{return value;}
    std::string toString()const{return fakeIpText;}
};
constexpr int WIFI_STA=1,WL_CONNECTED=3;
inline unsigned radioFailureStage=0;
struct FakeWiFi {
    bool connected=false,autoReconnect=false;unsigned reconnectCalls=0,modeCalls=0,beginCalls=0;
    bool mode(int){++modeCalls;return radioFailureStage!=4;} void setAutoReconnect(bool value){autoReconnect=value;}
    void begin(const char*,const char*){++beginCalls;} int status()const{return connected?WL_CONNECTED:0;}
    void reconnect(){++reconnectCalls;} IPAddress localIP()const{return IPAddress();}
    int RSSI()const{return fakeRssi;}
};
inline FakeWiFi WiFi;
struct Datagram {IPAddress ip;uint16_t port;std::string data;};
inline std::deque<Datagram> udpIncoming;
inline std::vector<Datagram> udpOutgoing;
class WiFiUDP {
    Datagram incoming{},outgoing{};
public:
    int begin(uint16_t){return 1;} void stop(){}
    int parsePacket(){
        if(udpIncoming.empty())return 0;
        incoming=udpIncoming.front();udpIncoming.pop_front();return int(incoming.data.size());
    }
    IPAddress remoteIP()const{return incoming.ip;} uint16_t remotePort()const{return incoming.port;}
    int read(uint8_t* bytes,size_t n){n=std::min(n,incoming.data.size());memcpy(bytes,incoming.data.data(),n);return int(n);}
    int beginPacket(IPAddress ip,uint16_t port){outgoing={ip,port,{}};return 1;}
    size_t write(const uint8_t* bytes,size_t n){outgoing.data.append(reinterpret_cast<const char*>(bytes),n);return n;}
    int endPacket(){udpOutgoing.push_back(outgoing);return 1;}
};

struct ble_gap_conn_desc {uint16_t conn_handle;};
class BLEServer;class BLECharacteristic;
class BLEServerCallbacks {
public:
    virtual ~BLEServerCallbacks()=default;
    virtual void onConnect(BLEServer*,ble_gap_conn_desc*){}
    virtual void onDisconnect(BLEServer*,ble_gap_conn_desc*){}
};
class BLECharacteristicCallbacks {
public:
    enum Status{SUCCESS_INDICATE,SUCCESS_NOTIFY,ERROR_INDICATE_DISABLED,ERROR_NOTIFY_DISABLED,
        ERROR_GATT,ERROR_NO_CLIENT,ERROR_NO_SUBSCRIBER,ERROR_INDICATE_TIMEOUT,ERROR_INDICATE_FAILURE};
    virtual ~BLECharacteristicCallbacks()=default;
    virtual void onWrite(BLECharacteristic*,ble_gap_conn_desc*){}
    virtual void onStatus(BLECharacteristic*,Status,uint32_t){}
};
struct Notification {uint16_t handle;uint32_t at;std::string data;};
inline std::vector<Notification> notifications;
inline uint16_t connectedHandle=0xffff;
inline bool notifyFails=false,subscribed=true;
class BLECharacteristic {
    std::unique_ptr<BLECharacteristicCallbacks> callbacks;
    std::string value;
public:
    static constexpr int PROPERTY_NOTIFY=1,PROPERTY_WRITE=2;
    void setCallbacks(BLECharacteristicCallbacks* c){callbacks.reset(c);}
    void setValue(const uint8_t* data,size_t n){value.assign(reinterpret_cast<const char*>(data),n);}
    std::string getValue()const{return value;}
    void write(uint16_t handle,const std::string& data){
        value=data;ble_gap_conn_desc desc{handle};if(callbacks)callbacks->onWrite(this,&desc);
    }
    void notify(){
        auto status=connectedHandle==0xffff?BLECharacteristicCallbacks::ERROR_NO_CLIENT:
            (!subscribed?BLECharacteristicCallbacks::ERROR_NO_SUBSCRIBER:
             (notifyFails?BLECharacteristicCallbacks::ERROR_GATT:BLECharacteristicCallbacks::SUCCESS_NOTIFY));
        if(status==BLECharacteristicCallbacks::SUCCESS_NOTIFY)notifications.push_back({connectedHandle,millis(),value});
        if(callbacks)callbacks->onStatus(this,status,0);
    }
    void asynchronousFailure(){if(callbacks)callbacks->onStatus(this,BLECharacteristicCallbacks::ERROR_GATT,1);}
};
class BLEService {
public:
    std::vector<std::unique_ptr<BLECharacteristic>> characteristics;
    BLECharacteristic* createCharacteristic(const char*,int){
        characteristics.emplace_back(new BLECharacteristic);return characteristics.back().get();
    }
    bool start(){return radioFailureStage!=2;}
};
class BLEAdvertising {
public:
    unsigned starts=0;
    void addServiceUUID(const char*){} bool start(){++starts;return radioFailureStage!=3;}
};
class BLEServer {
    std::unique_ptr<BLEServerCallbacks> callbacks;
public:
    BLEService service;BLEAdvertising advertising;
    void setCallbacks(BLEServerCallbacks* c){callbacks.reset(c);}
    BLEService* createService(const char*){return &service;}
    BLEAdvertising* getAdvertising(){return &advertising;}
    void startAdvertising(){advertising.start();}
    void connect(uint16_t handle){
        if(connectedHandle==0xffff)connectedHandle=handle;
        ble_gap_conn_desc desc{handle};callbacks->onConnect(this,&desc);
    }
    void disconnect(uint16_t handle){
        if(connectedHandle==handle)connectedHandle=0xffff;
        ble_gap_conn_desc desc{handle};callbacks->onDisconnect(this,&desc);
    }
};
class BLEDevice {
public:
    inline static std::vector<std::unique_ptr<BLEServer>> servers;
    inline static unsigned initCalls=0,initDelayMs=0;
    inline static void (*duringInit)()=nullptr;
    static bool init(const char*){++initCalls;fakeNow+=initDelayMs;if(duringInit)duringInit();return radioFailureStage!=1;}
    static int setMTU(unsigned){return 0;}
    static BLEServer* createServer(){servers.emplace_back(new BLEServer);return servers.back().get();}
};
