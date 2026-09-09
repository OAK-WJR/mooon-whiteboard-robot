#pragma once
#ifdef WBB_TOP_BOARD_V2
#ifdef WBB_CONTROL_TRANSPORT_NATIVE_TEST
#include "../control_tests/transport_stubs.h"
#else
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#endif
#include <atomic>
#include "control_session.h"

class ControlTransport {
public:
#ifdef WBB_TOP_BOARD_PICO
 // Includes rail freshness/error metadata and escaped, authenticated payload.
 static constexpr size_t maxLine=2304;
#else
 static constexpr size_t maxLine=1536;
#endif
 static constexpr uint16_t udpPort=4243;
 static constexpr const char* service="6e400001-b5a3-f393-e0a9-e50e24dcca9e";
 static constexpr const char* rxUUID="6e400002-b5a3-f393-e0a9-e50e24dcca9e";
 static constexpr const char* txUUID="6e400003-b5a3-f393-e0a9-e50e24dcca9e";
 using Receiver=void(*)(control::Link,uint64_t,const uint8_t*,size_t);
 bool begin(const char* name,const char* ssid,const char* password);
 void poll(Receiver receive);
 bool send(control::Link link,uint64_t peer,const char* line);
 bool alive(control::Link link,uint64_t peer)const;
 bool txPending(control::Link link,uint64_t peer)const;
 bool takeOverflow(){return overflow.exchange(false);}
 uint32_t dropped=0,reconnects=0;
 bool wifiUp=false,bleReady=false;
 std::atomic<bool> bleUp{false};
 std::atomic<uint32_t> blePeer{0};
 std::atomic<uint16_t> bleHandle{0xffff};
 std::atomic<bool> overflow{false},advertiseAgain{false};
 std::atomic<bool> notifyFailed{false};
 std::atomic<uint32_t> notifyFailures{0};
 struct RX {uint32_t peer;uint16_t size;uint8_t data[128];};
 QueueHandle_t rxQueue=nullptr;
private:
 struct TX {uint32_t peer;uint16_t size;char data[maxLine];};
 QueueHandle_t txQueue=nullptr;
 TX sending{};size_t sent=0;
 WiFiUDP udp;
 BLEServer* server=nullptr;
 BLECharacteristic* tx=nullptr;
 bool configured=false;
 uint32_t retryAt=0,lastNotify=0;
 void handleNotifyFailure();
};
#endif
