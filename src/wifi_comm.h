#ifndef WIFI_COMM_H
#define WIFI_COMM_H

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "config.h"

class WifiComm {
public:
    void begin(const char* ssid, const char* pass);
    bool isConnected() const { return WiFi.status() == WL_CONNECTED; }
    IPAddress localIP() const { return WiFi.localIP(); }

    // 接收 JSON 命令，返回 true 如果有新命令
    bool receive(JsonDocument& doc);

    // 发送 JSON 状态到发送方
    void send(const JsonDocument& doc);

private:
    WiFiUDP udp_;
    IPAddress remote_ip_;
    uint16_t remote_port_ = 0;
    bool has_remote_ = false;
};

#endif
