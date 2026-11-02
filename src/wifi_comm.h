#ifndef WIFI_COMM_H
#define WIFI_COMM_H

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "firmware_config.h"

class WifiComm {
public:
    void begin(const char* ssid, const char* pass);
    bool isConnected() const { return WiFi.status() == WL_CONNECTED; }
    IPAddress localIP() const { return WiFi.localIP(); }

    // Receive a JSON command; returns true if there is a new command
    bool receive(JsonDocument& doc);

    // Send JSON status to the sender
    void send(const JsonDocument& doc);

private:
    WiFiUDP udp_;
    IPAddress remote_ip_;
    uint16_t remote_port_ = 0;
    bool has_remote_ = false;
    bool active_ = false;   // only send/receive when WiFi is connected and UDP has begun (false in USB mode, UDP untouched)
};

#endif
