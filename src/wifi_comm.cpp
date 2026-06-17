#include "wifi_comm.h"

void WifiComm::begin(const char* ssid, const char* pass) {
    WiFi.begin(ssid, pass);
    Serial.print("WiFi connecting");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\nWiFi connected: %s\n", WiFi.localIP().toString().c_str());

    udp_.begin(UDP_LISTEN_PORT);
    Serial.printf("UDP listening on port %d\n", UDP_LISTEN_PORT);
}

bool WifiComm::receive(JsonDocument& doc) {
    int len = udp_.parsePacket();
    if (len == 0) return false;

    char buf[512];
    int n = udp_.read(buf, sizeof(buf) - 1);
    if (n <= 0) return false;
    buf[n] = '\0';

    // 记住发送方地址用于回传
    remote_ip_ = udp_.remoteIP();
    remote_port_ = udp_.remotePort();
    has_remote_ = true;

    DeserializationError err = deserializeJson(doc, buf);
    if (err) {
        Serial.printf("JSON parse error: %s\n", err.c_str());
        return false;
    }
    return true;
}

void WifiComm::send(const JsonDocument& doc) {
    if (!has_remote_) return;

    char buf[256];
    size_t n = serializeJson(doc, buf, sizeof(buf));

    udp_.beginPacket(remote_ip_, remote_port_);
    udp_.write((const uint8_t*)buf, n);
    udp_.endPacket();
}
