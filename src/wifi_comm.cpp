#include "wifi_comm.h"

void WifiComm::begin(const char* ssid, const char* pass) {
#if !USE_WIFI
    // USB serial mode: never touch WiFi/UDP — startup doesn't block, no flood of WiFiUdp errors,
    // and no RF noise on GPIO36/39 (ENC1). Commands go over serial, pose goes over the 'j' JSON stream.
    (void)ssid; (void)pass;
    WiFi.mode(WIFI_OFF);
    Serial.println("WiFi off (USB serial mode, USE_WIFI=0)");
    return;
#else
    WiFi.begin(ssid, pass);
    Serial.print("WiFi connecting");
    // Non-blocking: keep running after the timeout, don't hang startup (USB/serial mode needs no WiFi)
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
        delay(250);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\nWiFi connected: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\nWiFi not connected (timeout), turning off radio and using USB — removes ENC1 (GPIO36/39) RF noise");
        WiFi.mode(WIFI_OFF);   // turn off the WiFi radio, otherwise the GPIO36/39 encoder gets flooded with false counts
        return;                // USB mode needs no UDP
    }
    udp_.begin(UDP_LISTEN_PORT);
    active_ = true;            // only allow receive/send once connected and UDP has begun
#endif
}

bool WifiComm::receive(JsonDocument& doc) {
    if (!active_) return false;           // USB mode: don't touch UDP (otherwise WiFiUdp floods errors)
    int len = udp_.parsePacket();
    if (len == 0) return false;

    char buf[512];
    int n = udp_.read(buf, sizeof(buf) - 1);
    if (n <= 0) return false;
    buf[n] = '\0';

    // Remember the sender's address for replies
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
    if (!active_ || !has_remote_) return;

    char buf[256];
    size_t n = serializeJson(doc, buf, sizeof(buf));

    udp_.beginPacket(remote_ip_, remote_port_);
    udp_.write((const uint8_t*)buf, n);
    udp_.endPacket();
}
