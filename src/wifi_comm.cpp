#include "wifi_comm.h"

void WifiComm::begin(const char* ssid, const char* pass) {
#if !USE_WIFI
    // USB 串口工作模式: 完全不碰 WiFi/UDP —— 不阻塞启动, 不刷 WiFiUdp 报错,
    // 也消除 GPIO36/39(ENC1) 的射频噪声。命令走串口, 位姿走 'j' JSON 流。
    (void)ssid; (void)pass;
    WiFi.mode(WIFI_OFF);
    Serial.println("WiFi 关闭 (USB 串口模式, USE_WIFI=0)");
    return;
#else
    WiFi.begin(ssid, pass);
    Serial.print("WiFi connecting");
    // 非阻塞: 超时后继续运行, 不卡启动 (USB/串口模式无需 WiFi)
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
        delay(250);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\nWiFi connected: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\nWiFi 未连接 (超时), 关射频走 USB — 消除 ENC1(GPIO36/39) RF 噪声");
        WiFi.mode(WIFI_OFF);   // 关 WiFi 射频, 否则 GPIO36/39 编码器被灌假计数
        return;                // USB 模式不需要 UDP
    }
    udp_.begin(UDP_LISTEN_PORT);
    active_ = true;            // 连上且 UDP 已 begin, 才允许 receive/send
#endif
}

bool WifiComm::receive(JsonDocument& doc) {
    if (!active_) return false;           // USB 模式: 不碰 UDP (否则刷 WiFiUdp 报错)
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
    if (!active_ || !has_remote_) return;

    char buf[256];
    size_t n = serializeJson(doc, buf, sizeof(buf));

    udp_.beginPacket(remote_ip_, remote_port_);
    udp_.write((const uint8_t*)buf, n);
    udp_.endPacket();
}
