// Executes the actual read-only entry point; no MCU, USB device or motors used.
#include <cassert>
#include <iostream>
#include <ArduinoJson.h>
#include "../c6_selftest/src/main.cpp"

static JsonDocument latest() {
    const auto end = Serial.tx.size() - 1;
    auto start = Serial.tx.rfind('\n', end - 1);
    start = start == std::string::npos ? 0 : start + 1;
    JsonDocument d;
    assert(!deserializeJson(d, Serial.tx.substr(start, end - start)));
    return d;
}
static void stopped() {
    assert(levels[21] == LOW && modes[21] == OUTPUT);
    for (const auto& p : r3::headers) {
        assert(levels[p.fi] == LOW && levels[p.bi] == LOW);
        assert(modes[p.encA] == INPUT_PULLUP && modes[p.encB] == INPUT_PULLUP);
    }
    for (const auto& op : pinOperations) {
        assert(op.kind == "gpio" && op.value == LOW);
        assert((op.pin >= 0 && op.pin <= 5) || op.pin == 21);
    }
    for (int p : {12, 13, 16, 17, 14, 22}) assert(modes[p] != OUTPUT);
}
static void submit(const std::string& text) {
    Serial.feed(text);
    while (Serial.available()) { loop(); stopped(); }
}
int main() {
    setup(); stopped();
    assert(Serial.uartMode && Serial.baud == 115200 && Serial.rxPin == 17 && Serial.txPin == 16);
    auto d = latest();
    assert(d["motion_enabled"] == false && d["charge_enabled"] == false && d["power_disconnect"] == false);
    assert(d["motor_voltage_mv"].isNull() && d["motor_current_ma"].isNull() && d["rail_3v3_mv"].isNull());
    std::cout << "R3_DIAG_STATUS " << Serial.tx;
    for (const char* op : {"0f", "0r", "1f", "1r", "2f", "2r", "a", "w", "d 255", "go", "charge",
            "armbench 6000 6000 64", "armbench 12000 12000 64", "pulse 3 1 64 500"}) {
        submit(std::string(op) + '\n');
        assert(latest()["last_error"] == "BOARD_POWER_UNREVIEWED");
        assert(!rebootPending && restartCalls == 0);
    }
    for (const char* op : {"hello", "status", "p", "s", "stop", "x", "h", "?"}) {
        submit(std::string(op) + '\n'); assert(latest()["last_error"] == "");
    }
    levels[6] = 1; submit("p\n"); assert(latest()["motors"][0]["b"] == 1);
    levels[6] = 0; submit("p\n"); assert(latest()["motors"][0]["b"] == 0);
    submit("reboot" + std::string(200, ' ') + "\n");
    assert(latest()["last_error"] == "RX_OVERFLOW" && !rebootPending);
    submit(std::string("reboot\0ignored\n", 15)); assert(!rebootPending);
    auto dropped = txDropped; Serial.writeCapacity = 0;
    submit("hello\n"); assert(txDropped == dropped + 1);
    Serial.writeCapacity = 8192;
    Serial.connected = false; submit("status\n"); // UART has no host-present bit.
    assert(latest()["usb_disconnect_detect"] == false);
    fakeNow = 0xfffffff0U; submit("reboot\nreboot\n0f\n");
    auto requested = rebootAt;
    assert(rebootPending && restartCalls == 0 && latest()["last_error"] == "REBOOT_PENDING");
    fakeNow = requested + 99; loop(); assert(restartCalls == 0);
    Serial.writeCapacity = 0; fakeNow = requested + 100; loop();
    assert(restartCalls == 1); stopped();
    std::cout << "R3 diagnostic read-only pins, voltage/motion rejection, framing, TX pressure and reset wrap PASS\n";
}
