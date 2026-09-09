#pragma once
#include "bq76905_monitor.h"

#include <algorithm>
#include <array>
#include <functional>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace bq_model {
// Independent TI protocol oracle. No production register constants or private
// methods are imported. Literal addresses/bit meanings follow source_contract.json.
using namespace bq76905;
using Bytes = std::vector<std::uint8_t>;
#define CHECK(x) do { if (!(x)) throw std::runtime_error(std::string(__func__) + ":" + std::to_string(__LINE__) + " " #x); } while (0)

static std::uint8_t referenceCrc(const Bytes& bytes) {
    // Polynomial long division, intentionally different from driver's byte LFSR.
    CHECK(bytes.size() <= 4);
    std::uint64_t dividend = 0;
    for (auto byte : bytes) dividend = (dividend << 8) | byte;
    dividend <<= 8;
    for (int bit = static_cast<int>(bytes.size() * 8 + 7); bit >= 8; --bit)
        if (dividend & (std::uint64_t(1) << bit)) dividend ^= std::uint64_t(0x107) << (bit - 8);
    return static_cast<std::uint8_t>(dividend);
}
static std::uint16_t word(const Bytes& v, std::size_t at = 0) { return std::uint16_t(v.at(at) | (v.at(at + 1) << 8)); }
struct TestClock : Clock {
    std::uint32_t ms = 0;
    std::uint32_t nowMs() const override { return ms; }
};
struct Transfer { std::uint8_t address; Bytes tx; std::size_t rx_size; std::uint32_t at; };
struct Commit { std::uint16_t address; Bytes data; std::uint8_t checksum; std::uint8_t length; bool config; };

struct GateBlocks { bool dsg; bool chg; };
// Independent latched-fault routing oracle: SLUUCI8 tables 11-2/4 and
// 12-12/13/14. This isolates protection direction from the separate held-OFF
// override. It never drives a FET or grants a permission.
static GateBlocks decodeFaultRoutes(std::uint8_t safety_a, std::uint8_t safety_b,
                                   std::uint8_t dsg_map, std::uint8_t chg_map,
                                   std::uint8_t both_map) {
    struct Route { std::uint8_t status_a, status_b, dsg, chg, both; };
    const Route routes[] = {
        {0x40,0,    0x80,0,   0}, // CUV: DSG only.
        {0x80,0,    0,   0x80,0}, // COV: CHG only.
        {0x20,0,    0x40,0x40,0}, // SCD.
        {0x10,0,    0x20,0,   0}, // OCD1.
        {0x08,0,    0x10,0,   0}, // OCD2.
        {0x04,0,    0,   0x20,0}, // OCC.
        {0,   0x04, 0x08,0x08,0}, // HWD: Safety Status B bit2, not its enable bit0.
        {0,   0x80, 0x04,0,   0}, // OTD.
        {0,   0x40, 0,   0x04,0}, // OTC.
        {0,   0x20, 0x02,0,   0}, // UTD.
        {0,   0x10, 0,   0x02,0}, // UTC.
        {0,   0x08, 0x01,0x01,0}, // OTINT: Safety Status B bit3, each route bit0.
        {0,   0x02, 0,   0,   4}, // VREF.
        {0,   0x01, 0,   0,   2}, // VSS.
        {0x01,0,    0,   0,   1}  // REGOUT.
    };
    GateBlocks result{false, false};
    for (const auto& route : routes) {
        if (!((safety_a & route.status_a) || (safety_b & route.status_b))) continue;
        const bool shared = (both_map & route.both) != 0;
        result.dsg = result.dsg || shared || (dsg_map & route.dsg);
        result.chg = result.chg || shared || (chg_map & route.chg);
    }
    return result;
}

class Device : public I2c {
public:
    explicit Device(TestClock& c) : clock(c) { reset(); }
    TestClock& clock;
    std::map<std::uint16_t, std::uint8_t> ram;
    std::uint16_t battery = 0x04c0; // FULLACCESS, POR, SLEEP_EN. FET_EN=0.
    std::uint16_t device_number = 0x7605;
    std::uint16_t alarm_enable = 0xc200, alarms = 0, raw = 0;
    std::uint8_t regout = 0x1e, fet_control = 0, cb = 0;
    std::array<std::uint8_t, 4> safety{{0, 0, 0, 0}};
    int lower = 3700, upper = 3800, stack = 7500, ts = 10000;
    bool crc = false, otp_crc = false, busy = false, stuck_pending = false;
    bool never_enter = false, never_exit = false, never_scan = false;
    bool never_init = false, ignore_off = false, wrong_echo = false;
    bool bad_checksum = false, stop_separated_crc = false, corrupt_crc = false;
    bool omit_second_crc = false, reject_balance_clear = false, keep_por_on_exit = false;
    bool suppress_status_config_bit = false;
    int bad_length = -1, wake_naks = 0, delay_polls = 0, error_next = 0;
    std::uint16_t ignore_ram_write = 0;
    unsigned cancel_count = 0, calls_this_tick = 0, illegal_config_commits = 0;
    bool active = false;
    Transfer transfer{};
    std::vector<Transfer> trace;
    std::vector<Bytes> direct_writes;
    std::vector<std::uint16_t> commands;
    std::vector<Commit> commits;
    std::function<void(Device&, const Transfer&)> before_complete;
    std::function<void(Device&, const Transfer&)> after_complete;
    std::uint32_t enter_at = 0, exit_at = 0, init_at = 0, last_scan = 0;
    bool entering = false, exiting = false, init_pending = false;

    void reset() {
        ram.clear();
        ram[0x9015] = 0x0e; ram[0x9016] = 0x08; // Explicit model OTP: enabled 3.3 V, address 8.
        ram[0x9017] = otp_crc ? 1 : 0; ram[0x9018] = 0x34;
        ram[0x901b] = 0; ram[0x901e] = 0x18; ram[0x9020] = 0x02;
        ram[0x901c] = 0x00; ram[0x901d] = 0xc2;
        ram[0x9024] = 0xa1; ram[0x9026] = 0xff; ram[0x9027] = 0xef; ram[0x9028] = 0x06;
        battery = 0x04c0; crc = otp_crc; cb = 0; fet_control = 0; regout = 0x1e;
        alarm_enable = 0xc200; alarms = raw = 0;
        entering = exiting = false; init_pending = true; init_at = clock.ms + 20;
    }
    void events() {
        const auto reached = [this](std::uint32_t at) { return std::uint32_t(clock.ms - at) < 0x80000000u; };
        if (entering && reached(enter_at) && !never_enter) {
            battery |= 0x0020; battery &= ~0x000c; entering = false;
            safety.fill(0);
        }
        if (exiting && reached(exit_at) && !never_exit) {
            battery &= ~0x01a0; // Clear CFGUPDATE, POR and reload FET_EN.
            if (ram[0x901e] & 4) battery |= 0x0100;
            if (keep_por_on_exit) battery |= 0x0080;
            alarm_enable = ram[0x901c] | (ram[0x901d] << 8);
            regout = (regout & 0x10) | (ram[0x9015] & 0x0f);
            crc = (ram[0x9017] & 1) != 0;
            exiting = false; init_pending = true; init_at = clock.ms + 25;
        }
        raw = (battery & 0x80) ? 1 : 0;
        if (init_pending && !(battery & 0x20) && reached(init_at) && !never_init) {
            raw |= 4; init_pending = false;
        }
        if (!(battery & 0x20) && !never_scan && std::uint32_t(clock.ms - last_scan) >= 37) {
            last_scan = clock.ms; raw |= 0x80;
        }
        alarms |= raw & alarm_enable; // Raw pulses vanish; Alarm Status is latched/W1C.
    }
    bool start(std::uint8_t a, const std::uint8_t* data, std::size_t n, std::size_t rx) override {
        ++calls_this_tick;
        if (busy) return false;
        CHECK(!active); CHECK(n <= 69 && rx <= 68);
        transfer = Transfer{a, Bytes(data, data + n), rx, clock.ms};
        trace.push_back(transfer); active = true; remaining_polls = delay_polls;
        return true;
    }
    BusResult poll(std::uint8_t* destination, std::size_t capacity) override {
        ++calls_this_tick; CHECK(active);
        if (stuck_pending || remaining_polls-- > 0) return {BusState::Pending, 0, 0};
        if (before_complete) before_complete(*this, transfer);
        active = false;
        if (wake_naks > 0 && transfer.tx == Bytes{0x12}) {
            --wake_naks; return {BusState::Nak, 0, 0};
        }
        if (error_next == 1 || error_next == 2) {
            const int e = error_next; error_next = 0;
            return {e == 1 ? BusState::Nak : BusState::BusError, 0, 0};
        }
        events();
        Bytes response;
        if (transfer.rx_size) {
            CHECK(transfer.tx.size() == 1);
            const auto n = transfer.rx_size / (crc ? 2 : 1);
            const Bytes data = read(transfer.tx[0], n);
            for (std::size_t i = 0; i < data.size(); ++i) {
                response.push_back(data[i]);
                if (crc) {
                    Bytes prefix;
                    if (!i) {
                        if (!stop_separated_crc) { prefix.push_back(transfer.address << 1); prefix.push_back(transfer.tx[0]); }
                        prefix.push_back((transfer.address << 1) | 1);
                    }
                    prefix.push_back(data[i]);
                    auto c = referenceCrc(prefix);
                    if (corrupt_crc && i == 1) c ^= 1;
                    if (!(omit_second_crc && i == 1)) response.push_back(c);
                }
            }
        } else {
            Bytes data;
            for (std::size_t i = 1; i < transfer.tx.size();) {
                const auto byte = transfer.tx[i++];
                if (crc) {
                    if (i == transfer.tx.size()) return {BusState::Nak, i - 1, 0};
                    Bytes prefix;
                    if (data.empty()) prefix = {std::uint8_t(transfer.address << 1), transfer.tx[0]};
                    prefix.push_back(byte);
                    if (transfer.tx[i++] != referenceCrc(prefix)) return {BusState::Nak, i - 1, 0};
                }
                data.push_back(byte);
            }
            write(transfer.tx[0], data);
        }
        if (after_complete) after_complete(*this, transfer);
        const int error = error_next; error_next = 0;
        if (error == 3 && !response.empty()) response.pop_back();
        CHECK(response.size() <= capacity);
        std::copy(response.begin(), response.end(), destination);
        return {BusState::Complete, transfer.tx.size() - (error == 4 ? 1 : 0), response.size()};
    }
    void cancel() override { ++cancel_count; ++calls_this_tick; CHECK(active); active = false; }

private:
    int remaining_polls = 0;
    std::uint16_t selected = 0;
    Bytes staged;
    std::uint8_t checksum = 0;
    Bytes read(std::uint8_t reg, std::size_t n) {
        Bytes out(n, 0);
        const auto put = [&out](int v) {
            if (!out.empty()) out[0] = static_cast<std::uint8_t>(v);
            if (out.size() > 1) out[1] = static_cast<std::uint8_t>(static_cast<unsigned>(v) >> 8);
        };
        switch (reg) {
        case 0x12: put(suppress_status_config_bit ? battery & ~0x20 : battery); break;
        case 0x14: put(lower); break;
        case 0x16: put(0); break; // Shorted physical Cell2 in a 2S pack.
        case 0x1c: put(upper); break;
        case 0x26: put(stack); break;
        case 0x2a: put(ts); break;
        case 0x02:
            for (std::size_t i = 0; i < std::min(n, safety.size()); ++i) out[i] = safety[i];
            break;
        case 0x62: put(alarms); break;
        case 0x64: put(raw); break;
        case 0x66: put(alarm_enable); break;
        case 0x68: put(fet_control); break;
        case 0x69: put(regout); break;
        case 0x3e: put(wrong_echo ? selected ^ 1 : selected); break;
        case 0x40: {
            CHECK(n == 34); // Full buffer + checksum + length, 0x61 read LAST.
            unsigned data_length = 32;
            if (selected == 0x0001) { data_length = 2; put(device_number); }
            else if (selected == 0x0083) { data_length = 1; put(cb); }
            else {
                CHECK(selected >= 0x9000);
                for (unsigned i = 0; i < 32; ++i) out[i] = ram[selected + i];
            }
            unsigned sum = (selected & 255) + (selected >> 8);
            for (unsigned i = 0; i < data_length; ++i) sum += out[i];
            out[32] = static_cast<std::uint8_t>(255 - (sum % 256));
            if (bad_checksum) out[32] ^= 1;
            out[33] = static_cast<std::uint8_t>(bad_length >= 0 ? bad_length : data_length + 4);
            selected += 0x20; // Device's auto-increment side effect on read of 0x61.
            break;
        }
        default: throw std::runtime_error("Unexpected register read: " + std::to_string(reg));
        }
        return out;
    }
    void write(std::uint8_t reg, const Bytes& data) {
        if (reg == 0x3e) {
            CHECK(data.size() == 2); selected = word(data); staged.clear();
            if (selected == 0x0090 || selected == 0x0092 || selected == 0x009a || selected == 0x000e) {
                commands.push_back(selected);
                if (selected == 0x0090) { entering = true; enter_at = clock.ms + 23; }
                if (selected == 0x0092) { exiting = true; exit_at = clock.ms + 19; }
                if (selected == 0x009a) battery &= ~0x8040;
                if (selected == 0x000e) battery &= ~0x4000;
            } else CHECK(selected == 1 || selected == 0x83 || selected >= 0x9000);
        } else if (reg == 0x40) staged = data;
        else if (reg == 0x60) {
            CHECK(data.size() == 2); checksum = data[0];
            CHECK(data[1] == staged.size() + 4);
            unsigned sum = (selected & 255) + (selected >> 8) + checksum;
            for (auto b : staged) sum += b;
            CHECK((sum % 256) == 255);
            commits.push_back(Commit{selected, staged, checksum, data[1], (battery & 0x20) != 0});
            if (selected == 0x0083) {
                if (!(ram[0x9020] & 1) && !reject_balance_clear) cb = staged.at(0);
            } else if (!(battery & 0x20)) ++illegal_config_commits;
            else if (selected != ignore_ram_write) {
                for (unsigned i = 0; i < staged.size(); ++i) ram[selected + i] = staged[i];
            }
        } else {
            Bytes record{reg}; record.insert(record.end(), data.begin(), data.end()); direct_writes.push_back(record);
            if (reg == 0x68) {
                CHECK(data.size() == 1 && (data[0] & 3) == 0); // No CHG_ON / DSG_ON, ever.
                fet_control = data[0];
                if ((ram[0x901e] & 0x40) && !ignore_off) battery &= ~(data[0] & 0x0c);
            } else if (reg == 0x69) regout = data.at(0);
            else if (reg == 0x66) alarm_enable = word(data);
            else if (reg == 0x62) alarms &= ~word(data);
            else throw std::runtime_error("Unexpected write: " + std::to_string(reg));
        }
    }
};

} // namespace bq_model
