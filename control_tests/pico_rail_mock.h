#pragma once
// Fake only the worker/device boundary. The production monitor, service and
// runtime serializer run unchanged; this is not a silicon or timing simulator.
#include "../include/pico_rail_service.h"
#include "../src/ina3221_monitor.cpp"
#include "../src/bq76905_monitor.cpp"
#include "../src/pico_rail_service.cpp"
#include "bq76905/device_model.h"
#include <cassert>
#include <vector>

namespace pico_rail_mock {
class Bus final : public pico_rails::Bus {
public:
    bq_model::TestClock cellClock;
    bq_model::Device cellDevice{cellClock};
    bool cellPresent=false, cellDraining=false;
    unsigned cellStarts=0,cellPolls=0;
    class CellPort final : public bq76905::I2c {
    public:
        explicit CellPort(Bus& owner) : owner_(owner) {}
        bool start(uint8_t address,const uint8_t* tx,std::size_t txSize,std::size_t rxSize) override {
            assert(!owner_.active_ && address==0x08);
            if(owner_.cellDraining)return false;
            owner_.cellClock.ms=static_cast<uint32_t>(owner_.nowMicros()/1000);
            ++owner_.cellStarts;
            return owner_.cellDevice.start(address,tx,txSize,rxSize);
        }
        bq76905::BusResult poll(uint8_t* rx,std::size_t capacity) override {
            assert(!owner_.active_);++owner_.cellPolls;
            owner_.cellClock.ms=static_cast<uint32_t>(owner_.nowMicros()/1000);
            if(!owner_.cellPresent)owner_.cellDevice.error_next=1;
            return owner_.cellDevice.poll(rx,capacity);
        }
        void cancel() override { if(owner_.cellDevice.active)owner_.cellDevice.cancel(); }
    private:
        Bus& owner_;
    } cellPort{*this};
    bool present = false, openOk = true, setupPending = false, pending = false;
    bool nackNext = false, shortNext = false, corruptConfig = false, cleanupFails = false;
    unsigned opens = 0, closes = 0, starts = 0, polls = 0, cancels = 0;
    pico_rails::BusState link = pico_rails::BusState::Stopped;
    std::array<uint16_t, 3> voltages{5000, 5000, 0};
    std::vector<uint8_t> addresses;
    bool open() override {
        ++opens;
        if (!openOk) return false;
        link = setupPending ? pico_rails::BusState::Starting : pico_rails::BusState::Ready;
        return true;
    }
    void close() override {
        ++closes; cancel();
        cellPort.cancel();
        link = cleanupFails ? pico_rails::BusState::CleanupFault : pico_rails::BusState::Stopped;
    }
    pico_rails::BusState state() const override { return link; }
    bq76905::I2c& cells() override { return cellPort; }
    bool cellsIdle() const override { return !cellDevice.active && !cellDraining; }
    uint64_t nowMicros() const override {
        const uint32_t current = millis();
        elapsedUs_ += uint64_t(uint32_t(current - previousMs_)) * 1000;
        previousMs_ = current;
        return elapsedUs_;
    }
    bool start(uint8_t address, const uint8_t* tx, std::size_t txSize,
               uint8_t* rx, std::size_t rxSize) override {
        assert(!active_ && cellsIdle() && tx && ((txSize == 1 && rx && rxSize == 2) ||
                                (txSize == 3 && rxSize == 0)));
        ++starts; addresses.push_back(address);
        request_.assign(tx, tx + txSize); destination_ = rx; receiveSize_ = rxSize;
        active_ = true;
        return true;
    }
    ina3221::IoResult poll() override {
        assert(active_); ++polls;
        if (pending) return {ina3221::IoStatus::Pending, 0};
        active_ = false;
        if (!present || nackNext) {
            nackNext = false; destination_ = nullptr;
            return {ina3221::IoStatus::Nack, 0};
        }
        if (shortNext) {
            shortNext = false; destination_ = nullptr;
            return {ina3221::IoStatus::Complete, receiveSize_ ? receiveSize_ - 1 : 1};
        }
        if (request_.size() == 3) {
            assert(request_[0] == 0);
            config_ = uint16_t(request_[1]) << 8 | request_[2];
            if (config_ == 0x7122) { triggeredAt_ = nowMicros(); triggered_ = true; }
        } else {
            uint16_t value = 0;
            switch (request_[0]) {
            case 0xFE: value = 0x5449; break;
            case 0xFF: value = 0x3220; break;
            case 0x00: value = corruptConfig ? 0x7127 : config_; break;
            case 0x0F:
                if (triggered_ && nowMicros() - triggeredAt_ >= 3630) { value = 1; triggered_ = false; }
                break;
            case 0x02: value = voltages[0]; break;
            case 0x04: value = voltages[1]; break;
            case 0x06: value = voltages[2]; break;
            default: assert(false);
            }
            destination_[0] = value >> 8; destination_[1] = value;
        }
        destination_ = nullptr;
        return {ina3221::IoStatus::Complete, receiveSize_};
    }
    void cancel() override {
        if (active_) ++cancels;
        active_ = false; destination_ = nullptr;
    }
    bool active() const { return active_; }
private:
    mutable uint32_t previousMs_ = 0;
    mutable uint64_t elapsedUs_ = 0;
    uint64_t triggeredAt_ = 0;
    bool active_ = false, triggered_ = false;
    uint16_t config_ = 0x7127;
    uint8_t* destination_ = nullptr;
    std::size_t receiveSize_ = 0;
    std::vector<uint8_t> request_;
};
inline Bus bus;
}
namespace pico_rails {
inline Service& diagnostics() {
    static Service service(pico_rail_mock::bus);
    return service;
}
}
