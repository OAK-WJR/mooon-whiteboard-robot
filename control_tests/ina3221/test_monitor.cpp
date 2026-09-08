#include "ina3221_monitor.h"

#include <array>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

using ina3221::IoResult;
using ina3221::IoStatus;
using ina3221::Monitor;

#define CHECK(condition) do { if (!(condition)) { \
    std::cerr << __func__ << ':' << __LINE__ << ": " #condition "\n"; std::abort(); \
} } while (false)

struct TestClock : ina3221::Clock {
    uint64_t now = 0;
    uint64_t nowMicros() const override { return now; }
};

// Register-behavior model, not a second copy of the driver's state machine.
// Models MSB-first transfers, retained mode, clear-on-read CVRF, configuration
// writes, triggered conversion completion and a stable single-shot result bank.
struct ModelBus : ina3221::I2c {
    struct Transaction {
        uint8_t address;
        std::vector<uint8_t> tx;
        std::size_t rxSize;
    };
    explicit ModelBus(TestClock& time) : clock(time) {
        reg[0x00] = 0x7127;
        reg[0xFE] = 0x5449;
        reg[0xFF] = 0x3220;
    }
    TestClock& clock;
    std::array<uint16_t, 256> reg{};
    std::array<uint16_t, 3> nextBus{5000, 5008, 6000};
    std::vector<Transaction> trace;
    std::function<void(const Transaction&)> afterComplete;
    uint8_t* destination = nullptr;
    uint64_t started = 0;
    uint64_t conversionAt = 0;
    uint64_t conversionDelay = 3670;
    uint64_t startDelay = 0;
    uint64_t pollDelay = 0;
    bool active = false;
    bool converting = false;
    bool rejectStart = false;
    bool holdPending = false;
    bool neverReady = false;
    std::size_t failAt = 0;
    IoStatus failure = IoStatus::Nack;
    std::size_t shortAt = 0;
    unsigned starts = 0;
    unsigned polls = 0;
    unsigned cancels = 0;

    bool start(uint8_t address, const uint8_t* tx, std::size_t txSize,
               uint8_t* rx, std::size_t rxSize) override {
        ++starts;
        clock.now += startDelay;
        if (rejectStart) return false;
        CHECK(!active);
        CHECK((txSize == 1 && rxSize == 2) || (txSize == 3 && rxSize == 0));
        trace.push_back({address, std::vector<uint8_t>(tx, tx + txSize), rxSize});
        destination = rx;
        started = clock.now;
        active = true;
        return true;
    }
    IoResult poll() override {
        ++polls;
        CHECK(active);
        clock.now += pollDelay;
        if (holdPending || clock.now - started < 100) return {IoStatus::Pending, 0};
        active = false;
        if (trace.size() == failAt) return {failure, 0};
        const auto& t = trace.back();
        const uint8_t address = t.tx[0];
        if (t.rxSize == 0) {
            CHECK(address == 0); // No alert/shunt/limit register writes authorized.
            reg[0] = uint16_t((uint16_t(t.tx[1]) << 8) | t.tx[2]);
            converting = (reg[0] & 7) == 2;
            if (converting) {
                reg[0x0F] &= uint16_t(~1U);
                conversionAt = clock.now;
            }
        } else {
            if (converting && !neverReady && clock.now - conversionAt >= conversionDelay) {
                for (std::size_t i = 0; i < 3; ++i) reg[2 + i * 2] = nextBus[i];
                reg[0x0F] |= 1;
                converting = false;
            }
            destination[0] = uint8_t(reg[address] >> 8);
            destination[1] = uint8_t(reg[address]);
            if (address == 0x0F) reg[address] &= uint16_t(~1U);
        }
        if (afterComplete) afterComplete(t);
        return {IoStatus::Complete, trace.size() == shortAt ? 1 : t.rxSize};
    }
    void cancel() override {
        ++cancels;
        active = false;
        destination = nullptr;
    }
};

struct Fixture {
    TestClock clock;
    ModelBus bus{clock};
    Monitor monitor{bus, clock};
    void tick(uint64_t advance = 200) {
        const unsigned calls = bus.starts + bus.polls;
        monitor.tick();
        CHECK(bus.starts + bus.polls - calls <= 1);
        clock.now += advance;
    }
    void finish() {
        for (unsigned i = 0; i < 500 && monitor.state() == Monitor::State::Busy; ++i) tick();
        CHECK(monitor.state() != Monitor::State::Busy);
    }
    void init() {
        CHECK(monitor.begin());
        CHECK(bus.starts == 0);
        finish();
        CHECK(monitor.state() == Monitor::State::Idle);
        CHECK(!monitor.latest());
    }
    void scan() { CHECK(monitor.request()); finish(); }
    void fault(Monitor::Error expected) {
        CHECK(monitor.state() == Monitor::State::Fault);
        CHECK(monitor.error() == expected);
        CHECK(!monitor.latest());
        CHECK(!monitor.request());
        const auto before = bus.trace.size();
        tick();
        CHECK(bus.trace.size() == before); // Fault is sticky; no silent retry.
    }
};

void success_and_actual_wire_protocol() {
    Fixture f;
    CHECK(!f.monitor.request());
    f.init();
    f.scan();
    const auto sample = f.monitor.latest();
    CHECK(sample);
    CHECK((sample->millivolts == std::array<uint32_t, 3>{5000, 5008, 6000}));
    CHECK(sample->millivolts[std::size_t(ina3221::Rail::PicoUsbVbus)] == 5008);
    CHECK(sample->sequence == 1);
    CHECK(sample->readyObservedAtUs > sample->triggerRequestedAtUs);
    CHECK((f.bus.trace[2].tx == std::vector<uint8_t>{0, 0x71, 0x20}));
    CHECK((f.bus.trace[5].tx == std::vector<uint8_t>{0, 0x71, 0x22}));
    std::vector<uint8_t> busReads;
    for (const auto& t : f.bus.trace) {
        CHECK(t.address == 0x40);
        CHECK(t.tx[0] != 1 && t.tx[0] != 3 && t.tx[0] != 5); // No shunt telemetry.
        if (t.tx[0] == 2 || t.tx[0] == 4 || t.tx[0] == 6) busReads.push_back(t.tx[0]);
    }
    CHECK((busReads == std::vector<uint8_t>{2, 4, 6}));
    CHECK(f.bus.reg[0] == 0x7122); // Internal power-down does not rewrite MODE.
}

void stale_ready_and_atomic_publication() {
    Fixture f;
    f.bus.reg[0x0F] = 1;
    f.bus.reg[2] = 16000; // Retained old conversion must not be published.
    f.init();
    CHECK(f.bus.reg[0x0F] == 1); // Idle configuration alone does not clear CVRF.
    f.bus.afterComplete = [&](const ModelBus::Transaction&) { CHECK(!f.monitor.latest()); };
    f.scan();
    CHECK(f.monitor.latest()->millivolts[0] == 5000);
    const uint32_t first = f.monitor.latest()->sequence;
    f.bus.nextBus = {0, 5008, 0};
    CHECK(f.monitor.request());
    CHECK(!f.monitor.latest());
    CHECK(!f.monitor.request());
    f.finish();
    CHECK(f.monitor.latest()->sequence == first + 1);
    CHECK((f.monitor.latest()->millivolts == std::array<uint32_t, 3>{0, 5008, 0}));
}

void stale_flag_without_new_conversion_times_out() {
    Fixture f;
    f.init();
    f.bus.reg[0x0F] = 1;
    f.bus.neverReady = true;
    f.scan();
    f.fault(Monitor::Error::ConversionTimeout);
    for (const auto& t : f.bus.trace) CHECK(t.tx[0] != 2 && t.tx[0] != 4 && t.tx[0] != 6);
}

void identity_and_address_gates() {
    for (uint8_t address : {uint8_t(0x3F), uint8_t(0x44), uint8_t(0x80)}) {
        Fixture f;
        CHECK(!f.monitor.begin(address));
        f.fault(Monitor::Error::InvalidAddress);
        CHECK(f.bus.trace.empty());
    }
    for (uint8_t address = 0x40; address <= 0x43; ++address) {
        Fixture f;
        CHECK(f.monitor.begin(address)); f.finish();
        CHECK(f.monitor.state() == Monitor::State::Idle);
        for (const auto& t : f.bus.trace) CHECK(t.address == address);
    }
    for (unsigned die = 0; die < 2; ++die) {
        Fixture f;
        f.bus.reg[die ? 0xFF : 0xFE] = 0;
        CHECK(f.monitor.begin()); f.finish();
        f.fault(die ? Monitor::Error::WrongDie : Monitor::Error::WrongManufacturer);
        for (const auto& t : f.bus.trace) CHECK(t.rxSize == 2); // No writes to unknown chip.
    }
}

void configuration_loss_at_each_readout_boundary() {
    // Drop to POR after idle/trigger write, CVRF completion, or any bus read.
    for (unsigned point = 0; point < 6; ++point) {
        Fixture f;
        f.bus.afterComplete = [&](const ModelBus::Transaction& t) {
            const bool idle = t.tx.size() == 3 && t.tx[2] == 0x20;
            const bool trigger = t.tx.size() == 3 && t.tx[2] == 0x22;
            const bool ready = t.tx[0] == 0x0F && !f.bus.converting && f.bus.reg[2] != 0;
            if ((point == 0 && idle) || (point == 1 && trigger) || (point == 2 && ready) ||
                (point >= 3 && t.tx[0] == 2 + (point - 3) * 2)) f.bus.reg[0] = 0x7127;
        };
        CHECK(f.monitor.begin()); f.finish();
        if (f.monitor.state() == Monitor::State::Idle) f.scan();
        f.fault(Monitor::Error::ConfigurationChanged);
    }
}

void voltage_encoding_and_limits() {
    for (uint16_t valid : {uint16_t(0), uint16_t(8), uint16_t(26000)}) {
        Fixture f; f.init();
        f.bus.nextBus = {valid, valid, valid}; f.scan();
        const auto sample = f.monitor.latest();
        CHECK(sample);
        for (auto mv : sample->millivolts) CHECK(mv == valid);
    }
    for (uint16_t invalid : {uint16_t(1), uint16_t(5001), uint16_t(26008),
                             uint16_t(0x7FF8), uint16_t(0x8000), uint16_t(0xFFFF)}) {
        for (unsigned channel = 0; channel < 3; ++channel) {
            Fixture f; f.init(); f.bus.nextBus[channel] = invalid; f.scan();
            f.fault(Monitor::Error::InvalidVoltage);
        }
    }
}

void transfer_failures_at_every_operation() {
    Fixture baseline; baseline.init(); baseline.scan();
    const auto transfers = baseline.bus.trace.size();
    for (IoStatus status : {IoStatus::Nack, IoStatus::BusError}) {
        for (std::size_t i = 1; i <= transfers; ++i) {
            Fixture f; f.bus.failAt = i; f.bus.failure = status;
            CHECK(f.monitor.begin()); f.finish();
            if (f.monitor.state() == Monitor::State::Idle) f.scan();
            f.fault(status == IoStatus::Nack ? Monitor::Error::Nack : Monitor::Error::BusError);
        }
    }
    for (std::size_t i = 1; i <= transfers; ++i) {
        Fixture f; f.bus.shortAt = i;
        CHECK(f.monitor.begin()); f.finish();
        if (f.monitor.state() == Monitor::State::Idle) f.scan();
        f.fault(Monitor::Error::ShortRead);
    }
}

void pending_busy_and_late_backend_results() {
    { Fixture f; CHECK(f.monitor.begin()); f.bus.rejectStart = true; f.finish();
      f.fault(Monitor::Error::BusBusyTimeout); CHECK(f.bus.cancels == 0); }
    { Fixture f; CHECK(f.monitor.begin()); f.bus.holdPending = true; f.finish();
      f.fault(Monitor::Error::TransferTimeout); CHECK(f.bus.cancels == 1); }
    { Fixture f; CHECK(f.monitor.begin()); f.bus.startDelay = Monitor::ioTimeoutUs;
      f.tick(); f.fault(Monitor::Error::TransferTimeout); CHECK(f.bus.cancels == 1); }
    { Fixture f; CHECK(f.monitor.begin()); f.bus.startDelay = Monitor::ioTimeoutUs;
      f.bus.rejectStart = true; f.tick(); f.fault(Monitor::Error::BusBusyTimeout);
      CHECK(f.bus.cancels == 0); }
    { Fixture f; CHECK(f.monitor.begin()); f.tick(); f.bus.pollDelay = Monitor::ioTimeoutUs;
      f.tick(); f.fault(Monitor::Error::TransferTimeout); CHECK(f.bus.cancels == 1); }
}

void lost_progress_during_init_and_after_conversion() {
    { Fixture f; CHECK(f.monitor.begin()); f.tick();
      f.clock.now += Monitor::acquisitionTimeoutUs; f.tick();
      f.fault(Monitor::Error::AcquisitionTimeout); CHECK(f.bus.cancels == 1); }
    { Fixture f; f.init();
      f.bus.afterComplete = [&](const ModelBus::Transaction& t) {
          if (t.tx[0] == 6) f.clock.now += Monitor::acquisitionTimeoutUs;
      };
      f.scan(); f.fault(Monitor::Error::AcquisitionTimeout); }
    { Fixture f; f.init();
      f.bus.afterComplete = [&](const ModelBus::Transaction& t) {
          if (t.tx[0] == 0x0F && !f.bus.converting && f.bus.reg[2] != 0)
              f.clock.now += Monitor::conversionTimeoutUs;
      };
      f.scan(); f.fault(Monitor::Error::ConversionTimeout); }
}

void lifetime_64bit_clock_and_explicit_power_loss() {
    Fixture f;
    f.clock.now = uint64_t(0xFFFFFFFF) - 2000;
    f.init(); f.scan();
    CHECK(f.monitor.latest());
    const auto sample = *f.monitor.latest();
    f.clock.now = sample.triggerRequestedAtUs + Monitor::sampleLifetimeUs - 1;
    CHECK(f.monitor.latest());
    ++f.clock.now;
    CHECK(!f.monitor.latest()); CHECK(f.monitor.state() == Monitor::State::Expired);
    f.clock.now = sample.triggerRequestedAtUs + (uint64_t(1) << 32);
    CHECK(!f.monitor.latest()); // A 32-bit micros rollover cannot revive old data.
    f.scan(); CHECK(f.monitor.latest());
    f.monitor.invalidate();
    CHECK(!f.monitor.latest()); CHECK(!f.monitor.request());
    CHECK(f.monitor.state() == Monitor::State::Uninitialized);
}

void cancellation_and_explicit_recovery() {
    Fixture f;
    CHECK(f.monitor.begin()); f.tick();
    f.monitor.invalidate(); CHECK(f.bus.cancels == 1); CHECK(!f.bus.active);
    const auto n = f.bus.trace.size(); f.tick(); CHECK(f.bus.trace.size() == n);
    CHECK(f.monitor.begin()); f.finish(); f.scan(); CHECK(f.monitor.latest());
    CHECK(f.monitor.request()); f.tick();
    CHECK(f.monitor.begin()); CHECK(f.bus.cancels == 2);
    CHECK(!f.monitor.latest()); f.finish(); f.scan(); CHECK(f.monitor.latest());
    f.bus.nextBus[0] = 0xFFFF; f.scan(); f.fault(Monitor::Error::InvalidVoltage);
    f.bus.nextBus[0] = 5000; CHECK(f.monitor.begin()); f.finish(); f.scan();
    CHECK(f.monitor.latest());
    { Monitor temporary(f.bus, f.clock); CHECK(temporary.begin()); temporary.tick(); }
    CHECK(f.bus.cancels == 3); CHECK(!f.bus.active);
}

int main() {
    const std::pair<const char*, void(*)()> tests[] = {
        {"bus-only wire protocol and channel mapping", success_and_actual_wire_protocol},
        {"old CVRF, generation and atomic publication", stale_ready_and_atomic_publication},
        {"no new conversion times out", stale_flag_without_new_conversion_times_out},
        {"identity and address gates", identity_and_address_gates},
        {"configuration loss at six boundaries", configuration_loss_at_each_readout_boundary},
        {"zero, limits, sign, reserved and saturation", voltage_encoding_and_limits},
        {"NACK, bus error and short transfer at every operation", transfer_failures_at_every_operation},
        {"pending, contention and late backend returns", pending_busy_and_late_backend_results},
        {"lost progress and late conversion completion", lost_progress_during_init_and_after_conversion},
        {"sample expiry, 64-bit time and supply loss", lifetime_64bit_clock_and_explicit_power_loss},
        {"owned cancellation and explicit reprobe", cancellation_and_explicit_recovery},
    };
    for (const auto& test : tests) { test.second(); std::cout << "PASS " << test.first << '\n'; }
    std::cout << "PASS " << std::size(tests) << " runtime test groups\n";
}
