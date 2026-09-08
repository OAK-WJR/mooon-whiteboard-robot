#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace ina3221 {

enum class IoStatus { Pending, Complete, Nack, BusError };
struct IoResult {
    IoStatus status;
    std::size_t received;
};

// An adapter must implement bounded, nonblocking calls. A register read is one
// pointer write + repeated START + two-byte read. Buffers remain owned by Monitor
// until poll completes or cancel returns. cancel must prevent later buffer writes.
// start(false) means busy: no request was accepted and nothing may be cancelled.
// cancel must also safely tolerate a just-completed request without cancelling a
// different client's transaction. The adapter and clock must outlive Monitor.
class I2c {
public:
    virtual ~I2c() = default;
    virtual bool start(uint8_t address, const uint8_t* tx, std::size_t txSize,
                       uint8_t* rx, std::size_t rxSize) = 0;
    virtual IoResult poll() = 0;
    virtual void cancel() = 0;
};

// Monotonic 64-bit microseconds; do not pass a zero-extended wrapping micros().
class Clock {
public:
    virtual ~Clock() = default;
    virtual uint64_t nowMicros() const = 0;
};

// Candidate wiring only; these are channel indices, NOT allocated MCU GPIOs.
enum class Rail : std::size_t { RawLogic5V = 0, PicoUsbVbus = 1, Vmot = 2 };
struct Snapshot {
    std::array<uint32_t, 3> millivolts;
    uint32_t sequence;
    // Conservative acquisition interval, not simultaneous channel sampling.
    uint64_t triggerRequestedAtUs;
    uint64_t readyObservedAtUs;
};

class Monitor {
public:
    enum class State { Uninitialized, Busy, Idle, Ready, Expired, Fault };
    enum class Error {
        None, InvalidAddress, WrongManufacturer, WrongDie, Nack, BusError,
        ShortRead, BusBusyTimeout, TransferTimeout, ConversionTimeout,
        AcquisitionTimeout, ConfigurationChanged, InvalidVoltage
    };

    // Policy budgets, not TI timing specifications. See docs/INA3221_MONITOR.md.
    static constexpr uint32_t ioTimeoutUs = 5000;
    static constexpr uint32_t conversionTimeoutUs = 10000;
    static constexpr uint32_t acquisitionTimeoutUs = 30000;
    static constexpr uint32_t sampleLifetimeUs = 50000;
    static constexpr uint32_t pollIntervalUs = 500;

    Monitor(I2c& bus, Clock& clock) : bus_(bus), clock_(clock) {}
    ~Monitor();
    Monitor(const Monitor&) = delete;
    Monitor& operator=(const Monitor&) = delete;

    // begin probes both IDs and verifies explicit power-down configuration.
    // It schedules work; only tick performs I/O. No address scan or device reset.
    bool begin(uint8_t address = 0x40);
    bool request(); // Only after initialization or a completed/expired sample.
    void tick();   // At most one start OR poll per call. No sleeps or busy waits.
    void invalidate(); // Caller must use on known supply loss / bus reconfiguration.
    State state() const;
    Error error() const { return error_; }
    std::optional<Snapshot> latest() const;

private:
    enum class Step {
        Manufacturer, Die, IdleWrite, IdleVerify, DrainReady, Trigger,
        TriggerVerify, WaitReady, Bus1, Bus2, Bus3, FinalVerify
    };
    static constexpr uint16_t idleConfiguration = 0x7120;
    static constexpr uint16_t triggeredConfiguration = 0x7122;

    void queue(Step step, uint8_t reg, bool write = false, uint16_t value = 0);
    void complete(uint16_t value);
    bool checkDeadlines(uint64_t now);
    void fail(Error error);
    void cancelOwned();
    static uint64_t elapsed(uint64_t now, uint64_t since) { return now - since; }

    I2c& bus_;
    Clock& clock_;
    State state_ = State::Uninitialized;
    Error error_ = Error::None;
    Step step_ = Step::Manufacturer;
    uint8_t address_ = 0x40;
    std::array<uint8_t, 3> tx_{};
    std::array<uint8_t, 2> rx_{};
    std::size_t txSize_ = 0;
    std::size_t rxSize_ = 0;
    bool active_ = false;
    bool converting_ = false;
    bool pollDelayed_ = false;
    uint64_t pollFrom_ = 0;
    uint64_t operationFrom_ = 0;
    uint64_t acquisitionFrom_ = 0;
    uint64_t triggerFrom_ = 0;
    uint64_t readyAt_ = 0;
    uint32_t sequence_ = 0;
    std::array<uint32_t, 3> staging_{};
    std::optional<Snapshot> sample_;
};

} // namespace ina3221
