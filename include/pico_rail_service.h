#pragma once
#include "ina3221_monitor.h"
#include "bq76905_monitor.h"

namespace pico_rails {

enum class BusState { Stopped, Starting, Ready, Fault, Stopping, CleanupFault };
// Only the worker/platform boundary is substituted in native runtime tests.
// All methods are control-task-only and nonblocking. close() requests cleanup;
// Stopped and CleanupFault distinguish its eventual result.
class Bus : public ina3221::I2c, public ina3221::Clock {
public:
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual BusState state() const = 0;
    virtual bq76905::I2c& cells() = 0;
    virtual bool cellsIdle() const = 0;
};

struct Status {
    const char* state;
    const char* error;
    const char* bus;
    std::optional<ina3221::Snapshot> sample;
    uint32_t ageUs = 0;
    uint32_t spanUs = 0;
};

struct CellStatus {
    const char* state;
    const char* error;
    bool wakeRequested;
    bq76905::Diagnostics diagnostics;
    bq76905::Snapshot sample;
    uint32_t ageMs;
};

class Service {
public:
    static constexpr uint64_t startupTimeoutUs = 150000;
    static constexpr uint64_t samplePeriodUs = 40000;
    static constexpr uint64_t wakePulseUs = 100000, wakeReleaseUs = 100000;
    static constexpr uint64_t wakeLateUs = 150000;
    using WakeControl = bool (*)(bool active);
    explicit Service(Bus& bus) : bus_(bus), monitor_(bus, bus), cellClock_(bus), cellMonitor_(bus.cells(), cellClock_) {}
    bool begin(WakeControl wake = nullptr); // Once per boot; no load enable.
    void tick();
    void stop(); // Invalidates immediately; worker cleanup completes separately.
    void wakeAborted(); // Caller released wake as part of an independent stop.
    Status status() const; // No I/O; never exposes a stale or partial generation.
    CellStatus cellStatus() const;
private:
    class CellClock final : public bq76905::Clock {
    public:
        explicit CellClock(const Bus& bus) : bus_(bus) {}
        uint32_t nowMs() const override { return static_cast<uint32_t>(bus_.nowMicros() / 1000); }
    private:
        const Bus& bus_;
    };
    enum class Phase { NotStarted, Starting, Initializing, Running, Fault, Stopped };
    enum class CellPhase { NotStarted, Pulse, ReleaseWait, Running, Fault, Stopped };
    Bus& bus_;
    ina3221::Monitor monitor_;
    CellClock cellClock_;
    bq76905::Monitor cellMonitor_;
    Phase phase_ = Phase::NotStarted;
    uint64_t startedAt_ = 0;
    const char* error_ = "none";
    WakeControl wake_ = nullptr;
    bool wakeRequested_ = false, wakeControlFault_ = false;
    CellPhase cellPhase_ = CellPhase::NotStarted;
    uint64_t wakeAt_ = 0;
    const char* cellError_ = "none";
    void fail(const char* error);
    void failCells(const char* error);
    void stopCells();
    void tickWake();
};

// Factory owns the backend, worker and service for the complete firmware life.
Service& diagnostics();
} // namespace pico_rails
