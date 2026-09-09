#pragma once

#include "bq76905_monitor.h"
#include "ina3221_monitor.h"
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace pico_i2c {

constexpr int sdaGpio = 1, sclGpio = 2;
constexpr uint32_t busFrequencyHz = 100000;
constexpr std::size_t maxTx = 69, maxRx = 68;

enum class Code {
    None, Pending, Complete, ShortTransfer, Nak, Timeout, BusError,
    Cancelled, InvalidArgument, BufferTooSmall, NotReady
};
// These are BUFFER completion counts, never a per-byte ACK measurement.
// Unknown on error must not be represented as a known zero-byte wire transfer.
enum class CountEvidence { Unknown, BackendCompletion };
struct Result {
    Code code = Code::None;
    std::size_t transmitted = 0;
    std::size_t received = 0;
    CountEvidence evidence = CountEvidence::Unknown;
    int backendError = 0;
};
struct BusConfig { int sda; int scl; uint32_t frequencyHz; };
struct Transfer {
    uint8_t address;
    const uint8_t* tx;
    std::size_t txSize;
    uint8_t* rx;
    std::size_t rxSize;
};

class Clock {
public:
    virtual ~Clock() = default;
    virtual uint64_t nowUs() const = 0; // Monotonic, NOT zero-extended micros().
};
class EspClock final : public Clock {
public:
    uint64_t nowUs() const override;
};

// Worker-only boundary. Calls must stop waiting/issuing protocol work at their
// absolute deadline and quiesce buffer access before returning. Cleanup/scheduler
// latency needs physical qualification; late completion is rejected. A read
// MUST be TX + repeated START + RX (last byte NACK) + STOP, with no intermediate
// STOP, implicit retry, automatic bus-clear clocks, or GPIO reassignment.
// Stock IDF5.5.4 cannot meet the recovery rule. EspIdfBackend uses the project-local
// adaptation; no stock driver is silently substituted. Runtime acceptance is open.
class Backend {
public:
    virtual ~Backend() = default;
    virtual Result configure(const BusConfig& config, uint64_t deadlineUs) = 0;
    virtual Result transfer(const Transfer& transfer, uint64_t deadlineUs) = 0;
    virtual void shutdown() = 0; // Only on explicit requestStop, after transfer returns.
};

class Transport {
public:
    enum class State { Stopped, Starting, Ready, Fault, Stopping };
    static constexpr uint32_t bqDeadlineUs = 25000;
    static constexpr uint32_t inaDeadlineUs = 4000; // Below INA driver's 5ms I/O budget.
    static constexpr uint32_t setupDeadlineUs = 100000;

    class BqPort final : public bq76905::I2c {
    public:
        bool start(uint8_t address, const uint8_t* tx, std::size_t txSize, std::size_t rxSize) override;
        bq76905::BusResult poll(uint8_t* rx, std::size_t capacity) override;
        void cancel() override;
        Result lastResult() const { return last_; }
        bool idle() const; // Includes cancelled physical work still draining.
    private:
        friend class Transport;
        explicit BqPort(Transport& owner) : owner_(owner) {}
        Transport& owner_;
        bool outstanding_ = false, local_ = false;
        Result last_;
    };
    class InaPort final : public ina3221::I2c {
    public:
        bool start(uint8_t address, const uint8_t* tx, std::size_t txSize,
                   uint8_t* rx, std::size_t rxSize) override;
        ina3221::IoResult poll() override;
        void cancel() override;
        Result lastResult() const { return last_; }
    private:
        friend class Transport;
        explicit InaPort(Transport& owner) : owner_(owner) {}
        Transport& owner_;
        bool outstanding_ = false, local_ = false;
        uint8_t* destination_ = nullptr; // Control task only; NEVER visible to worker.
        std::size_t capacity_ = 0;
        Result last_;
    };

    Transport(Backend& backend, Clock& clock) : backend_(backend), clock_(clock), bq_(*this), ina_(*this) {}
    ~Transport(); // Must requestStop and observe Stopped before destroying.
    Transport(const Transport&) = delete;
    Transport& operator=(const Transport&) = delete;
    bool begin(); // Creates worker; configure runs on worker, never on caller.
    void requestStop(); // Cancels both logical requests; never aborts hardware in place.
    State state() const { return state_.load(std::memory_order_acquire); }
    BqPort& bq() { return bq_; }
    const BqPort& bq() const { return bq_; }
    InaPort& ina() { return ina_; }

private:
    enum class Phase { Free, Queued, Active, Done };
    struct Slot {
        std::atomic<Phase> phase{Phase::Free};
        std::atomic<uint32_t> generation{0};
        uint32_t requestGeneration = 0;
        uint64_t deadline = 0;
        uint8_t address = 0;
        std::size_t txSize = 0, rxSize = 0;
        std::array<uint8_t, maxTx> tx{};
        std::array<uint8_t, maxRx> rx{};
        Result result;
    };
    // S3 GCC does not promise is_always_lock_free. IDF supplies word atomics
    // (including its short critical-section fallback); no backend call holds
    // such a section or a shared application mutex. Keep publication word-sized.
    static_assert(sizeof(Phase) <= sizeof(uint32_t) && sizeof(State) <= sizeof(uint32_t),
                  "Slot publication must remain word-sized");
    Backend& backend_;
    Clock& clock_;
    std::array<Slot, 2> slots_;
    std::atomic<State> state_{State::Stopped};
    std::atomic<bool> stop_{false};
    TaskHandle_t task_ = nullptr;
    unsigned next_ = 0;
    BqPort bq_;
    InaPort ina_;

    bool submit(unsigned client, uint8_t address, const uint8_t* tx, std::size_t txSize,
                std::size_t rxSize, bool& outstanding, bool& local, Result& result);
    Result collect(unsigned client, uint8_t* rx, std::size_t capacity);
    void cancel(unsigned client);
    static void workerEntry(void* self);
    void worker();
    bool serviceOne();
};

// All begin/start/poll/cancel/requestStop calls belong to ONE control task.
// Worker is the only concurrent caller. Ports and monitors must not outlive
// Transport, which in turn must not outlive its Backend/Clock or be destroyed
// while its worker may still be in configure/transfer/shutdown.
} // namespace pico_i2c
