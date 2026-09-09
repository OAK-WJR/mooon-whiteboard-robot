#include "pico_i2c_transport.h"
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

using namespace pico_i2c;
#define CHECK(value) do { if (!(value)) { std::cerr << __func__ << ':' << __LINE__ << ": " #value "\n"; std::abort(); } } while (false)

template<class F> void until(F condition) {
    const auto limit = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!condition()) {
        CHECK(std::chrono::steady_clock::now() < limit);
        std::this_thread::yield();
    }
}
struct TestClock : Clock {
    std::atomic<uint64_t> now{1000};
    uint64_t nowUs() const override { return now.load(); }
};
// A deliberately controllable blocking worker boundary, not an ESP-IDF model.
// No test claims these calls establish physical ACKs, STARTs, or GPIO behavior.
struct TestBackend : Backend {
    struct Call { uint8_t address; std::vector<uint8_t> tx; std::size_t rxSize; uint64_t deadline; };
    std::mutex mutex;
    std::condition_variable changed;
    std::vector<Call> calls;
    std::atomic<unsigned> entries{0}, exits{0}, configures{0}, shutdowns{0};
    bool release = false;
    bool holdConfigure = false;
    bool configured = false;
    Result configuredResult{Code::Complete};
    Result next{Code::Complete, 0, 0, CountEvidence::BackendCompletion};
    bool useRequestedCounts = true;
    std::thread::id control = std::this_thread::get_id();

    Result configure(const BusConfig& config, uint64_t deadline) override {
        CHECK(std::this_thread::get_id() != control);
        CHECK(config.sda == 1 && config.scl == 2 && config.frequencyHz == 100000);
        CHECK(deadline > 0);
        std::unique_lock<std::mutex> guard(mutex);
        ++configures;
        changed.wait(guard, [&] { return !holdConfigure; });
        configured = true;
        return configuredResult;
    }
    Result transfer(const Transfer& request, uint64_t deadline) override {
        CHECK(std::this_thread::get_id() != control);
        CHECK(configured);
        std::unique_lock<std::mutex> guard(mutex);
        CHECK(entries == exits); // Never two physical transfers, even on two clients.
        calls.push_back({request.address, {request.tx, request.tx + request.txSize}, request.rxSize, deadline});
        ++entries;
        changed.wait(guard, [&] { return release; });
        release = false;
        // Touch TX/RX as late as possible to detect early storage reuse/free.
        CHECK(calls.back().tx == std::vector<uint8_t>(request.tx, request.tx + request.txSize));
        for (std::size_t i = 0; i < request.rxSize; ++i) request.rx[i] = uint8_t(i ^ request.address);
        Result result = next;
        if (useRequestedCounts) {
            result.transmitted = request.txSize;
            result.received = request.rxSize;
        }
        ++exits;
        return result;
    }
    void shutdown() override {
        CHECK(std::this_thread::get_id() != control);
        CHECK(entries == exits);
        ++shutdowns;
    }
    void finish(Result result = {Code::Complete, 0, 0, CountEvidence::BackendCompletion}, bool full = true) {
        std::lock_guard<std::mutex> guard(mutex);
        next = result; useRequestedCounts = full; release = true; changed.notify_all();
    }
};
struct Fixture {
    TestClock clock;
    TestBackend backend;
    Transport transport{backend, clock};
    void begin() {
        CHECK(transport.begin());
        until([&] { return transport.state() == Transport::State::Ready; });
    }
    void waitEntry(unsigned count) { until([&] { return backend.entries.load() >= count; }); }
    ~Fixture() {
        transport.requestStop();
        { std::lock_guard<std::mutex> guard(backend.mutex); backend.holdConfigure = false; backend.changed.notify_all(); }
        backend.finish();
        until([&] { return transport.state() == Transport::State::Stopped; });
        task_mock::joinAll();
    }
    bq76905::BusResult bqDone(uint8_t* rx, std::size_t capacity) {
        bq76905::BusResult result{};
        until([&] { result = transport.bq().poll(rx, capacity); return result.state != bq76905::BusState::Pending; });
        return result;
    }
    ina3221::IoResult inaDone() {
        ina3221::IoResult result{};
        until([&] { result = transport.ina().poll(); return result.status != ina3221::IoStatus::Pending; });
        return result;
    }
};

void maximum_frames_owned_copy_and_delivery() {
    Fixture f; f.begin();
    std::array<uint8_t, maxTx> tx;
    std::array<uint8_t, maxRx + 2> rx;
    for (unsigned i = 0; i < tx.size(); ++i) tx[i] = uint8_t(i);
    rx.fill(0xCC);
    bq76905::I2c& bq = f.transport.bq();
    CHECK(bq.start(0x08, tx.data(), tx.size(), maxRx));
    tx.fill(0xEE); // Caller TX may disappear immediately after successful start.
    f.waitEntry(1);
    CHECK(bq.poll(rx.data() + 1, maxRx).state == bq76905::BusState::Pending);
    CHECK(rx.front() == 0xCC && rx[1] == 0xCC && rx.back() == 0xCC);
    f.backend.finish();
    auto done = f.bqDone(rx.data() + 1, maxRx);
    CHECK(done.state == bq76905::BusState::Complete && done.transmitted == 69 && done.received == 68);
    for (unsigned i = 0; i < maxRx; ++i) CHECK(rx[i + 1] == uint8_t(i ^ 0x08));
    CHECK(rx.front() == 0xCC && rx.back() == 0xCC);
    CHECK(f.backend.calls[0].tx.size() == 69 && f.backend.calls[0].tx[0] == 0 && f.backend.calls[0].tx[68] == 68);
    CHECK(f.backend.calls[0].rxSize == 68); // One combined descriptor, no Wire-32 truncation.
    CHECK(f.transport.bq().lastResult().evidence == CountEvidence::BackendCompletion);
}

void cancel_active_caller_lifetime_and_storage_quarantine() {
    Fixture f; f.begin(); uint8_t pointer = 0x02;
    {
        std::array<uint8_t, 2> oldRx{0xCC, 0xCC};
        CHECK(f.transport.ina().start(0x40, &pointer, 1, oldRx.data(), oldRx.size()));
        f.waitEntry(1);
        auto before = std::chrono::steady_clock::now();
        CHECK(f.transport.ina().poll().status == ina3221::IoStatus::Pending);
        f.transport.ina().cancel();
        CHECK(std::chrono::steady_clock::now() - before < std::chrono::milliseconds(100));
        CHECK(f.transport.ina().lastResult().code == Code::Cancelled);
        CHECK(oldRx[0] == 0xCC && oldRx[1] == 0xCC);
    } // A late physical completion must not touch this former caller buffer (ASan).
    uint8_t newRx[2] = {0xDD, 0xDD};
    CHECK(!f.transport.ina().start(0x40, &pointer, 1, newRx, 2));
    uint8_t bqTx[] = {0x3E, 0x01, 0x00};
    CHECK(f.transport.bq().start(0x08, bqTx, 3, 0)); // May queue, cannot physically overlap.
    CHECK(f.backend.entries == 1 && f.backend.exits == 0);
    f.backend.finish(); f.waitEntry(2);
    CHECK(f.transport.ina().start(0x41, &pointer, 1, newRx, 2));
    f.backend.finish(); CHECK(f.bqDone(nullptr, 0).state == bq76905::BusState::Complete);
    f.waitEntry(3); CHECK(newRx[0] == 0xDD);
    f.backend.finish(); CHECK(f.inaDone().received == 2);
    CHECK(newRx[0] == 0x41 && newRx[1] == 0x40);
}

void queued_cancel_does_not_run_and_round_robin_is_fair() {
    Fixture f; f.begin(); uint8_t tx = 0, rx[2]{};
    CHECK(f.transport.bq().start(0x08, &tx, 1, 0)); f.waitEntry(1);
    CHECK(f.transport.ina().start(0x40, &tx, 1, rx, 2));
    f.transport.ina().cancel();
    CHECK(f.transport.ina().start(0x41, &tx, 1, rx, 2));
    f.backend.finish(); CHECK(f.bqDone(nullptr, 0).state == bq76905::BusState::Complete);
    CHECK(f.transport.bq().start(0x08, &tx, 1, 0));
    f.waitEntry(2); CHECK(f.backend.calls[1].address == 0x41);
    f.backend.finish(); CHECK(f.inaDone().status == ina3221::IoStatus::Complete);
    f.waitEntry(3); CHECK(f.backend.calls[2].address == 0x08);
    // A second cancellation after completion must not cancel the other client.
    f.transport.ina().cancel(); f.backend.finish();
    CHECK(f.bqDone(nullptr, 0).state == bq76905::BusState::Complete);
}

void errors_short_counts_and_unavailable_counts() {
    for (Code code : {Code::Nak, Code::Timeout, Code::BusError}) {
        Fixture f; f.begin(); uint8_t tx = 0, rx[2] = {0xCC, 0xCC};
        CHECK(f.transport.bq().start(0x08, &tx, 1, 2)); f.waitEntry(1);
        f.backend.finish({code, 0, 0, CountEvidence::Unknown, -42}, false);
        const auto r = f.bqDone(rx, 2);
        CHECK(r.state == (code == Code::Nak ? bq76905::BusState::Nak : bq76905::BusState::BusError));
        CHECK(rx[0] == 0xCC && f.transport.bq().lastResult().evidence == CountEvidence::Unknown);
        CHECK(f.transport.bq().lastResult().backendError == -42);
        CHECK(f.backend.entries == 1); // No automatic retry.
        CHECK(f.transport.ina().start(0x40, &tx, 1, rx, 2)); f.waitEntry(2);
        f.backend.finish({code, 0, 0, CountEvidence::Unknown, -43}, false);
        CHECK(f.inaDone().status == (code == Code::Nak ? ina3221::IoStatus::Nack : ina3221::IoStatus::BusError));
        CHECK(rx[0] == 0xCC);
    }
    { Fixture f; f.begin(); uint8_t tx = 0, rx[2] = {0xCC, 0xCC};
      CHECK(f.transport.bq().start(0x08, &tx, 1, 2)); f.waitEntry(1);
      f.backend.finish({Code::Complete, 1, 1, CountEvidence::BackendCompletion}, false);
      const auto r = f.bqDone(rx, 2);
      CHECK(r.state == bq76905::BusState::Complete && r.received == 1 && rx[1] == 0xCC);
      CHECK(f.transport.bq().lastResult().code == Code::ShortTransfer); }
    { Fixture f; f.begin(); uint8_t tx = 0, rx[2] = {0xCC, 0xCC};
      CHECK(f.transport.ina().start(0x40, &tx, 1, rx, 2)); f.waitEntry(1);
      f.backend.finish({Code::Complete, 1, 1, CountEvidence::BackendCompletion}, false);
      const auto r = f.inaDone(); CHECK(r.status == ina3221::IoStatus::Complete && r.received == 1 && rx[1] == 0xCC); }
    { Fixture f; f.begin(); uint8_t tx[3] = {0, 0x71, 0x22};
      CHECK(f.transport.ina().start(0x40, tx, 3, nullptr, 0)); f.waitEntry(1);
      f.backend.finish({Code::Complete, 2, 0, CountEvidence::BackendCompletion}, false);
      CHECK(f.inaDone().status == ina3221::IoStatus::BusError);
      CHECK(f.transport.ina().lastResult().code == Code::ShortTransfer); }
    for (auto invalid : {Result{Code::Complete, 1, 2, CountEvidence::Unknown},
                         Result{Code::Complete, 1, 3, CountEvidence::BackendCompletion},
                         Result{Code::Complete, 2, 2, CountEvidence::BackendCompletion},
                         Result{Code::ShortTransfer, 1, 2, CountEvidence::BackendCompletion},
                         Result{Code::Pending}}) {
        Fixture f; f.begin(); uint8_t tx = 0, rx[2] = {0xCC, 0xCC};
        CHECK(f.transport.bq().start(0x08, &tx, 1, 2)); f.waitEntry(1);
        f.backend.finish(invalid, false);
        CHECK(f.bqDone(rx, 2).state == bq76905::BusState::BusError && rx[0] == 0xCC);
    }
}

void timeout_in_queue_active_and_late_delivery() {
    Fixture f; f.begin(); uint8_t tx = 0, rx[2] = {0xCC, 0xCC};
    CHECK(f.transport.bq().start(0x08, &tx, 1, 0)); f.waitEntry(1);
    CHECK(f.transport.ina().start(0x40, &tx, 1, rx, 2));
    f.clock.now += Transport::inaDeadlineUs;
    CHECK(f.transport.ina().poll().status == ina3221::IoStatus::BusError);
    CHECK(f.transport.ina().lastResult().code == Code::Timeout);
    f.backend.finish(); CHECK(f.bqDone(nullptr, 0).state == bq76905::BusState::Complete);
    CHECK(f.backend.entries == 1); // Expired queued job never reached backend.
    CHECK(f.transport.ina().start(0x40, &tx, 1, rx, 2)); f.waitEntry(2);
    f.clock.now += Transport::inaDeadlineUs;
    CHECK(f.transport.ina().poll().status == ina3221::IoStatus::BusError);
    CHECK(!f.transport.ina().start(0x40, &tx, 1, rx, 2));
    f.backend.finish(); until([&] { return f.backend.exits == 2; });
    until([&] { return f.transport.ina().start(0x40, &tx, 1, rx, 2); });
    f.waitEntry(3); f.backend.finish(); until([&] { return f.backend.exits == 3; });
    f.clock.now += Transport::inaDeadlineUs;
    CHECK(f.inaDone().status == ina3221::IoStatus::BusError && rx[0] == 0xCC);
}

void invalid_arguments_capacity_and_busy_preserve_request() {
    Fixture f; f.begin(); uint8_t tx[70]{}, rx[2] = {0xCC, 0xCC};
    for (auto size : {std::size_t(0), std::size_t(70)}) {
        CHECK(f.transport.bq().start(0x08, tx, size, 0));
        CHECK(f.bqDone(nullptr, 0).state == bq76905::BusState::BusError);
    }
    CHECK(f.transport.bq().start(0x80, tx, 1, 0)); CHECK(f.bqDone(nullptr, 0).state == bq76905::BusState::BusError);
    CHECK(f.transport.bq().start(0x08, tx, 1, 69)); CHECK(f.bqDone(nullptr, 0).state == bq76905::BusState::BusError);
    CHECK(f.transport.ina().start(0x40, tx, 1, nullptr, 2)); CHECK(f.inaDone().status == ina3221::IoStatus::BusError);
    CHECK(f.backend.entries == 0);
    CHECK(f.transport.bq().start(0x08, tx, 1, 2)); f.waitEntry(1);
    CHECK(!f.transport.bq().start(0x09, tx, 1, 0));
    f.backend.finish(); CHECK(f.bqDone(rx, 1).state == bq76905::BusState::BusError);
    CHECK(rx[0] == 0xCC && f.transport.bq().lastResult().code == Code::BufferTooSmall);
    CHECK(f.transport.ina().start(0x40, tx, 1, rx, 2)); f.waitEntry(2);
    uint8_t wrongRx[2]{};
    CHECK(!f.transport.ina().start(0x41, tx, 1, wrongRx, 2));
    f.backend.finish(); CHECK(f.inaDone().status == ina3221::IoStatus::Complete);
    CHECK(rx[0] == 0x40 && wrongRx[0] == 0);
}

void lifecycle_failures_and_stop_during_setup_or_transfer() {
    { Fixture f; f.begin(); uint8_t tx=0;
      CHECK(f.transport.bq().idle());
      CHECK(f.transport.bq().start(0x08,&tx,1,0));CHECK(!f.transport.bq().idle());f.waitEntry(1);
      f.transport.bq().cancel();CHECK(!f.transport.bq().idle()); // Still physically active.
      f.backend.finish();until([&] { return f.transport.bq().idle(); }); }
    { Fixture f; task_mock::failCreate = true; CHECK(!f.transport.begin()); task_mock::failCreate = false;
      CHECK(f.backend.configures == 0); }
    { Fixture f; f.backend.configuredResult = {Code::BusError, 0, 0, CountEvidence::Unknown, -42};
      CHECK(f.transport.begin()); until([&] { return f.transport.state() == Transport::State::Fault; });
      uint8_t tx = 0; CHECK(f.transport.bq().start(0x08, &tx, 1, 0));
      CHECK(f.bqDone(nullptr, 0).state == bq76905::BusState::BusError);
      CHECK(f.backend.entries == 0 && f.backend.configures == 1); }
    { Fixture f; f.backend.holdConfigure = true; CHECK(f.transport.begin());
      until([&] { return f.backend.configures == 1; });
      f.clock.now += Transport::setupDeadlineUs;
      { std::lock_guard<std::mutex> guard(f.backend.mutex); f.backend.holdConfigure = false; f.backend.changed.notify_all(); }
      until([&] { return f.transport.state() == Transport::State::Fault; }); }
    { Fixture f; f.backend.holdConfigure = true; CHECK(f.transport.begin());
      until([&] { return f.backend.configures == 1; });
      f.transport.requestStop(); f.transport.requestStop();
      CHECK(f.transport.state() == Transport::State::Stopping); }
    { Fixture f; f.begin(); uint8_t tx = 0, rx[2] = {0xCC, 0xCC};
      CHECK(f.transport.ina().start(0x40, &tx, 1, rx, 2)); f.waitEntry(1);
      f.transport.requestStop(); f.transport.requestStop();
      CHECK(f.transport.state() == Transport::State::Stopping && f.backend.shutdowns == 0);
      f.backend.finish(); until([&] { return f.transport.state() == Transport::State::Stopped; });
      CHECK(rx[0] == 0xCC && f.backend.shutdowns == 1);
      f.transport.requestStop(); CHECK(f.transport.begin());
      until([&] { return f.transport.state() == Transport::State::Ready; }); }
}

void cancellation_completion_race_and_64bit_clock() {
    Fixture f;
    f.clock.now = uint64_t(0xFFFFFFFF) - 100;
    f.begin();
    for (unsigned i = 1; i <= 200; ++i) {
        uint8_t tx = uint8_t(i);
        std::array<uint8_t, 2> oldRx{0xCC, 0xCC};
        until([&] { return f.transport.ina().start(0x40, &tx, 1, oldRx.data(), 2); });
        f.waitEntry(i);
        f.backend.finish();
        if (i % 2 == 0) until([&] { return f.backend.exits == i; });
        // Covers both Active and the return/publication/cancel boundary.
        f.transport.ina().cancel();
        CHECK(oldRx[0] == 0xCC && oldRx[1] == 0xCC);
        f.clock.now += 1;
    }
    uint8_t tx = 2, rx[2]{};
    until([&] { return f.transport.ina().start(0x41, &tx, 1, rx, 2); });
    f.waitEntry(201); f.backend.finish();
    CHECK(f.inaDone().status == ina3221::IoStatus::Complete && rx[0] == 0x41);
    CHECK(f.clock.now > uint64_t(0xFFFFFFFF));
}

int main() {
    const std::pair<const char*, void(*)()> tests[] = {
        {"maximum combined descriptor, owned TX and bounded RX copy", maximum_frames_owned_copy_and_delivery},
        {"active cancellation, old caller lifetime and physical storage quarantine", cancel_active_caller_lifetime_and_storage_quarantine},
        {"queued cancellation and fair dual-client arbitration", queued_cancel_does_not_run_and_round_robin_is_fair},
        {"NAK/timeout/error, short counts and unknown-count rejection", errors_short_counts_and_unavailable_counts},
        {"queue/active/delivery deadlines", timeout_in_queue_active_and_late_delivery},
        {"argument bounds, insufficient RX and busy request preservation", invalid_arguments_capacity_and_busy_preserve_request},
        {"worker creation/setup failure and explicit draining stop", lifecycle_failures_and_stop_during_setup_or_transfer},
        {"200 cancellation/completion races and 64-bit clock boundary", cancellation_completion_race_and_64bit_clock},
    };
    for (const auto& test : tests) { test.second(); std::cout << "PASS " << test.first << '\n'; }
    std::cout << "PASS " << std::size(tests) << " transport/worker groups (no stock SDK hardware backend claim)\n";
}
