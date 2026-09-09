#include "pico_i2c_transport.h"
#include <cstring>
#include <esp_timer.h>

namespace pico_i2c {

uint64_t EspClock::nowUs() const { return static_cast<uint64_t>(esp_timer_get_time()); }

Transport::~Transport() { configASSERT(state() == State::Stopped); }

bool Transport::begin() {
    if (state() != State::Stopped) return false;
    stop_.store(false, std::memory_order_relaxed);
    next_ = 0;
    state_.store(State::Starting, std::memory_order_release);
    // Keep configure/transfers/cleanup on the same core as the allocated ISR.
    // IDF stack depth is bytes. Same priority as the Arduino loop, not above it.
#if CONFIG_FREERTOS_UNICORE
    constexpr BaseType_t workerCore = 0;
#else
    constexpr BaseType_t workerCore = 1;
#endif
    if (xTaskCreatePinnedToCore(workerEntry, "pico_i2c", 4096, this, 1, &task_, workerCore) != pdPASS) {
        state_.store(State::Stopped, std::memory_order_release);
        return false;
    }
    return true;
}

void Transport::requestStop() {
    if (stop_.load(std::memory_order_acquire) || state() == State::Stopped) return;
    bq_.cancel();
    ina_.cancel();
    state_.store(State::Stopping, std::memory_order_release);
    stop_.store(true, std::memory_order_release);
    // Nothing may touch task_ after publishing stop: worker may delete itself.
    // Its finite idle wait observes stop without a notification/handle race.
}

bool Transport::submit(unsigned client, uint8_t address, const uint8_t* tx, std::size_t txSize,
                       std::size_t rxSize, bool& outstanding, bool& local, Result& result) {
    if (outstanding) return false; // Keep prior result/destination and ownership.
    const bool invalid = address < 0x08 || address > 0x77 || !tx || txSize == 0 ||
                         txSize > maxTx || rxSize > maxRx;
    if (invalid || state() != State::Ready) {
        // Accept a local error for poll; false is reserved for contention.
        outstanding = local = true;
        result = {invalid ? Code::InvalidArgument : Code::NotReady};
        return true;
    }
    Slot& slot = slots_[client];
    if (slot.phase.load(std::memory_order_seq_cst) != Phase::Free) return false;
    slot.requestGeneration = slot.generation.fetch_add(1, std::memory_order_seq_cst) + 1;
    slot.address = address;
    slot.txSize = txSize;
    slot.rxSize = rxSize;
    slot.deadline = clock_.nowUs() + (client == 0 ? bqDeadlineUs : inaDeadlineUs);
    std::memcpy(slot.tx.data(), tx, txSize); // Never retain caller TX.
    result = {Code::Pending};
    local = false;
    outstanding = true;
    slot.phase.store(Phase::Queued, std::memory_order_seq_cst);
    xTaskNotifyGive(task_);
    return true;
}

Result Transport::collect(unsigned client, uint8_t* rx, std::size_t capacity) {
    Slot& slot = slots_[client];
    if (slot.phase.load(std::memory_order_seq_cst) != Phase::Done) {
        if (clock_.nowUs() >= slot.deadline) {
            cancel(client); // Logical timeout, storage remains owned by physical work.
            return {Code::Timeout};
        }
        return {Code::Pending};
    }
    Result result = slot.result;
    // Deadline covers delivery too: late polling must not return fresh success.
    if (clock_.nowUs() >= slot.deadline) result = {Code::Timeout};
    if (result.code == Code::Complete || result.code == Code::ShortTransfer) {
        if (result.received > capacity || (result.received && !rx)) {
            result.code = Code::BufferTooSmall;
        } else if (result.received) {
            std::memcpy(rx, slot.rx.data(), result.received); // Only caller task accesses its RX.
        }
    }
    slot.phase.store(Phase::Free, std::memory_order_seq_cst);
    return result;
}

void Transport::cancel(unsigned client) {
    Slot& slot = slots_[client];
    // Generation invalidation and phase publication need one total order.
    // Acquire/release alone on these two atomics could let both sides miss the
    // other's update, leaving a cancelled completion stranded in Done.
    slot.generation.fetch_add(1, std::memory_order_seq_cst);
    Phase expected = Phase::Queued;
    if (slot.phase.compare_exchange_strong(expected, Phase::Free, std::memory_order_seq_cst)) return;
    expected = Phase::Done;
    slot.phase.compare_exchange_strong(expected, Phase::Free, std::memory_order_seq_cst);
    // Active storage is never reset or reused here, even if physical work is late.
}

bool Transport::BqPort::start(uint8_t address, const uint8_t* tx, std::size_t txSize, std::size_t rxSize) {
    return owner_.submit(0, address, tx, txSize, rxSize, outstanding_, local_, last_);
}

bq76905::BusResult Transport::BqPort::poll(uint8_t* rx, std::size_t capacity) {
    if (!outstanding_) return {bq76905::BusState::BusError, 0, 0};
    if (!local_) last_ = owner_.collect(0, rx, capacity);
    if (last_.code == Code::Pending) return {bq76905::BusState::Pending, 0, 0};
    outstanding_ = false;
    if (last_.code == Code::Complete || last_.code == Code::ShortTransfer)
        return {bq76905::BusState::Complete, last_.transmitted, last_.received};
    if (last_.code == Code::Nak) return {bq76905::BusState::Nak, 0, 0};
    return {bq76905::BusState::BusError, 0, 0};
}

void Transport::BqPort::cancel() {
    if (!outstanding_) return;
    if (!local_) owner_.cancel(0);
    outstanding_ = false;
    last_ = {Code::Cancelled};
}

bool Transport::BqPort::idle() const {
    return !outstanding_ && owner_.slots_[0].phase.load(std::memory_order_seq_cst) == Phase::Free;
}

bool Transport::InaPort::start(uint8_t address, const uint8_t* tx, std::size_t txSize,
                               uint8_t* rx, std::size_t rxSize) {
    if (outstanding_) return false;
    if (rxSize && !rx) {
        outstanding_ = local_ = true;
        destination_ = nullptr;
        capacity_ = 0;
        last_ = {Code::InvalidArgument};
        return true;
    }
    if (!owner_.submit(1, address, tx, txSize, rxSize, outstanding_, local_, last_)) return false;
    destination_ = rx;
    capacity_ = rxSize;
    return true;
}

ina3221::IoResult Transport::InaPort::poll() {
    if (!outstanding_) return {ina3221::IoStatus::BusError, 0};
    if (!local_) last_ = owner_.collect(1, destination_, capacity_);
    if (last_.code == Code::Pending) return {ina3221::IoStatus::Pending, 0};
    outstanding_ = false;
    destination_ = nullptr;
    capacity_ = 0;
    if (last_.code == Code::Complete || last_.code == Code::ShortTransfer) {
        // INA's interface has no transmitted count: a short write cannot look complete.
        if (last_.transmitted != owner_.slots_[1].txSize) return {ina3221::IoStatus::BusError, 0};
        return {ina3221::IoStatus::Complete, last_.received};
    }
    if (last_.code == Code::Nak) return {ina3221::IoStatus::Nack, 0};
    return {ina3221::IoStatus::BusError, 0};
}

void Transport::InaPort::cancel() {
    destination_ = nullptr;
    capacity_ = 0;
    if (!outstanding_) return;
    if (!local_) owner_.cancel(1);
    outstanding_ = false;
    last_ = {Code::Cancelled};
}

void Transport::workerEntry(void* self) { static_cast<Transport*>(self)->worker(); }

void Transport::worker() {
    const uint64_t deadline = clock_.nowUs() + setupDeadlineUs;
    const Result setup = backend_.configure({sdaGpio, sclGpio, busFrequencyHz}, deadline);
    State starting = State::Starting;
    state_.compare_exchange_strong(starting,
        setup.code == Code::Complete && clock_.nowUs() < deadline ? State::Ready : State::Fault,
        std::memory_order_acq_rel); // An explicit stop must never be overwritten by Ready.
    for (;;) {
        if (stop_.load(std::memory_order_acquire)) break;
        if (state() == State::Ready && serviceOne()) continue;
        ulTaskNotifyTake(pdTRUE, 1); // One RTOS tick; lets explicit stop avoid a freed task handle.
    }
    // Explicit stop only. No automatic reconfigure, retry, reset or GPIO recovery.
    backend_.shutdown();
    for (auto& slot : slots_) slot.phase.store(Phase::Free, std::memory_order_seq_cst);
    state_.store(State::Stopped, std::memory_order_release);
    vTaskDelete(nullptr); // No access to this object after Stopped is published.
}

bool Transport::serviceOne() {
    for (unsigned offset = 0; offset < slots_.size(); ++offset) {
        const unsigned client = (next_ + offset) % slots_.size();
        Slot& slot = slots_[client];
        Phase expected = Phase::Queued;
        if (!slot.phase.compare_exchange_strong(expected, Phase::Active, std::memory_order_seq_cst)) continue;
        next_ = (client + 1) % slots_.size(); // A queued second client cannot be starved by immediate resubmission.
        const uint32_t generation = slot.requestGeneration;
        Result result{Code::Cancelled};
        if (slot.generation.load(std::memory_order_seq_cst) == generation) {
            if (clock_.nowUs() >= slot.deadline) result = {Code::Timeout};
            else {
                result = backend_.transfer({slot.address, slot.tx.data(), slot.txSize,
                                             slot.rx.data(), slot.rxSize}, slot.deadline);
                if (clock_.nowUs() >= slot.deadline) result = {Code::Timeout};
                else if (result.code == Code::Complete || result.code == Code::ShortTransfer) {
                    if (result.evidence == CountEvidence::Unknown || result.transmitted > slot.txSize ||
                        result.received > slot.rxSize) result = {Code::BusError};
                    else if (result.transmitted != slot.txSize || result.received != slot.rxSize)
                        result.code = Code::ShortTransfer;
                    else if (result.code == Code::ShortTransfer) result = {Code::BusError};
                } else if (result.code != Code::Nak && result.code != Code::Timeout && result.code != Code::BusError) {
                    result = {Code::BusError}; // Backend Pending is not physical completion.
                }
            }
        }
        slot.result = result;
        slot.phase.store(Phase::Done, std::memory_order_seq_cst);
        // Paired with cancel's CAS on Done. Covers cancellation both before and
        // after publishing completion, without freeing an Active buffer early.
        if (slot.generation.load(std::memory_order_seq_cst) != generation) {
            expected = Phase::Done;
            slot.phase.compare_exchange_strong(expected, Phase::Free, std::memory_order_seq_cst);
        }
        return true;
    }
    return false;
}

} // namespace pico_i2c
