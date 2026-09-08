#include "ina3221_monitor.h"

namespace ina3221 {

Monitor::~Monitor() { cancelOwned(); }

void Monitor::cancelOwned() {
    if (active_) {
        bus_.cancel();
        active_ = false;
    }
}

void Monitor::invalidate() {
    cancelOwned();
    sample_.reset();
    converting_ = false;
    pollDelayed_ = false;
    state_ = State::Uninitialized;
    error_ = Error::None;
}

bool Monitor::begin(uint8_t address) {
    invalidate();
    if (address < 0x40 || address > 0x43) {
        fail(Error::InvalidAddress);
        return false;
    }
    address_ = address;
    state_ = State::Busy;
    acquisitionFrom_ = clock_.nowMicros();
    queue(Step::Manufacturer, 0xFE);
    return true;
}

bool Monitor::request() {
    if (state_ != State::Idle && state_ != State::Ready) return false;
    sample_.reset(); // A previous generation must not masquerade as this request.
    state_ = State::Busy;
    acquisitionFrom_ = clock_.nowMicros();
    converting_ = false;
    queue(Step::DrainReady, 0x0F);
    return true;
}

void Monitor::queue(Step step, uint8_t reg, bool write, uint16_t value) {
    step_ = step;
    tx_ = {reg, static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
    rx_ = {};
    txSize_ = write ? 3 : 1;
    rxSize_ = write ? 0 : 2;
    operationFrom_ = clock_.nowMicros();
    pollDelayed_ = false;
}

void Monitor::fail(Error error) {
    cancelOwned();
    sample_.reset();
    converting_ = false;
    pollDelayed_ = false;
    error_ = error;
    state_ = State::Fault;
}

bool Monitor::checkDeadlines(uint64_t now) {
    if (elapsed(now, acquisitionFrom_) >= acquisitionTimeoutUs)
        fail(Error::AcquisitionTimeout);
    else if (converting_ && elapsed(now, triggerFrom_) >= conversionTimeoutUs)
        fail(Error::ConversionTimeout);
    else if (!pollDelayed_ && elapsed(now, operationFrom_) >= ioTimeoutUs)
        fail(active_ ? Error::TransferTimeout : Error::BusBusyTimeout);
    return state_ != State::Fault;
}

void Monitor::tick() {
    if (state_ != State::Busy || !checkDeadlines(clock_.nowMicros())) return;
    if (pollDelayed_) {
        if (elapsed(clock_.nowMicros(), pollFrom_) < pollIntervalUs) return;
        pollDelayed_ = false;
        operationFrom_ = clock_.nowMicros();
    }
    if (!active_) {
        active_ = bus_.start(address_, tx_.data(), txSize_, rx_.data(), rxSize_);
        checkDeadlines(clock_.nowMicros()); // A late backend return cannot revive work.
        return;
    }
    const IoResult result = bus_.poll();
    // Keep ownership until deadlines are checked, so a late/Pending operation is
    // cancelled before buffers are reused. cancel also tolerates just-completed I/O.
    if (!checkDeadlines(clock_.nowMicros())) return;
    if (result.status == IoStatus::Pending) return;
    active_ = false;
    if (result.status == IoStatus::Nack) { fail(Error::Nack); return; }
    if (result.status != IoStatus::Complete) { fail(Error::BusError); return; }
    if (result.received != rxSize_) { fail(Error::ShortRead); return; }
    complete(static_cast<uint16_t>((uint16_t(rx_[0]) << 8) | rx_[1]));
}

void Monitor::complete(uint16_t value) {
    switch (step_) {
    case Step::Manufacturer:
        if (value != 0x5449) { fail(Error::WrongManufacturer); return; }
        queue(Step::Die, 0xFF);
        break;
    case Step::Die:
        if (value != 0x3220) { fail(Error::WrongDie); return; }
        queue(Step::IdleWrite, 0x00, true, idleConfiguration);
        break;
    case Step::IdleWrite:
        queue(Step::IdleVerify, 0x00);
        break;
    case Step::IdleVerify:
        if (value != idleConfiguration) { fail(Error::ConfigurationChanged); return; }
        state_ = State::Idle;
        break;
    case Step::DrainReady:
        // DS Table 7-36: power-down writes leave old CVRF intact. Discard it.
        queue(Step::Trigger, 0x00, true, triggeredConfiguration);
        triggerFrom_ = operationFrom_;
        converting_ = true;
        break;
    case Step::Trigger:
        // Table 7-6 mode 010: all three bus channels, single shot, no shunt.
        // This write clears CVRF and starts a NEW conversion even if unchanged.
        queue(Step::TriggerVerify, 0x00);
        break;
    case Step::TriggerVerify:
        if (value != triggeredConfiguration) { fail(Error::ConfigurationChanged); return; }
        queue(Step::WaitReady, 0x0F);
        break;
    case Step::WaitReady:
        if ((value & 1U) != 0) {
            converting_ = false;
            readyAt_ = clock_.nowMicros();
            queue(Step::Bus1, 0x02);
        } else {
            queue(Step::WaitReady, 0x0F);
            pollDelayed_ = true;
            pollFrom_ = operationFrom_;
        }
        break;
    case Step::Bus1:
    case Step::Bus2:
    case Step::Bus3: {
        // Table 7-9: sign, 12 data bits, three reserved zero bits, 8mV/LSB.
        // ADC full scale 32.76V exceeds the allowed 26V INPUT range.
        if ((value & 0x8007U) != 0 || value > 26000U) {
            fail(Error::InvalidVoltage);
            return;
        }
        const std::size_t channel = step_ == Step::Bus1 ? 0 : (step_ == Step::Bus2 ? 1 : 2);
        staging_[channel] = uint32_t(value >> 3) * 8U;
        if (step_ == Step::Bus1) queue(Step::Bus2, 0x04);
        else if (step_ == Step::Bus2) queue(Step::Bus3, 0x06);
        else queue(Step::FinalVerify, 0x00);
        break;
    }
    case Step::FinalVerify:
        // Catch POR/default 0x7127 or a configuration change during readout.
        // Single-shot leaves a stable register bank; MODE is a retained R/W field.
        if (value != triggeredConfiguration) { fail(Error::ConfigurationChanged); return; }
        ++sequence_;
        if (sequence_ == 0) ++sequence_;
        sample_ = Snapshot{staging_, sequence_, triggerFrom_, readyAt_};
        state_ = State::Ready;
        break;
    }
}

std::optional<Snapshot> Monitor::latest() const {
    if (state_ != State::Ready || !sample_ ||
        elapsed(clock_.nowMicros(), sample_->triggerRequestedAtUs) >= sampleLifetimeUs)
        return std::nullopt;
    return sample_;
}

Monitor::State Monitor::state() const {
    if (state_ == State::Ready && !latest()) return State::Expired;
    return state_;
}

} // namespace ina3221
