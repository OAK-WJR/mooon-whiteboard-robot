#include "pico_rail_service.h"

namespace pico_rails {
namespace {
const char* busName(BusState state) {
    switch (state) {
    case BusState::Stopped: return "stopped";
    case BusState::Starting: return "starting";
    case BusState::Ready: return "ready";
    case BusState::Fault: return "fault";
    case BusState::Stopping: return "stopping";
    case BusState::CleanupFault: return "cleanup_fault";
    }
    return "fault";
}
const char* monitorError(ina3221::Monitor::Error error) {
    using E = ina3221::Monitor::Error;
    switch (error) {
    case E::None: return "none";
    case E::InvalidAddress: return "invalid_address";
    case E::WrongManufacturer: return "wrong_manufacturer";
    case E::WrongDie: return "wrong_die";
    case E::Nack: return "nack";
    case E::BusError: return "bus_error";
    case E::ShortRead: return "short_read";
    case E::BusBusyTimeout: return "bus_busy_timeout";
    case E::TransferTimeout: return "transfer_timeout";
    case E::ConversionTimeout: return "conversion_timeout";
    case E::AcquisitionTimeout: return "acquisition_timeout";
    case E::ConfigurationChanged: return "configuration_changed";
    case E::InvalidVoltage: return "invalid_voltage";
    }
    return "bus_error";
}
const char* cellDriverError(bq76905::Error error) {
    const char* const names[] = {"none", "invalid_options", "incomplete_profile", "nack", "bus_error",
        "short_transfer", "io_timeout", "wake_timeout", "initialization_timeout", "mode_timeout",
        "frame_timeout", "crc_mismatch", "crc_mode_mismatch", "echo_mismatch", "checksum_mismatch",
        "invalid_length", "wrong_device", "invalid_status", "not_full_access", "fets_not_off",
        "balancing_active", "config_not_active", "readback_mismatch", "unexpected_por",
        "unexpected_mode", "external_reset"};
    const auto index=static_cast<unsigned>(error);
    return index < sizeof(names)/sizeof(*names) ? names[index] : "bus_error";
}
}

bool Service::begin(WakeControl wake) {
    if (phase_ != Phase::NotStarted) return false;
    startedAt_ = bus_.nowMicros();
    phase_ = Phase::Starting;
    wake_ = wake;
    if (!bus_.open()) { fail("start_failed"); return false; }
    return true;
}

void Service::fail(const char* error) {
    error_ = error;
    phase_ = Phase::Fault;
    monitor_.invalidate();
    failCells("shared_bus_fault");
    bus_.close();
}

void Service::stop() {
    stopCells();
    if (phase_ == Phase::Stopped || phase_ == Phase::Fault) return;
    monitor_.invalidate();
    phase_ = Phase::Stopped;
    bus_.close();
}

void Service::stopCells() {
    if (cellPhase_ == CellPhase::Stopped || cellPhase_ == CellPhase::Fault) return;
    cellMonitor_.notifyResetOrPowerLoss();
    cellPhase_ = CellPhase::Stopped;
    if (wakeRequested_) {
        wakeRequested_ = false;
        if (!wake_ || !wake_(false)) { wakeControlFault_=true;cellPhase_ = CellPhase::Fault; cellError_ = "wake_gpio_fault"; }
    }
}

void Service::failCells(const char* error) {
    if (cellMonitor_.diagnostics().state != bq76905::State::Fault)
        cellMonitor_.notifyResetOrPowerLoss();
    cellPhase_ = CellPhase::Fault;
    cellError_ = error;
    if (wakeRequested_) {
        wakeRequested_ = false;
        if (!wake_ || !wake_(false)) { wakeControlFault_=true;cellError_ = "wake_gpio_fault"; }
    }
}

void Service::wakeAborted() {
    if (cellPhase_ == CellPhase::Pulse) failCells("wake_aborted");
}

void Service::tickWake() {
    const uint64_t now=bus_.nowMicros();
    if (cellPhase_ == CellPhase::NotStarted) {
        if (!wake_ || !wake_(true)) { wakeControlFault_=true;failCells("wake_gpio_fault"); return; }
        wakeRequested_=true;wakeAt_=now;cellPhase_=CellPhase::Pulse;
    } else if (cellPhase_ == CellPhase::Pulse && now-wakeAt_ >= wakePulseUs) {
        const bool late=now-wakeAt_ >= wakeLateUs;
        wakeRequested_=false;
        if (!wake_(false)) { wakeControlFault_=true;failCells("wake_gpio_fault"); return; }
        if (late) { failCells("wake_timeout"); return; }
        wakeAt_=now;cellPhase_=CellPhase::ReleaseWait;
    } else if (cellPhase_ == CellPhase::ReleaseWait && now-wakeAt_ >= wakeReleaseUs) {
        // Stock non-custom BQ76905 CRC mode; never guess/switch it on an error.
        // No battery-specific protection profile is silently invented. All
        // CHG/DSG outputs and system permissions remain OFF in this review.
        if (!cellMonitor_.beginAfterWake()) { failCells(cellDriverError(cellMonitor_.diagnostics().error)); return; }
        cellPhase_=CellPhase::Running;
    }
}

void Service::tick() {
    if (phase_ == Phase::NotStarted || phase_ == Phase::Stopped || phase_ == Phase::Fault) return;
    const auto busState = bus_.state();
    if (phase_ == Phase::Starting) {
        if (bus_.nowMicros() - startedAt_ >= startupTimeoutUs) { fail("setup_timeout"); return; }
        if (busState == BusState::Starting) return;
        if (busState != BusState::Ready) { fail("bus_fault"); return; }
        if (!monitor_.begin()) { fail(monitorError(monitor_.error())); return; }
        phase_ = Phase::Initializing;
        return;
    }
    if (busState != BusState::Ready) { fail("bus_fault"); return; }
    tickWake();
    if (wakeControlFault_) { stop(); return; }
    using M = ina3221::Monitor::State;
    // Never begin INA behind a BQ transfer, including cancelled physical work.
    // Finish the existing INA acquisition before advancing the other driver.
    if (monitor_.state() == M::Busy) monitor_.tick();
    else {
        const auto sample=monitor_.latest();
        const bool inaDue=monitor_.state()==M::Idle || !sample ||
            bus_.nowMicros()-sample->triggerRequestedAtUs >= samplePeriodUs;
        if (inaDue && bus_.cellsIdle()) {
            phase_=Phase::Running;
            monitor_.request(); // Schedules only; no I/O until a later loop.
        } else if (cellPhase_==CellPhase::Running) {
            cellMonitor_.tick();
            if (cellMonitor_.diagnostics().state==bq76905::State::Fault)
                failCells(cellDriverError(cellMonitor_.diagnostics().error));
        }
    }
    const auto state = monitor_.state();
    if (state == M::Fault) { fail(monitorError(monitor_.error())); return; }
}

CellStatus Service::cellStatus() const {
    CellStatus value{"not_started",cellError_,wakeRequested_,cellMonitor_.diagnostics(),cellMonitor_.snapshot(),0};
    switch (cellPhase_) {
    case CellPhase::NotStarted: break;
    case CellPhase::Pulse: value.state="waking";break;
    case CellPhase::ReleaseWait: value.state="settling";break;
    case CellPhase::Fault: value.state="fault";break;
    case CellPhase::Stopped: value.state="stopped";break;
    case CellPhase::Running:
        switch (value.diagnostics.state) {
        case bq76905::State::Detecting: value.state="detecting";break;
        case bq76905::State::Configuring: value.state="configuring";break;
        case bq76905::State::AwaitingScan: value.state="awaiting_scan";break;
        case bq76905::State::Monitoring: value.state=value.sample.fresh?"monitoring":"stale";break;
        case bq76905::State::Fault: value.state="fault";value.error=cellDriverError(value.diagnostics.error);break;
        case bq76905::State::Idle: value.state="not_started";break;
        }
        break;
    }
    if (value.sample.fresh) value.ageMs=cellClock_.nowMs()-value.sample.sampled_at_ms;
    return value;
}

Status Service::status() const {
    const auto busState = bus_.state();
    Status result{"not_started", error_, busName(busState), std::nullopt};
    if (busState == BusState::CleanupFault) {
        result.state = "fault";
        if (phase_ != Phase::Fault) result.error = "cleanup_failed";
        return result;
    }
    switch (phase_) {
    case Phase::NotStarted: return result;
    case Phase::Starting: result.state = "starting"; return result;
    case Phase::Initializing: result.state = "initializing"; return result;
    case Phase::Fault: result.state = "fault"; return result;
    case Phase::Stopped: result.state = "stopped"; return result;
    case Phase::Running: break;
    }
    if (busState != BusState::Ready) {
        result.state = "fault"; result.error = "bus_fault"; return result;
    }
    result.sample = monitor_.latest();
    if (result.sample) {
        const uint64_t age = bus_.nowMicros() - result.sample->triggerRequestedAtUs;
        if (age < ina3221::Monitor::sampleLifetimeUs) {
            result.ageUs = static_cast<uint32_t>(age);
            result.spanUs = static_cast<uint32_t>(result.sample->readyObservedAtUs -
                                                  result.sample->triggerRequestedAtUs);
            result.state = "ready";
            return result;
        }
        result.sample.reset();
    }
    result.state = monitor_.state() == ina3221::Monitor::State::Expired ? "expired" : "sampling";
    return result;
}
} // namespace pico_rails
