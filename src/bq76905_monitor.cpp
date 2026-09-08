#include "bq76905_monitor.h"

namespace bq76905 {
namespace {
// SLUUCI8 tables 11-1, 11-5, 11-9, 11-12/13, 12-4..12-11 and DM summary.
constexpr std::uint16_t kPor = 0x0080, kConfig = 0x0020;
constexpr std::uint16_t kFets = 0x000c, kAutonomous = 0x0100;
constexpr std::uint16_t kScan = 0x0080, kInit = 0x0004;
constexpr std::uint16_t kAlarmMask = 0xc285;

std::uint16_t u16(const std::uint8_t* p) {
    return static_cast<std::uint16_t>(p[0] | (static_cast<unsigned>(p[1]) << 8));
}
std::int32_t s16(const std::uint8_t* p) {
    const std::uint16_t v = u16(p);
    return v < 0x8000 ? v : static_cast<std::int32_t>(v) - 65536;
}
bool elapsed(std::uint32_t now, std::uint32_t from, std::uint32_t limit) {
    return static_cast<std::uint32_t>(now - from) >= limit;
}
bool due(std::uint32_t now, std::uint32_t at) {
    return static_cast<std::uint32_t>(now - at) < 0x80000000u;
}
std::uint8_t crcStep(std::uint8_t crc, std::uint8_t byte) {
    crc ^= byte;
    for (unsigned i = 0; i < 8; ++i)
        crc = static_cast<std::uint8_t>((crc << 1) ^ ((crc & 0x80) ? 0x07 : 0));
    return crc;
}
std::uint8_t sumCheck(std::uint16_t address, const std::uint8_t* p, std::size_t n) {
    unsigned sum = (address & 0xff) + (address >> 8);
    for (std::size_t i = 0; i < n; ++i) sum += p[i];
    return static_cast<std::uint8_t>(~sum);
}
Reading available(std::int32_t value) {
    Reading r; r.available = true; r.value = value; r.reason = Unavailable::None; return r;
}
Reading unavailable(Unavailable reason) {
    Reading r; r.reason = reason; return r;
}
void invalidate(Snapshot& s, Unavailable why) {
    s.fresh = s.status_available = s.cell_pair_valid = s.stack_consistent = false;
    s.lower_cell_mv = s.upper_cell_mv = s.stack_mv = unavailable(why);
    s.ts_adc_counts = s.ts_nominal_uv = unavailable(why);
    s.current_ma = unavailable(Unavailable::NoCurrentShunt);
    s.pack_temperature_mc = unavailable(Unavailable::TsIsVoltageInput);
}
bool inRange(int value, int lo, int hi) { return value >= lo && value <= hi; }
bool validOptions(const Options& o) {
    const std::uint32_t times[] = {o.io_timeout_ms, o.wake_timeout_ms,
        o.mode_timeout_ms, o.initialization_timeout_ms, o.frame_timeout_ms,
        o.poll_interval_ms, o.sample_interval_ms, o.max_sample_age_ms};
    for (auto t : times) if (!t || t >= 0x40000000u) return false;
    return o.address7 >= 0x08 && o.address7 <= 0x77 &&
        (o.startup_crc == CrcMode::Disabled || o.startup_crc == CrcMode::Enabled) &&
        o.io_timeout_ms <= o.mode_timeout_ms && o.io_timeout_ms <= o.wake_timeout_ms &&
        o.initialization_timeout_ms > o.wake_timeout_ms + o.mode_timeout_ms &&
        o.max_sample_age_ms > o.frame_timeout_ms + o.sample_interval_ms &&
        o.poll_interval_ms < o.mode_timeout_ms && o.stack_agreement_mv <= 1000;
}
} // namespace

Monitor::Monitor(I2c& bus, Clock& clock) : bus_(bus), clock_(clock) {
    invalidate(sample_, Unavailable::NoSample);
}

Monitor::~Monitor() { if (io_active_) bus_.cancel(); }

bool Monitor::validProfile(const ProtectionProfile& p) {
    if (!p.requested) return true;
    // At least 2 V is required at VC4A to qualify the upper measurement in 2S.
    if (!inRange(p.cuv_mv, 2000, 5500) || !inRange(p.cov_mv, 2001, 5500) ||
        !inRange(p.cuv_delay_scans, 1, 255) || !inRange(p.cov_delay_scans, 1, 255) ||
        !inRange(p.cuv_recovery_code, 0, 3) || !inRange(p.cov_recovery_code, 0, 3) ||
        !inRange(p.die_ot_c, 25, 150) || !inRange(p.die_ot_recovery_c, 25, 150) ||
        !inRange(p.die_ot_delay_scans, 1, 255) || p.die_ot_recovery_c >= p.die_ot_c)
        return false;
    const int hysteresis[] = {0, 50, 100, 200};
    return p.cuv_mv + hysteresis[p.cuv_recovery_code] <
           p.cov_mv - hysteresis[p.cov_recovery_code];
}

bool Monitor::beginAfterWake(const Options& o, const ProtectionProfile& p) {
    if (io_active_) bus_.cancel();
    io_active_ = io_waiting_ = false;
    op_ = Op::None; op_done_ = false; op_error_ = Error::None;
    diagnostic_ = Diagnostics();
    diagnostic_.generation = ++generation_;
    sample_ = Snapshot(); sample_.generation = generation_;
    invalidate(sample_, Unavailable::NoSample);
    working_ = sample_;
    initializing_ = frame_active_ = false;
    options_ = o; profile_ = p;
    if (!validOptions(o)) { fail(Error::InvalidOptions); return false; }
    if (!validProfile(p)) { fail(Error::IncompleteProtectionProfile); return false; }
    buildEntries();
    begun_at_ = clock_.nowMs();
    initializing_ = true; initial_por_ = false;
    diagnostic_.state = State::Detecting;
    go(Stage::Detect);
    return true;
}

void Monitor::buildEntries() {
    entry_count_ = 0;
    const auto add = [this](std::uint16_t a, std::uint8_t n, std::uint16_t v) {
        entries_[entry_count_++] = Entry{a, n, v};
    };
    add(0x901e, 1, 0x40); // HOST_FETOFF_EN only: no host ON, FET_EN, SFET or PWM.
    add(0x901b, 1, 2);
    add(0x9019, 2, 0x01c0); // TSMODE=1; CCMODE=3; default ADC conversion speeds.
    add(0x9020, 1, 0x03); // CB_NO_CMD=1; retain standard 1 ms settling delay.
    add(0x901c, 2, kAlarmMask); // Latch FULLSCAN, INITCOMP and POR, plus stock mask.
    if (profile_.requested) {
        add(0x902e, 2, static_cast<std::uint16_t>(profile_.cuv_mv));
        add(0x9030, 1, static_cast<std::uint16_t>(profile_.cuv_delay_scans));
        add(0x9031, 1, static_cast<std::uint16_t>(profile_.cuv_recovery_code));
        add(0x9032, 2, static_cast<std::uint16_t>(profile_.cov_mv));
        add(0x9034, 1, static_cast<std::uint16_t>(profile_.cov_delay_scans));
        add(0x9035, 1, static_cast<std::uint16_t>(profile_.cov_recovery_code));
        add(0x904c, 1, static_cast<std::uint16_t>(profile_.die_ot_c));
        add(0x904d, 1, static_cast<std::uint16_t>(profile_.die_ot_delay_scans));
        add(0x904e, 1, static_cast<std::uint16_t>(profile_.die_ot_recovery_c));
    }
    const std::uint16_t a = profile_.requested ? 0xc1 : 0x01;
    const std::uint16_t b = profile_.requested ? 0x02 : 0x00;
    add(0x9024, 1, a); // No current protection pretence on shorted SRP/SRN.
    add(0x9025, 1, b); // No TS thermistor protections; optional INTERNAL OT only.
    // These bit layouts are NOT Enabled Protections A/B. SLUUCI8 tables
    // 12-12/13: bit7 means CUV for DSG, COV for CHG; bit0 is OTINT on each.
    add(0x9026, 1, profile_.requested ? 0x81 : 0x00);
    add(0x9027, 1, profile_.requested ? 0x81 : 0x00);
    // Table 12-14: shared VREF(bit2), VSS(bit1), REGOUT(bit0). Retain the
    // stock VREF/VSS routing and add REGOUT; OTINT does not live in this byte.
    add(0x9028, 1, 0x07);
    entry_index_ = 0;
}

void Monitor::notifyResetOrPowerLoss() { fail(Error::ExternalReset); }

void Monitor::fail(Error error) {
    if (io_active_) bus_.cancel();
    io_active_ = io_waiting_ = false;
    diagnostic_.failing_address = op_address_;
    diagnostic_.error = error; diagnostic_.state = State::Fault;
    diagnostic_.configuration_verified = diagnostic_.voltage_profile_verified = false;
    diagnostic_.fets_off_observed = false;
    diagnostic_.generation = ++generation_;
    sample_.generation = generation_;
    invalidate(sample_, Unavailable::Fault);
    initializing_ = frame_active_ = false;
    op_ = Op::None; op_done_ = false;
}

void Monitor::go(Stage next) {
    stage_ = next; stage_at_ = next_at_ = clock_.nowMs();
    op_ = Op::None; op_done_ = false; op_error_ = Error::None;
}

void Monitor::tick() {
    if (diagnostic_.state == State::Idle || diagnostic_.state == State::Fault) return;
    const auto now = clock_.nowMs();
    if (stage_ == Stage::Detect && elapsed(now, begun_at_, options_.wake_timeout_ms)) {
        fail(Error::WakeTimeout); return;
    }
    if (initializing_ && elapsed(now, begun_at_, options_.initialization_timeout_ms)) {
        fail(Error::InitializationTimeout); return;
    }
    if (frame_active_ && elapsed(now, frame_at_, options_.frame_timeout_ms)) {
        fail(Error::FrameTimeout); return;
    }
    if ((stage_ == Stage::WaitConfig || stage_ == Stage::WaitExit ||
         stage_ == Stage::WaitAwake) && elapsed(now, stage_at_, options_.mode_timeout_ms)) {
        fail(Error::ModeTimeout); return;
    }
    if ((stage_ == Stage::WaitStartup || stage_ == Stage::StartupStatus) &&
        elapsed(now, startup_at_, options_.frame_timeout_ms)) {
        fail(Error::FrameTimeout); return;
    }
    if (op_done_ && op_error_ != Error::None) {
        // Only pre-response address NAKs are retried during the externally driven wake.
        if (stage_ == Stage::Detect && op_error_ == Error::Nak) {
            op_ = Op::None; op_done_ = false; op_error_ = Error::None;
            next_at_ = now + options_.poll_interval_ms;
        } else { fail(op_error_); return; }
    }
    if (op_ != Op::None && !op_done_) { stepOperation(); return; }
    if (due(now, next_at_)) runStage();
}

void Monitor::startOp(Op op, std::uint16_t address, std::uint8_t size,
                      std::uint16_t value, bool data_memory) {
    op_ = op; op_address_ = address; op_size_ = size; op_dm_ = data_memory;
    op_step_ = 0; op_done_ = false; op_error_ = Error::None;
    for (auto& byte : op_data_) byte = 0;
    if (op == Op::Command) { op_size_ = 2; value = address; }
    op_data_[0] = static_cast<std::uint8_t>(value);
    op_data_[1] = static_cast<std::uint8_t>(value >> 8);
}

void Monitor::prepareIo(std::uint8_t reg, const std::uint8_t* data,
                        std::size_t write_size, std::size_t read_size) {
    io_register_ = reg; tx_[0] = reg; tx_size_ = 1; logical_rx_size_ = read_size;
    const bool crc = options_.startup_crc == CrcMode::Enabled;
    rx_size_ = read_size * (crc ? 2 : 1);
    for (std::size_t i = 0; i < write_size; ++i) {
        tx_[tx_size_++] = data[i];
        if (crc) {
            std::uint8_t c = 0;
            if (!i) { c = crcStep(c, static_cast<std::uint8_t>(options_.address7 << 1)); c = crcStep(c, reg); }
            tx_[tx_size_++] = crcStep(c, data[i]);
        }
    }
    io_at_ = clock_.nowMs(); io_waiting_ = true;
}

void Monitor::stepOperation() {
    if ((io_active_ || io_waiting_) && elapsed(clock_.nowMs(), io_at_, options_.io_timeout_ms)) {
        if (io_active_) bus_.cancel();
        io_active_ = io_waiting_ = false;
        op_error_ = Error::IoTimeout; op_done_ = true; return;
    }
    if (io_waiting_) {
        if (bus_.start(options_.address7, tx_, tx_size_, rx_size_)) {
            io_active_ = true; io_waiting_ = false;
        }
        return;
    }
    if (io_active_) {
        const BusResult r = bus_.poll(rx_, sizeof(rx_));
        if (r.state == BusState::Pending) return;
        io_active_ = false;
        if (r.state != BusState::Complete) {
            op_error_ = r.state == BusState::Nak ? Error::Nak : Error::BusError;
        } else if (r.transmitted != tx_size_ || r.received != rx_size_) {
            op_error_ = Error::ShortTransfer;
        } else {
            const bool crc = options_.startup_crc == CrcMode::Enabled;
            for (std::size_t i = 0; i < logical_rx_size_; ++i) {
                decoded_[i] = rx_[i * (crc ? 2 : 1)];
                if (crc) {
                    std::uint8_t c = 0;
                    if (!i) {
                        c = crcStep(c, static_cast<std::uint8_t>(options_.address7 << 1));
                        c = crcStep(c, io_register_);
                        c = crcStep(c, static_cast<std::uint8_t>((options_.address7 << 1) | 1));
                    }
                    if (crcStep(c, decoded_[i]) != rx_[2 * i + 1]) {
                        op_error_ = Error::CrcMismatch; break;
                    }
                }
            }
        }
        if (op_error_ != Error::None) { op_done_ = true; return; }
        completeIo(); return;
    }
    const std::uint8_t address[] = {static_cast<std::uint8_t>(op_address_),
                                  static_cast<std::uint8_t>(op_address_ >> 8)};
    if (op_ == Op::Read) prepareIo(static_cast<std::uint8_t>(op_address_), nullptr, 0, op_size_);
    else if (op_ == Op::Write) prepareIo(static_cast<std::uint8_t>(op_address_), op_data_, op_size_, 0);
    else if (op_ == Op::Command) prepareIo(0x3e, op_data_, 2, 0);
    else if (op_step_ == 0) prepareIo(0x3e, address, 2, 0);
    else if (op_step_ == 1) prepareIo(0x3e, nullptr, 0, 2);
    else if (op_ == Op::IndirectRead) prepareIo(0x40, nullptr, 0, 34);
    else if (op_step_ == 2) prepareIo(0x40, op_data_, op_size_, 0);
    else if (op_step_ == 3 && op_dm_) prepareIo(0x12, nullptr, 0, 2);
    else {
        const std::uint8_t commit[] = {sumCheck(op_address_, op_data_, op_size_),
                                      static_cast<std::uint8_t>(op_size_ + 4)};
        prepareIo(0x60, commit, 2, 0); // 0x61 LAST commits; never written first.
    }
}

void Monitor::completeIo() {
    if (op_ == Op::Read) {
        for (std::size_t i = 0; i < op_size_; ++i) op_data_[i] = decoded_[i];
        op_done_ = true; return;
    }
    if (op_ == Op::Write || op_ == Op::Command) { op_done_ = true; return; }
    if (op_step_ == 1 && u16(decoded_) != op_address_) {
        op_error_ = Error::EchoMismatch; op_done_ = true; return;
    }
    if (op_ == Op::IndirectRead && op_step_ == 2) {
        // Read all 32 buffer bytes + checksum + length. Reading 0x61 advances
        // the window AFTER this transfer; every following op reselects its address.
        const unsigned length = decoded_[33];
        if (length < 5 || length > 36 || length - 4 < op_size_ ||
            (!op_dm_ && length - 4 != op_size_)) op_error_ = Error::InvalidLength;
        else if (sumCheck(op_address_, decoded_, length - 4) != decoded_[32])
            op_error_ = Error::ChecksumMismatch;
        else for (std::size_t i = 0; i < op_size_; ++i) op_data_[i] = decoded_[i];
        op_done_ = true; return;
    }
    if (op_ == Op::IndirectWrite && op_step_ == 3 && op_dm_) {
        diagnostic_.last_battery_status = u16(decoded_);
        if (!checkStatus(true, initial_por_)) return;
    } else if (op_ == Op::IndirectWrite &&
               ((op_step_ == 3 && !op_dm_) || op_step_ == 4)) {
        op_done_ = true; return;
    }
    ++op_step_;
}

bool Monitor::checkStatus(bool require_config, bool allow_por) {
    const auto s = diagnostic_.last_battery_status;
    if (s & 0x0201) { fail(Error::InvalidStatus); return false; }
    if ((s & 0x0c00) != 0x0400) { fail(Error::NotFullAccess); return false; }
    if (!allow_por && (s & kPor)) { fail(Error::UnexpectedPor); return false; }
    if (s & kFets) { fail(Error::FetsNotOff); return false; }
    if (require_config && !(s & kConfig)) { fail(Error::ConfigNotActive); return false; }
    if (!require_config && (s & kConfig)) { fail(Error::UnexpectedMode); return false; }
    if (!initializing_ && (s & (0xc040 | kAutonomous))) {
        fail(Error::UnexpectedMode); return false;
    }
    diagnostic_.fets_off_observed = true;
    return true;
}

void Monitor::runStage() {
    const auto read = [this](std::uint8_t a, std::uint8_t n) { startOp(Op::Read, a, n); };
    const auto write = [this](std::uint8_t a, std::uint8_t n, std::uint16_t v) { startOp(Op::Write, a, n, v); };
    const auto command = [this](std::uint16_t a) { startOp(Op::Command, a); };
    const auto again = [this]() {
        op_ = Op::None; op_done_ = false; next_at_ = clock_.nowMs() + options_.poll_interval_ms;
    };
    const auto status = [this]() { diagnostic_.last_battery_status = u16(op_data_); };
    switch (stage_) {
    case Stage::Detect:
        if (!op_done_) { read(0x12, 2); break; }
        status();
        if ((u16(op_data_) & 0x0201) != 0) { fail(Error::InvalidStatus); break; }
        if ((u16(op_data_) & 0x0c00) == 0) { again(); break; } // Still initializing.
        initial_por_ = (u16(op_data_) & kPor) != 0;
        go(Stage::DeviceId); break;
    case Stage::DeviceId:
        if (!op_done_) { startOp(Op::IndirectRead, 0x0001, 2); break; }
        diagnostic_.device_number = u16(op_data_);
        if (u16(op_data_) != 0x7605) { fail(Error::WrongDevice); break; } // SLUAAM4 §2.1.
        diagnostic_.response_confirmed = true;
        go(Stage::InitialOff); break;
    case Stage::InitialOff:
        if (!op_done_) { write(0x68, 1, 0x0c); break; }
        go(Stage::CheckOff); break;
    case Stage::CheckOff:
        if (!op_done_) { read(0x12, 2); break; }
        status();
        // Stock HOST_FETOFF_EN is zero: ACK to 0x68 alone proves nothing.
        // An already active or already CONFIG_UPDATE device is refused, not taken over.
        if (!checkStatus(false, initial_por_)) break;
        if (u16(op_data_) & kAutonomous) { fail(Error::UnexpectedMode); break; }
        go((u16(op_data_) & 0x4000) ? Stage::ExitDeepSleep : Stage::ClearBalance); break;
    case Stage::ExitDeepSleep:
        if (!op_done_) { command(0x000e); break; }
        go(Stage::WaitAwake); break;
    case Stage::WaitAwake:
        if (!op_done_) { read(0x12, 2); break; }
        status(); if (!checkStatus(false, initial_por_)) break;
        if (u16(op_data_) & 0x4000) again(); else go(Stage::ClearBalance);
        break;
    case Stage::ClearBalance:
        if (!op_done_) { startOp(Op::IndirectWrite, 0x0083, 1, 0); break; }
        go(Stage::CheckBalance); break;
    case Stage::CheckBalance:
        if (!op_done_) { startOp(Op::IndirectRead, 0x0083, 1); break; }
        if (op_data_[0]) { fail(Error::BalancingActive); break; }
        go(Stage::EnterConfig); break;
    case Stage::EnterConfig:
        if (!op_done_) { command(0x0090); break; }
        diagnostic_.state = State::Configuring; go(Stage::WaitConfig); break;
    case Stage::WaitConfig:
        if (!op_done_) { read(0x12, 2); break; }
        status();
        if (!(u16(op_data_) & kConfig)) {
            if (checkStatus(false, initial_por_)) again();
        } else if (checkStatus(true, initial_por_)) go(Stage::ReadCrcConfig);
        break;
    case Stage::ReadCrcConfig:
    case Stage::AuditCrc:
        if (!op_done_) { startOp(Op::IndirectRead, 0x9017, 2, 0, true); break; }
        if ((u16(op_data_) & 1) != (options_.startup_crc == CrcMode::Enabled ? 1 : 0)) {
            fail(Error::CrcModeMismatch); break;
        }
        go(stage_ == Stage::ReadCrcConfig ? Stage::WriteConfig : Stage::AuditRegout); break;
    case Stage::WriteConfig: {
        const Entry& e = entries_[entry_index_];
        if (!op_done_) { startOp(Op::IndirectWrite, e.address, e.size, e.value, true); break; }
        go(Stage::VerifyConfig); break;
    }
    case Stage::VerifyConfig:
    case Stage::AuditConfig: {
        const Entry& e = entries_[entry_index_];
        if (!op_done_) { startOp(Op::IndirectRead, e.address, e.size, 0, true); break; }
        const auto value = e.size == 1 ? op_data_[0] : u16(op_data_);
        if (value != e.value) { fail(Error::ReadbackMismatch); break; }
        if (stage_ == Stage::VerifyConfig) go(Stage::GuardConfig);
        else if (++entry_index_ < entry_count_) { op_ = Op::None; op_done_ = false; }
        else go(Stage::AuditCrc);
        break;
    }
    case Stage::GuardConfig:
        if (!op_done_) { read(0x12, 2); break; }
        status(); if (!checkStatus(true, initial_por_)) break;
        go(++entry_index_ < entry_count_ ? Stage::WriteConfig : Stage::ReadRegout); break;
    case Stage::ReadRegout:
        if (!op_done_) { read(0x69, 1); break; }
        if (op_data_[0] & 0xe0) { fail(Error::ReadbackMismatch); break; }
        regout_ = static_cast<std::uint8_t>(op_data_[0] & 0x0f); // Preserve rail/enable.
        go(Stage::ReadRegoutConfig); break;
    case Stage::ReadRegoutConfig:
    case Stage::AuditRegoutConfig:
        if (!op_done_) { startOp(Op::IndirectRead, 0x9015, 2, 0, true); break; }
        // EXIT_CFGUPDATE reloads REGOUT from DM. Do not let an existing live
        // override conceal a different exit voltage, enable state, or I2C address.
        if (op_data_[0] != regout_ ||
            (op_data_[1] == 0 ? 0x08 : op_data_[1]) != options_.address7) {
            fail(Error::ReadbackMismatch); break;
        }
        go(stage_ == Stage::ReadRegoutConfig ? Stage::WriteRegout : Stage::AuditFetControl); break;
    case Stage::WriteRegout:
        if (!op_done_) { write(0x69, 1, regout_); break; } // TS_ON=0, never change LDO rail.
        go(Stage::VerifyRegout); break;
    case Stage::VerifyRegout:
    case Stage::AuditRegout:
        if (!op_done_) { read(0x69, 1); break; }
        if (op_data_[0] != regout_) { fail(Error::ReadbackMismatch); break; }
        go(stage_ == Stage::VerifyRegout ? Stage::ArmAlarms : Stage::AuditRegoutConfig); break;
    case Stage::ArmAlarms:
        if (!op_done_) { write(0x66, 2, kAlarmMask); break; }
        go(Stage::ClearAlarms); break;
    case Stage::ClearAlarms:
        if (!op_done_) { write(0x62, 2, kScan | kInit | 1); break; }
        go(Stage::GuardExit); break;
    case Stage::GuardExit:
        if (!op_done_) { read(0x12, 2); break; }
        status(); if (checkStatus(true, initial_por_)) go(Stage::ExitConfig); break;
    case Stage::ExitConfig:
        if (!op_done_) { command(0x0092); break; }
        go(Stage::WaitExit); break;
    case Stage::WaitExit:
        if (!op_done_) { read(0x12, 2); break; }
        status();
        if (u16(op_data_) & kConfig) {
            if (checkStatus(true, initial_por_)) again();
        } else if (checkStatus(false, false)) go(Stage::RuntimeOff);
        break;
    case Stage::RuntimeOff:
        if (!op_done_) { write(0x68, 1, 0x0c); break; }
        go(Stage::DisableSleep); break;
    case Stage::DisableSleep:
        if (!op_done_) { command(0x009a); break; }
        go(Stage::CheckRuntime); break;
    case Stage::CheckRuntime:
    case Stage::FrameGuard:
    case Stage::SampleGuard:
    case Stage::SampleEnd:
        if (!op_done_) { read(0x12, 2); break; }
        status(); if (!checkStatus(false, false)) break;
        if (u16(op_data_) & (0xc040 | kAutonomous)) { fail(Error::UnexpectedMode); break; }
        if (stage_ == Stage::CheckRuntime) go(Stage::VerifyFetControl);
        else if (stage_ == Stage::FrameGuard) { entry_index_ = 0; go(Stage::AuditConfig); }
        else if (stage_ == Stage::SampleGuard) {
            sample_read_at_ = clock_.nowMs(); go(Stage::Cell1);
        } else {
            working_.battery_status = u16(op_data_); publish();
            frame_active_ = initializing_ = false;
            diagnostic_.configuration_verified = true;
            diagnostic_.voltage_profile_verified = profile_.requested;
            diagnostic_.state = State::Monitoring; go(Stage::Idle);
        }
        break;
    case Stage::VerifyFetControl:
    case Stage::AuditFetControl:
        if (!op_done_) { read(0x68, 1); break; }
        if (op_data_[0] != 0x0c) { fail(Error::ReadbackMismatch); break; }
        go(stage_ == Stage::VerifyFetControl ? Stage::VerifyAlarms : Stage::FrameBalance); break;
    case Stage::VerifyAlarms:
        if (!op_done_) { read(0x66, 2); break; }
        if (u16(op_data_) != kAlarmMask) { fail(Error::ReadbackMismatch); break; }
        diagnostic_.state = State::AwaitingScan; startup_at_ = clock_.nowMs();
        go(Stage::WaitStartup); break;
    case Stage::WaitStartup:
        if (!op_done_) { read(0x62, 2); break; }
        if (!(u16(op_data_) & kInit)) { go(Stage::StartupStatus); break; }
        frame_at_ = clock_.nowMs(); frame_active_ = true;
        working_ = Snapshot(); working_.generation = generation_;
        go(Stage::FrameGuard); break;
    case Stage::StartupStatus:
    case Stage::ScanStatus:
        if (!op_done_) { read(0x12, 2); break; }
        status(); if (!checkStatus(false, false)) break;
        go(stage_ == Stage::StartupStatus ? Stage::WaitStartup : Stage::WaitScan);
        next_at_ = clock_.nowMs() + options_.poll_interval_ms;
        break;
    case Stage::Idle:
        if (!elapsed(clock_.nowMs(), sample_.sampled_at_ms, options_.sample_interval_ms)) break;
        frame_at_ = clock_.nowMs(); frame_active_ = true;
        working_ = Snapshot(); working_.generation = generation_;
        go(Stage::FrameGuard); break;
    case Stage::FrameBalance:
        if (!op_done_) { startOp(Op::IndirectRead, 0x0083, 1); break; }
        if (op_data_[0]) { fail(Error::BalancingActive); break; }
        working_.balancing_mask = op_data_[0]; go(Stage::ClearScan); break;
    case Stage::ClearScan:
        if (!op_done_) { write(0x62, 2, kScan); break; }
        go(Stage::WaitScan); break;
    case Stage::WaitScan:
        if (!op_done_) { read(0x62, 2); break; }
        if (!(u16(op_data_) & kScan)) { go(Stage::ScanStatus); break; }
        go(Stage::SampleGuard); break;
    case Stage::Cell1:
        if (!op_done_) { read(0x14, 2); break; }
        working_.lower_cell_mv = available(s16(op_data_)); go(Stage::Cell5); break;
    case Stage::Cell5:
        if (!op_done_) { read(0x1c, 2); break; }
        working_.upper_cell_mv = available(s16(op_data_)); go(Stage::Stack); break;
    case Stage::Stack:
        if (!op_done_) { read(0x26, 2); break; }
        working_.stack_mv = available(u16(op_data_)); go(Stage::Ts); break;
    case Stage::Ts:
        if (!op_done_) { read(0x2a, 2); break; }
        working_.ts_adc_counts = available(s16(op_data_)); go(Stage::Safety); break;
    case Stage::Safety:
        if (!op_done_) { read(0x02, 4); break; }
        working_.safety_alert_a = op_data_[0]; working_.safety_status_a = op_data_[1];
        working_.safety_alert_b = op_data_[2]; working_.safety_status_b = op_data_[3];
        go(Stage::Alarms); break;
    case Stage::Alarms:
        if (!op_done_) { read(0x62, 2); break; }
        working_.alarm_status = u16(op_data_); go(Stage::RawAlarms); break;
    case Stage::RawAlarms:
        if (!op_done_) { read(0x64, 2); break; }
        working_.alarm_raw = u16(op_data_); go(Stage::SampleEnd); break;
    }
}

void Monitor::publish() {
    const auto lower = working_.lower_cell_mv.value, upper = working_.upper_cell_mv.value;
    if (lower <= 0 || lower > 5500) working_.lower_cell_mv = unavailable(Unavailable::OutOfRange);
    if (upper <= 0 || upper > 5500) working_.upper_cell_mv = unavailable(Unavailable::OutOfRange);
    if (lower < 2000 || !working_.lower_cell_mv.available)
        working_.upper_cell_mv = unavailable(Unavailable::LowerCellBelow2V);
    if (working_.stack_mv.value <= 0 || working_.stack_mv.value > 11000)
        working_.stack_mv = unavailable(Unavailable::OutOfRange);
    const auto ts = working_.ts_adc_counts.value;
    // Nominal scale: VBG (1.1962 V) * (5/3) / 32768. Integer arithmetic avoids
    // signed overflow, and the value is explicitly nominal (not a calibrated ADC).
    const auto uv = static_cast<std::int32_t>((static_cast<std::int64_t>(ts) * 5981000 + 49152) / 98304);
    if (ts < 0 || uv > 1800000) {
        working_.ts_adc_counts = working_.ts_nominal_uv = unavailable(Unavailable::OutOfRange);
    } else working_.ts_nominal_uv = available(uv);
    working_.cell_pair_valid = working_.lower_cell_mv.available && working_.upper_cell_mv.available;
    auto diff = lower + upper - working_.stack_mv.value;
    if (diff < 0) diff = -diff;
    working_.stack_consistent = working_.cell_pair_valid && working_.stack_mv.available &&
                               diff <= options_.stack_agreement_mv;
    working_.cell_pair_valid = working_.cell_pair_valid && working_.stack_consistent;
    working_.current_ma = unavailable(Unavailable::NoCurrentShunt);
    working_.pack_temperature_mc = unavailable(Unavailable::TsIsVoltageInput);
    working_.sequence = sample_.sequence + 1;
    working_.sampled_at_ms = clock_.nowMs();
    working_.read_span_ms = working_.sampled_at_ms - sample_read_at_;
    working_.fresh = working_.status_available = true;
    sample_ = working_;
}

Snapshot Monitor::snapshot() const {
    Snapshot s = sample_;
    if (s.fresh && elapsed(clock_.nowMs(), s.sampled_at_ms, options_.max_sample_age_ms))
        invalidate(s, Unavailable::Stale);
    return s;
}

PermissionResult Monitor::requestPermission(PermissionRequest request) const {
    if (!request.charge && !request.discharge) return PermissionResult::HeldOff;
    if (!diagnostic_.configuration_verified) return PermissionResult::NotInitialized;
    const Snapshot s = snapshot();
    if (!s.fresh) return PermissionResult::StaleMeasurements;
    if (!s.cell_pair_valid) return PermissionResult::InvalidCells;
    if (s.safety_status_a || s.safety_status_b || s.safety_alert_a || s.safety_alert_b ||
        (s.battery_status & 0x3000)) return PermissionResult::SafetyFault;
    if (!diagnostic_.voltage_profile_verified) return PermissionResult::ProtectionProfileMissing;
    return PermissionResult::SystemProtectionUnqualified;
}

} // namespace bq76905
