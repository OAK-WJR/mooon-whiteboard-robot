#ifndef WHITEBOARDBOT_BQ76905_MONITOR_H
#define WHITEBOARDBOT_BQ76905_MONITOR_H

#include <cstddef>
#include <cstdint>

// Standalone, allocation-free C++11. No Arduino, Wire, board profile, or secrets.
namespace bq76905 {

enum class BusState { Pending, Complete, Nak, BusError };
struct BusResult {
    BusState state;
    std::size_t transmitted;
    std::size_t received;
};

class I2c {
public:
    virtual ~I2c() {}
    // Nonblocking. false means BUSY/not started, with NO bus side effects.
    // Read transactions MUST use a repeated START after the register pointer.
    // tx remains alive until completion/cancel. Accept up to 69 TX / 68 RX bytes.
    virtual bool start(std::uint8_t address7, const std::uint8_t* tx,
                       std::size_t tx_size, std::size_t rx_size) = 0;
    // Complete must report exact buffer counts, including CRC but excluding the
    // address/RW byte and ACK bits. Pending
    // must return promptly; copy received bytes into rx only within capacity.
    virtual BusResult poll(std::uint8_t* rx, std::size_t capacity) = 0;
    // Nonblocking cancellation; release transaction/buffer ownership before return.
    virtual void cancel() = 0;
};

class Clock {
public:
    virtual ~Clock() {}
    virtual std::uint32_t nowMs() const = 0; // Monotonic modulo 2^32.
};

enum class CrcMode { Disabled, Enabled };
struct Options {
    std::uint8_t address7 = 0x08;
    // Select from the actual SKU/OTP and existing BQ state, NOT bus guessing.
    // Stock BQ76905 starts without CRC. This driver never changes the CRC bit.
    CrcMode startup_crc = CrcMode::Disabled;
    std::uint32_t io_timeout_ms = 100;
    std::uint32_t wake_timeout_ms = 2000;
    std::uint32_t mode_timeout_ms = 1000;
    std::uint32_t initialization_timeout_ms = 15000;
    std::uint32_t frame_timeout_ms = 1500;
    std::uint32_t poll_interval_ms = 5;
    std::uint32_t sample_interval_ms = 250;
    std::uint32_t max_sample_age_ms = 3000;
    // Diagnostic tolerance for sequential ADC readings; not a protection limit.
    std::uint16_t stack_agreement_mv = 250;
};

// Optional, explicit voltage + INTERNAL DIE overtemperature monitoring profile.
// Every field is required if requested. These are NOT full pack protections:
// SRP/SRN have no shunt and TS is NOT a thermistor. No battery defaults are guessed.
struct ProtectionProfile {
    bool requested = false;
    int cuv_mv = -1;
    int cov_mv = -1;
    int cuv_delay_scans = -1;
    int cov_delay_scans = -1;
    int cuv_recovery_code = -1; // 0: no auto recovery; 1:50, 2:100, 3:200 mV.
    int cov_recovery_code = -1;
    int die_ot_c = -1;
    int die_ot_recovery_c = -1;
    int die_ot_delay_scans = -1;
};

enum class State { Idle, Detecting, Configuring, AwaitingScan, Monitoring, Fault };
enum class Error {
    None, InvalidOptions, IncompleteProtectionProfile, Nak, BusError,
    ShortTransfer, IoTimeout, WakeTimeout, InitializationTimeout, ModeTimeout,
    FrameTimeout, CrcMismatch, CrcModeMismatch, EchoMismatch, ChecksumMismatch,
    InvalidLength, WrongDevice, InvalidStatus, NotFullAccess, FetsNotOff,
    BalancingActive, ConfigNotActive, ReadbackMismatch, UnexpectedPor,
    UnexpectedMode, ExternalReset
};
enum class Unavailable {
    None, NoSample, Stale, Fault, OutOfRange, LowerCellBelow2V,
    NoCurrentShunt, TsIsVoltageInput
};
struct Reading {
    bool available = false;
    std::int32_t value = 0; // Ignore and serialize as null when unavailable.
    Unavailable reason = Unavailable::NoSample;
};
struct Snapshot {
    std::uint32_t generation = 0;
    std::uint32_t sequence = 0;
    std::uint32_t sampled_at_ms = 0;
    std::uint32_t read_span_ms = 0; // Sequential registers, NOT an atomic ADC frame.
    bool status_available = false;
    bool fresh = false;
    bool cell_pair_valid = false;
    bool stack_consistent = false;
    std::uint16_t battery_status = 0;
    std::uint16_t alarm_status = 0;
    std::uint16_t alarm_raw = 0;
    std::uint8_t safety_alert_a = 0;
    std::uint8_t safety_status_a = 0;
    std::uint8_t safety_alert_b = 0;
    std::uint8_t safety_status_b = 0;
    std::uint8_t balancing_mask = 0;
    Reading lower_cell_mv;  // Direct 0x14, physical VC1-VC0.
    Reading upper_cell_mv;  // Direct 0x1C, physical VC5-VC4A, NOT 0x16.
    Reading stack_mv;
    Reading ts_adc_counts;  // Direct 0x2A, signed raw code.
    Reading ts_nominal_uv;  // Nominal bandgap scale, not calibrated temperature.
    Reading current_ma;
    Reading pack_temperature_mc;
};

struct Diagnostics {
    State state = State::Idle;
    Error error = Error::None;
    std::uint32_t generation = 0;
    std::uint16_t device_number = 0;
    std::uint16_t last_battery_status = 0;
    std::uint16_t failing_address = 0;
    bool response_confirmed = false;
    bool configuration_verified = false;
    bool voltage_profile_verified = false;
    bool fets_off_observed = false; // Observation only; becomes false on fault.
};

struct PermissionRequest { bool charge; bool discharge; };
enum class PermissionResult {
    HeldOff, NotInitialized, StaleMeasurements, InvalidCells, SafetyFault,
    ProtectionProfileMissing, SystemProtectionUnqualified
};

class Monitor {
public:
    Monitor(I2c& bus, Clock& clock);
    ~Monitor();
    Monitor(const Monitor&) = delete;
    Monitor& operator=(const Monitor&) = delete;
    // Caller owns external wake sequencing / permission pull-downs. Starting or
    // restarting always invalidates earlier samples and never grants power.
    bool beginAfterWake(const Options& options = Options(),
                        const ProtectionProfile& profile = ProtectionProfile());
    void notifyResetOrPowerLoss();
    // At most ONE adapter start or poll per tick, no loops waiting for hardware.
    void tick();
    Snapshot snapshot() const; // Returns a copy, invalidates aged readings here too.
    Diagnostics diagnostics() const { return diagnostic_; }
    bool inhibitRequired() const { return true; }
    // Deliberately no ON/toggle path: current and pack-temperature protections,
    // and the system permission circuit, are not qualified by this module.
    PermissionResult requestPermission(PermissionRequest request) const;
    static bool validProfile(const ProtectionProfile& profile);

private:
    enum class Stage {
        Detect, DeviceId, InitialOff, CheckOff, ExitDeepSleep, WaitAwake,
        ClearBalance, CheckBalance, EnterConfig, WaitConfig, ReadCrcConfig,
        WriteConfig, VerifyConfig, GuardConfig, ReadRegout, ReadRegoutConfig, WriteRegout,
        VerifyRegout, ArmAlarms, ClearAlarms, GuardExit, ExitConfig, WaitExit,
        RuntimeOff, DisableSleep, CheckRuntime, VerifyFetControl, VerifyAlarms,
        WaitStartup, StartupStatus, Idle,
        FrameGuard, AuditConfig, AuditCrc, AuditRegout, AuditRegoutConfig, AuditFetControl, FrameBalance,
        ClearScan, WaitScan, ScanStatus, SampleGuard, Cell1, Cell5, Stack, Ts,
        Safety, Alarms, RawAlarms, SampleEnd
    };
    enum class Op { None, Read, Write, Command, IndirectRead, IndirectWrite };
    struct Entry { std::uint16_t address; std::uint8_t size; std::uint16_t value; };
    I2c& bus_;
    Clock& clock_;
    Options options_;
    ProtectionProfile profile_;
    Diagnostics diagnostic_;
    Snapshot sample_, working_;
    Stage stage_ = Stage::Detect;
    Op op_ = Op::None;
    std::uint8_t op_step_ = 0;
    std::uint16_t op_address_ = 0;
    std::uint8_t op_size_ = 0;
    std::uint8_t op_data_[32] = {};
    bool op_dm_ = false;
    bool op_done_ = false;
    Error op_error_ = Error::None;
    bool io_active_ = false, io_waiting_ = false;
    std::uint8_t tx_[69] = {}, rx_[68] = {}, decoded_[34] = {};
    std::size_t tx_size_ = 0, rx_size_ = 0, logical_rx_size_ = 0;
    std::uint8_t io_register_ = 0;
    std::uint32_t io_at_ = 0, begun_at_ = 0, stage_at_ = 0;
    std::uint32_t next_at_ = 0, frame_at_ = 0, sample_read_at_ = 0, startup_at_ = 0;
    std::uint32_t generation_ = 0;
    bool initializing_ = false, frame_active_ = false, initial_por_ = false;
    std::uint8_t regout_ = 0;
    Entry entries_[19] = {};
    std::size_t entry_count_ = 0, entry_index_ = 0;

    void buildEntries();
    void go(Stage next);
    void fail(Error error);
    bool checkStatus(bool require_config, bool allow_por);
    void runStage();
    void startOp(Op op, std::uint16_t address, std::uint8_t size = 0,
                 std::uint16_t value = 0, bool data_memory = false);
    void stepOperation();
    void completeIo();
    void prepareIo(std::uint8_t reg, const std::uint8_t* data,
                   std::size_t write_size, std::size_t read_size);
    void publish();
};

} // namespace bq76905
#endif
