#include "device_model.h"
using namespace bq76905;
using namespace bq_model;

struct Rig {
    TestClock clock;
    Device chip;
    Monitor monitor;
    Rig() : chip(clock), monitor(chip, clock) {}
    void tick() {
        chip.events(); chip.calls_this_tick = 0; monitor.tick();
        CHECK(chip.calls_this_tick <= 1); ++clock.ms;
    }
    void until(const std::function<bool()>& predicate, unsigned max = 20000) {
        for (unsigned i = 0; i < max && !predicate(); ++i) tick();
        CHECK(predicate());
    }
    void sample(unsigned sequence = 1) {
        until([&]() { return monitor.snapshot().sequence >= sequence || monitor.diagnostics().state == State::Fault; });
        if (monitor.diagnostics().state == State::Fault)
            throw std::runtime_error("Unexpected driver error " + std::to_string(int(monitor.diagnostics().error)) +
                                     " at address " + std::to_string(monitor.diagnostics().failing_address));
        CHECK(monitor.snapshot().sequence >= sequence);
    }
    void fault(Error error) {
        until([&]() { return monitor.diagnostics().state == State::Fault; });
        CHECK(monitor.diagnostics().error == error);
        CHECK(!monitor.diagnostics().configuration_verified);
        CHECK(!monitor.diagnostics().voltage_profile_verified);
        CHECK(!monitor.diagnostics().fets_off_observed);
        CHECK(!monitor.snapshot().fresh && !monitor.snapshot().lower_cell_mv.available);
        CHECK(!monitor.snapshot().current_ma.available);
        CHECK(monitor.inhibitRequired());
    }
};

static ProtectionProfile exampleProfile() {
    // Synthetic validation fixture; NOT a recommendation for GNB charging.
    ProtectionProfile p; p.requested = true;
    p.cuv_mv = 2800; p.cov_mv = 4200;
    p.cuv_delay_scans = 10; p.cov_delay_scans = 11;
    p.cuv_recovery_code = 2; p.cov_recovery_code = 2;
    p.die_ot_c = 100; p.die_ot_recovery_c = 90; p.die_ot_delay_scans = 12;
    return p;
}

static void ti_golden_vectors() {
    // SLUAAM4 §4 captured waveforms, independent of driver and model defaults.
    CHECK(referenceCrc({0x10, 0x3e, 0x22}) == 0x63);
    CHECK(referenceCrc({0x00}) == 0x00);
    CHECK(referenceCrc({0x10, 0x14, 0x11, 0xc5}) == 0x79);
    CHECK(referenceCrc({0x0b}) == 0x31);
    Rig r; r.chip.otp_crc = true; r.chip.reset();
    r.chip.lower = 0x0bc5; r.chip.stack = r.chip.lower + r.chip.upper;
    Options o; o.startup_crc = CrcMode::Enabled;
    CHECK(r.monitor.beginAfterWake(o)); r.sample();
    CHECK(r.monitor.snapshot().lower_cell_mv.value == 3013);
    CHECK(r.chip.ram.at(0x9017) == 1); // CRC not disabled or migrated by driver.
}

static void source_table_and_sequence() {
    Rig r; r.chip.wake_naks = 3;
    CHECK(r.monitor.beginAfterWake());
    CHECK(!r.monitor.diagnostics().response_confirmed); r.sample();
    const auto s = r.monitor.snapshot();
    CHECK(s.lower_cell_mv.value == 3700 && s.upper_cell_mv.value == 3800);
    CHECK(s.stack_mv.value == 7500 && s.cell_pair_valid);
    CHECK(s.ts_adc_counts.value == 10000 && s.ts_nominal_uv.value == 608419);
    CHECK(!s.current_ma.available && s.current_ma.reason == Unavailable::NoCurrentShunt);
    CHECK(!s.pack_temperature_mc.available && s.pack_temperature_mc.reason == Unavailable::TsIsVoltageInput);
    CHECK(r.monitor.diagnostics().device_number == 0x7605);
    CHECK(r.monitor.diagnostics().response_confirmed && r.monitor.diagnostics().configuration_verified);
    CHECK(!r.monitor.diagnostics().voltage_profile_verified);
    CHECK(r.monitor.requestPermission({true, false}) == PermissionResult::ProtectionProfileMissing);
    CHECK(r.monitor.requestPermission({false, false}) == PermissionResult::HeldOff);
    CHECK(r.chip.illegal_config_commits == 0);
    CHECK(r.chip.commands == std::vector<std::uint16_t>({0x0090, 0x0092, 0x009a}));
    const std::map<std::uint16_t, Bytes> expected = {
        {0x0083,{0}}, {0x901e,{0x40}}, {0x901b,{2}}, {0x9019,{0xc0,1}},
        {0x9020,{3}}, {0x901c,{0x85,0xc2}}, {0x9024,{1}}, {0x9025,{0}},
        {0x9026,{0}}, {0x9027,{0}}, {0x9028,{7}}
    };
    CHECK(r.chip.commits.size() == expected.size());
    for (const auto& c : r.chip.commits) {
        CHECK(c.data == expected.at(c.address));
        if (c.address >= 0x9000) CHECK(c.config);
        if (c.address == 0x901b) { CHECK(c.checksum == 0x52); CHECK(c.length == 5); }
        if (c.address == 0x0083) { CHECK(c.checksum == 0x7c); CHECK(c.length == 5); }
    }
    CHECK(r.chip.regout == 0x0e); // TS_ON cleared; existing 3.3 V LDO preserved.
    CHECK(r.chip.ram[0x9015] == 0x0e && r.chip.ram[0x9016] == 0x08);
    bool cell1 = false, cell5 = false;
    for (const auto& t : r.chip.trace) {
        if (t.rx_size && t.tx[0] == 0x14) cell1 = true;
        if (t.rx_size && t.tx[0] == 0x1c) cell5 = true;
        if (t.rx_size) CHECK(t.tx[0] != 0x16 && t.tx[0] != 0x36 && t.tx[0] != 0x3a && t.tx[0] != 0x3c);
        // Correct 2S write and one-byte width; 0x901C must not be overwritten as Vcell high byte.
        if (t.tx == Bytes({0x60,0x52,0x05})) CHECK(t.rx_size == 0);
    }
    CHECK(cell1 && cell5);
    CHECK(r.chip.direct_writes.front() == Bytes({0x68,0x0c}));
    r.sample(2); CHECK(r.monitor.snapshot().read_span_ms > 0);
}

static void verified_profile_is_not_charge_permission() {
    Rig r; CHECK(r.monitor.beginAfterWake(Options(), exampleProfile())); r.sample();
    CHECK(r.monitor.diagnostics().voltage_profile_verified);
    CHECK(r.chip.ram[0x902e] == 0xf0 && r.chip.ram[0x902f] == 0x0a);
    CHECK(r.chip.ram[0x9032] == 0x68 && r.chip.ram[0x9033] == 0x10);
    CHECK(r.chip.ram[0x9030] == 10 && r.chip.ram[0x9034] == 11);
    CHECK(r.chip.ram[0x9031] == 2 && r.chip.ram[0x9035] == 2);
    CHECK(r.chip.ram[0x904c] == 100 && r.chip.ram[0x904d] == 12 && r.chip.ram[0x904e] == 90);
    CHECK(r.chip.ram[0x9024] == 0xc1 && r.chip.ram[0x9025] == 2);
    CHECK(r.chip.ram[0x9026] == 0x81 && r.chip.ram[0x9027] == 0x81 && r.chip.ram[0x9028] == 7);
    for (auto q : {PermissionRequest{true,false}, PermissionRequest{false,true}, PermissionRequest{true,true}})
        CHECK(r.monitor.requestPermission(q) == PermissionResult::SystemProtectionUnqualified);
    CHECK((r.chip.battery & 0x010c) == 0);
}

static void protection_direction_is_independent_of_held_off() {
    struct Case { std::uint8_t a, b; bool profile_dsg, profile_chg, plain_dsg, plain_chg; };
    const Case cases[] = {
        {0x40,0, true, false, false,false}, // CUV must not add a CHG protection block.
        {0x80,0, false,true,  false,false}, // COV must not add a DSG protection block.
        {0x01,0, true, true,  true, true }, // REGOUT shared even without a profile.
        {0,0x08, true, true,  false,false}, // OTINT shared via the TWO per-FET bit0s.
        {0,0x02, true, true,  true, true }, // VREF: retained stock common route.
        {0,0x01, true, true,  true, true }, // VSS: retained stock common route.
        {0xc0,0, true, true,  false,false}, // Combined CUV/COV.
        {0x41,0, true, true,  true, true }, // CUV cannot override a common REGOUT block.
        {0x20,0, false,false, false,false}, // Unused SCD bit must not be copied from enable mask.
        {0x10,0, false,false, false,false}, // OCD1.
        {0x08,0, false,false, false,false}, // OCD2.
        {0x04,0, false,false, false,false}, // OCC.
        {0,0x04, false,false, false,false}, // HWD.
        {0,0x80, false,false, false,false}, // OTD (TS is not a thermistor).
        {0,0x40, false,false, false,false}, // OTC.
        {0,0x20, false,false, false,false}, // UTD.
        {0,0x10, false,false, false,false}  // UTC.
    };
    for (bool with_profile : {false, true}) {
        Rig r; r.monitor.beginAfterWake(Options(), with_profile ? exampleProfile() : ProtectionProfile()); r.sample();
        for (const auto& c : cases) {
            const auto blocks = decodeFaultRoutes(c.a, c.b, r.chip.ram.at(0x9026),
                                                   r.chip.ram.at(0x9027), r.chip.ram.at(0x9028));
            CHECK(blocks.dsg == (with_profile ? c.profile_dsg : c.plain_dsg));
            CHECK(blocks.chg == (with_profile ? c.profile_chg : c.plain_chg));
            // Direction is checked separately from final driver output. Even the
            // direction NOT blocked by CUV/COV remains held OFF in this release.
            r.chip.safety = {{0, c.a, 0, c.b}};
            r.sample(r.monitor.snapshot().sequence + 1);
            for (auto request : {PermissionRequest{true,false}, PermissionRequest{false,true}, PermissionRequest{true,true}})
                CHECK(r.monitor.requestPermission(request) == PermissionResult::SafetyFault);
            CHECK(r.monitor.inhibitRequired());
            CHECK(r.chip.fet_control == 0x0c && (r.chip.battery & 0x010c) == 0);
            CHECK(r.chip.ram.at(0x901e) == 0x40);
        }
    }
}

static void missing_parameters_refused_without_bus_access() {
    const auto good = exampleProfile();
    int ProtectionProfile::* fields[] = {&ProtectionProfile::cuv_mv, &ProtectionProfile::cov_mv,
        &ProtectionProfile::cuv_delay_scans, &ProtectionProfile::cov_delay_scans,
        &ProtectionProfile::cuv_recovery_code, &ProtectionProfile::cov_recovery_code,
        &ProtectionProfile::die_ot_c, &ProtectionProfile::die_ot_recovery_c, &ProtectionProfile::die_ot_delay_scans};
    for (auto f : fields) {
        Rig r; auto p = good; p.*f = -1;
        CHECK(!r.monitor.beginAfterWake(Options(), p)); r.fault(Error::IncompleteProtectionProfile);
        CHECK(r.chip.trace.empty());
    }
    auto p = good; p.cuv_mv = 4300; CHECK(!Monitor::validProfile(p));
    p = good; p.cuv_delay_scans = 0; CHECK(!Monitor::validProfile(p));
    p = good; p.cov_recovery_code = 4; CHECK(!Monitor::validProfile(p));
    Rig r; Options o; o.io_timeout_ms = 0;
    CHECK(!r.monitor.beginAfterWake(o)); r.fault(Error::InvalidOptions);
    CHECK(r.chip.trace.empty());
}

static void acknowledge_is_not_identity_or_fets_off() {
    { Rig r; r.chip.device_number = 0x7607; r.monitor.beginAfterWake(); r.fault(Error::WrongDevice); CHECK(r.chip.direct_writes.empty()); }
    { Rig r; r.chip.battery = 0xffff; r.monitor.beginAfterWake(); r.fault(Error::InvalidStatus); }
    { Rig r; r.chip.battery = 0x0cc0; r.monitor.beginAfterWake(); r.fault(Error::NotFullAccess); CHECK(r.chip.commits.empty()); }
    { Rig r; r.chip.battery |= 0x010c; r.chip.ignore_off = true; r.monitor.beginAfterWake(); r.fault(Error::FetsNotOff); CHECK(r.chip.commits.empty()); }
    { Rig r; r.chip.battery |= 0x0100; r.monitor.beginAfterWake(); r.fault(Error::UnexpectedMode); CHECK(r.chip.commits.empty()); }
    { Rig r; r.chip.battery |= 0x4000; r.monitor.beginAfterWake(); r.sample(); CHECK(r.chip.commands.front() == 0x000e); }
}

static void config_entry_exit_and_readback_are_real() {
    { Rig r; r.chip.never_enter = true; r.monitor.beginAfterWake(); r.fault(Error::ModeTimeout); CHECK(r.chip.commits.size() == 1); }
    { Rig r; r.chip.never_exit = true; r.monitor.beginAfterWake(); r.fault(Error::ModeTimeout); CHECK(r.monitor.snapshot().sequence == 0); }
    { Rig r; r.chip.ignore_ram_write = 0x901b; r.monitor.beginAfterWake(); r.fault(Error::ReadbackMismatch); CHECK(r.chip.commands.back() == 0x0090); }
    { Rig r; r.chip.keep_por_on_exit = true; r.monitor.beginAfterWake(); r.fault(Error::UnexpectedPor); }
    { Rig r; r.chip.cb = 3; r.chip.reject_balance_clear = true; r.monitor.beginAfterWake(); r.fault(Error::BalancingActive); CHECK(r.chip.commands.empty()); }
    { Rig r; r.chip.ram[0x9017] = 1; r.monitor.beginAfterWake(); r.fault(Error::CrcModeMismatch); }
    { Rig r; r.chip.suppress_status_config_bit = true; r.monitor.beginAfterWake();
      r.fault(Error::ModeTimeout); CHECK(r.chip.commits.size() == 1); }
    { Rig r; r.chip.ram[0x9015] = 0x08; r.monitor.beginAfterWake(); r.fault(Error::ReadbackMismatch);
      CHECK(r.chip.commands.back() == 0x0090); CHECK(r.chip.regout == 0x1e); }
    { Rig r; r.chip.ram[0x9016] = 0x09; r.monitor.beginAfterWake(); r.fault(Error::ReadbackMismatch);
      CHECK(r.chip.commands.back() == 0x0090); }
}

static void indirect_protocol_integrity() {
    { Rig r; r.chip.wrong_echo = true; r.monitor.beginAfterWake(); r.fault(Error::EchoMismatch); }
    { Rig r; r.chip.bad_checksum = true; r.monitor.beginAfterWake(); r.fault(Error::ChecksumMismatch); }
    for (int n : {0, 4, 5, 7, 37, 255}) {
        Rig r; r.chip.bad_length = n; r.monitor.beginAfterWake(); r.fault(Error::InvalidLength);
    }
    // A corrupt unused byte in a 32-byte DM window still invalidates its checksum.
    Rig r; r.monitor.beginAfterWake();
    r.chip.before_complete = [](Device& d, const Transfer& t) {
        if (t.tx == Bytes{0x40} && d.commits.size() > 1) d.bad_checksum = true;
    };
    r.fault(Error::ChecksumMismatch);
}

static void transport_faults_and_deadlines() {
    for (int e : {1, 2, 3, 4}) {
        Rig r; r.monitor.beginAfterWake(); r.sample();
        r.chip.error_next = e;
        r.fault(e == 1 ? Error::Nak : e == 2 ? Error::BusError : Error::ShortTransfer);
    }
    { Rig r; r.chip.wake_naks = 10000; r.monitor.beginAfterWake(); r.fault(Error::WakeTimeout); CHECK(r.chip.commits.empty()); }
    { Rig r; r.chip.stuck_pending = true; r.monitor.beginAfterWake(); r.fault(Error::IoTimeout); CHECK(r.chip.cancel_count == 1); }
    { Rig r; r.chip.busy = true; r.monitor.beginAfterWake(); r.fault(Error::IoTimeout); CHECK(r.chip.cancel_count == 0); }
    { Rig r; r.chip.delay_polls = 12; r.monitor.beginAfterWake(); r.sample(); }
}

static void crc_failures_and_startup_mode() {
    for (int kind : {0, 1, 2}) {
        Rig r; r.chip.otp_crc = true; r.chip.reset();
        r.chip.corrupt_crc = kind == 0; r.chip.stop_separated_crc = kind == 1;
        r.chip.omit_second_crc = kind == 2;
        Options o; o.startup_crc = CrcMode::Enabled;
        r.monitor.beginAfterWake(o);
        r.fault(kind == 2 ? Error::ShortTransfer : Error::CrcMismatch);
    }
    // Known framing mismatch must not cause a write-mode heuristic fallback.
    { Rig r; r.chip.otp_crc = true; r.chip.reset(); r.monitor.beginAfterWake();
      r.until([&] { return r.monitor.diagnostics().state == State::Fault; }); CHECK(r.chip.direct_writes.empty()); }
    { Rig r; Options o; o.startup_crc = CrcMode::Enabled; r.monitor.beginAfterWake(o);
      r.fault(Error::CrcMismatch); CHECK(r.chip.direct_writes.empty()); }
}

static void reset_invalidates_generation_and_partial_samples() {
    Rig r; r.monitor.beginAfterWake(); r.sample();
    const auto generation = r.monitor.snapshot().generation;
    r.chip.after_complete = [](Device& d, const Transfer& t) {
        if (t.tx == Bytes{0x1c}) d.reset(); // POR between two cell reads and final status.
    };
    r.fault(Error::UnexpectedPor);
    CHECK(r.monitor.snapshot().generation != generation);
    CHECK(r.monitor.snapshot().sequence == 1); // Partial second frame never published.
    r.chip.after_complete = nullptr;
    CHECK(r.monitor.beginAfterWake()); r.sample();
    CHECK(r.monitor.snapshot().sequence == 1 && r.chip.ram[0x901b] == 2);
    r.monitor.notifyResetOrPowerLoss(); r.fault(Error::ExternalReset);
    // Active CRC can be lost to the actual OTP default on POR. No framing fallback.
    Rig c; c.chip.crc = true; c.chip.ram[0x9017] = 1; c.chip.battery &= ~0x80;
    Options o; o.startup_crc = CrcMode::Enabled;
    c.monitor.beginAfterWake(o); c.sample(); c.chip.reset();
    c.fault(Error::CrcMismatch);
}

static void por_during_config_cannot_escape_guard() {
    Rig r; r.monitor.beginAfterWake();
    r.chip.after_complete = [](Device& d, const Transfer& t) {
        if (t.tx == Bytes({0x40, 2}) && (d.battery & 0x20)) d.reset();
    };
    r.fault(Error::ConfigNotActive);
    CHECK(r.chip.illegal_config_commits == 0);
    CHECK(r.monitor.snapshot().sequence == 0);
}

static void fullscan_is_latched_and_fresh() {
    { Rig r; r.chip.never_init = true; r.monitor.beginAfterWake(); r.fault(Error::FrameTimeout); CHECK(r.monitor.snapshot().sequence == 0); }
    { Rig r; r.chip.never_scan = true; r.monitor.beginAfterWake(); r.fault(Error::FrameTimeout); CHECK(r.monitor.snapshot().sequence == 0); }
    { Rig r; r.monitor.beginAfterWake(); r.sample();
      r.clock.ms += 3000;
      CHECK(!r.monitor.snapshot().fresh && r.monitor.snapshot().lower_cell_mv.reason == Unavailable::Stale);
      CHECK(r.monitor.requestPermission({true,true}) == PermissionResult::StaleMeasurements); }
    { Rig r; r.monitor.beginAfterWake(); r.sample(); r.chip.never_scan = true;
      r.chip.after_complete = [](Device& d, const Transfer& t) {
          if (t.tx == Bytes({0x62,0x80,0})) d.reset();
      };
      r.fault(Error::UnexpectedPor); }
}

static void cell_and_ts_validity() {
    { Rig r; r.chip.lower = 1999; r.chip.stack = 5799; r.monitor.beginAfterWake(); r.sample();
      const auto s = r.monitor.snapshot(); CHECK(s.lower_cell_mv.available);
      CHECK(!s.upper_cell_mv.available && s.upper_cell_mv.reason == Unavailable::LowerCellBelow2V);
      CHECK(!s.cell_pair_valid && r.monitor.requestPermission({true,false}) == PermissionResult::InvalidCells); }
    { Rig r; r.chip.lower = 2000; r.chip.stack = 5800; r.monitor.beginAfterWake(); r.sample(); CHECK(r.monitor.snapshot().upper_cell_mv.available); }
    for (int v : {-1, 0, 6000, -32768}) {
        Rig r; r.chip.upper = v; r.monitor.beginAfterWake(); r.sample(); CHECK(!r.monitor.snapshot().upper_cell_mv.available);
    }
    { Rig r; r.chip.stack = 65535; r.monitor.beginAfterWake(); r.sample(); CHECK(!r.monitor.snapshot().stack_mv.available); }
    { Rig r; r.chip.stack = 7000; r.monitor.beginAfterWake(); r.sample(); CHECK(!r.monitor.snapshot().stack_consistent); }
    for (int v : {-1, 32767}) {
        Rig r; r.chip.ts = v; r.monitor.beginAfterWake(); r.sample(); CHECK(!r.monitor.snapshot().ts_adc_counts.available);
    }
}

static void runtime_drift_and_safety_readback() {
    for (std::uint16_t a : {0x901b, 0x9019, 0x901e, 0x9020, 0x9024, 0x9026, 0x9027, 0x9028}) {
        Rig r; r.monitor.beginAfterWake(); r.sample(); r.chip.ram[a] ^= 1; r.fault(Error::ReadbackMismatch);
    }
    { Rig r; r.monitor.beginAfterWake(); r.sample(); r.chip.cb = 1; r.fault(Error::BalancingActive); }
    { Rig r; r.monitor.beginAfterWake(); r.sample(); r.chip.battery |= 0x0100; r.fault(Error::UnexpectedMode); }
    { Rig r; r.monitor.beginAfterWake(); r.sample(); r.chip.battery |= 4; r.fault(Error::FetsNotOff); }
    { Rig r; r.monitor.beginAfterWake(); r.sample(); r.chip.regout |= 0x10; r.fault(Error::ReadbackMismatch); }
    { Rig r; r.monitor.beginAfterWake(Options(), exampleProfile()); r.sample(); r.chip.safety = {{0x40,0x80,1,2}};
      r.sample(2); const auto s = r.monitor.snapshot();
      CHECK(s.safety_alert_a == 0x40 && s.safety_status_a == 0x80 && s.safety_alert_b == 1 && s.safety_status_b == 2);
      CHECK(r.monitor.requestPermission({true,true}) == PermissionResult::SafetyFault); }
}

static void clock_wrap_and_scheduler_stall() {
    { Rig r; r.clock.ms = 0xfffffe00u; r.chip.reset(); r.monitor.beginAfterWake(); r.sample(2); CHECK(r.monitor.snapshot().fresh); }
    { Rig r; r.monitor.beginAfterWake(); r.until([&] { return r.chip.active; }); r.clock.ms += 101; r.fault(Error::IoTimeout); }
    { Rig r; r.monitor.beginAfterWake(); r.clock.ms += 15001; r.fault(Error::WakeTimeout); }
}

static void restart_cancels_only_owned_transaction() {
    Rig r; r.monitor.beginAfterWake(); r.until([&] { return r.chip.active; });
    CHECK(r.monitor.beginAfterWake()); CHECK(r.chip.cancel_count == 1);
    r.sample();
    const auto n = r.chip.trace.size(); r.monitor.notifyResetOrPowerLoss();
    for (unsigned i = 0; i < 50; ++i) r.tick();
    CHECK(r.chip.trace.size() == n); // Fault latches; no automatic retry/ON after failure.
}

static void every_initialization_transfer_can_fail_closed() {
    Rig baseline; baseline.monitor.beginAfterWake(Options(), exampleProfile()); baseline.sample();
    const auto count = baseline.chip.trace.size();
    CHECK(count > 200);
    for (std::size_t index = 0; index < count; ++index) {
        Rig r; r.monitor.beginAfterWake(Options(), exampleProfile());
        // Report one byte fewer AFTER the transaction side effects. This models
        // ambiguous completion even when the length write already committed RAM.
        r.chip.after_complete = [index](Device& d, const Transfer&) {
            if (d.trace.size() == index + 1) d.error_next = 4;
        };
        r.fault(Error::ShortTransfer);
        CHECK(r.monitor.snapshot().sequence == 0);
        const auto transactions = r.chip.trace.size();
        for (unsigned i = 0; i < 5; ++i) r.tick();
        CHECK(r.chip.trace.size() == transactions);
    }
    std::cout << "  fault injection covered all " << count << " initialization bus transfers\n";
}

int main() {
    const std::pair<const char*, void(*)()> tests[] = {
        {"TI captured CRC vectors", ti_golden_vectors},
        {"source command table and real mode sequence", source_table_and_sequence},
        {"explicit profile is never charge permission", verified_profile_is_not_charge_permission},
        {"CUV/DSG, COV/CHG and shared faults remain held OFF", protection_direction_is_independent_of_held_off},
        {"every missing profile parameter rejected", missing_parameters_refused_without_bus_access},
        {"ACK alone cannot prove identity or FET OFF", acknowledge_is_not_identity_or_fets_off},
        {"entry/exit/readback require actual state", config_entry_exit_and_readback_are_real},
        {"indirect echo/checksum/length", indirect_protocol_integrity},
        {"NAK/short read/short write/busy/timeout", transport_faults_and_deadlines},
        {"CRC faults and startup mode", crc_failures_and_startup_mode},
        {"POR invalidates published and partial samples", reset_invalidates_generation_and_partial_samples},
        {"POR during config before commit", por_during_config_cannot_escape_guard},
        {"latched scan and stale data", fullscan_is_latched_and_fresh},
        {"physical 2S and TS validity", cell_and_ts_validity},
        {"runtime config drift and safety flags", runtime_drift_and_safety_readback},
        {"clock wrap and scheduler stall", clock_wrap_and_scheduler_stall},
        {"explicit restart and latched failure", restart_cancels_only_owned_transaction},
        {"ambiguous completion at every initialization transfer", every_initialization_transfer_can_fail_closed}
    };
    unsigned passed = 0;
    for (const auto& test : tests) {
        try { test.second(); ++passed; std::cout << "PASS " << test.first << '\n'; }
        catch (const std::exception& e) { std::cerr << "FAIL " << test.first << ": " << e.what() << '\n'; return 1; }
    }
    std::cout << passed << " test groups passed (independent TI protocol model; no hardware).\n";
}
