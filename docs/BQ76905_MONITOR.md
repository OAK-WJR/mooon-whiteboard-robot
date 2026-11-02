# BQ76905 standalone monitor driver

This delivery implements **2S monitoring, volatile configuration verification and data invalidation after faults**. `Monitoring` only means the configure-and-read-back flow is done; it **does not mean charging or discharging is allowed**. The current implementation always requires the external inhibit signal to stay active; there is no path that toggles CHG_ON, DSG_ON or FET_ENABLE, or that lifts the inhibit automatically.

The original standalone delivery was not connected to the main firmware. On 2026-09-08 the [main loop, GPIO39 wake and host cells diagnostics integration](PICO_CELL_RUNTIME.md) was done; it still did not access hardware, a serial port or USB. The standalone driver delivery and examples below are kept as historical interface notes; mechanical evidence is unrelated to the software test results.

## Files and re-running

| File | Contents |
|---|---|
| `include/bq76905_monitor.h` | Public C++11 interface with no Arduino dependency |
| `src/bq76905_monitor.cpp` | Fixed-memory, non-blocking finite state machine |
| `control_tests/bq76905/test_monitor.cpp` | Independent TI protocol simulator, source-table examples and fault injection |
| `control_tests/bq76905/source_contract.json` | Kept document identifiers, 32 checks against the actual source text, and known conflicts |
| `control_tests/bq76905/run_tests.py` | Only compiles the driver and tests above; does not run the project firmware build |
| `control_tests/bq76905/test_report.json` | This run's build and test results, with the matching code digests |
| `control_tests/bq76905/.gitignore` | Isolates this folder's temporary build output |
| `docs/BQ76905_MONITOR.md` | This document and the constraints still to integrate |

Run from the repository root:

```sh
python3 control_tests/bq76905/run_tests.py
```

By default it uses `clang++ -std=c++11 -Wall -Wextra -Werror -pedantic`, with both AddressSanitizer and UndefinedBehaviorSanitizer enabled. `--compiler g++` can be passed explicitly; hosts without sanitizers can use `--no-sanitizers`, and the report records this limit. All build output stays in `.build/` in this test folder.

This verification passed **18 test groups, incomplete-completion injection at each of all 296 initialization transfers, and 32 kept source-text checks**. 296 is the total number of simulated bus transactions needed from full explicit voltage/internal over-temperature configuration to the first snapshot; it is not measured device timing or a performance promise. Each interruption point runs separately from the initial state, and the test verifies that no half frame is published and nothing is released.

The simulator implements protocol rules such as the actual registers, delayed mode switching, CFG write conditions, W1C latches, length commit and the 0x61 auto window shift; it does not import the driver's internal command constants. The CRC input/output bytes from the TI waveforms are a fixed external reference. Tests cover NAK, short read, short write, bus error, always busy, always Pending, wrong echo/length/checksum/config read-back, CFG not entered/not exited, POR while running and before config commit, CRC falling back to the OTP default, stale data, timer wraparound, lower cell below 2 V, upper cell abnormal, TS out of range, protection status and config drift while running.

The protection direction test also independently decodes the routing bytes actually written, using TRM tables 11-2/4 and 12-12/13/14, and checks 34 fault combinations with and without a profile. It verifies in isolation "which channel this protection adds a turn-off condition to", while confirming item by item that the actual CHG/DSG are still held OFF and that all charge/discharge requests are still rejected. Checking only the final double OFF would hide routing errors, so it is not used as the only evidence that the direction is correct.

This is software and protocol simulation, **not simulation/certification of the battery, protection MOSFETs, power chain or real charging**.

## Sources used and corrections

First ran `hyperresearch search`, which found no local vault; then read the existing `cell_monitor_integration/sources/bq76905.pdf`, `bq76905_trm.pdf` and their kept text, and checked TI's official documents. No external search key was used. The SHA of the original PDFs and the checks against the actual text tables are saved in the test contract; the original documents were not rewritten.

| Conclusion | Basis |
|---|---|
| Cell1 voltage `0x14`, physical Cell5 voltage `0x1C`, both I2/mV | [SLUUCI8](https://www.ti.com/lit/ug/sluuci8/sluuci8.pdf), tables 5-1, 11-1 |
| 2S upper cell on VC5–VC4A, lower cell on VC1–VC0; `VcellMode=2` | SLUUCI8 tables 5-8, 12-6 |
| `VcellMode` is at `0x901B`, width **H1** | SLUUCI8 table 12-6, data memory summary |
| Stack `0x26` is U2/mV; TS `0x2A` is a signed ADC code | SLUUCI8 table 11-1 |
| Device ID returns `0x7605` | [SLUAAM4 §2.1](https://www.ti.com/document-viewer/lit/html/SLUAAM4/GUID-4CF2CBF4-59FC-4D52-BF6B-6DBF6FD985E0) |
| CRC is off by default; changing the bit only takes effect after leaving CFG | SLUUCI8 §9.1, table 12-4 |
| DSG/CHG bit7 are CUV/COV respectively, and each bit0 is OTINT; Both bit0 is REGOUT | SLUUCI8 tables 12-12, 12-13, 12-14 |
| Fixed waveform reference for the CRC first byte and following bytes | [SLUAAM4 §4](https://www.ti.com/document-viewer/lit/html/SLUAAM4/GUID-8BA27471-7E9C-46CC-9D03-ECBD6DC8419B) |
| Electrical input conditions and standard device information | [SLUSE97](https://www.ti.com/lit/ds/symlink/bq76905.pdf) |

Three conflicts in the documents are handled explicitly:

1. The old address text in TRM §5.1.2 cannot override the real command table; the 2S upper cell cannot be read from Cell2/`0x16`.
2. The summary table in software guide §3.4 lists Vcell Mode as H2; here H1 is used per the TRM, to avoid overwriting the adjacent `0x901C` Alarm Mask.
3. The REGOUT default summary `0x08` does not match the 3.3 V default text. The driver reads the actual `REGOUT CONTROL` and DM `REGOUT Config`, and requires the low four bits to match exactly; it does not guess the voltage, and does not change its config. Leaving CFG reloads this value, so on a mismatch it refuses to continue before leaving.

## Run sequence and inhibit control

1. The external circuit holds the inhibit first; call `beginAfterWake()` after the TS/wake timing is done. The driver does not generate a GPIO wake pulse.
2. Read the real Battery Status; only in this first read-only phase are NAKs retried, with a deadline (the actual backend cannot tell an address NAK from a register-byte NAK); an uninitialized SEC also gets only a limited wait. Confirm `0x7605` through an indirect `DEVICE_NUMBER` read, with echo, length and checksum.
3. Write `FET CONTROL=0x0C` to request double OFF, and check the actual CHG/DSG bits in Battery Status. HOST_FETOFF_EN is not on by default, so **a successful write is not by itself evidence of double OFF**. It refuses to take over a device with autonomous FET control already on, an actual output high, SEALED, or already put in CFG by another host; it does not blindly send FET_ENABLE, which would toggle the state, and does not try to guess the unseal key.
4. If in DEEPSLEEP, send EXIT_DEEPSLEEP and wait for status confirmation. Write `CB_ACTIVE_CELLS=0` and read back zero; an abnormal device that already has writes disabled but is still balancing is refused.
5. Send SET_CFGUPDATE `0x0090`, and start DM configuration only **after seeing CFGUPDATE=1**. Before each DM commit, check CFG/SEC/POR/actual FET state again; after each write, read back and compare, then check CFG state.
6. Verify that the CRC bit matches the call parameter; configure the items in the table below. Clear TS_ON, but keep the existing LDO enable and voltage. Check that the LDO value and I2C address that DM will load on exit also match the current values.
7. While measurement is paused, set and clear the needed latched events; send EXIT_CFGUPDATE `0x0092`, and wait for CFGUPDATE=0 and POR=0. Request double OFF again, disable SLEEP, and confirm the actual outputs, autonomous control, FET CONTROL and Alarm Enable.
8. Use the enabled and cleared **Alarm Status latches INITCOMP/FULLSCAN** to wait for a real new measurement. The bits with the same names in Raw Status are short pulses and cannot be polled as latched events. While waiting, alternate with reading Battery Status to detect POR.
9. Each round, first check status and audit all configured DM, CRC, LDO, I2C address, FET CONTROL and balancing state; clear FULLSCAN and wait for the next latch. Read Cell1, Cell5, Stack, TS, Safety, Alarm; check Battery Status again, and only then publish the snapshot.

An indirect read fetches the 32-byte buffer, checksum and length (reading `0x61` comes last); the checksum is computed over the data length the device returns, and the range and the command's expected length are checked. Reading length auto-shifts the window, so the next operation always selects the address again. Writes go in the order address, buffer data, checksum/length; length is committed last. On any length/checksum failure, no partial data is used.

| Config | Current written value/source |
|---|---|
| FET Options `0x901E` | `0x40`: only HOST_FETOFF allowed; HOST_FETON, FET_EN, SFET body diode override, PWM, etc. disabled |
| Vcell Mode `0x901B` | `2`, single byte |
| DA Config `0x9019` | `0x01C0`: TSMODE=voltage input, CCMODE=current conversion off, standard ADC speed |
| Balancing Configuration `0x9020` | `0x03`: balancing write commands disabled, standard 1 ms wait kept; the actual balancing state must also be zero |
| Default Alarm Mask `0x901C` | `0xC285`: standard sources plus FULLSCAN, INITCOMP, POR |
| Enabled Protections A/B | No explicit profile: `0x01/0x00`, with only REGOUT among these enable bits; with a full voltage/internal over-temperature profile: `0xC1/0x02` |
| DSG FET Protections A `0x9026` | With profile: `0x81` = CUV(bit7) + OTINT(bit0); without profile: `0x00` |
| CHG FET Protections A `0x9027` | With profile: `0x81` = COV(bit7) + OTINT(bit0); without profile: `0x00` |
| Both FET Protections B `0x9028` | `0x07` in both modes: keeps the default shared turn-off mapping for VREF(bit2)/VSS(bit1) and adds REGOUT(bit0); this byte has no OTINT bit |
| Current and TS temperature protection | Disabled: the current hardware cannot provide real protection decisions for these quantities |
| I2C Config, REGOUT Config, I2C Address | **Read-only check, not changed** |

The no-profile mode does not depend on the chip's default COV/CUV thresholds; it explicitly turns off unconfigured protection items, and always requires the external inhibit. Even with the voltage/internal over-temperature items configured, the current outputs are still OFF.

This round fixed an implementation error that reused the Enabled Protections A/B masks directly for FET routing. The two register groups have different bit definitions; for example, the OTINT enable is at `0x9025.bit1`, but its turn-off routing is at `0x9026.bit0` and `0x9027.bit0`, while `0x9028.bit1` is VSS. What VREF/VSS keep here is the default turn-off routing; this is not grounds for claiming the diagnostic function is fully verified.

The protection directions of the explicit profile are below; they only show the extra protection turn-off condition that each fault applies:

| Fault | DSG protection turn-off | CHG protection turn-off |
|---|---|---|
| CUV | Yes | No |
| COV | No | Yes |
| Internal OTINT | Yes | Yes |
| REGOUT | Yes | Yes |
| VREF / VSS | Yes | Yes |

The CUV mapping only adds a protection turn-off to DSG, and the COV mapping only adds one to CHG. **Outputs in a direction that this protection does not block are still bound by the global held OFF and the permission inhibit; this delivery does not approve resuming charging or any release.**

## Interface for the main work stream

Per the CM3 convention given by the main work stream this round: GPIO39/contact19 is `WAKE_REQUEST_N`, GPIO41/contact22 is `CHARGE_REQUEST`, and GPIO40 stays NC. This only records the handoff convention; the standalone driver does not bind these pins, does not output requests, and did not change the main schematic, board profile or main firmware.

`I2c` and `Clock` must outlive `Monitor`; Monitor cannot be copied. The I2C adapter must meet these requirements:

- `start()`, `poll()` and `cancel()` all return promptly. `start=false` means no bus operation has been done yet; Pending must not imply a blocking wait.
- Reads use a **repeated START** after the register pointer, and STOP after NACK on the last byte. It cannot be swapped for another read style that uses STOP while still using the CRC here.
- Supports up to 69 bytes TX and 68 bytes RX. It must not use a fixed 32-byte Wire buffer and silently truncate.
- The completion count is the number of bytes in the passed buffer, including CRC, **not including address/RW and ACK bits**. Short counts on read or write both report errors.
- Before cancel returns, DMA/IRQ and the like must no longer access the old buffer. When Monitor is destroyed or explicitly restarted, it cancels any transaction it still holds.
- This driver owns this BQ's indirect command window exclusively; other I2C devices can be arbitrated by the adapter, but another BQ client must not interleave writes to `0x3E…0x61`.

Each `tick()` calls adapter start or poll at most once; on timeout it can call cancel once instead; there is no loop that waits on hardware. Default parameters are driven at a 1 ms tick in host tests; for platform integration we suggest keeping this scheduling interval first and measuring the real run time. Slower calls may hit frame timeouts; real-time performance cannot simply be claimed as verified.

```cpp
// adapter and clock are implemented by the platform; this code is not connected to the current top_v2_main.cpp.
bq76905::Monitor monitor(adapter, clock);
bq76905::Options options;
options.startup_crc = bq76905::CrcMode::Disabled; // must match the actual device/existing running state
// With the external inhibit already active, after the hardware wake timing ends:
bool accepted = monitor.beginAfterWake(options);

// Each scheduler iteration:
monitor.tick();
auto diagnostic = monitor.diagnostics();
auto frame = monitor.snapshot();
// Use the values only when frame.fresh / each Reading.available is true.
// Unavailable current_ma / pack_temperature_mc must be output as null in JSON.
// diagnostic.configuration_verified cannot be used as charge permission.

// Call immediately on a known chip reset, power loss or entry into SHUTDOWN:
monitor.notifyResetOrPowerLoss();
```

Default time parameters: single transaction 100 ms, first response window 2000 ms, mode confirmation 1000 ms, full initialization 15000 ms, startup event/single-round acquisition 1500 ms, poll interval 5 ms, minimum interval between rounds 250 ms, maximum data age 3000 ms. These are explicit software deadlines and have not been verified as safety response times of the real power chain. The clock uses monotonic `uint32_t` milliseconds and handles wraparound; wall-clock time is never used.

Errors latch Fault, which invalidates the config and profile verification flags, makes the snapshot unavailable and changes generation; it does not automatically retry configuration or restore permission. After fixing the cause and completing the external wake/inhibit conditions again, an explicit `beginAfterWake()` starts a new generation. Data is invalidated as soon as POR is observed; when CRC falls back to a different OTP default with POR, this may first show up as a CRC/communication error, and is invalidated the same way. The host cannot know about a physical POR in the moment before it reads the status, so the external default inhibit cannot rely on this polling window.

If the fault happens inside CFG, the chip may still be stuck in CONFIG_UPDATE. To avoid applying a partial config to the running state, the driver does not blindly send EXIT_CFGUPDATE on a fault; a later begin also refuses to take over a chip that is already in CFG. In this case the integrator must run a verified BQ reset/recovery flow while the external inhibit is active, then reinitialize. Rebooting only the MCU or sending another wake pulse is not guaranteed to clear the BQ's CFG state. The current API does not send RESET, SHUTDOWN or unseal commands.

`fets_off_observed` is the last observed value, not physical output feedback or a hard-wired guarantee. On a communication fault, the driver cannot guarantee that another OFF command would be carried out by the chip; the integrator must keep a separate inhibit circuit. After a fault/expiry `status_available=false`; old status numbers must not be taken as the current state.

## Data validity and CRC

- The lower cell is physical Cell1 and the upper cell is physical Cell5. The upper cell can only be valid when the lower cell measurement is valid and at least 2000 mV; when the lower cell is below 2 V, the upper cell returns the unavailable reason `LowerCellBelow2V`.
- A cell value that is non-positive or above 5500 mV, or a Stack that is non-positive or above the two-cell 11000 mV, is unavailable. The difference between Cell1+Cell5 and Stack must also meet the explicit `stack_agreement_mv` (default 250 mV) before the pair is marked valid. This tolerance is a diagnostic parameter for sequential sampling and cannot replace protection thresholds or calibration.
- Reading registers is not an atomic ADC snapshot; `read_span_ms` records the software read span, which may cross a conversion update. Passing communication and status checks does not prove that the samples happened at the same time, or that the harness has no open circuit.
- TS returns the raw signed ADC code and nominal microvolts, converted as 1.1962 V × 5/3 ÷ 32768; only the nominal 0–1.8 V range is accepted. There is no thermistor conversion; it is not battery temperature.
- SRP/SRN connect to RAW B−, so current is always `available=false` with reason `NoCurrentShunt`; the current command is not read, and a raw zero is not taken as 0 A.
- `snapshot()` recomputes the expired state when called, so even if the scheduler pauses, the getter never marks over-age values as fresh. Old copies cached by the integrator must still check age/generation.
- CRC is chosen explicitly by `Options.startup_crc` and checked against the DM bit. A standard non-customized device starts with it off by default; an existing running state or custom OTP must be selected to match reality. It does not auto-detect and switch, change the CRC bit, or downgrade a CRC error to no CRC.
- CRC8 polynomial 0x07, initial value 0; each data byte is checked separately. The first write byte includes the write address/register; the first read byte includes the address and register before and after the repeated START. The implementation verifies after the full transaction ends; any CRC failure discards the whole frame. The adapter may abort on a wrong CRC early when the hardware supports it; the current interface does not require byte-by-byte early NACK.

## Explicit protection parameters and refusing release

`ProtectionProfile.requested=false` is the default. If it is set to true, all of the parameters below must be given explicitly and be valid; if any one is missing, it is refused before any bus access:

| Parameter | Unit/constraint | DM address |
|---|---|---|
| `cuv_mv` / `cov_mv` | mV; the driver limits CUV ≥2000, COV>CUV, both ≤5500 | `0x902E` / `0x9032`, each I2 |
| `cuv_delay_scans` / `cov_delay_scans` | 1–255 ADSCANs, not milliseconds | `0x9030` / `0x9034` |
| `cuv_recovery_code` / `cov_recovery_code` | 0=no auto recovery; 1/2/3=50/100/200 mV; recovery windows must not overlap | `0x9031` / `0x9035` |
| `die_ot_c` / `die_ot_recovery_c` | Internal chip temperature of 25–150 °C; recovery must be lower | `0x904C` / `0x904E` |
| `die_ot_delay_scans` | 1–255 internal temperature measurements | `0x904D` |

It is only a set of voltage/internal chip over-temperature settings that have been read back. Numbers such as 2800/4200 mV in the tests are test inputs, **not approved for charging the GNB battery**.

`requestPermission({charge, discharge})` has no bus or GPIO side effects, and returns based on the current evidence: not initialized, data expired, cells invalid, protection fault, missing profile, or `SystemProtectionUnqualified`. Both false returns `HeldOff`. Even if the profile is fully written and read back and the voltages are normal, it returns `SystemProtectionUnqualified`, and `inhibitRequired()` is always true.

Full protection release for the whole system still lacks: battery temperature and current protection information, all thresholds approved for the specific battery, and joint verification of the protection/charging module and the actual CHG/DSG permission circuit. No user-checkable "confirmed" boolean can bypass these gaps. To implement a real power release, the main work stream must separately complete and review the system protection logic; this driver's current API provides no way to enable it.

The remaining integration work is explicitly limited to: platform non-blocking I2C/clock adapter, external wake/default inhibit control, reset notification, and serialization of snapshots with null semantics. Later hardware tests still need to verify the actual OTP/CRC/LDO, 68-byte reads and cancel behavior, sampling error, bus timeouts, power loss/reset and protection circuit action. This delivery does not claim that these have passed.
