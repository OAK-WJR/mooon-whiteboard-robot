# ESP32-S3-Pico-M firmware review profile

Select **`top_board_v2_pico`** explicitly. Its protocol identity is
`top_board_v2_pico_review`. The default environment is unchanged. This is the
current 14-sheet electrical handoff, with motion and charging unqualified.

The pin authority is the adopted
[controller contract](../hardware/top_board_v2/kiss_6v/power_integration/controller_contract.json),
[logic recovery contract](../hardware/top_board_v2/kiss_6v/power_integration/logic_recovery_contract.json),
and [integration README](../hardware/top_board_v2/kiss_6v/power_integration/README.md).
Use MCU1's `physical_contacts` together with `pin_nets`: the older
`gpio_assignments` subsection is not the complete pin authority. There are now
**21 application GPIOs**, including the adopted logic status/recovery, wake and
charge request connections. The verifier also exports the actual native main
schematic to a temporary XML netlist and compares the observed consumers to it.

## Selection and build

```sh
# Reproducible validation, including cross-builds, without the private config:
python3 control_tests/verify_pico_profile.py --build

# Native checks only, using the installed ArduinoJson and host C++ compiler:
python3 control_tests/verify_pico_profile.py

# Explicit ordinary PlatformIO compile target (no upload or monitor):
./.venv/bin/pio run -e top_board_v2_pico
```

The verifier compiles disposable public copies of `src/`, `include/`, and the
focused tests. It replaces `include/config.h` **in the temporary copy only** with
a failing sentinel, and checks compiler dependencies. It never reads, copies,
hashes, or changes the real private config. It uses the project's existing
PlatformIO cache; it does not run the older verification scripts, which have
broader side effects and may read private configuration.

`firmware_config.h` selects public Pico configuration before any legacy include.
The environment also pre-includes `pico_build_config.h` across translation units.
Wrong MCU, native-CDC console, mixed R3/Pico flags, stale preloaded configuration,
missing explicit profile, and incompatible entry points fail compilation.
Runtime credentials still come from the existing USB provisioning/NVS record.

Changed paths in the firmware handoff (frozen during the host follow-up):

| Purpose | Paths |
|---|---|
| New public profile/selection/safe defaults | `include/board_profile_pico.h`, `include/pico_build_config.h`, `include/firmware_config.h`, `include/pico_gpio_safe.h` |
| Explicit build environment | `platformio.ini` |
| Runtime, UART selection, diagnostics, framing | `src/top_v2_main.cpp`, `src/top_v2_platform.h`, `src/main.cpp`, `src/control_transport.h` |
| Checked inhibited motor path and encoder selection | `src/motor_driver.cpp`, `src/motor_driver.h`, `src/odometry.cpp` |
| Route remaining configuration consumers through selection | `src/robot_controller.h`, `src/path_planner.h`, `src/trajectory_follower.h`, `src/wifi_comm.h`, `src/motor_test_main.cpp` |
| Focused validation | `control_tests/test_pico_profile.cpp`, `control_tests/verify_pico_profile.py`, `control_tests/pico_mock/sdkconfig.h`, `control_tests/pico_mock/driver/gpio.h`, one macro guard in `control_tests/runtime_stubs.h` |
| Handoff | `docs/PICO_PROFILE.md` |

`esp32-s3-devkitc-1` supplies the generic S3 toolchain/Arduino variant, with 16 MB
flash, the framework's `default_16MB.csv` partition table, and `qio_qspi` memory
mode. It does not define this carrier's application pins. PSRAM is not enabled
(`BOARD_HAS_PSRAM` is absent); no memory capacity or board boot behavior has been
measured. The manufacturer identifies an S3R2/16 MB flash/2 MB PSRAM module, but
its current page also contains an Octal-PSRAM pin warning. The profile reserves
GPIO33–37 regardless and leaves PSRAM qualification open.
[Waveshare product documentation](https://docs.waveshare.com/ESP32-S3-Pico)

## Application pins

Numbers in parentheses are **physical module contacts**, not GPIO numbers.

| Header / physical wheel | FI | BI | Encoder A | Encoder B |
|---|---|---|---|---|
| M0 → H2 → Motor1 | GPIO4 (26) | GPIO5 (27) | GPIO10 (35) | GPIO11 (1) |
| M1 → H1 → Motor3 | GPIO6 (29) | GPIO7 (31) | GPIO12 (2) | GPIO13 (4) |
| M2 → H3 → Motor2 | GPIO8 (32) | GPIO9 (34) | GPIO14 (5) | GPIO15 (6) |

The existing controller arrays retain physical Motor1/2/3 order, so the firmware
`MOTOR0/1/2_*` and `ENC0/1/2_*` macros resolve through **M0/M2/M1**. The profile's
`headers` array is in connector order. Encoder direction defaults are neutral
`+1/+1/+1`, without inheriting the old harness's sign correction.

| Signal | GPIO (contact) | Current runtime behavior |
|---|---|---|
| MOTOR_ARM_REQUEST | 16 (7) | Checked LOW before motor initialization; held LOW |
| WD_HEARTBEAT_OD | 17 (9) | Open drain; checked 2µs LOW pulses earned by control progress |
| I2C_SDA / I2C_SCL | 1 (24) / 2 (25) | Reserved by profile; bus not initialized |
| LOGIC_FAULT_N | 18 (10) | Input, no internal pull; report raw active-low level |
| LOGIC_PGOOD | 38 (17) | Input, no internal pull; report raw level |
| LOGIC_CYCLE_REQUEST | 42 (21) | Checked LOW; Q40 shutdown request inactive |
| WAKE_REQUEST_N | 39 (19) | Released open drain; R101 10k to MAIN_3V3 keeps wake inactive |
| CHARGE_REQUEST | 41 (22) | Checked push-pull LOW; R112 330Ω to U111.6, R113 10k pulldown |

Application code leaves factory UART43/44, native USB19/20, straps0/3/45/46,
LED21, and memory33–37 reserved. UART0 alone owns TX43/RX44 for the factory
CH343P/CH334F data/recovery path. GPIO40 (contact20) remains NC and untouched. In particular,
physical contact **21 is GPIO42**, while **GPIO21 belongs to the RGB LED**.
Contact30 is CHIP_PU/RUN; contact37 is the module regulator enable and stays NC.
No external JTAG use of GPIO42 or eFuse changes are introduced.

## Startup and status semantics

`pico::SafeDefaults` uses checked IDF GPIO calls, preloading ARM, cycle and charge
LOW before enabling their push-pull outputs. For WDI and wake, it first selects
INPUT to disable any retained output, preloads HIGH, and then selects open-drain
mode. It never drives either signal push-pull HIGH. `holdLow()` checks ARM,
cycle and charge LOW and wake released HIGH; it never requests wake LOW or
charge HIGH. FLT/PGOOD use the hardware's external pullups. This follows the
[IDF GPIO configuration API](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html)
and the locally installed framework headers.

On GPIO failure, the startup state stays latched. The code attempts to release
ARM/cycle/charge/WDI/wake to INPUT so their external default biases can act, reports both the initiating and
release error, and retains USB status/stop/software-reboot access. A successful
later write cannot qualify a previously failed startup. Calls and external bias
assumptions are not proof of physical LOW or motor power isolation. Wake's R101
pullup drives U94's input inactive; charge's R113 pulldown is on the far side of
R112 and keeps U111's final AND input inactive after a successful GPIO release.
A failed release remains reported rather than claiming that hardware bias won.

The Pico motor driver uses checked FI/BI LOW initialization, never attaches a
PWM peripheral, and rejects nonzero speed requests even below the command gate.
`pwm_ready=false` with no GPIO error is intentional. GPIO/driver failures remain
reported. The normal motion commands remain rejected as
`BOARD_POWER_UNREVIEWED`; no arm edge is generated.

`logic_flt_n` and `logic_pgood` are sampled digital levels, or `null` after a GPIO
startup fault. They do not establish battery health, USB sustain, voltage
windows, or VMOT discharge. The [INA runtime](PICO_RAIL_RUNTIME.md) now supplies
`motor_voltage_mv`, `usb_vbus_mv` and `raw_logic_5v_mv` only for a complete,
unexpired conversion group, with `rail_monitor` age/sequence/error metadata.
No real-device readings are claimed. `motor_current_ma` and `rail_3v3_mv` remain
`null`. Encoder counts are raw diagnostics; pose and wheel-speed fields are
`null`, with `odometry_qualified=false` until the new harness is qualified.

`source_cycle_enabled`, `motor_arm_requested`, `motion_enabled`, and
`charge_enabled` remain false. `watchdog_ready` describes the **task watchdog**;
`external_watchdog_ready` now describes the GPIO17 progress service below.
Neither flag measures the external supervisor, motor isolation, or supply health.

`source_cycle` and `logic_cycle` commands return `SOURCE_CYCLE_UNAVAILABLE` after
normal authentication/ownership checks, including when a caller supplies
plausible measurement values. The cycle GPIO never requests HIGH. Hardware
qualification must establish USB sustain, ARM off and measured VMOT, measured
RAW5 below 1.8 V before release, and restored voltage/PGOOD with timeout and
missing-reading handling. Battery-only loss of power, a command ACK, and
`esp_restart()` cannot establish recovery success.

## External watchdog service and boot/ROM behavior

The watchdog follow-up first read the current native schematic/contract and the
local [TI TPS3851 Rev. B text](../hardware/top_board_v2/kicad/r3_watchdog/sources/tps3851.txt),
then checked the [TI primary datasheet](https://www.ti.com/lit/ds/symlink/tps3851.pdf).
The earlier watchdog handoff's native export on 2026-09-07 had **nine sheets**
(the current main has 14); its watchdog connection was:

| Net | Connected watchdog/controller pins |
|---|---|
| `EN_CHIP_PU` | MCU1.30 (RUN), U20.3 (**MR input**), XCWD1.EN |
| `SAFETY_CLEAR_N` | U20.7 (**WDO output**), U20.8 (**RESET output**), U21.1/U21.13 (latch clears), U30.4 (FLT), R703 pullup |
| `WD_HEARTBEAT_OD` | MCU1.9 (GPIO17), U20.6 (WDI), U21.3 (heartbeat-seen clock), R702 pullup, XCWD1.WDI |

**Correction to the earlier handoff:** TPS3851 timeout does not reset the MCU
on this wiring. Its outputs clear motor permission, while RUN drives its MR
input. They are not connected to RUN or logic-power shutdown. Consequently,
bootloader/ROM dwell without WDI may repeatedly clear the latch but cannot
create an MCU reset loop through U20. No hardware boot-bypass correction is
required for the current connection. Independent MCU reset recovery is not
provided by U20; the application task watchdog is a separate mechanism.

U20 is TPS3851H33E, with SET1 tied HIGH and CWD pulled to 3V3 through 10kΩ.
The selected watchdog timeout and reset-output duration are each
170/200/230ms (minimum/typical/maximum). WDI responds to falling edges and ignores
them during RESET or WDO assertion; the minimum LOW pulse is 50ns. A RUN reset
asserts MR, which holds the safety clear for the reset delay after RUN rises.
These are safety-latch intervals, not an application-start deadline. See TI
§5.6, §6.3.2, §6.3.4 and §7.1.1.1.

Pico enrolls the setup/control task in the existing task watchdog with a
**5-second startup budget before UART, NVS and radio initialization**. No external
pulses are emitted during those blocking calls. If a call returns at or beyond
the budget, startup restarts without renewing that deadline. Successful startup
changes the task watchdog to its normal 3-second budget without subscribing
twice. Registration/reconfiguration/reset errors prevent external readiness.
The checked API semantics come from the installed S3 `esp_task_wdt.h`; this
deadline requires the SDK watchdog/panic machinery to operate correctly.

`pico::ExternalWatchdog` runs only in the control task. A completed iteration
must return from transport processing and the actual robot update, retain
checked ARM/cycle/charge LOW and wake released outputs, and successfully service the task watchdog before
it earns a pulse. The nominal falling-edge interval is **40ms**. Every pulse uses
checked OD LOW, a 2µs minimum software delay, and checked OD release HIGH.
The scheduler requires progress no older than **100ms** and a previous pulse no
older than **120ms**, leaving 50ms before the supervisor's minimum timeout.
There is no timer, ISR, separate pulse task, or heartbeat driven by host traffic.

The service enters `acquiring` after startup and advertises `active` only after
500ms of sustained fresh service. That interval exceeds a previously asserted
WDO's maximum hold plus the next allowed pulse. An independent native timing
model checks emitted pulses against all nine min/typ/max tWD/tRST combinations.
The `active` state is software evidence, not readback of U20 or the latch;
GPIO18/38 still report unrelated raw logic-supply FLT/PGOOD only.

Freshness is checked before replacing the previous progress timestamp and again
after work returns. A stalled loop emits no pulses. A late control completion,
live RX loop without control updates, or task-watchdog failure stops service;
later progress cannot clear that fault without reboot. WDI SDK failures use the
existing latched GPIO-fault path, including output-release error reporting.
ARM remains LOW during heartbeat release, so the heartbeat-seen latch cannot
produce a fresh ARM clock or motor permit. Status/stop/software-reboot remain
available; GPIO-fault recovery retains its USB-only path. Healthy reboot ACK
draining continues to earn progress pulses until `esp_restart()`.

## Transports

The existing USB protocol now uses UART0 `Serial` at 115200 on RX44/TX43 for
Pico. Both the actual `HardwareSerial` type and `Serial0` alias are checked in
the cross-build. The pinned core's `begin()` configures hardware flow control
disabled; no RTS/CTS pins are assigned. Arduino's
[UART API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/serial.html)
documents the explicit RX/TX interface and flow-control modes. The UART boolean
indicates driver readiness, not a connected host or usable USB power; lease
expiry still governs an absent host.

Wi-Fi STA/UDP, BLE GATT, authentication, framing, ownership, USB provisioning,
and the existing software-reboot/ACK-drain path are retained. Pico's added
diagnostics require a 1792-byte payload buffer and 2304-byte framed transport
limit; the legacy profiles keep their original sizes. Larger UDP replies can
require IP fragmentation; real network behavior remains untested.

## Explicit host selection

`scripts/robot_control.py` now accepts `top_board_v2_pico_review`. For subsequent
device sessions, replace `PORT` with an explicit local serial path:

```sh
./.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind ch343p --profile top_board_v2_pico_review status
./.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind ch343p --profile top_board_v2_pico_review reboot
./.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind ch343p --profile top_board_v2_pico_review --wifi-config wifi.json --key-file control.key provision
```

These are usage templates; validation below does not execute device commands.
`--usb-kind ch343p` itself binds the Pico application profile; the matching
`--profile` makes that selection visible. Conflicting native/CP2102N/R3 selections
fail before I/O. Wi-Fi and BLE retain their existing authenticated workflows;
select `--profile top_board_v2_pico_review` explicitly with either wireless link
and its provisioned key, without a USB-kind override.

CH343 selection requires VID:PID **1A86:55D3**, the exact requested path, and a
unique serial number or USB location for reconnects. This is a CH343-family
descriptor match, not proof of a particular Pico module or carrier. Other WCH
IDs, ambiguous bridges, and a reused path with another identity are rejected;
nonstandard descriptors need a separate review. The family ID is documented in
the [WCH driver source](https://github.com/WCHSoftGroup/ch343ser_linux/blob/main/driver/ch343.c).

The host validates the actual protocol-2 Pico schema, including normal and
latched GPIO-fault status. The three measured rails require consistent freshness
metadata; all are `null` together when unavailable. Unwired measurements and
odometry stay `null`; motion, charging, PWM, and source-cycle capability flags stay false.
`external_watchdog_ready` must agree with `external_watchdog_state`; only
`active` can report true, with a healthy task watchdog and no control timing
fault. Acquiring/stale/init-failed status remains diagnosable. Raw FLT/PGOOD bits
never qualify power recovery. Pico payloads
must occupy fewer than 1792 UTF-8 bytes; the larger limit is enabled only for an
explicit Pico selection. Legacy/R3 firmware cannot pass a Pico hello, status,
or restart check, and credentials are not provisioned before a matching hello.
Motion and source-cycle requests are also rejected locally before transmission.

Software reboot/provisioning success requires the same device and profile, a
new boot identifier, recent idle status with zero PWM, an active external
watchdog service, and a software reset reason. Reconnect verification waits for
watchdog acquisition. Returning to a GPIO fault remains diagnosable but does not pass the
healthy idle return check. These results never report a physical power cycle
or measured source-cycle success. ROM `reset`/`bootloader` select ESP32-S3 UART
recovery and `--no-stub read-mac`; a ROM-only result does not verify the Pico
application or custom PCB identity. Serial open requests disable flow control
and deassert DTR/RTS; physical reset-line behavior remains untested.

`scripts/pcb_control.py` supplies shared bridge/recovery helpers. Its own CLI
and `scripts/pcb_diagnostics.py` retain the older protocol-1 diagnostic workflow;
use `robot_control.py` for the integrated Pico profile.

The host follow-up changes only `scripts/robot_control.py`,
`scripts/pcb_control.py`, `control_tests/test_pico_host.py`,
`control_tests/pico_host_status_fixture.cpp`, and this document.

## Remaining integration

Other open items are measured watchdog pulse/tolerance/latch behavior,
measurement calibration and electrical qualification, power-cycle interlocks and acceptance, encoder polarity/CPR,
FI/BI modulation and harness qualification, charging permit integration,
memory/flash boot validation, and complete S3 USB/radio/RTOS behavior.

The current 14-sheet main connects its isolated cell wake request to GPIO39
and the final charging AND request to GPIO41. GPIO39 now performs a finite
open-drain wake; GPIO41 stays inactive. Main I2C GPIO1/2 runs BQ readout and INA
diagnostics through the shared backend. Battery-specific protection and
charging enable remain unqualified; GPIO40 stays unused. See
[PICO_CELL_RUNTIME.md](PICO_CELL_RUNTIME.md) for the current integration and
verification checkpoint. No physical bus/readout acceptance has been performed.

## Validation evidence and limits

The focused native suite uses ASAN/UBSAN and compiles the actual selected profile,
runtime, controller, driver, encoder ISRs, and transport sources. It compares
observed pin initialization order and per-pin ISR wheel responses against the
current MCU40 and recovery contracts, rather than generating another set of
pin assertions from the same table. It covers retained HIGH latches, bit42 in
GPIO masks, configuration/preload/release failures, later LOW-write failures,
all six FI/BI failure paths, unavailable telemetry, forged measurement inputs,
status frame size at counter limits, and UART/UDP/BLE reboot including queued
BLE status replies. Selection rejection checks and relevant legacy/R3 runtime,
GPIO, motor-fault, transport, and reboot regressions are included.

The `--build` run also compiles/links the Pico and R3 integrated firmware from
the public-only copy, then verifies their source dependency files contain no
private-config dependency. Native SDK calls, interrupts, NVS, clocks, and radio
boundaries are test doubles. HMAC uses macOS CommonCrypto in native tests.
These checks do **not** prove S3 runtime scheduling, real USB recovery, RF
coexistence, electrical timing, motor isolation, or any power/charging behavior.
No serial/USB device, flashing, hardware operation, or purchase is part of this
validation.

Verified on 2026-09-07 with `python3 control_tests/verify_pico_profile.py --build`:
29 Pico native scenarios, nine rejected configuration variants, the framework C
pre-include check, and six relevant legacy/R3 regression suites passed. Pico
linked at 56,740 bytes RAM / 1,178,264 bytes flash; R3 at 51,652 bytes RAM /
1,417,536 bytes flash. Those sizes describe these builds only, not runtime headroom
or hardware qualification.

Host follow-up verified offline on 2026-09-07:

```sh
./.venv/bin/python -B -m unittest discover -s control_tests -p 'test_*host.py' -q
# Original host handoff: 45 tests passed, including 19 Pico tests.
./.venv/bin/python -B -m unittest discover -s usb_diagnostics/tests -p 'test_*.py' -q
# 13 legacy diagnostic/recovery tests passed.
```

The Pico host fixture compiles the production runtime/serializer with
ASAN/UBSAN and native SDK doubles in the same public-only temporary copy. Its
normal, counter-limit (over 1200 bytes), and GPIO-fault envelopes are consumed
by the actual host parser. Tests cover profile/descriptor rejection, capability
and null-field enforcement, authentication, reconnect identity, reboot with
lost ACK, provisioning, and the installed esptool UART strategy without calling
its reset methods. Serial inventories, ports, ROM subprocesses, and radio links
are mocked. The original host-only follow-up left its firmware snapshot and
hardware files unchanged. The subsequent watchdog work changes firmware as
listed below, while leaving hardware untouched.

### Watchdog follow-up paths and verification

| Purpose | Changed paths |
|---|---|
| Progress service and checked OD pulses | `include/pico_watchdog.h` (new), `include/pico_gpio_safe.h` |
| Bounded setup, loop integration, capability status | `src/top_v2_main.cpp` |
| Host schema/restart gating | `scripts/robot_control.py` |
| Actual-runtime watchdog checks and verifier | `control_tests/test_pico_watchdog.cpp` (new), `control_tests/test_pico_profile.cpp`, `control_tests/verify_pico_profile.py` |
| Clock, critical-section, UART, task-WDT and GPIO failure injection | `control_tests/mock/Arduino.h`, `control_tests/mock/freertos/task.h`, `control_tests/runtime_stubs.h`, `control_tests/pico_mock/driver/gpio.h` |
| Production status envelopes and host checks | `control_tests/pico_host_status_fixture.cpp`, `control_tests/test_pico_host.py` |
| Corrected wiring/operation handoff | `docs/PICO_PROFILE.md` |

Run `python3 -B control_tests/verify_pico_profile.py --build` for the 29 retained
Pico scenarios plus 14 watchdog scenarios, nine rejected profile configurations,
the C pre-include check, six legacy/R3 regression suites, and public-only Pico/R3
cross-builds. Watchdog checks include slow and overdue startup, task-WDT failure,
60-second pre-application ROM dwell, clock wrap, ignored WDI during WDO assertion,
stale/missing control progress, slow control and transport calls, LOW/release/
cleanup SDK failures, and BLE reboot backlog. The host commands above now run
46 host tests (20 Pico) and 13 legacy diagnostic/recovery tests.

Verified on 2026-09-07: all of the above native, selection, regression, host and
public-only cross-build checks passed. The final Pico source snapshot was also
cross-built after making watchdog status use one shared timestamp: 56,772 bytes
RAM / 1,179,632 bytes flash. R3 remained 51,652 bytes RAM / 1,417,536 bytes flash.
Compiler dependencies contained no private-config include. These are build
sizes and offline test results, not measured runtime margins.

No private config is read/copied/hashed, no serial/USB hardware is opened, and no
flashing or hardware operation is performed. Cross-build and native models do
not establish real S3 scheduling, pulse widths, R702 rise time, supervisor/latch
levels, radio coexistence, recovery reset-line behavior, or sustained board
operation. Those remain bench acceptance items. The watchdog follow-up added no
ADC driver, wake pulse, charging enable, motion qualification or source-cycle success.

### 13-sheet wake/charge safe-default follow-up

Only `include/board_profile_pico.h`, `include/pico_gpio_safe.h`,
`control_tests/test_pico_gpio_safe.cpp` (new), `control_tests/test_pico_profile.cpp`,
`control_tests/verify_pico_profile.py` and this document change in this follow-up.
The runtime, host scripts, watchdog scheduler and hardware files are untouched.
The existing 40 ms progress-earned WD service now also checks the added safe
defaults through its existing `holdLow()` call. Failure to maintain them prevents
a heartbeat and enters the existing GPIO-fault/release path.

The new standalone GPIO test exercises every actual startup SDK call, retained
push-pull LOW wake state, repeated holds, safe defaults during a heartbeat,
hold failures before a WDI edge, and cleanup failure on each of the five outputs.
It checks software operations and release/error reporting, not physical voltages.
The runtime suite adds wake/charge initialization, hold and release failures;
all its command/reboot scenarios reject wake LOW, charge HIGH and any GPIO40 use.

The verifier now requires installed `kicad-cli` (PATH or the local KiCad app).
It exports the current main **only to its temporary directory**, verifies all
40 contacts and the **21 application GPIOs**, checks R101/R112/R113 and U94/U111
connectivity, and exports again at the end to catch a concurrent pin change.
Compilation still uses the public-only sentinel copy and dependency checks.

Verified on 2026-09-07 with `python3 -B control_tests/verify_pico_profile.py --build`:
24 standalone GPIO scenarios, 38 Pico runtime scenarios, 14 retained watchdog
scenarios, nine rejected profile configurations, the C pre-include check and six
legacy/R3 regression suites passed under ASan/UBSan. Both native exports confirmed
13 sheets and the same 21 application GPIO connections. Public-only Pico linked
at 56,772 bytes RAM / 1,180,588 bytes flash; R3 at 51,652 / 1,417,536 bytes.
Compiler dependency checks found no private-config dependency.

`./.venv/bin/python -B -m unittest discover -s control_tests -p 'test_pico_host.py' -q`
also passed all 20 existing offline Pico host tests using actual firmware status
envelopes. No host schema or script change was needed. Motion/charging/source-cycle
flags remain false; these checks do not establish physical wake/charge levels,
hardware timing or integrated S3 operation. No device I/O or flashing was performed.

### Fourteenth-sheet voltage monitor follow-up

The main now adds U120 INA3221 on existing GPIO1/2 I2C, address0x40, without changing any of the21 application GPIOs or40 module contacts. The native verifier expects14 sheets; the September7 thirteen-sheet build record above remains historical. Voltage monitor transport/runtime are still separate work; no hardware reads or source-cycle qualification are claimed by this schematic update.
