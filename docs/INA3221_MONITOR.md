# Independent INA3221 voltage monitor

**Current integration (2026-09-08):** the independent driver below is now used
by [Pico runtime rail diagnostics](PICO_RAIL_RUNTIME.md) through the project-local
I²C backend. The original handoff scope/history is retained below; its statement
that no runtime integration was included no longer describes the current app.
No real-device readings, calibration or physical acceptance are claimed.

The new driver supports a **review-only** three-bus INA3221 candidate:
RAW_LOGIC_5V, PICO_USB_VBUS_RAW (factory USB, MCU contact 40), and VMOT,
in that channel order, all referenced to protected GND. It does not change
top_v2_main, the Pico watchdog/safe defaults, host capabilities, ADC/wake pins,
the main schematic or the independent BQ work. No firmware I²C adapter or runtime
integration is included in this bounded handoff.

The [circuit review](../hardware/top_board_v2/kiss_6v/system_monitor_review/README.md)
contains all 16 physical pins, proposed passives and primary source archive.
TI [SBOS576C Rev. C, September 2025](https://www.ti.com/lit/ds/symlink/ina3221.pdf)
allows the analog terminals to remain at 0–26 V with VS off (§§7.3.1, 8.3).
It does **not** specify a guaranteed unpowered input-current/backfeed maximum.
SCL is limited to VS+0.3 V: VS and bus pullups must use the same MAIN_3V3 domain.
Valid conversions require VS within 2.7–5.5 V. This is adequate evidence to build
the standalone driver, not to claim electrical qualification or USB source-cycle
readiness.

## Source-based register and timing choices

| Item | TI source | Implemented behavior |
| --- | --- | --- |
| Identity | §§7.6.2.19–20, registers FE/FF | Require 0x5449 and 0x3220 before any configuration write. |
| Address | Table 7-1 | Explicit 0x40 default (A0=GND); only 0x40–0x43 accepted, no scan. |
| Configuration | Table 7-6 | All three enabled, AVG=1, VBUSCT=100 (1.1 ms), VSHCT=100 unused. Write/readback 0x7120 to initialize idle; write/readback **0x7122** for bus-only single-shot. POR 0x7127 is rejected. |
| New conversion | §7.3.1, Table 7-36 | Read/discard old Mask/Enable 0x0F; write own trigger; require newly observed CVRF bit 0 before bus reads. Trigger writes clear CVRF; power-down writes do not. Mask reads clear CVRF. |
| Result bank | §7.3.1, §§7.6.2.3, 5, 7 | Read 0x02/0x04/0x06 after all channels finish. Single-shot stops updating the bank. Read configuration again before publishing all three together. |
| Encoding | Tables 7-9/13/17 | Two bytes MSB-first; sign bit 15, data 14:3, reserved 2:0; 8 mV/LSB. Reject sign/reserved bits, any decoded value >26000 mV, including full-scale 0x7FF8. Zero is valid only after the completed-conversion sequence. |
| Conversion time | §6.5 | CT=100 is 1.1 ms nominal, 1.21 ms max per conversion: three bus-only conversions total 3.3 ms nominal / 3.63 ms summed maximum. §7.3.1 additionally states 40 µs recovery from power-down, without a separately specified maximum. |
| Device bus timeout | §6.5 footnote 3 | 28–35 ms SCL-low interface reset; not an application transaction deadline. |

The trigger mode bits are retained R/W configuration fields; internal power-down
after completion does not mean rewriting MODE to 000. §7.3.1 explicitly describes
retriggering by writing the same mode again. The implementation requires exact
0x7122 readback before and after the data bank. No shunt conversion, shunt register
read, resistance assumption or current member exists. **No shunt does not mean a
measured current of zero.**

The driver uses these separate engineering budgets: 5 ms per queued I²C operation
(including contention), 10 ms from trigger scheduling to observed CVRF, 30 ms for
the entire initialization or scan, 500 µs between not-ready status polls, and
50 ms sample lifetime measured conservatively from trigger scheduling. Those
budgets include software scheduling margin; they are not new TI specifications.
No minimum ADC time is invented to reject a legitimately early CVRF. A delayed
callback cannot publish success after a deadline. Three channels are sequential,
not simultaneous: the snapshot exposes the acquisition interval.

## API and integration contract

`include/ina3221_monitor.h` and `src/ina3221_monitor.cpp` use standard C++17 with
no Arduino, GPIO or private configuration dependency. Inject `I2c` and `Clock`:

- `I2c::start/poll/cancel` must return promptly and never busy-wait. A read is
  a register-pointer write, repeated START, and two-byte read with final NACK/STOP.
  A write contains register, MSB, LSB. `poll` reports received-byte count and
  distinguishes Pending, Complete, Nack and BusError.
- `start(false)` accepted no work. Only accepted requests are cancelled. Cancel
  must prevent late writes to buffers and safely tolerate a just-completed
  operation without touching another client's transaction. Bus arbitration and
  buffer lifetimes are the adapter's responsibility. This interface is not an
  implemented ESP-IDF or Wire backend.
- `Clock` supplies monotonic **64-bit** microseconds. Do not zero-extend a wrapping
  32-bit `micros()` value. The clock and bus objects outlive the monitor.
- The monitor exclusively owns this INA3221's configuration and Mask/Enable
  reads. Sharing the physical bus with a different address is fine; a second
  owner of this device can clear CVRF or retrigger without a detectable config
  difference and would break the freshness proof.

Call `begin(address)` once, then call `tick()` from a cooperative task (roughly
1 ms or faster). Begin only schedules work. Wait for `Idle`, call `request()`,
and continue ticking. Each tick performs at most one backend start **or** poll;
it has no delays, retry loops, timers, autonomous pulses or GPIO access.
`latest()` returns an optional copy only after the whole scan and final config
check succeed. Keep that optional alive while iterating its voltage array.

A new request immediately removes the previous cached generation. Any ID,
configuration, I²C, length, range or timing error discards staging and published
data and latches `Fault`; explicit `begin()` is required to reprobe. Expired
samples return no value even if tick has stopped, and a new request is permitted
after expiration. `sequence` advances only on successful publication. Call
`invalidate()` on known supply loss or bus/domain reconfiguration; it cancels
owned I/O and requires initialization again.

There is no claim that I²C identity/configuration proves physical supply voltage
or detects an unannounced power failure immediately after the last read. Results
are timestamped samples with a bounded age, not live wiring/power guarantees.
A consumer must not retain a copied sample indefinitely or reuse a pre-switch
sample for post-switch qualification. Lack of I²C replies, a missing sample or
POR/default zeros are never successful source-cycle evidence.

## Offline validation

Run from the repository root:

```sh
python3 control_tests/ina3221/verify.py
python3 control_tests/ina3221/verify.py --target-cxx .platformio/packages/toolchain-xtensa-esp-elf/bin/xtensa-esp32s3-elf-g++
```

The verifier copies only the new header, implementation and test source to a
temporary directory and supplies a poison `config.h` there. It never reads,
copies or hashes the repository's private config, enumerates ports, connects to
devices, or flashes. It compiles the **actual driver**, then runs its register
model tests under AddressSanitizer and UndefinedBehaviorSanitizer. The optional
installed target compiler only builds an ESP32-S3 object, with exceptions/RTTI
disabled; it does not download a toolchain or run firmware.

The tests exercise real transaction ordering/byte encoding/channel mapping,
stale ready flags, no new conversion, legitimate zero, saturation/sign/reserved
bits, configuration loss across readout, NACK/bus error/short completion at every
transfer, contention, pending/late backend results, lost progress, sample expiry
across the 32-bit time boundary, cancellation and explicit reprobe. This validates
software behavior against a datasheet-based model, not silicon timing, physical
USB sustain or an entire ESP32-S3 runtime.

Validation result (2026-09-07): **11 runtime test groups passed** under ASan/UBSan;
the installed ESP32-S3 compiler also built the standalone driver object with
`-Wall -Wextra -Werror -pedantic -fno-exceptions -fno-rtti`.

Remaining work after candidate adoption: an actual bounded I²C adapter and
electrical validation, then explicit runtime/host integration with measured
thresholds and the existing disarmed source-cycle gates. None of those later
steps is represented as complete by this driver.
