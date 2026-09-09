# Project-local ESP32-S3 I²C master adaptation

Implemented for ESP-IDF **5.5.4**, Arduino library commit **735507283d**.
This is a hardware-backend candidate with offline verification; the application
now instantiates it for [INA diagnostics](../../docs/PICO_RAIL_RUNTIME.md) and
[BQ wake/readout](../../docs/PICO_CELL_RUNTIME.md).
No silicon waveform or complete runtime acceptance is claimed.

`upstream/` preserves Espressif's Apache-2.0 sources. `source_manifest.json`
records their URLs and hashes; all three match the installed commit's published
sources. `LICENSE` retains the upstream license. Run `python3 adapt.py` here to
reproduce `i2c_master.c`, `i2c_private.h`, `namespace.h` and `PATCH.diff`.

The C++ `EspIdfBackend` calls `src/pico_i2c_native.c`, which includes the adapted
master translation unit. Public master symbols have a `wbb_idf554_` prefix.
There is no override or edit of the installed SDK, and no silent fallback to
stock master transfers. The stock common peripheral allocation layer remains
shared so a second driver cannot independently claim the same hardware port.
The shared `i2c_bus_t` layout is checked against actual SDK and project DWARF:
52 bytes with the same twelve member offsets. Compile-time guards reject the
wrong SoC/IDF version or changed base layout. The new deadline/core/fault fields
belong only to the project-private master object.

## Changed behavior

- Automatic software and hardware bus-clear implementations are removed. The
  reset API and async callback setup reject calls. Queue depth must be zero;
  upstream asynchronous transfer code has no supported entry in this backend.
- Each request supplies one 64-bit `esp_timer` deadline. Command-semaphore waits,
  FIFO chunks, completion waits and the NACK STOP wait recheck it. RTOS waits
  use remaining whole ticks; a final sub-tick remainder yields and rechecks.
  No new software-issued command is intentionally started after expiry.
- Completion after the deadline is rejected. NACK maps to a distinct terminal
  error; arbitration/error maps conservatively. No request is retried internally.
  NACK with completed STOP permits a later explicit request; timeout/bus error
  keeps the backend unavailable until explicit shutdown/configuration.
- Cleanup masks and clears interrupts before clearing stored buffer pointers.
  The transport worker is pinned to one CPU, so configuration, transfer,
  quiescence and interrupt deallocation occur on the interrupt-allocation CPU.
  Aborting the peripheral FSM can release bus lines, but does not run a recovery
  clock train. A stuck target can remain stuck; automatic recovery is not hidden.
- GPIO1/2 use external pullups, 100kHz, addresses0x08/0x40 only. The SCL timeout
  allocation is2ms and still requires validation with the real isolated bus.
  Setup does not probe, issue a wake pulse or enable any load.
- Setup failures retain their partial context until explicit shutdown. A failed
  shutdown retains context and exposes its error; destroying a backend with an
  unreleased context asserts. Transport `Stopped` describes worker termination,
  not a successful release of every backend resource; callers must also inspect
  `shutdownResult()`.

The deadline bounds the algorithm's waits and new command issue. It is **not a
measured hard real-time return guarantee**: cleanup and RTOS/interrupt latency
can add time. The caller rejects late results and keeps active storage quarantined
until return. Hardware timing, ISR delays under Wi-Fi/BLE and stuck-bus cleanup
must be measured before claiming a physical upper bound.

## Verification

From the repository root:

```sh
python3 -B control_tests/pico_i2c_backend/verify_driver.py --build
python3 -B control_tests/pico_i2c_backend/verify_adapter.py
python3 -B control_tests/pico_i2c/verify.py --tsan
```

The driver harness extracts and compiles the actual changed C wait/dispatch/
quiesce functions. Its RTOS/FIFO boundaries are fake;13 cases cover long chunks,
expiry before commands, late completion, NACK, stuck STOP and clearing buffer
references with interrupts masked. It does not simulate the entire peripheral.
Adapter tests exercise actual C++ result mapping and ownership through a fake C
API. Shared-worker tests exercise cancellation races and storage ownership.

Public-only builds use a poison replacement for the private configuration.
The Pico link audit forces the native backend symbols into the ELF, verifies
the namespaced transfer symbols, rejects stock master transfer symbols, and
compares the shared-base ABI. This prevents an unused-code compile from being
misreported as a working link. There is no device I/O or flashing.

Still required: actual START/STOP/ACK and long-frame traces, power sequencing,
bus capacitance/pullups, deadline/ISR behavior, full application scheduling,
battery-specific permission/recovery and physical tests. BQ/INA initialization,
readout and USB freshness metadata are now connected, with mock-boundary runtime
tests. BQ long transactions advance between INA acquisitions, with physical-idle
checks before admitting INA. See PICO_CELL_RUNTIME_VERIFICATION.json for the
current application link without force-link flags. Neither driver completion
nor an input voltage proves source capacity.
