# Shared Pico I²C transport — INA and BQ runtime candidates connected

The shared client adapters and dedicated FreeRTOS worker are implemented and tested. The application now instantiates the project-local **ESP-IDF5.5.4 hardware-backend candidate** for the [INA3221 runtime diagnostics](PICO_RAIL_RUNTIME.md). Its [source/verification notes](../third_party/esp_idf_i2c_no_clear/README.md) describe the driver adaptation. There are no real-device readings or physical bus acceptance results; [BQ finite wake and readout](PICO_CELL_RUNTIME.md) are now integrated, with charging and motion still inhibited.

The original worker handoff below is retained as the core design history. New backend source and pinned-core changes belong to the later main integration; the original claim that no backend was supplied is superseded by the linked candidate.

## Primary-source decision

`hyperresearch search` was attempted for the missing I²C and atomic API facts;
there is no initialized local vault. The installed public headers identify
Arduino **3.3.9** and ESP-IDF **5.5.4**:

- `.platformio/packages/framework-arduinoespressif32/cores/esp32/esp_arduino_version.h`
- `.platformio/packages/framework-arduinoespressif32-libs/esp32s3/include/esp_common/include/esp_idf_version.h`
- The same S3 tree's `esp_driver_i2c/include/driver/i2c_master.h`,
  `driver/i2c/include/driver/i2c.h`, and FreeRTOS headers.

The [IDF 5.5.4 I²C guide](https://docs.espressif.com/projects/esp-idf/en/v5.5.4/esp32s3/api-reference/peripherals/i2c.html)
and versioned Espressif sources give these concrete limits:

| API | Relevant finding | Consequence |
| --- | --- | --- |
| New `i2c_master_transmit_receive` | One write/read operation has repeated START and final read NACK/STOP; the public return is a status, not actual partial TX/RX lengths. | It can establish full API completion, not per-byte ACK counts. |
| New synchronous implementation | `s_i2c_transaction_start` collapses non-DONE statuses into `ESP_ERR_INVALID_STATE`; timeout/busy paths call `s_i2c_hw_fsm_reset(..., true)`. | Cannot honestly label every error NACK vs timeout, or claim no implicit clear clocks. |
| New async callbacks | Error reporting is restricted; async use has device/queue limitations and is not a way to obtain missing partial counts. | Not used as an apparent nonblocking shortcut. |
| Legacy `i2c_master_cmd_begin` | Supports chained long transfers and distinguishes `ESP_FAIL` for ACK error from `ESP_ERR_TIMEOUT`; automatically invokes `i2c_hw_fsm_reset` on timeout/busy and after ten ACK errors. | It also fails the strict recovery requirement. Neither API has a public switch to disable these internal paths. |

Exact version-pinned review locations:

- New driver: [timeout/busy recovery calls](https://github.com/espressif/esp-idf/blob/v5.5.4/components/esp_driver_i2c/i2c_master.c#L525-L571),
  [pre-transfer recovery](https://github.com/espressif/esp-idf/blob/v5.5.4/components/esp_driver_i2c/i2c_master.c#L642-L650),
  [merged non-DONE return](https://github.com/espressif/esp-idf/blob/v5.5.4/components/esp_driver_i2c/i2c_master.c#L681-L688),
  and [reset's clear_bus branch](https://github.com/espressif/esp-idf/blob/v5.5.4/components/esp_driver_i2c/i2c_master.c#L111-L144).
- Legacy driver: [pre-transfer and terminal error paths](https://github.com/espressif/esp-idf/blob/v5.5.4/components/driver/i2c/i2c.c#L1472-L1544),
  including increment-before-threshold ACK-error counting; [clear/reset implementation](https://github.com/espressif/esp-idf/blob/v5.5.4/components/driver/i2c/i2c.c#L651-L697),
  with [9-clock/10-error constants and a separate 50 ms recovery wait](https://github.com/espressif/esp-idf/blob/v5.5.4/components/driver/i2c/i2c.c#L91-L124).

The installed S3 `soc_caps.h` enables `SOC_I2C_SUPPORT_HW_CLR_BUS`: recovery
uses the peripheral to generate clocks, not necessarily a software GPIO mode
change. Avoiding an explicit reset call does not remove those recovery clocks.
The legacy recovery wait also means its transfer timeout alone is not a hard
wall-clock cap across cleanup.

The original remaining hardware-backend work was a reviewed SDK change exposing
a **no-bus-clear** error/timeout path with unambiguous terminal status, followed
by an implementation of the small `Backend` interface below. Alternatively,
changing the recovery requirement would need an explicit project decision.
The later main implementation preserves the recovery requirement and uses a project-local namespaced adaptation; it does not patch the installed SDK or use GPIO bit-banging.

The main owner can choose among these backend paths; none is implemented here:

| Option | Required work and cost |
| --- | --- |
| Patch the version-pinned legacy IDF driver | Add a reviewed path that prevents implicit clear clocks on busy/timeout/ACK-error paths, preserves safe peripheral/FIFO state after errors, and retains the documented NACK/timeout distinction. Requires rebuilding and maintaining the SDK component plus fault-path tests. Merely deleting reset calls is not sufficient. |
| Patch the new IDF driver | Add recovery opt-out and preserve distinct terminal NACK/timeout status through the synchronous API. Requires SDK maintenance and validation of its transaction/ISR state machine. Failed-transfer byte counts still need explicit evidence or must remain Unknown. |
| Implement a dedicated S3 peripheral backend | Own command/FIFO/interrupt handling, repeated START, bounded abort and peripheral cleanup without bus-clear pulses. Avoids changing the SDK transfer driver but has the largest implementation and silicon-validation burden. |
| Explicitly permit stock automatic recovery | Then a worker around the legacy command API is a smaller implementation. This changes the current requirement and allows fault-triggered bus-clear clocks; main must accept their system effects and reconcile recovery waits with the deadline policy. It does not add measured partial ACK counts. |

## Current bus and interfaces

Per the main owner's 14-page handoff, SDA is GPIO1, SCL GPIO2, at **100 kHz**.
R95/R96 are the shared 4.7 kΩ MAIN_3V3 pullups. U120 is INA3221 address **0x40**;
its channels are RAW_LOGIC_5V, PICO_USB_VBUS_RAW (physical MCU contact 40), and
VMOT. Both sense inputs of each channel share the rail tap through the adopted
10 Ω/differential 100 nF filter. There is no current shunt. The adapter reserves
no additional GPIO and performs no pin configuration itself.

The adapters inherit the actual existing interfaces:

| Port | Interface | Caller buffer behavior |
| --- | --- | --- |
| `transport.bq()` | `bq76905::I2c` | start accepts TX/RX sizes; poll receives the caller's RX pointer/capacity. |
| `transport.ina()` | `ina3221::I2c` | start records the caller's RX pointer in the control-task port only; poll copies completed bytes. |

Both use bounded **69-byte TX / 68-byte RX** storage, matching the BQ CRC frame
maximum. There is one slot per client, one worker and at most one backend call
in progress. A read is passed as one combined TX/RX descriptor. The backend
contract requires repeated START, last-byte NACK and final STOP; **physical
framing is not verified or implemented by the transport core**. Wire is not
used. No claim is based on a Wire buffer size or silently truncated transfer.

## Ownership, cancellation and scheduling

All frontend calls belong to one control task; the worker is the only concurrent
caller. `begin()` allocates a 4096-byte-stack FreeRTOS worker at priority1, pinned to core1 on S3 (core0 on a single-core target), so configuration, ISR allocation and cleanup stay on the same core.
`Backend::configure({1, 2, 100000}, deadline)` runs there, never on the control
task. Caller code must wait for `State::Ready` before starting monitor init.
The core has no implicit hardware initialization. The separately constructed EspIdfBackend performs it only after an explicit begin/configure sequence.

`start()` copies TX into its slot before publishing Queued. A second request on
that client returns false without changing its pending request or destination.
Invalid arguments/not-ready state are accepted as local errors for the next
poll, so false continues to mean contention. The worker claims slots with atomic
state transitions and alternates between queued clients. It holds no shared
application lock while calling the blocking backend.

`poll()` is the only place that copies bytes into caller memory. It copies at
most the reported count and never truncates to fit a smaller destination.
`cancel()` drops the INA caller pointer, advances the generation and cancels
Queued/Done work immediately. An Active slot remains occupied until the old
backend call actually returns, even after cancellation, caller timeout or an
attempt to submit another generation. The other client can queue but cannot
physically overlap. A cancelled completion is discarded using the generation
and phase checks; it cannot fill a new request's buffer.
Slot phase and generation operations use sequential consistency: if cancel
misses Done, the worker's post-publication check must observe the invalidated
generation. Acquire/release on separate atomics alone is insufficient for that
two-sided cleanup rule. The stress test exercises this race but is not a proof
of all possible weak-memory executions.

No frontend call waits for hardware, a worker result, or a hardware-held mutex.
There are bounded copies and atomic operations plus a task notification on
submission. This is not a formal wait-free/real-time guarantee. The S3 compiler
does not promise `is_always_lock_free`; IDF provides hardware or short
critical-section atomic implementations, not a lock held across I²C. See
[IDF atomic implementation](https://github.com/espressif/esp-idf/blob/v5.5.4/components/newlib/src/stdatomic.c)
and [atomic helpers](https://github.com/espressif/esp-idf/blob/v5.5.4/components/newlib/priv_include/esp_stdatomic.h).
Keep transport storage in internal RAM when adding a hardware backend.

`requestStop()` invalidates both logical requests and returns immediately. The
worker finishes any old physical call, then invokes `Backend::shutdown()` and
publishes Stopped. Repeated stop is harmless. Idle waiting is one RTOS tick,
avoiding notification through a task handle after deletion. Keep the transport,
backend and clock alive until Stopped; monitors must be destroyed or cancelled
before their port owner. Destruction asserts that the worker is stopped.

## Results and timing boundaries

`lastResult()` distinguishes Complete, ShortTransfer, Nak, Timeout, BusError,
Cancelled, InvalidArgument, BufferTooSmall and NotReady. `CountEvidence` marks
counts as Unknown or BackendCompletion. An error with Unknown counts is not
evidence that zero bytes reached the device; a cancelled write may still have
had physical effects. Cancellation is not a rollback.

Only terminal backend completion with known, in-range counts can produce a
complete/short frontend result. BQ receives both counts; INA receives RX count
and treats short TX as an error because its interface has no TX-count field.
The existing monitor code then rejects short completion. Timeout/cancel/error
maps conservatively to each interface's BusError where it lacks a dedicated
enumerator; the detailed code remains available through `lastResult()`.
No retry, bus reset or recovery sequence is scheduled automatically.

These counts mean buffers completed by the backend/API, excluding address/RW
bytes and ACK bits. They are **not measured ACK counters**. A future stock-style
synchronous backend could use full requested lengths only after the API confirms
the complete command; partial lengths on failure remain unknown unless the SDK
actually exposes them. I²C fixed-length reads also cannot distinguish a target
returning an idle-high byte stream from intended data solely by byte count.
Protocol checks, CRC and range/configuration validation remain the drivers' job.

The monotonic clock is 64-bit microseconds (`EspClock` uses `esp_timer_get_time`).
Software deadlines are 100 ms for configuration, 25 ms for BQ and 4 ms for INA,
measured from submission through delivery. Queued expiry does not start a
physical transfer. Late backend completion or late poll is rejected. An active
timeout does not reuse storage early. A backend must stop further protocol waits/work at its absolute deadline and quiesce buffer access before return. Cleanup/scheduling latency still needs physical qualification; the worker cannot safely preempt a hung hardware routine.
If it violates that contract, logical cancellation stays prompt but the occupied
slot and draining stop remain pending instead of pretending the bus is free.

**Current runtime scheduling:** at 100 kHz, a 69-byte BQ write needs
about `(1 address + 69 data) × 9 / 100000 = 6.3 ms`, before overhead/stretching.
That already exceeds the INA driver's existing 5 ms I/O budget. Simultaneously
requesting an INA sample behind such a write can legitimately time out. The service now finishes each INA acquisition and admits the next only when
`BqPort::idle()` confirms both logical and physical completion, including
cancelled work. BQ advances between these acquisitions, also during configuration.
This adapter does not extend deadlines or fabricate success. A maximum
69-TX/68-RX combined descriptor would need about 12.51 ms before overhead.
Long-frame and cancelled-work tests exercise this schedule; physical scheduling
margins remain unqualified. Returning busy alone would not remove the INA
driver's separate 5 ms bus-busy timeout.

## Offline verification and remaining acceptance

```sh
python3 -B control_tests/pico_i2c/verify.py --tsan --build
```

The tests execute the real transport and worker on a native thread-backed
FreeRTOS boundary. The deliberately blocking fake backend exercises two-client
arbitration, maximum frame copies, pending/queued cancellation, destroyed old
caller buffers, storage quarantine, error/short/unknown counts, capacity checks,
late completion, queue expiry, creation/configuration failure, and explicit stop
during setup or transfer. These are adapter tests, not a model claiming to prove
the separately implemented ESP-IDF hardware backend's behavior.

The native build uses ASan/UBSan, and `--tsan` runs a separate ThreadSanitizer
build. `--build` copies public project files with a poison replacement config,
compiles Pico/R3 projects, checks that the new adapter object exists, and audits
source dependencies. The real private config is never read/copied/hashed.
No serial/USB/device inventory, flashing or hardware operation is performed.

Final verification on **2026-09-08**, after the sequential-consistency fix:

- **8/8 groups pass under ASan/UBSan and separately under TSan**, including
  200 cancellation/completion races and crossing the 32-bit microsecond boundary.
- Public-only `top_board_v2_pico`: adapter object, project and poison-config
  dependency audit pass; RAM 56,772 bytes, flash 1,180,952 bytes.
- Public-only `top_board_v2_r3`: the same checks pass; RAM 51,652 bytes,
  flash 1,417,536 bytes.

At that original worker checkpoint, firmware did not instantiate the adapter;
unused code could be removed by the linker. Those historical builds establish
compilation/dependency isolation. The newer INA runtime additionally verifies
the actual application link, but still does not establish physical I²C communication.

Remaining acceptance is the candidate SDK/backend qualification, repeated
START/STOP/error behavior on silicon, real partial-count evidence where available,
worker/control scheduling margins, I²C power-domain behavior and pullups, and
full runtime integration. The original worker handoff supplied no voltage or
control integration. The later INA runtime above supersedes that measurement
limitation, followed by BQ wake/readout integration. Charging permissions, BQ
recovery, motion and USB source-cycle qualification remain open. Current evidence
is recorded in [PICO_CELL_RUNTIME_VERIFICATION.json](PICO_CELL_RUNTIME_VERIFICATION.json).
