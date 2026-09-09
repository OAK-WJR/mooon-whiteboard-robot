# Pico I²C hardware backend: project-local candidate implemented

The main agent read the transport core and independently checked Espressif v5.5.4 source on 2026-09-08. A project-local backend candidate is implemented and now instantiated by the [INA runtime diagnostics](PICO_RAIL_RUNTIME.md). The [BQ wake/readout runtime](PICO_CELL_RUNTIME.md) is now connected too. Physical qualification remains open; offline execution is not evidence of physical I²C communication.

The project-local no-automatic-bus-clear requirement remains in force for the next implementation. No user approval is required to investigate or implement a conforming backend in isolated public files. No SDK file, installed toolchain, bus pin or real device was changed by this review.

## Confirmed upstream behavior

- [New master driver](https://github.com/espressif/esp-idf/blob/v5.5.4/components/esp_driver_i2c/i2c_master.c): `s_i2c_send_commands` calls `s_i2c_hw_fsm_reset(..., true)` on hardware timeout and a still-busy timed-out bus; `s_i2c_transaction_start` also does this for previous timeout/busy state. On S3 the clear path calls `i2c_ll_master_clr_bus`, so avoiding explicit GPIO toggles in a wrapper does not remove recovery clock pulses.
- The same source builds a combined transfer with repeated START and final-byte NACK/STOP. Successful API completion supplies buffer-length evidence; it does not expose individual ACK counts. Non-DONE statuses require conservative mapping.
- [Legacy driver](https://github.com/espressif/esp-idf/blob/v5.5.4/components/driver/i2c/i2c.c): timeout/busy recovery and the ten-ACK-error clear path likewise prevent an unchanged public-API wrapper from meeting the requirement.
- Stock per-call timeout handling is not by itself a proof of a single absolute deadline across all internal waits, FIFO chunks and cleanup. A future backend must qualify this as well as error handling.

## Selected implementation and remaining work

The main agent implemented the project-local, version-pinned Espressif master adaptation in [third_party/esp_idf_i2c_no_clear](../third_party/esp_idf_i2c_no_clear/README.md), with automatic bus-clear paths removed, distinct failure mapping and one transaction deadline. Namespaced symbols keep it separate from stock master APIs. The installed common peripheral allocator remains shared, with its52-byte base ABI compared using actual SDK/project DWARF. Upstream license, original files and a reproducible patch are retained. No installed SDK was modified.

The candidate passed its initial forced-link Pico build and native changed-function tests. The adapter/core test scopes and remaining silicon/runtime limits are listed in the source README. Cleanup/scheduling latency is not a physically measured hard deadline guarantee.

Keep TX/RX storage owned by the worker until physical completion, even on cancellation. Do not retry a possibly partially completed BQ write. Runtime must schedule long BQ initialization/configuration separately from short INA reads: a maximum69-byte BQ write takes at least6.3ms at100kHz, exceeding the present4ms INA transport/5ms monitor I/O budgets. The current service advances BQ between INA acquisitions and starts INA only after BQ physical work, including cancelled work, has drained. This schedule is exercised offline; silicon scheduling margins remain unmeasured.

The INA service now uses the control-task watchdog sequence and publishes USB freshness/error status. BQ wake/readout now uses the same sequence. Battery-specific protection/permissions, recovery, complete supervision and physical backend/rail qualification remain required. Motion, charging and source-cycle success remain disabled until their own prerequisites are implemented and verified.
