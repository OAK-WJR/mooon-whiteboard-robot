# Pico three-rail voltage diagnostics: in the firmware, not yet measured

2026-09-08: the main loop now creates the project's ESP-IDF 5.5.4 I²C backend and runs
INA3221 initialization, single-shot sampling and status output. This feature is still review firmware: it has not been flashed
or connected to a real bus, and no actual voltages have been read. The current main schematic has fourteen pages; full-board layout and routing are still not done.

## Actual connections and behavior

| INA channel | Schematic sense point | Status field |
| --- | --- | --- |
| 1 | PS2 output RAW_LOGIC_5V | `raw_logic_5v_mv` |
| 2 | Factory USB VBUS at Pico physical contact 40 | `usb_vbus_mv` |
| 3 | VMOT after motor power switch U30 | `motor_voltage_mv` |

The address is fixed at 0x40, GPIO1/2, 100kHz, external pull-ups. No other addresses are scanned. [BQ cell monitoring](PICO_CELL_RUNTIME.md) now shares this bus,
running a limited GPIO39 wake and a read-only monitoring config; the GPIO41 charge request and GPIO42 power cycling are still disabled.
There is no current shunt resistor, so `motor_current_ma` is always empty; 3.3V has no measurement channel either.

The main task requests startup of the separate I²C worker task only after safe GPIO, UART, wireless init and the task watchdog succeed.
Init failure, NACK, bus error, short read, config change or timeout locks this diagnostic run,
clears the readings and requests a bus cleanup; it does not auto-scan, retry or output recovery clocks. Reboot, GPIO faults and
watchdog failure likewise revoke the readings immediately. Cleanup finishes in the worker task, and a cleanup failure is shown separately; USB
reboot handling does not wait for it. "Immediately" here means logical revocation by the control task, not a guaranteed physical cleanup delay.

The worker task's wait limit is controlled by the original transport layer; the service also sets a 150ms startup observation deadline. Each main loop iteration does at most
one non-blocking I²C submit or poll, and does it before the task watchdog reset and the external watchdog progress pulse.
It uses the 64-bit monotonic `esp_timer`, and does not treat the 32-bit `micros()` as a time that never wraps.

Each time, it first verifies the chip identity and config, discards the old conversion-ready bit, then triggers a single-shot conversion on all three channels.
Only after confirming the new conversion is done, all three registers are read, and the final config matches, does it publish a full group. About every 40ms it requests
the next sample; the previous group is cleared during the request, so a brief `sampling`/null value is expected behavior.
Long BQ messages now advance between INA acquisition groups. The INA starts only after both the actual BQ transaction and the cancel cleanup have ended,
to avoid using up the INA's 4ms transfer/5ms operation budget while queued behind the BQ.

## Status shared by USB, Wi-Fi and BLE

The new `rail_monitor` object contains:

| Field | Meaning |
| --- | --- |
| `state` | `not_started`, `starting`, `initializing`, `sampling`, `ready`, `expired`, `fault` or `stopped` |
| `error` | `none` or the specific failure reason, such as `nack`, `transfer_timeout`, `configuration_changed` |
| `bus` | Worker task state; `cleanup_fault` means cleanup did not succeed |
| `seq` | Number of the current complete sample group; `null` when there is no valid sample |
| `age_us` | Microseconds elapsed since the trigger request **when the message is built**; must be under 50000 |
| `span_us` | Conservative acquisition window from the trigger request to the observed conversion done; the three channels are not sampled at the same time |

Integer millivolt values for the three rails are output only in `ready` with the bus available and no fault. All are integer multiples of 8mV, in the range
0–26000mV; zero is valid only after the full conversion flow. When lost, expired, stopped or in error, all three values
and the sample metadata are `null`; zero or the previous group's readings are never used instead.

`age_us` **does not include message transmission and computer-side queueing time**. BLE fragmentation in particular can mean the whole message arrives
after more than 50ms; the host passing the structure check also does not mean the data is still fresh when received. The existing status command is for diagnostics;
these readings are not used to approve motion, charging or power switching. A real voltage being present also does not prove USB can carry a sustained load.

The Pico's JSON payload limit is 1792 bytes, and the line buffer including escaping/HMAC/newline is 2304 bytes;
the old R3/native configs are unchanged. Parse with the matching updated `scripts/robot_control.py`.

## Verification scope and remaining work

```sh
python3 -B control_tests/verify_pico_rails.py --build
python3 -B control_tests/verify_pico_profile.py
python3 -B -m unittest discover -s control_tests -p 'test_pico_host.py'
```

18 new runtime scenarios run the real main loop, INA driver, diagnostic service and serialization; only the worker task,
device and SDK boundaries are replaced. They cover the three-rail mapping, whole-group updates, expiry, time wraparound, absent device/NACK, short read,
timeout, chip config reset, invalid voltage, GPIO/watchdog faults, USB reboot and cleanup failure.
The public config build also checks that the actual application link keeps the project backend; forced linking is not allowed as a substitute for evidence of runtime integration.

These tests cannot prove actual voltage accuracy, I²C electrical timing, real-time behavior under wireless load or whole-robot stability.
BQ wake and monitoring are integrated; battery-specific protection thresholds, permission and recovery transactions, power dynamics verification,
full layout and routing, and real-board tests are still not done. For the current software checkpoint, see
[PICO_CELL_RUNTIME_VERIFICATION.json](PICO_CELL_RUNTIME_VERIFICATION.json).
INA calibration, input error and power-off leakage budgets are also not done; the existing failure record of the MOS transient simulation is still kept.
