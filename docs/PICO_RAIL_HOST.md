# Pico INA3221 host acceptance

`scripts/robot_control.py` accepts INA telemetry only for the explicitly selected
`top_board_v2_pico_review` profile. The local authority for voltage representation,
generation and timing is `include/ina3221_monitor.h` and `src/ina3221_monitor.cpp`;
the runtime service adds the bus/startup states below.

Every Pico status requires a `rail_monitor` object with all six fields:

| Field | Accepted values |
| --- | --- |
| `state` | `not_started`, `starting`, `initializing`, `sampling`, `ready`, `expired`, `fault`, `stopped` |
| `error` | `none`, `start_failed`, `setup_timeout`, `bus_fault`, `cleanup_failed`, `invalid_address`, `wrong_manufacturer`, `wrong_die`, `nack`, `bus_error`, `short_read`, `bus_busy_timeout`, `transfer_timeout`, `conversion_timeout`, `acquisition_timeout`, `configuration_changed`, `invalid_voltage` |
| `bus` | `stopped`, `starting`, `ready`, `fault`, `stopping`, `cleanup_fault` |
| `seq` | Integer `1..4294967295` when ready; otherwise `null` |
| `age_us` | Integer `0..49999` when ready; otherwise `null` |
| `span_us` | Integer `0..29999`, no greater than `age_us`, when ready; otherwise `null` |

Only `state=ready`, `bus=ready`, `error=none` permits a complete sample.
`motor_voltage_mv`, `raw_logic_5v_mv`, and `usb_vbus_mv` must then all be integers,
each a multiple of 8 within `0..26000` mV. Zero is a valid measured value. Every
other state requires all three voltages and all three sample metadata fields to
be explicitly `null`. Missing fields, mixed samples, booleans, numeric strings and
floating point values fail validation. `fault` requires a non-`none` error;
every other state, including `stopped`, requires `none`. Retained errors therefore
remain represented as `fault`.

GPIO faults, `reboot_pending`, or watchdog states `stale`, `init_failed`, and
`gpio_fault` cannot accompany ready rail telemetry. Ready telemetry requires
`reboot_pending` to be explicitly `false`. Watchdog `startup`/`acquiring` and INA
readiness are separate states; acquisition alone is not a rail fault.

`age_us` and `span_us` describe the sample **at firmware serialization**. Age is
measured from the conversion trigger request; span describes the conservative
interval from that request to observing conversion readiness. Channels are not
simultaneously sampled. UART/radio transport and BLE fragmentation add latency:
a frame serialized as ready can arrive more than 50 ms later. The host validates
the reported bounds without inventing a receipt-time age or rejecting a delayed
BLE frame solely because of transit time. Acceptance does **not** establish
freshness at receipt, source sustain, USB sustain, or physical power-cycle proof.
HMAC authenticates exact payload bytes; it does not establish transit time.

`seq` identifies a completed monitor generation, skips zero on uint32 wrap, and
is not the command `id`. Polling may report the same generation more than once.
The host returns each validated frame without carrying a previous voltage into
sampling, expiration, or fault states.

`motor_current_ma` and `rail_3v3_mv` remain required and `null`. Motion, charging,
source cycling, PWM and odometry remain unqualified; blocked commands remain
blocked even with measured rails. The CLI's `hardware_power_measured` and
`physical_power_cycle` qualification flags remain `false`.

Pico payloads must contain fewer than **1792 UTF-8 bytes**; command lines must
contain fewer than **2304 bytes**, including their newline. USB responses retain
the JSON envelope; wireless responses require HMAC-SHA256 over the exact payload
string. The larger payload budget is selected from the expected profile before
parsing the response. R3/legacy retain their 1200-byte payload and 1536-byte command
line bounds and their previous telemetry contract.

Run the owned offline suites from the repository root:

```sh
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m unittest discover -s control_tests -p 'test_pico*host.py' -v
```

`control_tests/test_pico_rail_host.py` uses independent contract examples, fake
USB streams, and simulated BLE notifications/clocks. It checks adversarial
types, missing/partial samples, range and timing boundaries, state contradictions,
unchanged capabilities, UTF-8 byte bounds, HMAC tampering, and complete controller
status exchanges. The delayed BLE case explicitly accepts serialization-time
ready telemetry after more than 50 ms of simulated transit.

`control_tests/test_pico_host.py` additionally compiles the actual production
serializer with ASan/UBSan and a native mock device boundary. Its `rails_ready`,
`rails_large`, `rails_expired`, `rails_sampling`, `rails_fault`, and `rails_cleanup`
modes check real monitor samples, resampling/expiration, NACK after a sample, and
cleanup failure. Emitted and HMAC-wrapped response lines must fit the 2304-byte
budget. Its existing public
staging helper excludes the private configuration by path before any read/copy;
compilation uses public defaults and checks compiler dependencies. These tests
use locally installed dependencies and require no device, USB, serial, or network
access. Runtime and native fixture sources are maintained separately.
