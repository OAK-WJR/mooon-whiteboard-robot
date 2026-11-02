# Long-run boundary fixes — 2026-09-07

This round changed the main runtime firmware's encoder counting, odometry input checks and stall cooldown timing. The PCB is still `motor_controller / top_board_v2_kicad_r2` in EasyEDA; power and real-board verification are not done; nothing was flashed and no motor was started.

## Defects and fixes

| Trigger | Old behavior | Behavior after fix |
|---|---|---|
| Encoder runs forward/reverse for a long time up to the integer limit | The ISR adds/subtracts on a signed `long`, and overflow is undefined behavior; sign reversal and differences can also overflow | Internally uses `uint32_t` modular arithmetic; direction correction, the signed window exposed to callers, and differences between adjacent samples all handle wraparound explicitly |
| `Odometry::update` gets a NaN or infinite time value | NaN pollutes wheel speed and pose, and stays in the state | Rejects that update; keeps pose, wheel speed and the last valid count baseline |
| Stall cooldown expiry time crosses the `millis()` wraparound, or is exactly zero | Absolute time comparison and the zero sentinel fail; the target board may retry early | Computes elapsed time from a fixed 32-bit start time, and records cooldown state with a separate boolean |
| ISR and read/clear access the count at the same time | Only disables interrupts on the current core, which is not full mutual exclusion on the dual-core ESP32 | ISR, snapshot and clear share a FreeRTOS critical section; direction conversion happens outside the critical section |

Encoder telemetry `encoder_counts` is still a direction-corrected signed 32-bit window, ranging from −2147483648 to 2147483647 and wrapping around after overflow; it is not a total distance that grows without limit. Wheel speed uses modular differences; the absolute net pulse change between two valid samples must be less than 2³¹. The main runtime loop also has a separate 100ms delayed-stop mechanism. The new count handling does not calibrate encoder CPR, direction or motor parameters, and does not remove electrical noise.

The basis for modular arithmetic is that C++ has different rules for unsigned integers and for signed overflow; it does not rely on the result of casting an out-of-range unsigned value to signed. [C++ working draft](https://eel.is/c++draft/basic.fundamental)

The critical-section approach is based on `task.h` and the Xtensa/RISC-V `portmacro.h` in the installed ESP-IDF 5.5.4. The official documentation states that disabling interrupts only affects the current core, and that SMP shared data should use critical sections with a spinlock; the matching implementation on the single-core C6 still uses its port's critical-section API. [Espressif FreeRTOS documentation](https://docs.espressif.com/projects/esp-idf/en/v5.5.4/esp32/api-reference/system/freertos_idf.html#critical-sections)

## Reproduction and verification scope

Results from before the change are saved in [odometry_before.json](../hardware/top_board_v2/validation/reverification/odometry_before.json) and [stall_timing_before.json](../hardware/top_board_v2/validation/reverification/stall_timing_before.json), both recording the hash of the source under test. Count overflow was actually reported by UBSAN, and the invalid-time test actually got NaN. Desktop `long` is 64-bit and ESP32's is 32-bit: in this desktop test the old cooldown logic showed up as never retrying after crossing the boundary; on the target board the expiry addition wraps first, and it may also retry early. The desktop behavior is not presented as a target-board measurement.

New tests run the real `odometry.cpp`, GPIO interrupt callbacks and `robot_controller.cpp` / `motor_driver.cpp`:

- Full-range forward and reverse wraparound of the three ISR counters.
- 18 telemetry boundaries with direction correction, and 24 wheel-speed integration cases crossing the boundary in forward and reverse.
- 120,000 simulated GPIO edges fed through the registered interrupt handlers, checking wheel speed and reset.
- Zero, negative, NaN and infinite time values, and recovery of valid updates afterward.
- Normal cooldown, crossing the timer wraparound, and expiry time exactly zero, tested on each of the three motors, 9 cooldown cases in total; checks that it stays stopped before expiry, that the original retry policy takes effect after expiry, and that it does not resume after an explicit stop.

Running `.venv/bin/python control_tests/verify.py` runs the full suite and six firmware build environments; each run generates [software_verification.json](../hardware/top_board_v2/validation/reverification/software_verification.json), and its current hash and results are what count. When ASAN/UBSAN find a problem, the check fails outright instead of only warning in the log.

These tests simulate GPIO, the 32-bit clock and the mutex API on the host. They are not a 50-day real-hardware endurance test, and do not simulate C6 interrupt latency, dual-core scheduling, RF coexistence, motor temperature rise or real power cuts. The stall threshold and limited auto-retry policy keep the original configuration and have not passed thermal safety acceptance; the board-level startup limit stays `BOARD_POWER_UNREVIEWED`.
