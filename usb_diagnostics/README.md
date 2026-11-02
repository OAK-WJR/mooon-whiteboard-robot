# USB diagnostics

By default it only collects data and the motors do not run. While running it outputs JSON every 100ms; the host saves the raw frames as `telemetry.jsonl` and the conclusions as `report.json`. This gives reviewable fault evidence; it cannot make the MCU identify every PCB fault by itself.

## Quick use: existing C6 adapter board

Run from the WhiteBoardBot root folder. First lift the wheels off the surface and disconnect motor power, then connect the USB data cable:

```sh
./.venv/bin/pio run -d usb_diagnostics -e adapter -t upload
./.venv/bin/python scripts/pcb_diagnostics.py --list
./.venv/bin/python scripts/pcb_diagnostics.py --seconds 10
```

If there is only one Espressif USB serial port, it is chosen automatically; if there are several, add `--port` with a device from `--list`. The host checks the firmware protocol and board type, and never sends motor test commands to a program that does not match. Reports are written to `diagnostic-reports/<time>/` in the current folder by default.

The build command for the new board is below. **Only diagnostics without motion are enabled for now**; its power supply is not finalized, so it cannot be used for prototype acceptance or connected directly to 12V:

```sh
./.venv/bin/pio run -d usb_diagnostics -e top_board_v2
```

On `adapter`, ENC0_A is GPIO16; on the new board draft `top_board_v2`, ENC0_A is GPIO7. The two configs must not be mixed. On the old board, GPIO16 may output UART during the ROM stage; the firmware changes it to an input after entering `setup()`, which cannot retroactively remove the hardware conflict during the ROM stage.

The diagnostic firmware now supports USB `reboot`: it first stops the outputs, replies with the accepted status, then reboots. AI can call `scripts/pcb_control.py reboot --port PORT --profile top_board_v2` (use `adapter` for the old adapter board); the host verifies the new session, the reset reason and that every channel is idle, and does not judge success from the ACK alone. `reset` / `bootloader` call esptool's native USB ROM recovery and do not erase or write Flash; recovery is not guaranteed when the USB peripheral or the power supply itself has failed.

The integrated board also has a USB/Wi-Fi/BLE main firmware that uses a different protocol through `scripts/robot_control.py`; see the [unified control notes](../control_tests/README.md). The current USB diagnostic firmware stays on protocol 1, and the main firmware is protocol 2; the two must not be mixed.

## Motor bench test

There is currently no onboard voltage/current sensing. Powered tests must use an **external current-limited supply that matches the verified motor/board ratings**, measure the motor terminal voltage by hand, and keep the wheels in the air. First do static checks with power off, and start the test parameters at a low duty cycle. `--bench-current-limit` only records the supply current limit you actually set; the script does not set the supply for you, and does not measure this current.

The command format is below; replace the parts in angle brackets with values you have actually confirmed:

```text
./.venv/bin/python scripts/pcb_diagnostics.py --run-motors \
  --motor-voltage <6 or 12> --supply-voltage <measured volts> \
  --bench-current-limit <amps, calculated and set> --duty 64 --pulse-ms 500
```

The script tests, in order, the forward/reverse response of the three wheels, changes on undriven channels, encoder A/B phases and count direction, then the software stop after the heartbeat stops. Before each pulse it waits for the encoders to be still; if any item needs checking, the following actions are canceled. Only after the single-wheel tests pass can `--all-at-once` be added to test all three starting together, based on the calculated total startup current.

Each run lasts at most 1000ms; losing the heartbeat for 350ms revokes the command automatically, and the enable is also cleared when the time is up normally, so the next run must arm again. USB disconnect, receive overflow or an invalid command returns the motor control pins to low. Low/low means the driver is high-impedance and coasting; **this is not a mechanical emergency stop, and does not mean the motor power is physically disconnected**. The MT3608 EN on the existing v2 draft cannot replace power isolation.

## What the report can tell

| Evidence | Can determine | Still cannot determine alone |
|---|---|---|
| Protocol, frame number, session number, reset reason | Communication integrity, reboots, whether the app is alive | With no enumeration at all, whether it is the cable, 3V3, soldering or a damaged MCU |
| PWM init status per channel | Whether the six LEDC outputs were allocated successfully | Whether the correct waveform really appears at the driver input pads and motor outputs |
| A/B levels, edges on both phases, directional counts, illegal transitions | Motor motion response, signs of a missing phase, direction reversal, signs of crosstalk | Whether no response is caused by undervoltage, duty cycle, mechanical load, the driver or the encoder |
| Software stop reason and command zeroing | Software stop logic after link loss/timeout | Power transistor short, physical power cut, or whether the real wheels have stopped |
| `motor_voltage_mv/current_ma/rail_3v3_mv = null` | No matching sensing capability now | Must not be read as 0V, 0A or normal power |

`USB_CAPTURE_PASSED` only means this USB capture passed; `BENCH_CHECKS_PASSED` only means the bench checks that ran passed. Neither is a whole-board manufacturing release result. `REVIEW` keeps the ambiguity, and never turns "the encoder did not change" directly into "the ESP32 is broken".

## Full acceptance matrix for the new PCB

| Stage | Method and record | Pass condition |
|---|---|---|
| Power off | Multimeter/fixture measurement of input polarity, GND, 3V3, VMOT, every header pin and the motor harness; record resistances | Matches the final netlist, with no unexpected shorts/opens; MCU push-pull outputs are not used to test for shorts |
| USB-only power | Battery/motor power removed; verify both USB plug orientations, the data lines, cold boot/reset/BOOT download | MCU enumerates stably and repeated uploads succeed; measured 3V3 within spec; USB does not power the motors or back-feed external inputs |
| Both voltages | Apply 6V and 12V separately per the final spec, and calibrate voltage sensing against a multimeter | Actual VMOT matches the motor rated voltage; mismatched modes are prevented from running; no chip pin exceeds its limits |
| Current sensing/protection | Check current sensing with a controllable load and calibrated instruments, then verify hardware current limit/cutoff and fault latch | Acts within the design threshold and response time; logic/USB keep working and report the fault; no fault injection with GPIO or arbitrary bare shorts |
| Hand-turned encoders | Motors unpowered; turn the three wheels by hand one at a time; USB records the edges and direction of both phases | Both phases valid on every channel; no abnormal transitions on the other channels; the measured directions and physical motor mapping are recorded in the config |
| Single-wheel run | Each wheel forward and reverse, stepped duty cycles; check FI/BI/FO/BO and the encoders with USB + oscilloscope | All three wheels respond; non-target channels do not move; reversing flips the count sign; waveforms and phases match the datasheet |
| Simultaneous start | Three wheels start together, load changes; oscilloscope records VMOT, 3V3 and the reset pin, USB records current and encoders | No out-of-spec dips/spikes and no resets; total current and the temperature rise of all contacts within the approved design ratings |
| Stop/regeneration | Cut the heartbeat, unplug USB, reset the MCU, turn off motor power; check the real driver outputs and VMOT transients | Not driven by default; the physical power-off stage acts as designed; regeneration/hot-plug voltage does not exceed the design limit |
| Thermal and stall | After the parameters are fixed, test at the approved current/time in a controlled fixture with current limiting | Temperature rise of drivers, regulators, connectors and copper and the protection action pass; the stall current is unknown now, so "pass" cannot be filled in in advance |
| Fault injection | Disconnect a single phase/motor connection, simulate voltage/current alarms, USB message faults; record expected vs. measured | Observable faults are reported and the stop conditions are met; detection blind spots are written into the report; failures are not masked by repeatedly "retrying startup" |

The final PCB must add voltage/current feedback, a default-off power switch, independent hardware protection and enough test points. Shared current sensing can help locate faults wheel by wheel; to directly tell apart each driver's current/output fault while running together, three sensing channels or a changed base board test interface are needed. High-speed spikes and USB signal quality still need an oscilloscope; USB logs cannot replace it.

## Software verification

```sh
c++ -std=c++17 -I usb_diagnostics/include usb_diagnostics/tests/test_safety.cpp -o /tmp/wbb-diag-safety
/tmp/wbb-diag-safety
./.venv/bin/python -m unittest discover -s usb_diagnostics/tests -p 'test_*.py' -v
c++ -std=c++17 -fsanitize=address,undefined \
  -I usb_diagnostics/tests/mock -I usb_diagnostics/include \
  usb_diagnostics/tests/test_firmware.cpp -o /tmp/wbb-diag-firmware
/tmp/wbb-diag-firmware
```

The last item runs the real firmware's command parser and loop, simulating USB/GPIO/PWM, and covers lost heartbeat, USB disconnect, a long message overflowing before the newline, PWM faults and reboot while running. The full two-board and host verification can be run with `.venv/bin/python control_tests/verify.py`. The output is **simulated data**; this round did not connect to a real ESP32, and did not measure USB download, motor rotation, current or temperature rise.
