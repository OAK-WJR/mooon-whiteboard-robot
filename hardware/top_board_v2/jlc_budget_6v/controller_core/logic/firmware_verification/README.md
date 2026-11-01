# KISS C6 firmware verification evidence

`control_tests/verify_kiss_c6.py` writes these files. Each run deletes the
previous logs and JSON first, so the folder only ever holds one run.

| File | What it is |
| --- | --- |
| `report.json` | Every step with its exit code, `all_passed`, the step that failed (if any), source hashes, and the SHA-256 of the netlist that was checked |
| `profile.json` | Board constants compiled from `include/board_profile_kiss_c6.h` |
| `observed.json` | Pins the compiled firmware actually configured: outputs, encoder interrupts, FI/BI enable order, encoder edge-to-wheel map, console kind, ADC and plain-input pins, untouched pins |
| `status.json`, `fault_status.json`, `rails.json` | Real status, GPIO-fault status and rail report frames from the compiled firmware. The host validator in `scripts/robot_control.py` is run on each one. |
| `*.log` | Output of each step (gitignored) |

The verifier compares the firmware with
`hardware/top_board_v2/easyeda/logic_netlist.json`, the EasyEDA export of the
2026-10-29 board. It maps module pads to GPIOs with the ESP32-C6-MINI-1
datasheet pin table. It does not use the KiCad files in the folder above.

Superseded, kept only as history:

- `cross_build_before_schematic_integration.json` (2026-09-09)
- `toolchain_evidence.json`

Both describe the September CH343G/UART0 board with ARM on GPIO14. They are
not evidence for the current board.

No hardware is involved in any of this. Nothing is uploaded, and the private
`include/config.h` is never read.
