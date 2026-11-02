# Top board simplified scope

> **Superseded since 2026-10-28:** the current board `top_board_v2_logic` (EasyEDA) charges the 2S battery over USB-C on the board with an IP2326, and on 2026-10-29 the CH343G was removed in favor of the C6's native USB; see [hardware/README.md](../../README.md). The external charging, "the top board does not charge" and the six-page, 70-component KiCad drawing below are all records from 2026-09.

The user's latest instruction: "No need to deal with the charger and motors, only the top PCB". The current task and budget only cover five finished top PCBs with the components on them and soldering; target ¥30, cap ¥50, CNY preferred. The same figure in USD is only a possible fallback. The base board, motors, battery, external charger and harness are not counted; historical material is kept, and there is no more buying or extended research.

| Top board function | Design responsibility |
| --- | --- |
| ESP32 | Wi-Fi/BLE, six encoder inputs, six base board drive signals, control and diagnostics |
| USB | Communication, download, separate EN/BOOT auto recovery, logic power for diagnostics |
| Power | 3.3V for the controller, about 6V output, necessary input protection and decoupling; stopped by default on power-up/reset |
| Interface and mounting | Reuse the base board H-bridges, a triangular outline no larger than the base board, and the original 19 contacts; fixed position on top, bottom side kept as empty as possible |

External charging is accepted, and the new top board adds no charging, balancing or their dedicated peripherals. USB does not charge the battery or power the motors directly. Protection, connectors and peripheral components actually fitted on the board still count in the top BOM; if it relies on external protection, the input requirements must be listed explicitly, and the `VBAT_PROTECTED` label cannot be treated as a protection circuit.

None of the old plan's separate controller, per-cell digital telemetry, isolated I²C, duplicate voltage supervision, multi-level fault latches or extra charging port are added. Stop timeouts, the task watchdog, reset reason, encoder counts, communication errors and test logs are handled by the firmware. Diagnostics only report data that can actually be observed, and cannot guarantee that USB will find every hardware fault.

Layout prefers two layers and single-sided assembly on top, and must finally meet power return, RF and connector clearance. First place and route by hand in KiCad, using 30°/60° directions for the triangle, then complete the EasyEDA import and round-trip check. The three drive channels already on the base board are not redesigned; the board-to-board pin order must not be changed at will to save top board cost.

The current [six-page schematic](controller_core/logic/README.md) merges the controller, USB, interfaces, two dividers and the power candidates, 70 components in total, ERC 0 errors, 0 warnings. The [component list](TOP_COMPONENTS.md) is not yet a full BOM; the [cost review](COST_REVIEW.md) has not passed the budget. The new firmware passed an offline check against the current netlist, and motion stays disabled. The earlier three-page drawing, the complex power drafts and the module carrier boards are all historical.

| Top board verification condition | Result to verify |
| --- | --- |
| USB only | Communication, diagnostics, auto recovery; logic power budget and inrush meet requirements |
| External battery input only | Controller and wireless work; stopped by default on reset |
| Both inputs connected at once, or plugged/unplugged | Back-feed, switching and startup behavior meet the design conditions |
| Communication loss, program hang, reset/download | Stop and recovery behavior correct; motion does not resume automatically |
| Low input voltage, load step, faults and regeneration | Input protection, supply transients, temperature rise and default-off behavior meet requirements |
| ADC power-up/power-down and wireless running | Injection current and input range pass, conversion/calibration valid, unmeasured state not disguised as a value |
| Manufacturing and interface | DRC, 19 contact positions/pin order, actual footprints, assembly clearance, antenna area and import round trip consistent |

The above are whole-board verification requirements still to be done. The existing ERC, wiring comparison, partial USB/RC simulation and offline firmware checks do not replace real power, wireless, interface and real-board acceptance. There is currently a MINI two-layer placement candidate; full-board routing is not done, and there is no full quote or manufacturing release. The MINI's part number on the global site is Standard Only, so it is not taken as a low-cost final design that has passed the budget.
