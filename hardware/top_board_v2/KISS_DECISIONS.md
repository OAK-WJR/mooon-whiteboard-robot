# KISS design constraints — added by the user, 2026-09-07

Within the existing requirements for USB control/recovery, Wi-Fi/BLE, whole-robot 6V or 12V motors, 2S charging and protection, prefer fewer components, config items and special operating modes. Simple and checkable comes before adding features.

- The whole robot uses one motor voltage: all three at 6V or all three at 12V; no per-motor voltage conversion.
- Reuse the three RZ7889-MS and 19 board-to-board contacts of the existing bottom controller board; top_board_v2 adds no duplicate motor H-bridges. The new motor power section means the shared VMOT regulation/cutoff, not redoing the three drive channels. The current motor is confirmed as the MECCANIXITY 12V 300RPM encoder N20, with future compatibility for an all-6V GA12-N20 robot; see [base board and motor review](bottom_board_review/README.md).
- Keep one CP2102N USB data/recovery path; the native USB pins can be used for other necessary functions after the boot behavior is checked.
- The hardware watchdog uses a fixed 200ms setting; pick a device that also monitors the MAIN voltage, saving a separate supervisor chip and an adjustable timing capacitor.
- One dual flip-flop handles both "heartbeat seen after a fault" and "new enable edge received". An output AND gate lets the request going low revoke permission directly; each of these three logic functions has a matching failure scenario.
- Prefer reusing component part numbers and common packages that have already been reviewed. Any new component must map to a clear function or a failure mode that was found.
- After the full schematic is finalized, first clean out duplicate functions and trial options, then do one coherent layout and routing pass; a mechanical draft or an unfinished schematic is not called the latest routed PCB.
- The base board's 19 mating contacts, triangular shape and four-layer direction keep their existing constraints. Cutting components must not come at the cost of wrong mating, missing a real power cutoff, or dropping battery protection.

The existing watchdog module has 10 components and has been merged into the 147-component, 10-page R3 integrated draft. The final count, cost and conditions for ordering the board follow the actual BOM and verification after the whole board is done.
