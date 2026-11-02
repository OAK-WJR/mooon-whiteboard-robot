# 3.3V distribution to the controller and encoders, and ground return routing

This document records the controller power stage: **66 trace segments, 32 vias and two ground pours** added, two BOOT trace segments replaced with a new path, and four components (C5, C6, R21, R23) adjusted. At that time there were **319 trace segments, 88 vias, 6 ground pours, 51 unconnected items**. The one original M2/PWR1 assembly boundary overlap is still there; no rules were relaxed and no check waivers were added.

[Current PCB](top_board_v2_logic.kicad_pcb) · [Full-board preview](preview/placement.png) · [Controller power close-up](preview/mcu_supply_detail.png) · [Native connectivity report of that stage](../archive/controls_before_routing/top_board_v2_logic.verification.json)

## Connections and layout

The 3.3V trunk comes straight out of L1's output pad, using 0.8mm copper and two 0.4mm-hole vias at the output end. It goes to C5/C6 beside the controller, U3.3, CH343 VIO, the boot config pull-ups, the six encoder pull-ups and the M0.1 encoder power. The original thin trace is kept only as the U4 output feedback sense, and is not used to power the new loads.

The trunk runs on the back layer along the right side of the board to the controller; where it crosses the back-layer serial lines, it changes layers with two short front-side bridges and paired vias. BOOT was only rerouted locally to free that area. After R21 and R23 were moved, VIO and the middle encoder pull-ups were routed through the actual gap between the original M2 header pins. The original 19 mating pads and all holes, orientations and outlines are unchanged.

C5 was turned and C6 moved down, forming a local 10µF / 100nF decoupling loop; local power is 0.65mm, with 0.4mm breakouts at the module's small pads. C5/C6 each have a nearby via at their ground ends. A front-side ground pour and 12 ground vias were added under the module, with 0.3mm via holes, placed in and around the gaps between the ground pads. A back-side ground pour filling the board outline was added, keeping the original antenna keep-out and mounting hole keep-outs. The ground pours connect to the controller's **30 physical ground pads**, U4/U5 and the USB ground.

The manufacturer stresses that the module's base board should have enough ground copper and ground vias, and that wireless communication must be verified in the final structure. The specific trace widths and via arrangement used here are design choices and are not the manufacturer's certification of this board; the power pin numbers for the bare chip in the same guide were not applied to the MINI module. [Espressif PCB guide: positioning a module on a base board](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32c6/pcb-layout-design.html#general-principles-of-pcb-layout-for-modules-positioning-a-module-on-a-base-board).

## Verification scope

- All 22 MAIN_3V3 real netlist pads are in the same copper connection group.
- All 30 ground pad objects of the controller were checked one by one, including the nine separate pads that share pin name 49; all connect to the common ground group of the regulators and USB. One pin 49 was not used to stand for the other eight.
- All physical pad groups that were connected on the original board stay connected; all 246 pin nets match the current schematic XML.
- The original 309 trace/via records are kept verbatim, and only the two explicitly listed BOOT segments were replaced; the full records of the 70 footprints that did not move are unchanged, including the 19 mating contacts and the Type-C locating holes.
- Four temporary copies each cut one of: the 3.3V trunk, the MCU power breakout, the VIO branch and the rerouted BOOT trace. The native connectivity graph detected the expected break in each, and DRC unconnected items rose from **51 to 52**. The dangling warnings in the controls are kept in the report and are not used as waivers for the main board.
- Two layers, board outline, design rules and net classes are unchanged; the original M2/PWR1 assembly issue is still reported. The max nominal residual of the 19-point top/bottom board registration is still about 0.00103mm.

The [version record](verification/mcu_supply_layout_revision.json) checks that the existing 123 firmware and schematic source files did not change. The schematic, BOM and firmware did not gain components or change function because of this, and unrelated firmware tests were not re-run.

## Still not done

The later [control routing stage](MOTOR_CONTROL_ROUTING_REVIEW.md) connected ARM, GPIO15 and the M2 controls; the power input, VMOT distribution, the remaining motor/encoder lines and the sensing lines are still not done. For the current totals and ground pour definitions, see the [main board notes](README.md). Finishing the controller power net does not mean the power input is usable, let alone prove that real startup, auto reset, USB download or wireless work.

This round ran no closed-loop regulator model, load transient, temperature rise, actual voltage drop or RF test. C5's exact store part number and effective capacitance, the input connector and protection, the ADC power-off issue, USB current budget/suspend/inrush/back-feed, assembly clearance and actual RF tolerance are still not decided. The nominal gap of about 0.046mm from the original antenna to the lower board is still not enough to prove the clearance after manufacturing. The cost of five finished top boards does not meet the target, and there is no manufacturing release.

Verify this stage's archive in `controller_core`:

```sh
python3 verify_mcu_supply.py --board archive/controls_before_routing/top_board_v2_logic.kicad_pcb --baseline archive/mcu_before_supply/top_board_v2_logic.kicad_pcb
```

The [255-segment checkpoint](../archive/mcu_before_supply/CHECKPOINT.md) keeps the main board from before this round. `route_mcu_supply.py` only accepts that old version's hash and cannot be applied again to the current main board; it uses explicit coordinates and does not call an autorouter.
