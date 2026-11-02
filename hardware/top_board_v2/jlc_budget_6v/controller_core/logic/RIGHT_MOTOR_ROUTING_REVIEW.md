# M0 control signals and controller ground return check (historical 554-segment stage)

This page records the stage before the power distribution was routed; for the current board, see the [later notes](VMOT_DISTRIBUTION_REVIEW.md).

This round connected **U3.28 / GPIO22 → M0.3 / FI → R12.1** and **U3.29 / GPIO23 → M0.2 / BI → R13.1**. M0 mates with the original base board H2, which is physical Motor1; the two M2 control lines and all six encoder signals stay connected. The M1 FI/BI controller paths are still not done, so this cannot be used to claim that the three motor channels work.

At this stage the main board had **554 trace segments, 156 vias, two ground pours, 21 unconnected items**. 38 segments and 12 vias were added, replacing six local 3.3V segments and two vias; no components were added and no footprints moved. All 74 footprints, the 19 mating contacts, the board outline and the rules are kept.

[PCB of that stage](../archive/vmot_before_distribution/top_board_v2_logic.kicad_pcb) · [Preview of that stage](../archive/vmot_before_distribution/preview/placement.png) · [Close-up of that stage](../archive/vmot_before_distribution/preview/right_motor_pair_detail.png) · [Connection report of that stage](../archive/vmot_before_distribution/top_board_v2_logic.verification.json)

## Paths and return

The two control lines exit from the right side of the module, change layers to cross the UART, then run along the right board edge in 30°/60° directions around the 3.3V power area to the original M0 pull-downs and header. The new control traces are 0.25mm wide, and vias are 0.6mm outer diameter with a 0.3mm hole; the corners at the interface end have chamfer segments. Explicit coordinates were used, with no autorouting.

During routing, it was found that the controller's original ground return along the board edge had been split. In the end, an explicit local ground path was added near C6, joining the original C5/C4 and the controller's lower-left ground return through a new ground via. The short ground trace between the capacitor pads is 0.2mm and the later part is 0.3mm; one parallel power via of C5 was moved, and the C5-to-C6 power now connects on the back side, keeping the two input vias and the 0.65mm power width. The decoupling capacitors were not connected to the separate feedback sense branch.

This short ground return passed the copper connection and clearance checks; power impedance, RF return and temperature rise still need later verification.

## Verification results

The KiCad native connection graph checks all 30 physical ground pads of the controller one by one, including the nine that share pin name 49; it also confirms that the 22 MAIN_3V3 pads, the six encoder lines, the four complete M0/M2 control lines, all six pull-down ground ends and all original physical pad connection groups stay connected. The 246 physical pin nets match the current XML netlist item by item.

660 old copper records are kept verbatim, and only the eight items listed in the [coordinate record](right_motor_pair_review.routing.json) were replaced. The full records of the 74 footprints, the 19 contacts, the outline, pour and keep-out definitions, DRC rules and effective project settings are unchanged.

Eight separate fault copies each remove one of: the two M0 controller breakouts, the two outer paths, the two interface connections, the C5/C6 power bridge and the new controller ground via. Each detected the specified endpoints separating, and the unconnected count rose from **21 to 22**; the fault copies did not replace the main board. The short, clearance, board edge and isolated copper checks were not relaxed.

The current DRC has only the original **M2/PWR1 assembly boundary overlap** left, with no new errors other than unconnected items. The [version record](verification/right_motor_pair_layout_revision.json) checks that the firmware and schematic source files did not change; this round did not re-run the earlier firmware or subcircuit simulations.

## Still not done

The [21 connections remaining at that time](../archive/vmot_before_distribution/remaining_connections.json) include the two M1 control lines, the ADC signals and four sensing ground ends, the power input and VMOT distribution. Full input connector and protection, ADC power-off, actual USB auto recovery, default shutoff, load dynamics, temperature rise, RF, assembly and the JLCEDA import round trip still need verification. The budget for five finished top boards has not passed, and there is no manufacturing release.

Verify the board of that stage in `controller_core`:

```sh
python3 verify_right_motor_pair.py --board archive/vmot_before_distribution/top_board_v2_logic.kicad_pcb --baseline archive/right_motor_before_pair/top_board_v2_logic.kicad_pcb
```

The [522-segment checkpoint](../archive/right_motor_before_pair/CHECKPOINT.md) saves the main board, preview and original report from before the routing. `route_right_motor_pair.py` only accepts that base and cannot be applied again to the current board.
