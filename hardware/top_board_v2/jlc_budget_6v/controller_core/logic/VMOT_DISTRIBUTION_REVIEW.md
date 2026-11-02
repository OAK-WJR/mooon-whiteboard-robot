# VMOT distribution and ADC ground check

This file records the 566-segment stage; for the current main board, see the [USB/OR input routing check](LOGIC_INPUT_ROUTING_REVIEW.md).

This round used **1.2mm front-side copper** to connect the C19 output to the three VMOT contacts 2/3/4 of PWR1, with R35's input end sensing from that wide copper path; it also connected the four ground ends of C20/R34 and C21/R36. All **74 physical GND pads** are now in the same group in the KiCad copper connection graph.

At this stage the main board had **566 trace segments, 158 vias, two ground pours, 13 unconnected items**. 12 segments and two ground vias were added, and all 710 old copper records are kept; all 74 footprints, the 19 mating contacts, the board outline, pour definitions and DRC rules are unchanged; no components were added, and the schematic, BOM and firmware were not changed.

[PCB of this stage](../archive/logic_before_input/top_board_v2_logic.kicad_pcb) · [Full-board preview](../archive/logic_before_input/preview/placement.png) · [Power distribution close-up](../archive/logic_before_input/preview/vmot_distribution_detail.png) · [Native connection report](../archive/logic_before_input/top_board_v2_logic.verification.json)

## Power and return paths

The new load supply starts from C19's output pad, goes in 30°/60° directions to the middle power contact, then splits to the contacts on both sides. The original 1.0mm segment from L2 to C19 is kept, and the new distribution segments are 1.2mm; loads were not connected to R28's separate thin feedback trace or R30's thin bleed trace. Trace width and connectivity checks do not replace closed-loop stability, voltage drop, startup surge or thermal verification.

The two ADC lower legs and filter capacitors each connect to the common ground through 0.3mm copper and 0.6/0.3mm ground vias. C21/R36's via moved to the right to avoid ENC1_B on the back side; both holes are outside the SMD pads. The sensing inputs, MCU signals and power-off protection are still not done; this round only connected the existing ground ends.

## Verification

The native connection graph confirms that the nine VMOT pads are connected: L2.2, C18.1, C19.1, R28.1, R30.1, R35.1 and PWR1.2/3/4. All 74 physical ground pads, 22 MAIN_3V3 pads, the M0/M2 control lines, the six encoder lines and all previously connected physical pad groups stay correct; the 246 physical pin nets match the XML netlist item by item.

Seven fault copies each remove one of: two new power path segments, two contact distribution segments, the original L2-to-C19 segment and two new ground vias; each detected the specified endpoints disconnecting, and the unconnected count rose from **13 to 14**. Short segments inside the same pad copper were not used as valid open-circuit controls.

Another copy removed all six old 0.3mm back-side VMOT feedback/bleed traces. The inputs of R28 and R30 disconnected as expected, and the unconnected count rose to **15**; L2, C18, C19, R35 and the three power contacts still connect to each other. This check confirms that the load copper path does not depend on those thin traces; disconnecting the feedback branch was not mistakenly called normal whole-board operation.

Main board DRC has only the original **M2/PWR1 assembly boundary overlap** left, with no new short, copper clearance, board edge or isolated copper errors. The [version record](../archive/logic_before_input/verification/vmot_distribution_layout_revision.json) checks that 123 firmware and schematic source files did not change; the earlier firmware and subcircuit simulations did not become real-board evidence because of this round's copper connection check.

## Not done

The [13 remaining connections](../archive/logic_before_input/remaining_connections.json) include the two M1 control lines, the two ADC signals, the battery and USB inputs and the power OR. M1 still needs local adjustment near the controller; the current board cannot drive the three motors for acceptance. Full input connector and protection, ADC power-off, USB auto recovery, default shutoff, power dynamics, temperature rise, RF, assembly, a full quote for five finished boards and the JLCEDA import round trip have still not passed.

Verify the board of this stage in `controller_core`:

```sh
python3 verify_vmot_distribution.py --board archive/logic_before_input/top_board_v2_logic.kicad_pcb --baseline archive/vmot_before_distribution/top_board_v2_logic.kicad_pcb
```

The [554-segment checkpoint](../archive/vmot_before_distribution/CHECKPOINT.md) saves the main board and preview from before the routing. `route_vmot_distribution.py` only accepts that base and cannot be applied again to the current board. No manufacturing release.
