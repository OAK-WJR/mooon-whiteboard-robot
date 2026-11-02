# Routing of enable, boot config and the base board control interface

This document records the control routing stage: **59 trace segments and 22 vias** added, three segments and one via of R25's original power branch removed, and R20, R21 and R25 adjusted. At that time there were **375 trace segments, 109 vias, two ground pours, 35 unconnected items**. No components were added, and the six-page schematic, the 70-part BOM and the firmware were not changed. For the later encoder changes and the current totals, see the [main board notes](README.md).

[Current PCB](top_board_v2_logic.kicad_pcb) · [Full-board preview](preview/placement.png) · [Control area close-up](preview/motor_controls_detail.png) · [Native verification of that stage](../archive/encoder_before_links/top_board_v2_logic.verification.json)

## Connected

| Net | Actual pad connections checked this round |
| --- | --- |
| MOTOR_ARM_REQUEST | U3.19 / GPIO14 → U5.2 / EN, R24.1 / 10kΩ pull-down |
| JTAG_SEL_HIGH | U3.20 / GPIO15 → R25.2; R25.1 to MAIN_3V3 |
| M2_FI | U3.26 / GPIO20 → M2.3, R20.1 |
| M2_BI | U3.27 / GPIO21 → M2.2, R21.1 |
| M0_FI / M0_BI | M0.3 → R12.1; M0.2 → R13.1, not yet connected to the controller |
| M1_FI / M1_BI | M1.3 → R16.1; M1.2 → R17.1, not yet connected to the controller |
| Ground of the control pull-downs | R12.2, R13.2, R16.2, R17.2, R20.2, R21.2 → common ground |

The four mating connectors keep their original positions, hole sizes and orientations. M0/M1/M2 are all wired as **pin 2 BI, pin 3 FI**; pins 4/5 of M2 are encoder B/A, not yet fully connected in this stage. Physical Motor1/2/3 map to M0/M2/M1; physical motor numbers must not be guessed from connector numbers.

ARM exits from below the controller, crosses the back-side power trace on the front, then runs along the left side to the U5 EN/pull-down node. The GPIO15 pull-up moved close to the module, and the original long power branch was deleted. The two M2 control lines exit from the inner side of the module and change layers to avoid the enable, boot config and 3.3V. All coordinates are given explicitly, using directions in multiples of 30°, with no autorouting.

The six control pull-downs got nearby ground vias. M2's R20/R21 were moved to a spot that avoids the power capacitors, header holes and existing traces. After clearing the old pour cache, four historical local pour definitions already covered by the full-board ground pour were deleted, to avoid low-priority areas leaving isolated copper; the full-board back-side ground, the module front-side ground and all keep-outs were kept. One new ground via under the module connects the return area between the new signal paths.

## Verified scope

The KiCad native connection graph checks the eight new net groups and the six pull-down ground ends; all physical pad groups that were connected before stay connected. All 22 MAIN_3V3 pads and the controller's 30 physical ground pads stay connected, with the nine that share pin name 49 checked one by one. All 246 pin nets are the same as the current XML.

403 existing trace/via records are kept verbatim; the only deletion is the explicitly recorded original power branch of R25. The full records of the 71 footprints that did not move are unchanged, including the original 19 mating contacts, the module, the Type-C locating holes and the four mounting holes. The kept pour and keep-out definitions were also compared item by item; DRC rules, net classes and the board outline are unchanged.

Five temporary copies each cut one of: the controller breakouts of ARM, GPIO15, M2_FI and M2_BI, and the ground return of R20. The native connectivity graph detected the expected break in each, and the unconnected count rose from **35 to 36**. These controls only prove that the check can recognize the matching copper breaks; they are not a real-hardware auto shutoff test.

At this stage DRC found no new short, copper clearance, board edge, via or isolated copper issues, and still reports the one original **M2/PWR1 assembly boundary overlap**. The max nominal residual of the 19-point top/bottom board registration is still about 0.00103mm; the antenna projection and its nominal tolerance issue of about 0.046mm are not resolved. The [stage version record](../archive/encoder_before_links/verification/motor_controls_layout_revision.json) checks that 123 firmware and schematic source files did not change; unrelated firmware tests were not re-run.

## Not done

The encoder connections left unfinished in this stage were partly connected in the later [encoder routing stage](ENCODER_ROUTING_REVIEW.md); the [current unconnected details](remaining_connections.json) record what actually remains. The M0/M1 control lines to the controller, sensing, power input and VMOT distribution are still not done. The board still cannot be manufactured as a complete finished board or accepted at power-up.

Default-off still needs verification of actual power-up/undervoltage/power-off, USB reset and download, base board load and EN behavior; GPIO15's role still depends on the eFuse conditions described in the current documents. The existing offline tests and partial simulations did not become real-board evidence because of this. ADC power-off, USB power draw/back-feed, full input protection, power dynamics and temperature rise, RF, assembly, tolerances, BOM and the budget for five finished boards are still not decided.

Verify this stage's archive in `controller_core`:

```sh
python3 verify_motor_controls.py --board archive/encoder_before_links/top_board_v2_logic.kicad_pcb --baseline archive/controls_before_routing/top_board_v2_logic.kicad_pcb
```

The [319-segment checkpoint](../archive/controls_before_routing/CHECKPOINT.md) keeps the main board from before this round. `route_motor_controls.py` only accepts that old version's hash and cannot be applied again to the current main board.
