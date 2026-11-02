# Routing M1 encoder A/B to the controller

This document records the M1 stage: completed **M1.5 / ENC1_A → U3.5 / GPIO2** and **M1.4 / ENC1_B → U3.6 / GPIO3**, keeping the R18/R19 pull-ups. M1 mates with the original base board H1, which is physical Motor3; motor numbers must not be guessed from connector numbers. The M2 encoder paths stay connected; at the end of this stage the two M0 controller leads were still not done; for later progress, see the [main board notes](README.md).

At that time the main board had **456 trace segments, 131 vias, two ground pours, 25 unconnected items**. 40 segments and 16 vias were added, replacing 6 segments and 3 vias. All 74 footprint records are kept; component positions, BOM, schematic and firmware were not changed.

[Current PCB](top_board_v2_logic.kicad_pcb) · [Full-board preview](preview/placement.png) · [Left encoder close-up](preview/left_encoder_pair_detail.png) · [Native copper connection report of that stage](../archive/right_encoder_before_pair/top_board_v2_logic.verification.json)

## Paths and return

M1 A exits from the left side of the controller, crosses the power with a short bridge on the front, and runs along the left board edge to the already connected R18. M1 B changes layers from the inner side of the module, crosses the existing 3.3V and 6V copper on the front, changes layers at the U5 enable line and the feedback sense line, and finally joins the original encoder B net with a 30° branch. All new signal traces are 0.25mm, with coordinates given explicitly and no autorouting.

A 3.3V back-side trace beside the controller was moved inward, keeping its 0.65mm width and the original power via; the C4 ground via moved in by 0.3mm, keeping its short ground trace. The local path where ARM crosses the power made room for the encoder lead; the connections of U3.19, U5.2 and R24.1 are unchanged.

After the copper was refilled, the ground pour on the bottom side of the controller was found to have lost its common return; it was reconnected with two explicit ground branches and two vias. The checks cover each of U3's 30 physical ground pads, including the nine that share pin name 49; possible ground connections inside the module were not used in place of top board copper connections.

## Verification and limits

The KiCad native connection graph confirms the full M1 A/B paths and all original physical pad connection groups. All 22 MAIN_3V3 pads, the 30 controller ground pads and the six control pull-down ground ends stay connected; the 246 physical pin nets match the schematic XML. 531 old copper records are kept verbatim, and only the nine items listed in the [coordinate plan](left_encoder_pair_review.routing.json) were replaced; the 19 mating contacts, board outline, footprints, ground pour/keep-out definitions, net classes and rules are all kept.

Nine temporary copies each break one of: the M1 A/B controller breakouts and interface branches, the rerouted 3.3V branch, the C4 ground trace, the ARM path and the two new ground vias. Each copy detected the matching break, and unconnected items rose from **25 to 26**. The ground pour runs in parallel with the short ground traces, so the two controller ground controls actually removed the vias; deleting redundant copper was not taken as an open circuit.

DRC still has the original **M2/PWR1 assembly boundary overlap**, with no new copper clearance, short, dangling or isolated copper errors. The drawing uses the existing two-layer rules; these results do not prove manufacturing tolerance, temperature rise, load dynamics, USB or wireless stability. The [stage version record](../archive/right_encoder_before_pair/verification/left_encoder_pair_layout_revision.json) checks that 123 firmware and schematic source files did not change; the existing firmware tests were not re-run.

Later the [M0 encoder controller routing](RIGHT_ENCODER_ROUTING_REVIEW.md) was completed; the [current remaining details](remaining_connections.json) still include the M0/M1 control leads to the controller, sensing, power input and VMOT distribution. ADC power-off, full input connector and protection, actual default-off, USB auto recovery, RF and assembly verification are still not done. The budget for five finished top boards is still not met, and there is no manufacturing release.

Verify this stage's archive in `controller_core`:

```sh
python3 verify_left_encoder_pair.py --board archive/right_encoder_before_pair/top_board_v2_logic.kicad_pcb --baseline archive/left_encoder_before_pair/top_board_v2_logic.kicad_pcb
```

The [422-segment checkpoint](../archive/left_encoder_before_pair/CHECKPOINT.md) keeps the main board from before this round. `route_left_encoder_pair.py` only accepts that base hash, and cannot be applied again to the current board.
