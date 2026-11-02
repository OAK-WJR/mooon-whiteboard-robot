# M0 encoder routing and six-line connection check (historical 522-segment stage)

This page records the stage before the M0 control lines were routed; for the current board, see the [later notes](RIGHT_MOTOR_ROUTING_REVIEW.md).

This round completed **M0.5 / ENC0_A → U3.12 / GPIO0** and **M0.4 / ENC0_B → U3.13 / GPIO1**, keeping the R14/R15 pull-ups. M0 mates with the original base board H2, which is physical Motor1. With this, all six A/B signals of the three encoder groups on the top are connected in CAD; this is not a result for the actual harness, encoder direction or a loaded test.

At this stage the main board had **522 trace segments, 146 vias, two ground pours, 23 unconnected items**. 78 segments and 22 vias were added, replacing 12 segments and 7 vias. All 74 footprints and the original 19 mating contacts are unchanged; component positions, BOM, schematic and firmware were not changed.

[PCB of that stage](../archive/right_motor_before_pair/top_board_v2_logic.kicad_pcb) · [Preview of that stage](../archive/right_motor_before_pair/preview/placement.png) · [Close-up of that stage](../archive/right_motor_before_pair/preview/right_encoder_pair_detail.png) · [Connection report of that stage](../archive/right_motor_before_pair/top_board_v2_logic.verification.json)

## Paths and local adjustments

M0 A exits straight from the bottom side of the controller, and M0 B changes layers from the inner side of the module. The two signals pass between the boot config and the power supply, each changing layers to cross the existing UART and 3.3V distribution, then enter the M0 interface from the right side of U4. The short back-side channel near the boot config uses 0.2mm width, and the other new encoder traces are 0.25mm; new vias keep 0.6mm outer diameter and 0.3mm hole. All paths use explicit coordinates and directions in multiples of 30°, with no autorouting.

The R25 power via moved down, and the GPIO15 signal path is kept; the U4 input back-side branch moved sideways, keeping its 0.6mm width, and the C8 ground via and its short connection were adjusted to match. U4's feedback sense and its two explicit ground returns are kept, and the encoders do not connect to the power sense branch. The small bridge of M2 encoder A inside the module was rerouted to make way for the pin 14 ground breakout; the M2 interface end and the controller pin numbers are unchanged.

The new traces had split the ground pour on the bottom side of the controller. Explicit ground branches have now been added to connect pins 11/52 and pin 14 back to the common ground; the related ground vias moved to spots that do not block signals. All 30 physical ground pads of the controller were checked one by one, including the nine that share pin name 49; ground connections inside the module were not relied on in place of top board copper connections.

## Verification

The KiCad native connection graph confirms all six encoder nets, each covering the controller pin, the matching header pin and the pull-up end. All physical pad groups that were connected before stay connected; all 22 MAIN_3V3 pads, 30 controller ground pads, six control pull-down ground ends and 246 physical pin nets stay correct. 568 old copper records are kept verbatim, and only the 19 items listed in the [coordinate plan](right_encoder_pair_review.routing.json) were replaced; footprints, board outline, rules, net classes, ground pours and keep-out definitions are unchanged.

Eight temporary copies each cut one of: the M0 A/B controller breakouts and interface branches, the R25 power, the U4 input branch, the rerouted small M2 encoder A bridge and the new lower-left ground via. Each detected the matching break, and the unconnected count rose from **23 to 24**. In another copy, after the new ground via at pin 14 was removed, all pad connection groups were still kept and the unconnected count was still 23, confirming that there is already a parallel ground return there; the isolated copper warning produced by that fault copy is kept in the report, and only proves that the pad connections are kept. Deleting a redundant via was not mistakenly recorded as a passed open-circuit test, and the main board's isolated copper check was not relaxed.

The current DRC reports no new short, copper clearance, board edge, via, dangling or isolated copper errors, and still reports the original **M2/PWR1 assembly boundary overlap**. The [version record](verification/right_encoder_pair_layout_revision.json) checks that 123 firmware and schematic source files did not change; the existing firmware checks were not re-run.

## Still not done

The [23 connections remaining at that time](../archive/right_motor_before_pair/remaining_connections.json) include the four control lines from M0/M1 to the controller, sensing and four sensing ground ends, the power input and VMOT distribution. Full input connector and protection, ADC power-off, actual default-off, USB power and auto recovery, load dynamics, temperature rise, RF and assembly verification are still not done. Passing the two-layer copper connection checks cannot prove operating stability; the budget for five finished top boards is still not met, and there is no manufacturing release.

Verify the board of that stage in `controller_core`:

```sh
python3 verify_right_encoder_pair.py --board archive/right_motor_before_pair/top_board_v2_logic.kicad_pcb --baseline archive/right_encoder_before_pair/top_board_v2_logic.kicad_pcb
```

The [456-segment checkpoint](../archive/right_encoder_before_pair/CHECKPOINT.md) saves the main board from before this round. `route_right_encoder_pair.py` only accepts that base hash and cannot be applied again to the current board.
