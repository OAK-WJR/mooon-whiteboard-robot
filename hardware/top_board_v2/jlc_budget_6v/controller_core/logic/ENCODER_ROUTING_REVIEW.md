# Encoder interface and M2 A/B routing check

This document records the six-interface pull-up and M2 encoder stage: **57 trace segments and 11 vias** added, replacing 10 existing segments and 2 vias. At that time there were **422 trace segments, 118 vias, two ground pours, 27 unconnected items**. For the later M1 controller connections and the current counts, see the [main board notes](README.md). All component positions, the six-page schematic, the 70-part BOM and the firmware are unchanged.

[Current PCB](top_board_v2_logic.kicad_pcb) · [Full-board preview](preview/placement.png) · [Encoder area close-up](preview/encoder_links_detail.png) · [Native connection check of that stage](../archive/left_encoder_before_pair/top_board_v2_logic.verification.json)

## Connected scope

| Net | Pad connections checked this round | Status to the controller |
| --- | --- | --- |
| ENC0_A | M0.5 → R14.2 | U3.12 not yet connected |
| ENC0_B | M0.4 → R15.2 | U3.13 not yet connected |
| ENC1_A | M1.5 → R18.2 | U3.5 not yet connected |
| ENC1_B | M1.4 → R19.2 | U3.6 not yet connected |
| ENC2_A | M2.5 → R22.2 → U3.15 / GPIO6 | Full path connected |
| ENC2_B | M2.4 → R23.2 → U3.16 / GPIO7 | Full path connected |

The other end of the six pull-up resistors was already connected to MAIN_3V3 in the previous power stage. Each mating header is still **pin 4 encoder B, pin 5 encoder A**; physical Motor1/2/3 map to M0/M2/M1. What is checked here is the top copper connection; it cannot replace checking the actual harness pin order and encoder direction.

M2 A/B exit from the bottom side of the controller and change layers to cross the existing power, boot config and control lines. The local paths of GPIO15, M2_FI and BOOT and one ground via made way for the encoder breakout; the physical pad connections before and after the replacement were all re-checked. ENC2_B exits at 0.2mm between the module pad and the boot config trace, and the other new signals are 0.25mm; existing rules were not relaxed. New vias are all 0.6mm outer diameter with a 0.3mm hole. The outer bends use 30°/60° turns; all coordinates are given explicitly, with no autorouting.

## Check results

The KiCad native copper connection graph confirms the six listed connections, and all original physical pad connection groups are still connected. The 22 MAIN_3V3 pads, the 30 physical ground pads of the controller and the six control pull-down ground ends all keep their original connections; the nine ground pads that share pin name 49 were checked one by one. All 246 physical pin nets match the current XML.

The full records of the 74 footprints are kept verbatim, including the original 19 mating contacts and four mounting holes. 472 existing trace/via records are kept verbatim, and the 12 replaced records are listed in the [coordinate plan](encoder_links_review.routing.json). The two ground pours and all keep-out definitions, the board outline, design rules and net classes are unchanged, and the pours were refilled from a cleared cache.

11 temporary copies each cut one of: the six interface-to-pull-up connections, the two M2 encoder controller breakouts, and the rerouted M2_FI, GPIO15 and BOOT. The native connection graph of each copy detected the expected break, and unconnected items rose from **27 to 28**; deleting only a short trace inside a pad was not used as an open-circuit control. These checks prove that the matching copper breaks can be detected by CAD; they are not a real-board USB fault diagnosis test.

At this stage DRC reported no new short, copper clearance, via, board edge or isolated copper errors; the original **M2/PWR1 assembly boundary overlap** remains. The max nominal residual of the 19-point top/bottom board registration is about 0.00103mm; the nominal clearance of about 0.046mm from the antenna to the lower board still has no manufacturing tolerance or RF verification. The [stage version record](../archive/left_encoder_before_pair/verification/encoder_links_layout_revision.json) checks that 123 firmware and schematic source files did not change; unrelated firmware tests were not re-run.

## Not yet done

The remaining M1 encoder controller connections from this stage were completed in [later routing](LEFT_ENCODER_ROUTING_REVIEW.md); the M0 encoder, the M0/M1 control leads to the controller, sensing and power still have unfinished items, see the [current remaining details](remaining_connections.json). The ADC power-off issue, full input connector and protection, USB power and auto recovery, actual default-off, power dynamics, temperature rise, assembly and wireless tests are still not done.

This round added no component cost and did not change the conclusion that the current plan fails the budget. The full BOM and assembly quote for the five finished top boards are still missing; a partial CAD pass cannot be a release for manufacturing or whole-robot operation.

Verify this stage's archive in `controller_core`:

```sh
python3 verify_encoder_links.py --board archive/left_encoder_before_pair/top_board_v2_logic.kicad_pcb --baseline archive/encoder_before_links/top_board_v2_logic.kicad_pcb
```

The [375-segment checkpoint](../archive/encoder_before_links/CHECKPOINT.md) keeps the main board from before this round. `route_encoder_links.py` only accepts that old version's hash, and cannot be applied again to the current board.
