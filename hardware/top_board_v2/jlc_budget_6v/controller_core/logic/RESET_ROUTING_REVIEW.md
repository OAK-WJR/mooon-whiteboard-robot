# Hand-routing check of auto reset and serial

This document records the reset stage: **85 trace segments, 18 vias and one back-side ground pour** added, and the position or orientation of 11 components adjusted. At that time the whole board had **255 trace segments, 56 vias, 98 unconnected items**, and the one original M2/PWR1 assembly boundary overlap remained. No components were added, and the schematic, BOM and firmware did not change; the budget for five finished top boards is still not met.

[Current PCB](top_board_v2_logic.kicad_pcb) · [Full-board preview](preview/placement.png) · [Reset control area](preview/reset_control_detail.png) · [Controller EN/BOOT area](preview/mcu_reset_detail.png) · [Native check report of that stage](../archive/mcu_before_supply/top_board_v2_logic.verification.json)

## Local nets connected

| Function | Copper connections checked |
| --- | --- |
| DTR | U1.10 → R3.1, R6.2, Q2 emitter |
| RTS | U1.11 → R4.1, R7.2, Q1 emitter |
| EN drive | R3.2 → Q1 base; Q1 collector → R5.1 |
| Controller EN | R5.2, 22kΩ pull-up R8.2, 1µF C4.1 → U3.8 |
| BOOT drive | R4.2 → Q2 base; Q2 collector, pull-up R9.2 → U3.23 / GPIO9 |
| GPIO8 boot config | R10.2 → U3.22 / GPIO8 |
| Controller serial receive | U1.2 / TXD → U3.30 / GPIO17 |
| Controller serial transmit | U3.31 / GPIO16 → R11.2; R11.1 → U1.3 / RXD |

R5, R8 and C4 moved to the side of the module's EN pin; Q2 was turned so its base and collector traces are separate. R20 moved slightly to leave real copper clearance for the EN branch; R25 avoids the existing power feedback trace. The serial lines were placed by hand along the right side of the control area, and R11 stays near the controller's transmit pin. Directions are multiples of 30°, and no autorouting was used.

The R6/R7 pull-ups connect to the existing CH343 VIO decoupling node; the power ends of R8/R9/R10 are joined on another group of local copper. At that time these two supply groups were not yet connected to the controller's power/ground; those connections were made in the later [controller power stage](MCU_SUPPLY_REVIEW.md), and the current state follows the [main board notes](README.md). Connectivity of the nets in the table does not mean the board can already boot, run diagnostics or auto-download.

## Actual verification

The KiCad native connectivity graph checks **33 local connection groups**, including the existing power area, USB, reset and serial; all **246 physical pin nets** match the current XML. The old version's **208 trace/via records** are kept verbatim, and the full records of the 63 footprints that were not adjusted are also the same, including the controller, the Type-C locating holes and the original 19 mating contacts.

Four temporary copies each cut one of the controller breakouts for EN, GPIO9, GPIO17 receive and GPIO16 transmit. The connectivity graph detected the matching break in each, and DRC unconnected items rose from **98 to 99** in each. After the front-layer breakout of GPIO17 was cut, its via had only a back-layer connection left, producing the expected dangling via warning, which is kept in the control record; it was not added to the main board's check waivers.

The main board has no new short, copper clearance, board edge or assembly boundary errors. The custom rules, the expanded KiCad design parameters and the net classes are the same as in the previous version. The 19-point top/bottom board registration was recomputed, and the max nominal residual is still about 0.00103mm; the 2D antenna projection is unchanged, and the existing tolerance and RF limits are still unresolved.

## Transistor part number and limits

Checked the JLCPCB basic part **JSCJ MMBT3904(RANGE:100-300) / C20526**, SOT-23, supporting Economic and Standard assembly. The manufacturer's Rev.2.1 gives 1 base, 2 emitter, 3 collector, matching this drawing's symbol and the native pad numbers. [JLC part page](https://jlcpcb.com/partdetail/21237-MMBT3904/C20526), [manufacturer datasheet](https://datasheet.lcsc.com/datasheet/pdf/9335632bcfa83df8351353eab379720e.pdf?productCode=C20526).

This part number is still a candidate whose pinout has been checked; it is not yet in the main BOM, and there is no price or stock confirmation for this batch. The PZT3904 link inherited from the original generic symbol cannot be used to check the SOT-23 footprint. The manufacturer's hFE=100–300 test condition is VCE=1V, IC=10mA, which is not a guarantee for this circuit's low current or for the full temperature range; the old generic NPN simulation did not become a manufacturer model because of this. The basis and limits are saved in the [part check record](../sources/reset_transistor_review.json).

## Remaining work

Later stages connected the MAIN_3V3 distribution and the controller ground; the later control stage also connected GPIO15 and the M2 controls; the power input, VMOT distribution and the remaining base board signals are still incomplete; input protection, ADC power-off, USB power draw and back-feed, power dynamics, RF, full finished-board cost and the JLCEDA round trip are still to be done. There is no real-board evidence yet for actual reset timing, boot config, USB download or fault recovery.

The [version record](verification/reset_layout_revision.json) checks that 123 firmware and schematic source files did not change; old firmware tests were not re-run or re-attributed. The previous version is saved in the [checkpoint before reset routing](../archive/reset_before_control/README.md).

Verify the archived reset-stage copper connections from `controller_core`:

```sh
python3 verify_reset_control.py --board archive/mcu_before_supply/top_board_v2_logic.kicad_pcb --baseline archive/reset_before_control/top_board_v2_logic.kicad_pcb
```

`route_reset_control.py` only accepts the previous 170-segment base and cannot be applied again to the current board. The check does not access a real serial port.
