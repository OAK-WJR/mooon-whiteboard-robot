# Five top boards: current layout and cost scope

> **Superseded since 2026-10-28:** the current board `top_board_v2_logic` (EasyEDA) charges the 2S battery over USB-C on the board with an IP2326, and on 2026-10-29 the CH343G was removed in favor of the C6's native USB; see [hardware/README.md](../../README.md). The external charging, "the top board does not charge" and the six-page, 70-component KiCad drawing below are all records from 2026-09.

The budget only counts **5 top PCBs, the components on them and soldered finished boards**, total target ¥30, cap ¥50, CNY preferred; the same figure in USD is only a possible fallback. The base board, motors, battery, charger and external accessories are not counted, and no more of them are being bought.

- [Two-layer PCB layout preview](controller_core/logic/preview/placement.png) / [KiCad project and verification scope](controller_core/logic/README.md)
- [Six-page schematic PDF](controller_core/logic/top_board_v2_review.pdf) / [current component list](TOP_COMPONENTS.md)
- [Cost review](COST_REVIEW.md) / [power candidates](controller_core/power_review/README.md) / [firmware](controller_core/logic/FIRMWARE.md)
- [Top board simplified scope](KISS_SCOPE.md) / [structured budget constraints](budget_constraints.json)

The current C6-MINI space/wiring review draft has 70 physical components and 246 physical pins, ERC 0 errors, 0 warnings. The PCB has a two-layer placement and four mounting holes, with the 19 original interface pads kept in place; there are still 10 unconnected items and one interface assembly boundary overlap, and it is not fully routed or released for manufacturing. 73 offline firmware checks and the current C6/R3 cross-compiles pass, and motion stays disabled.

The MINI's global-site part number C5736265 is marked Standard Only, so the original economic assembly fees cannot be applied. The current module has no low-cost final decision; China-site assembly eligibility and a full quote have not been obtained. Of the 70 components, 63 references have store part numbers and 7 are still to be chosen; the full input connector and protection are still not drawn. Neither external charging nor the smaller module has shown that the whole batch meets the budget.

The MINI pre-order reference price on the China-site public page is ¥17.21 each, ¥86.05 for five, and the page shows no stock; this already exceeds the ¥50 cap for five complete finished top boards. That page is a cached public reference returned by a web-access tool, not a live order quote. The CNY references for the three chosen chips and the other items with missing prices are in the [cost review](COST_REVIEW.md), not mixed with the global site's USD fees.

USB power budget, inrush and back-feed, ADC power-off and external divider calibration, power dynamics, the antenna and the actual board-stack clearance are not resolved. Placement follows the triangular board outline, the original 19 contacts and single-sided assembly on top; later hand routing will use 30°/60° directions, then the EasyEDA import will be checked. At this stage only a reviewable placement candidate is kept; it is not called a finished board.

## Historical screening records (the prices and selection status below do not represent the current six-page drawing)

Below are the earlier charging/controller/power candidate analyses and the state where they were paused. The current state follows the entry points above, the six-page netlist and COST_REVIEW; the historical drafts are not routed further.

## Cost change after external charging

Recomputed only with the **historical prices saved on 2026-09-08** below: removing one IP2326 per board, and the loading fee for one extended part in the example, lowers the partial reference cost for five boards by **$3.098 + $3.07 = $6.168**. The original $51.699 becomes **$45.531**; if the historical dual TPS62933 candidate is still used, it is **$47.474**. This is only a conditional calculation of the chips and a few fixed fees, not a current quote, and not the price of the full new architecture.

These top board figures still lack the PCB, the other on-board components and assembly, necessary on-board protection, connectors and similar costs; the USB low-voltage supply problem of the dual TPS54302 also still exists. **This cannot be used to declare five complete boards under $50.** The updated structured calculation is in [external_charging_revision in the price record](candidate_cost_reference.json).

The later switch to CH343G adds $3.216 for five chips at the saved chip reference price; the two historical partial subtotals above become **$48.747 / $50.690**. They are still not the current full BOM or a quote, and missing components and fees cannot be counted as zero. See `usb_recovery_circuit_revision` in the same price record.

## Historical on-board charging candidate prices (for comparison; no longer the current architecture)

The USD unit prices below are **LCSC retail reference prices**, not JLCPCB PCBA billing prices. Quantities use the price tier matching the actual use for 5 boards, not the 1000-piece price. The JLCPCB public pages were checked for package, extended part category and assembly type; the newly obtained C6 China-site SMT CNY public price is listed separately above, keeping the currencies separate. A full live PCBA order quote has still not been obtained this time; nothing was ordered or pre-ordered.

| Function candidate | Model / part number | Per board | This batch | Reference unit price USD | Batch reference subtotal USD | JLCPCB type |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| Main controller RF module | ESP32-C6-WROOM-1-N4 / C6034714 | 1 | 5 | 4.1266 | 20.6330 | Extended part, Economic / Standard |
| Separate USB serial recovery | CH340C / C84681 | 1 | 5 | 0.5900 | 2.9500 | Extended part, Economic / Standard |
| Motor and logic buck chip | TPS54302DDCR / C311983 | 2 | 10 | 0.3028 | 3.0280 | Extended part, Economic / Standard |
| On-board 2S boost charging | IP2326 / C2832094 | 1 | 5 | 0.6196 | 3.0980 | Extended part, Economic / Standard |
| **Core components above only** | **Not a full BOM** | | | | **29.7090** | |

This candidate set did not pass the budget. The chip reference prices above plus the Economic setup fee of $8.18, the stencil at $1.53 and loading fees for 4 extended parts at $12.28 already give an example subtotal of **$51.699**. This is still a screening calculation that mixes LCSC reference prices with JLCPCB public fees; **it cannot be called a supplier quote, nor used to claim that no alternative can get under $50**.

The subtotal above still lacks the PCB, solder joints, possible X-ray, loss/spares/minimum order quantity, inductors, capacitors, passive R/C parts, connectors, battery discharge protection, power switches, fault shutdown, ESD, reverse-polarity protection, USB power switching, harness, mounting parts, tax and shipping. Missing items cannot be set to zero, and chip prices cannot be written up as the full power supply price.

Fee source: [JLCPCB PCBA fee table, page updated 2026-08-20](https://jlcpcb.com/help/article/pcb-assembly-price). Economic SMT is $0.0016/joint; each extended part is $3.07; QFN and other packages with hidden joints may trigger X-ray inspection charged by quantity, with the exact quantity and charge set by the final order. No coupons or promotional subsidies are counted.

Price and assembly type sources:

- C6: [LCSC price](https://www.lcsc.com/product-detail/C6034714.html), [JLC Economic eligibility](https://jlcpcb.com/partdetail/ESPRESSIFINC-ESP32_C6_WROOM_1N4/C6034714).
- CH340C: [LCSC price](https://www.lcsc.com/product-detail/USB_CH340C_C84681.html), [JLC category](https://jlcpcb.com/partdetail/WCH_Jiangsu_Qin_Heng-CH340C/C84681). Do not mix it up with C7464026, which has the same name but a different part number.
- TPS54302: [LCSC price](https://www.lcsc.com/product-detail/C311983.html), [JLC category](https://jlcpcb.com/partdetail/TexasInstruments-TPS54302DDCR/C311983).
- IP2326: [LCSC price](https://www.lcsc.com/product-detail/Battery-Management-ICs_INJOINIC-IP2326_C2832094.html), [JLC category](https://jlcpcb.com/partdetail/Injoinic-IP2326/C2832094).

## Selection conclusions and missing evidence

1. **First reduce duplicate functions and the number of extended part types, then start the new schematic.** Reuse the base board's three drive channels; prefer single-sided assembly on top and a two-layer PCB, though the final layer count must still meet return path, power and wireless layout needs. The original board outline and 19 contacts are the interface basis; the old modules' stacked use of space is not carried over.
2. **The C6-WROOM has been put into the three-page logic drawing as a candidate pending cost confirmation; it has not yet replaced the old PCB or firmware.** GPIO and module physical pin numbers have been checked; the remaining power, peripheral and firmware adaptation still needs to be done. JLCPCB confirms it can be assembled as Economic. The S3-WROOM-1-N16R2 / C2913205 checked separately is Standard Only, so it cannot be estimated with Economic fees. The old Motor3 fault was not shown to be caused by the C6 chip model.
3. **Reusing the TPS54302 as the same chip for both buck rails can be studied.** TI Rev. C (2026-03) §6.3.10 says it can enter 100% duty cycle when BOOT-SW is above the typical 2.1V; this is not a guarantee of a constant 6V across the full battery charge range and at full load. §6.3.5 requires EN to be driven open-drain/open-collector, and it enables by default when floating; the motor power must separately ensure default-off. §6.3.7 gives ±2.5% reference voltage across temperature. Still to compute: feedback error, bootstrap refresh, low-battery dropout, three-motor start/stall, inductor saturation, temperature rise and back-feed. [TI datasheet](https://www.ti.com/lit/ds/symlink/tps54302.pdf).
4. **The IP2326 has been removed from the current top board candidates because of the external charging decision.** Its 2S charge balancing, NTC, input detection and dedicated charging port are no longer being integrated. The original parameter research is kept only as a historical basis; battery discharge protection is still a current requirement. [Historical material: INJOINIC datasheet, hosted by LCSC](https://datasheet.lcsc.com/datasheet/pdf/925274b0e33bb8cb0aa1e158750a3907.pdf?productCode=C2832094).

The alternative TPS62130RGTR / C43590 is $1.3226 at LCSC 1+, and JLC can assemble it as Economic, but it is QFN; per TI it supports up to 6V and 100% duty cycle. It adds one more part type than the dual TPS54302 route in the table above, so for now it is kept as a dropout/performance alternative and not set as the current plan. [LCSC](https://www.lcsc.com/product-detail/C43590.html), [JLC](https://jlcpcb.com/partdetail/TexasInstruments-TPS62130RGTR/C43590), [TI](https://www.ti.com/lit/gpn/tps62130). Source of the S3 limit: [JLC exact part page](https://jlcpcb.com/partdetail/3198303-ESP32_S3_WROOM_1N16R2/C2913205).

**Extra power candidate TPS62933DRLR / C3200405**: the JLC exact part page lists it as an extended part that can be assembled as Economic/Standard; the LCSC same-day 10+ price is $0.4971. With two per board, ten for the whole batch, the chip reference subtotal is $4.971, $1.943 more than the dual TPS54302's $3.028; with the other assumptions unchanged, the incomplete subtotal above becomes **$53.642**. Its 3.8–30V input range is worth using for the USB low-voltage supply re-check, but it did not make this combination pass the budget. This part number has not been put into the schematic yet, and must not be mixed up with the PG version TPS62933P. [LCSC price](https://www.lcsc.com/product-detail/C3200405.html), [JLC category](https://jlcpcb.com/partdetail/TexasInstruments-TPS62933DRLR/C3200405), [TI Rev.D, 2022-08](https://www.ti.com/lit/ds/symlink/tps62933.pdf).

Currently kept: three identical 6V encoder N20s, base board reuse, Wi-Fi/BLE, USB diagnostics and button-free reset/download recovery, necessary battery/power protection, default stop, battery fixing on top, and interface checks. The user has agreed to switch to external charging, so on-board charging is no longer kept. The assembly method and other functions do not change automatically because of this choice.

Next: complete battery/USB power switching, default-off motor power and sensing, check the three-motor load, keep integrating power sensing and calibration, then fill in the BOM and price it from the actual BOM/CPL. **No final total price has been obtained, the budget has not passed, and the new PCB layout for this candidate set has not started.** The current three-page drawing has no on-board charging circuit, and there are no whole-robot simulation or real-board pass conclusions.

Interface re-checks unrelated to the budget have continued: the [base board copper connection review](../bottom_board_review/native_copper/README.md) checked the saved source file and the import model, including an open-circuit comparison of Motor3's six paths. Connectivity in the import model holds, but native EDA copper pour differences, the real harness and real-board verification are still unresolved; this result must not be written up as the new top board or the whole robot being stable.

**Shared firmware progress, 2026-09-08:** fixed the issue where the control lease expired during the command signature check and control was cleared but the stop call was missed; the new boundary regression and related restart tests pass, and the C6 R3 config cross-compile passes. [Fix and offline verification scope](../../../control_tests/LEASE_DEADLINE_FIX.md). This fix is independent of the final controller and power choices, did not open the motion gate, and did not change this folder's budget or manufacturing status.

## Historical draft status

The original module version is still in [kiss_6v](../kiss_6v/README.md); the latest partial native PCB has 62 components on the board, 103 footprints off the board awaiting placement, 375 trace segments and 74 vias; 7 DRC items, 235 unconnected. The original preview and some review reports still match the 55-component stage and do not represent the latest 62-component state. The new copper connections in the partial recovery area have a native connection check, but the full difference review is not done; there is no manufacturing release where it was paused.

The [paused state list](paused_module_state.json) records the differences between the core files and the original review list; it is only a record of the current file state and cannot replace a whole-board review. The old review_manifest.json matches an earlier checkpoint and cannot be taken as proof that all the latest files pass.
