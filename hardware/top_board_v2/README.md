> English overview and current status: [../README.md](../README.md). The notes
> below are the working notes from September 2026, written in Chinese at the
> time and translated into English in November 2026.
> In this curated snapshot many linked review files (`kiss_6v/`, `kicad/`,
> `validation/`, previews, PDFs) are left out to keep the repository small;
> they are in the full-history repository.

# top_board_v2 — five finished top boards

**Only covers the five top PCBs, the components on them and soldering; total target ¥30, cap ¥50.** The base board, motors, battery and charger are not counted, and no more of them are being bought. The top keeps the ESP32, USB auto recovery, the power supply and the original base board interface; there is no charging or balancing on the board.

It is currently a **C6-MINI two-layer layout review draft, and full-board routing is not done**: [PCB preview](jlc_budget_6v/controller_core/logic/preview/placement.png), [KiCad project](jlc_budget_6v/controller_core/logic/top_board_v2_logic.kicad_pro), [six-page schematic PDF](jlc_budget_6v/controller_core/logic/top_board_v2_review.pdf). 70 components, 246 physical pins, ERC 0 errors/0 warnings; 73 offline firmware checks and the C6/R3 cross-compiles pass. The 19 mating pad positions are kept, the max nominal registration deviation is about 0.00103mm, and the top board outline lies inside the base board.

**The budget has not passed yet.** The MINI's part number on the global site is Standard Only, so the economic assembly pricing cannot be reused; this layout is a space/wiring study, not the low-cost final design. Neither the [cost review](jlc_budget_6v/COST_REVIEW.md) nor the [current list with part numbers filled for 63/70 references](jlc_budget_6v/TOP_COMPONENTS.md) has a complete finished-board total.

The PCB still has 153 unconnected items and one M2/PWR1 assembly boundary overlap. Input protection, USB power draw/inrush, ADC power-off and external divider calibration, power dynamics, the antenna and real-hardware verification are not done; motion stays disabled, and it cannot be ordered for manufacturing. For the current files and full scope, see the [review notes](jlc_budget_6v/controller_core/logic/README.md) and [firmware notes](jlc_budget_6v/controller_core/logic/FIRMWARE.md).

## Historical versions

The previous direction was [whole-robot 6V, ready-made modules, reused base board](kiss_6v/README.md). The [two-layer electrical working draft](kiss_6v/electrical_layout/README.md) had started partial layout and routing, and is now paused for cost; the full board is not done and cannot be ordered. The 147-component R3 below is kept as a historical review reference; the dual-voltage fully custom architecture is no longer being extended.

For the historical module version, see the [carrier board preview at the 55-component stage](kiss_6v/electrical_layout/carrier_preview.png) or open the [KiCad electrical PCB where it was paused](kiss_6v/electrical_layout/top_board_v2_6v.kicad_pcb). The preview only crops the area near the board outline, and does not yet show the 7 components of the recovery area; the latest native file has 62 components on the board and 103 footprints off the board awaiting placement. The top has no height limit; the board outline and the original 19 mating contacts are kept; charging, protection, full wiring and assembly clearance are still to be done. The old [C6 top mounting draft](kiss_6v/kicad_installation/README.md) and the [Pico joint mounting study](kiss_6v/pico_joint_installation/README.md) are both historical material. The [module review](kiss_6v/MODULE_REVIEW.md) keeps the specific basis and limits.

**For the historical R3 status and file entry points, see [R3_STATUS.md](R3_STATUS.md).** It is the native KiCad schematic review draft for the 2S battery and separate USB recovery direction, integrated into 10 pages and 147 components, with a fixed watchdog and fault latch; R3 power hardware and PCB layout/routing are not done. There is a [schematic PDF](kicad/r3_system/top_board_v2_r3.pdf) and a [KiCad review project with its local symbols/footprints included](review_r3_schematic/top_board_v2_r3.kicad_pro). The old architecture's pricing reference is in the [cost update](procurement/R3_COST_UPDATE.md); it does not apply to the current 6V module version and is not a full PCBA quote.

**The version that was routed and round-tripped into EasyEDA is still R2:** D3/D5 polarity was fixed in KiCad first, then imported into EasyEDA `motor_controller → top_board_v2_kicad_r2`. Use [R2 verification and file entry points](R2_VERIFICATION.md) as the reference. The EasyEDA delivery uses the `.epro2` that passed round-trip verification; the old `.epro` changes clearance rules and has been removed from the delivery entry points, see the [round-trip record](ROUNDTRIP_VERIFICATION.md). What is kept below is the record of the previous corner-fix revision; the whole robot is still not released for manufacturing.

Below is the historical corner-fix revision record: the revision was once synced to EasyEDA **motor_controller → top_board_v2_kicad**, and was not released for manufacturing. The whole-robot 6V/12V dual-voltage requirement of that time has been replaced by the 6V single-voltage decision above.

Current files: [KiCad PCB](kicad/top_board_v2_refined_4layer.kicad_pcb), [KiCad project](kicad/top_board_v2_refined_4layer.kicad_pro), [standalone EasyEDA project](top_board_v2_kicad.epro), [full project backup](motor_controller_with_kicad_4layer.epro). The original top_board_v2 and the other three PCBs are kept; open the new board with _kicad in its name to see this round's changes.

![Revised board read back from EasyEDA](validation/reverification/eda_render_after.png)

59 ordinary trace right angles were changed to segmented 30° turns; the remaining 17 flagged points are at pad/via connections. Explicitly specified turn coordinates were used; no autorouter was called. Fixed connectors stayed in place and were locked.

Four-layer decision: L1 components and main signals, L2 GND reference plane, L3 3V3 distribution and a few local connections, L4 the remaining signals. The actual manufacturer stackup, copper thickness and USB impedance are still to be confirmed.

KiCad: 49 components, 215 pads, 539 copper trace segments, 93 vias, 0 unconnected, 1 M2/PWR assembly boundary overlap. After the EDA split them into 574 segments, the copper union and netlist comparison passed, and PCB DRC returned 0 items. The 19 top/bottom board pins match one by one, with a max center difference of about 0.00103 mm; the top board outline is inside the base board, at about 81% of its area.

Items not yet released include: the fixed supply setting of about 6.6V, the passive path after the boost turns off, missing sensing and a separate power cutoff, the USB power path, the base board projection under the antenna, USB impedance, assembly boundaries and 1 schematic warning. 12V must not be connected to the existing BAT 1S socket.

Added USB / Wi-Fi / BLE main firmware and a JSON host program that AI can call, supporting status, stop, config, reboot and USB ROM recovery. The current integrated board firmware rejects motor start with BOARD_POWER_UNREVIEWED. There are no real-board test results.

- [Full verification report](VERIFICATION.md)
- [USB, Wi-Fi and BLE control notes](../../control_tests/README.md)
- [USB diagnostics and three-wheel test method](../../usb_diagnostics/README.md)
- [ngspice plot](validation/reverification/power_simulation.png), [simulation script](simulate_power.py)
- [Pin and outline diagram](validation/reverification/mating_verification.png)

The current choice is three identical 6V, 300rpm encoder N20 motors; the MECCANIXITY 12V 300RPM model provided earlier is kept as a historical reference and cannot be used as a performance basis for the 6V version. The three RZ7889-MS on the base board are reused; for details and the socket label conflict, see [base board and motor review](bottom_board_review/README.md). The user has provided the rated/stall current table and the encoder wire color functions, see [current motor inputs](jlc_budget_6v/motor_selection/README.md); still to confirm: that the real motors match the chosen table, the actual plug pin order, encoder calibration, the board spacing, and the positions of the battery and metal structure.
