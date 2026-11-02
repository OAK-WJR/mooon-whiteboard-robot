# Review of the actual bottom controller board project

> **Superseded since 2026-10-28:** the current board `top_board_v2_logic` (EasyEDA) charges the 2S battery over USB-C on the board with an IP2326, and on 2026-10-29 the CH343G was removed in favor of the C6's native USB; see [hardware/README.md](../../README.md). The external charging and "the top board does not charge" below are records from 2026-09.

**The current entry point is the [5-board budget version](../jlc_budget_6v/README.md); the original ready-made module plan is paused for budget.** The base board connection evidence below stays valid; the 12V/6V motor link records at the end are the historical selection process.

**The user has now confirmed 6V, 300rpm and provided the parameters and an encoder wire color diagram.** The [latest wiring and load check](../jlc_budget_6v/motor_selection/README.md) re-reads this base board's 90 numbered pads and maps them to the current top board's 19 contacts: the encoder black wire (positive) should go to pin 2 on the board, 3V3, and the blue wire (negative) to pin 5, GND. The actual mating face/factory harness pin order still needs checking; the wire color diagram cannot be taken as a numbered plug diagram. Text below of the "no specs provided" kind is the historical state before the extra screenshots.

2026-09-08 addition: the [copper connection re-check of the saved base board](native_copper/README.md) compared the original copper data and checked the KiCad import model. 21 nets are connected in that model, but DRC and fill outline differences are not resolved, so this cannot establish that the original board or the hardware fully passes.

2026-09-07 (America/Los_Angeles). This time the saved `motor_controller_kicad_r2.epro2` was read directly, its base board schematic and PCB preview were viewed, and the base board's real pad nets were extracted. This is not a check of an unsaved online version or of the hardware.

- Schematic page UUID: `e928385bc2a56e10`; PCB UUID: `a7681f7930a373e5`.
- [Project schematic preview](bottom_schematic.webp) / [project PCB preview](bottom_pcb.webp), both extracted directly from that EDA project; the circuit was not redrawn.
- The base board actually has 24 footprints: 3 motor drivers, 3 motor sockets, 4 board-to-board sockets, 9 capacitors, 5 mechanical holes. All 3 drivers are **RZ7889-MS / C22462543**. There is no MCU, buck/boost chip, charger chip or inductor.
- All **90 numbered pad nets** in the modern `.epro2` are identical, item by item, to the 90 numbered pad nets of `bottom_board` in the existing `validation/reverification/refined_export.epro`, including unconnected mechanical pads. This is not proof of copper connectivity, DRC or hardware continuity.
- The [record of this extraction and check](inspection.json) saves the SHA-256 of both projects, all pad nets of the 24 footprints, 43 interface checks and the hashes of the two original previews.

## Reuse limits

**Keep reusing the base board's three drive channels; no duplicate H-bridges are added on top_board_v2.** The top board handles the ESP32 main controller, USB, wireless communication and protected power. The user has accepted [off-robot external battery charging](../jlc_budget_6v/external_charging/README.md), so the top board no longer handles charging or balancing. Currently the whole robot shares one fixed 6V motor supply, fed into the existing base board's H4; the three motors have the same model/rated voltage across the robot.

Pin 1 of base board H4 goes to GND, and pins 2/3/4 together go to `BAT+`. Pin 4 of all three RZ7889-MS connects directly to this `BAT+`, and pin 3 to GND. So in the new system this net name stands for the controlled VMOT output by the top board; the name must not be used as a reason to connect the raw 2S battery here directly, bypassing the top board's regulation and cutoff parts.

The RZ7889-MS working voltage range is recorded in the [manufacturer manual](https://atta.szlcsc.com/upload/public/pdf/source/20240621/B131634ECE1FC6FF8A8E32F28CF9A987.pdf); the driver chip covering 6V/12V does not mean the existing traces, connectors, capacitor voltage ratings and cooling have been verified with two sets of real loads. The base board's actual capacitor voltage ratings, stall current and copper/temperature rise still need to be part of the whole-robot review.

## Actual interface

| Top board interface | Base board interface | Physical motor |
|---|---|---|
| M0 | H2 | Motor1 |
| M2 | H3 | Motor2 |
| M1 | H1 | Motor3 |

On each base board H1/H2/H3, pin 2 connects to the matching driver BI (pin 1), pin 3 to FI (pin 2), pin 4 to pin 4 of the matching motor socket, and pin 5 to pin 3 of the motor socket. **H2.1 is 3V3; H1.1/H3.1 are GND**, so the three five-pin interfaces must not be treated as identical.

All three motor sockets are **ZX-ZH1.5-6PLT / C19272418**, with the same PCB nets, as below. Numbers are this board's footprint pin numbers, not harness colors, and not the left-right order seen from the motor end.

| Motor socket pin on this board | Actual PCB net/destination |
|---|---|
| 1 | Matching driver BO, driver pins 7/8 |
| 2 | 3V3 (intended as encoder power) |
| 3 | Encoder A back to the top board |
| 4 | Encoder B back to the top board |
| 5 | GND |
| 6 | Matching driver FO, driver pins 5/6 |
| 7/8 | Unconnected connector mechanical pads |

**Found a label conflict to verify:** the saved schematic preview labels 2=GND, 5=VCC next to the motor socket, while the PCB nets are 2=3V3, 5=GND. This only confirms that the label in the drawing does not match the actual nets; it cannot yet be said that the real harness is reversed. This must be decided from the exact motor, the socket view and the end-to-end harness definition; this board's pins are not swapped because of a generic N20 wiring diagram or wire colors. The original 19 board-to-board contacts are kept.

## Motors the user has specified

- Current: MECCANIXITY, N20, DC12V, 300RPM, with encoder, two-pack. Product page [Amazon B0F8NHMBWF](https://www.amazon.com/dp/B0F8NHMBWF). The web request returned an Amazon block page; the full specs and current price were not read. 12V/300RPM/two-pack come from the product title the user gave; current parameters from other stores are not applied to it.
- Future: a 6V encoder version of the GA12-N20 sold in China. Product page [Taobao item 565304239241](https://item.taobao.com/item.htm?id=565304239241). The exact speed/gear ratio SKU, stall current, encoder voltage, output type and terminal definitions are not yet confirmed; the price parameters in the share link are not taken as a quote.
- "No motor model/link provided" is no longer listed as missing information. What is still needed is the electrical specs and harness definitions of these two exact versions, and the mounting space.
- The dual-voltage plan above has been narrowed by the user's latest requirement to three 6V versions; no per-motor voltage settings are added, and the motors' rated voltage is never guessed automatically.

Later, the [Amazon-hosted product image](https://m.media-amazon.com/images/I/61qXv1v1T-L._AC_SL3840_.jpg) referenced by a resale page with the same ASIN was found: in the picture, one end of the harness is a six-position plug and the other end is six bare wires. This photo cannot prove the plug pitch, pin functions or the harness actually shipped, but it shows the included wires cannot be taken as an existing adapter cable with plugs on both ends. The ZX-ZH1.5-6PLT used on the base board still needs the correct matching terminals and end-to-end wire order; the wire colors in the picture must not replace the VCC/GND/M+/M−/A/B definitions. The image source chain and hash are kept only in the full-history repository; no unknown stall current was filled in and no reversal was concluded from it.

This time only extraction, checking and recording were done; the base board and its traces were not changed, and no serial port was opened, no battery was charged and no motor was operated.
