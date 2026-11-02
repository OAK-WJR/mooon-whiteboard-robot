# USB and logic power input routing check

USB VBUS is connected to the anode of D1; the cathodes of D1/D2 join and go to the U4 input and the C7/C8 decoupling. A front-side ground jumper was added near U4, connecting back to the common ground. The battery-side input of D2 is still to be routed; this stage cannot prove that dual-input power works.

The main board has **587 trace segments, 167 vias, two ground pours, 10 unconnected items**. 21 segments and nine vias were added, and all 724 old copper records are kept; the 74 footprints, 19 mating contacts, board outline, pour definitions, schematic, BOM and firmware are unchanged.

[Current PCB](top_board_v2_logic.kicad_pcb) · [Full-board preview](preview/placement.png) · [Input path close-up](preview/logic_input_detail.png) · [Native connection report](top_board_v2_logic.verification.json)

New power traces are 0.8mm wide, routed in 30°/60° directions plus horizontal and vertical, with local layer changes to cross existing signals and 3.3V copper. The USB power tap branches from the connector's existing VBUS via, avoiding the MH4 mounting area; it does not depend on the CH343's 0.3/0.4mm back-side power branch. The power copper from the original connector pads to the via and in the C7/C8 area keeps its original width. The new ground jumper is 0.4mm wide, near U4's output inductor, connecting the input-side ground to the common ground where C11 sits.

New vias are 0.6mm pad, 0.3mm drill. The connection graph confirms that all 74 physical ground pads are in the same group, but this does not prove that return impedance, switching noise or transient behavior pass.

## Verification scope

- The native netlist defines the diode symbol as pin 1 K, pin 2 A; in the PCB, D1's A goes to VBUS_RAW, D2's A to VBAT_PROTECTED, and both K to LOGIC_VIN_OR. This check covers the symbol and copper nets; it does not replace hardware verification of polarity, leakage or back-feed into other chip pins.
- The seven pads of the USB input net and the six pads of the logic OR net are all connected; all original connection groups, the 22 MAIN_3V3 pads, nine VMOT pads, six encoder lines and the M0/M2 control signals stay connected. The 246 pin nets match the current XML netlist item by item.
- Eight fault copies each remove one of the seven power paths or the new U4 ground via; in each, the native connection graph detected the break at the matching endpoints, and DRC unconnected items rose from 10 to 11.
- Another copy removed the seven old CH343 back-side power branch segments; the two USB VBUS contacts still connect to D1 and U2; the CH343's two power inputs disconnect as expected, with 12 unconnected items. This verifies that the new power tap path is independent.
- Main board DRC shows no new short, copper clearance, board edge or isolated copper errors; the original M2/PWR1 assembly boundary overlap remains. That issue was not masked or waived.

The [version record](verification/logic_input_layout_revision.json) checks the unchanged firmware and schematic sources. The existing offline tests did not become real-board evidence for the new PCB because of this.

## Still to do

The [10 unconnected items](remaining_connections.json) include the two M1 control lines, two VBAT_PROTECTED connections, and three connections for each of the two ADC channels. The full input connector and protection, ADC power-off, USB current budget/inrush/suspend/switching back-feed, auto recovery, power dynamics and temperature rise, the actual board stack and RF, the price of the five finished boards and the JLCEDA import round trip still need verification. No manufacturing release.

Re-check the current board in the `controller_core` folder:

```sh
python3 verify_logic_input.py --board logic/top_board_v2_logic.kicad_pcb --baseline archive/logic_before_input/top_board_v2_logic.kicad_pcb
```

The [566-segment checkpoint](../archive/logic_before_input/CHECKPOINT.md) saves the main project and preview from before the change. `route_logic_input.py` only accepts that base, and cannot be applied again to the current board.
