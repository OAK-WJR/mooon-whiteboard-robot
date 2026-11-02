# Hand-routing check of the top USB interface

The USB stage added **67 trace segments, 14 vias and one back-side ground pour**, and adjusted the positions of U2, C1, C2 and C3; at that time the whole board had **170 trace segments, 38 vias, 123 unconnected items**, with no components added. That version is saved in the [checkpoint before reset routing](../archive/reset_before_control/README.md). After this, the [local routing of reset and serial](RESET_ROUTING_REVIEW.md) was done; the current state follows the [main board notes](README.md); the cost of the five finished boards still does not meet the target.

[Current PCB](top_board_v2_logic.kicad_pcb) · [USB close-up](preview/usb_interface_detail.png) · [Full-board preview](preview/placement.png) · [Native check report of the USB stage](../archive/reset_before_control/top_board_v2_logic.verification.json)

## Wiring and layout

U2 was turned to 90° and moved close to the Type-C connector; the two data lines pass through its pads 1/6 and 3/4 respectively, then go to the CH343G. The data contacts for both plug orientations are connected: A6/B6 are D+, A7/B7 are D−. The mirrored contacts cross over with one back-layer jumper; CC1 and CC2 each keep a separate 5.1kΩ pull-down. ST's layout notes ask for the protection device to be near the point where interference enters, and for short signal, VBUS and ground return paths; these are the layout basis, not proof that the whole board passes ESD. [ST USBLC6-2 §2.3](https://www.st.com/resource/en/datasheet/usblc6-2.pdf).

C1, C2 and C3 sit near the CH343G's VDD5, V3 and VIO pins respectively, still using the original three 100nF capacitors. The locating holes and the original Type-C footprint are all kept, and VBUS routes around the two locating holes. The USB ground pour connects with the existing U5 ground pour; U4's ground pour and the full-board power distribution are still not fully connected. The WCH manual asks for these three decoupling capacitors to be near their pins, and defines the CH343 as a USB full-speed device. [WCH v2.0 §7.1](https://files.waveshare.com/wiki/ESP32-S3-LCD-1.3/CH343DS1_EN.pdf).

All new copper was placed by hand using explicit coordinates, with no autorouting, in directions that are multiples of 30°. The 0.2mm data trace width is only the current geometric parameter; no manufacturing stackup was obtained that could confirm 90Ω differential impedance, and no signal integrity or eye diagram verification was done.

## Checks actually done

- The KiCad native connectivity graph checks **21 local connection groups**, including the two original power areas, USB data, CC, VBUS and decoupling. The contacts for both plug orientations connect to the matching CH343G data pins.
- The nets of all **246 physical pins** match the existing XML netlist. The original **127 trace/via records** are kept verbatim; the 19 mating contacts, the full Type-C footprint and locating holes, the module and the board outline are unchanged.
- Native DRC leaves **123 unconnected items and one original M2/PWR1 assembly boundary overlap**, with no new short, copper clearance, locating hole clearance or assembly boundary problems.
- Temporary copies each break the last segment of D+ or D− before the CH343G; the native connectivity graph detected the break from both sets of Type-C contacts to that input, and DRC unconnected items became 124 in each. This is a CAD open-circuit control, not a real unplugging test.
- `.kicad_dru` stays the same. The original 85-byte project file only had metadata, and the new project saves the default parameters explicitly; after a temporary old version was expanded through native SaveBoard, it was confirmed that both versions have the same PCB design parameters, rule severities and net classes, and no new DRC exclusions were added.
- The [top/bottom board registration](bottom_alignment.json) was recomputed, and the max nominal deviation of the 19 matching points is still about 0.00103mm; the [2D antenna projection](antenna_projection.json) is unchanged, and the existing manufacturing tolerance and RF limits remain.

## Not yet done

After the USB stage, the local traces of UART and auto reset to the controller were completed; the later [controller power stage](MCU_SUPPLY_REVIEW.md) connected MAIN_3V3 and the controller ground; USB power to D1, the power input and VMOT distribution are still not done. The current checks do not prove that USB enumeration, download, reset, fault recovery or Wi-Fi/BLE work on a real board. USB inrush, suspend current, back-feed, ADC power-off, power dynamics, RF clearance, the finished-board quote and the JLCEDA round-trip import are still not done.

The previous version is saved in the [checkpoint before USB routing](../archive/usb_before_interface/README.md). The [version record](verification/usb_layout_revision.json) verifies that 123 firmware and schematic source files did not change; old firmware tests were not re-run or re-attributed. The ADC condition model stays bound to the saved old PCB and was not re-verified because of this round's layout.

Reproduce the saved USB-stage copper connection check from `controller_core`; for the current full local copper check command, see the [reset routing notes](RESET_ROUTING_REVIEW.md):

```sh
python3 verify_usb_interface.py --board archive/reset_before_control/top_board_v2_logic.kicad_pcb --baseline archive/usb_before_interface/top_board_v2_logic.kicad_pcb
```

`route_usb_interface.py` is this round's explicit coordinate plan and only accepts the previous 103-segment base; it must not be applied again directly to the current board. The checks above do not access a real serial port.
