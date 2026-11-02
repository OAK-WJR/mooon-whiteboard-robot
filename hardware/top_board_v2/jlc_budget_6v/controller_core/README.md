# External charging version (2026-09, superseded): the controller project and USB subcircuit of that time

> **Superseded (2026-10-28/10-29).** The current board is EasyEDA `top_board_v2_logic` (see [hardware/README.md](../../../README.md)): on-board IP2326 charging. Since 2026-10-29 the CH343G and its DTR/RTS auto-reset transistors have been removed in favor of the C6's native USB Serial/JTAG; EN has the R8 pull-up and C4 delay, and BOOT (GPIO9) has the R9 10k pull-up plus BOOT/GND recovery pads. Reference Q1 is now the 2N7002 for IO8 isolation (gate to 3V3); IO8 connects through it to the VBUS divider and has the R53 100k pull-up to 3V3, for USB power detection (high = USB present; for the low-level margin without USB, see the [firmware notes](logic/FIRMWARE.md#io8-usb-power-detection)). Current pins follow [`include/board_profile_kiss_c6.h`](../../../../include/board_profile_kiss_c6.h) and the EasyEDA netlist; for flashing, see the [firmware notes](logic/FIRMWARE.md). The external charging, six-page KiCad project and CH343G subcircuit below are 2026-09 history.

**Entry point at that time: [six-page schematic and two-layer layout](logic/README.md)**: 70 physical components, 246 physical pins, C6-MINI review candidate; ERC 0 errors/0 warnings, 73 offline checks and the C6/R3 cross-compiles of that time pass. Full-board PCB routing is not done; the MINI's Standard Only assembly limit on the global site is recorded in the [cost review](../COST_REVIEW.md); there is no low-cost or manufacturing final decision.

Below is the separate USB subcircuit built on 2026-09-08 and its verification basis, including USB-C, data protection, the CH343G and EN/BOOT auto recovery, **20 physical components and 74 physical pins** in total. Its connections are fully kept in the six-page project above.

## Verification basis of the separate USB subcircuit

- [Open the KiCad project](usb_recovery.kicad_pro) / [schematic](usb_recovery.kicad_sch)
- [View the whole drawing](preview/usb_recovery.png)
- [Reset timing diagram](preview/reset_sequence.png) / [verification record](verification/report.json)

The CH343G's VDD5/VBUS connects to USB, and VIO shares the controller's 3.3V with the ESP32. It uses the manufacturer's dual-supply connection, removing the old draft's separate serial and reset signal isolators. V3 only gets decoupling and does not power the ESP32; DTR has no pull-down, to avoid entering TNOW mode. Basis: [WCH V2.0 manual](https://files.waveshare.com/wiki/ESP32-S3-LCD-1.3/CH343DS1_EN.pdf) §4–5, [local copy](sources/ch343.pdf).

Auto recovery uses two cross-connected NPNs: DTR high with RTS low pulls EN low; the reverse pulls GPIO9 low; when the two are the same, both control lines are released. GPIO8 keeps its pull-up. The [Espressif auto download notes](https://docs.espressif.com/projects/esptool/en/latest/esp32c6/advanced-topics/boot-mode-selection.html) give this connection and the EN delay requirement.

The EN branch adds 680Ω to limit capacitor discharge current; the EN pull-up is 22kΩ and the delay capacitor 1µF. This differs from the manufacturer's usual 10kΩ value, so the low-level margin and startup hold time were checked separately. Under explicit model conditions, the estimated upper bound of RTS peak current is about 6.49mA; in the three timing cases with R +1% and C +20%, there is still about 14ms of BOOT hold time after EN crosses 0.75VDD, more than the manual's 3ms. The host must actually follow the simulated sequence of a 100ms reset followed by a 50ms BOOT hold; the model cannot be taken as a measurement of the existing serial driver.

## Verification scope

The actual netlist was checked for USB D+/D−, CC1/CC2, the VIO/USB power isolation and the physical pin numbers of the two transistors. The checker can reject deliberately introduced swapped data lines, swapped reset emitters and a CC2 short. Native ERC is **0 errors, 3 warnings**; the three warnings are the TX, RX and GPIO8 interfaces not yet connected to the MCU; they are shown as they are, and no rules were turned off.

ngspice ran **72 DC states and 3 startup timing cases**. The reset base resistor was also disconnected, to check that the failure of EN to go low is correctly detected. SPICE uses a generic NPN model, covering the connection logic and limited parameter variation; it is not the chip manufacturer's model or an all-temperature guarantee. The saturation voltage calculation refers to the specified test points of the [onsemi MMBT3904L datasheet](https://www.onsemi.com/pdf/datasheet/mmbt3904lt1-d.pdf), and these are explicitly listed as calculation assumptions; the final transistor to buy has not been chosen.

The power symbols in the drawing mean that **USB VBUS, the controller's 3.3V and the common system ground are external interface conditions**. They do not mean that regulation, USB/battery switching, discharge protection or the full power supply design have passed.

The MCU has been connected in the six-page project, and the boot pin assignment has been re-checked and fixed. Still to do: power and default stop, actual USB enumeration/current budget, plugging and unplugging while powered off and TNOW boot mode, measured driver and esptool behavior, and ESD/RF/harness and base board interface tests. The CH343G cannot simply take on the configurable-EEPROM capability of the P version; successful enumeration also does not mean the whole board is automatically allowed to draw 500mA from USB. The RF and encoder power draw in the USB-only diagnostic mode must be included in the system budget.

Addition: [DTR startup bias check](logic/usb_startup_review/README.md): 24 limited-condition model scenarios support keeping the current 4.7k pull-up, and a comparison with a wrong pull-down detected insufficient high-level margin; no parts were added, and no pass conclusion for actual CH343 sampling/USB power was obtained. The [power state check](logic/USB_POWER_MODES.md) confirmed that the stop gate does not also forbid wireless init; the USB/battery power policy still needs to be finished.

## Components and cost

The [JLCPCB C2844153](https://jlcpcb.com/partdetail/WCH_Jiangsu_Qin_Heng-CH343G/C2844153) public page at that time listed SOP16, extended part, Economic assembly supported, $1.2332 at 1+, $6.166 for five. That is $3.216 more than the historical CH340C reference price of $2.95 for five; removing the isolation peripherals simplifies the circuit, but **this does not confirm that the batch cost dropped or is under budget**. Public component prices are not a full PCBA quote. The other components have not been individually assigned JLCPCB part numbers and priced.

The controller has been put into the logic drawing as a candidate pending cost confirmation; the charger and battery protection were not chosen as a result, and the PCB layer count and layout are not frozen. The wiring at that time followed [logic/c6_pin_plan.json](logic/c6_pin_plan.json); it is the 2026-09 KiCad pin table (UART 16/17, ARM GPIO14), now replaced by `include/board_profile_kiss_c6.h`. The parent [c6_pin_plan.json](c6_pin_plan.json) is kept only as historical input to the generator; in it, the GPIO22 power request was replaced by GPIO14 in the MINI drawing of that time, and M0_BI is GPIO23, so the historical table cannot be used directly for wiring. The current MINI's 53 physical pin numbers were checked independently against the manufacturer's v1.5 manual.

Rebuild and verify:

```sh
cd hardware/top_board_v2/jlc_budget_6v/controller_core
python3 build_usb.py
python3 verify_usb.py
python3 plot_usb.py
```

`build_usb.py` only writes to this folder; `verify_usb.py` exports the actual netlist through KiCad, then runs the checks and ngspice; it does not access a serial port or the real firmware config. The preview is exported from KiCad as SVG and rasterized; the old PCB and firmware were not changed. The whole board is not done and not released for manufacturing.
