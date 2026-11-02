# USB/battery power modes of the top PCB

This table checks the current top schematic and firmware. **The wireless power software policy is implemented; full USB power management and real-board verification are not done**. The motors and the external battery are outside this scope; only the conditions of the top power interface are used here.

2026-10-29 update: the current board is EasyEDA `top_board_v2_logic`, and connections are checked against the EasyEDA netlist. The CH343G analysis of the 2026-09 KiCad version is kept at the end and is superseded. The circuit behavior in the table below has been rewritten for the current netlist; the row-by-row re-analysis of current, suspend and inrush is still not done.

## Current actual connections

- **Logic power**: USB VBUS through D1 and battery VBAT through D2 join (diode OR) at the input of U4 (3V3 buck). U4's EN is controlled by SW1; when SW1 is off, 3V3 is off.
- **Motor power**: U5's VIN connects directly to VBAT; EN is MOT_EN = GPIO0, with the R24 pull-down.
- **Charging**: VBUS also feeds the IP2326 (U6, through R37 and the boost inductor L3), which charges the 2S battery; the balance tap connects to U6 through J3 and R43. Charging is not controlled by SW1 or the firmware.
- **USB detection**: VBUS goes through the R52 1k/R10 2.2k divider, then through Q1 (2N7002, gate to 3V3) to IO8, and IO8 also has the R53 100k pull-up to 3V3. With USB, IO8 is high; when 3V3 is off, Q1 is off and IO8 is not back-fed from VBUS. Without USB, R10 pulls IO8 low through Q1: with D1 reverse leakage within 0.3mA, IO8 stays at or below about 0.72V, under the C6's VIL limit (0.25 × VDD ≈ 0.825V). See the [firmware notes](FIRMWARE.md#io8-usb-power-detection) for the calculation.
- **USB data**: D+/D− connect to GPIO13/12 through R51/R50 22Ω. There is no bridge chip, ACT# or bridge suspend state; the whole C6 is the USB device.

`VBAT` is an interface name; the full input connector and protection are not yet in place, so the label cannot be treated as a protection function.

## States that must be told apart

| Input/event | Behavior of the current circuit or firmware | Evidence needed to complete the design |
| --- | --- | --- |
| USB only (no battery), not yet enumerated | With SW1 on, U4 powers the controller and encoders through D1. With no battery there is no 0.6A charge current; with default USB power or a USB-A cable, the CC gate (U8/Q2) pulls the IP2326's EN low, so it does not charge at all. The IP2326's BAT pins (U6.21/22) and the IO4 divider R33 are on the same VBAT net; what voltage the IP2326 puts out at BAT with no battery is not verified. If it holds VBAT_SENSE at 6.5–8.8V for 1 second, the firmware treats it as a qualified battery and starts wireless; otherwise it stays in `waiting_voltage` and does not init wireless | Real `bMaxPower`, current before enumeration and plug-in inrush; measured VBAT_SENSE reading with no battery, and whether wireless starts |
| USB only (no battery), configured | USB CDC can be used for control; the firmware has no USB configuration state or current measurement | Current and 3.3V minimum under the full top board and base board interface load, IP2326 no-load current, diagnostics and reset reliability; enumeration does not automatically grant 500mA |
| USB only (no battery), computer suspended | The firmware does not receive the suspend state; the controller, encoders and IP2326 keep drawing power as usual | Suspend current and resume path of the whole device (including IP2326 no-load current) |
| SW1 off, USB connected for charging | 3V3 is off, and the IP2326 charges as usual; Q1 is off and IO8 has no voltage | Real-board check that there is no back-fed voltage on IO8 and 3V3 |
| Battery only | D2/U4 power the controller. R10 pulls IO8 low through Q1 (at or below about 0.72V with D1 leakage within 0.3mA), `charge="battery"` | Measured IO8 voltage on battery alone (including after D1 heats up); D1 reverse leakage, USB port voltage; load transients of wireless and controller power |
| USB and battery connected together | The two paths feed U4 through the diodes according to their voltages, and usually the higher battery path carries the controller load. The IP2326 charges only when the USB-C source advertises ≥1.5A (CC gate U8/U9/Q2, threshold 0.65V): about 0.6A into the battery, about 1.2A on the 5V side (about 1.3A worst case), within the 1.5A budget; default USB power, USB-A cables and a suspended computer never trigger charging | Current direction of both paths, USB ground/supply disturbance, plug/unplug transients and whether they trigger a reset; whether the USB source can keep the CDC connection under the charge current |
| USB kept, battery disconnected | U4's input switches from the battery path to the USB path. Only after wireless has tried to start (`wireless_start_attempted=true`, including `init_failed`), and VBAT_SENSE leaves 6.0–8.8V or sampling fails, does the firmware revoke control and auto-reset; when wireless has not started, an out-of-range reading only keeps it in `waiting_voltage`. With USB still present, the IP2326 output may hold VBAT within range, and then it does not reset. Detection and reset have a delay | 3.3V waveform during the switch, IP2326 output with no battery, USB current budget; measure whether VBAT_SENSE leaves the range after the battery is pulled; also withdraw motion permission once the power conditions no longer hold |
| Battery kept, USB unplugged | The controller keeps running on the battery; USB CDC disconnects, and the USB control lease becomes invalid | No unexpected EN/BOOT pulses; the USB control lease becomes invalid and related motion stops; a later wireless takeover follows the existing protocol |
| Both disconnected, or repeated low voltage | Capacitors discharge and the controller resets; the two sensing channels may lose power asynchronously | Measured waveforms of EN/BOOT, 3.3V, the sensing pins and ARM; no false start, lockup or abnormal powering of the sensing pins |

The USB-IF update lists the suspend current limit as 2.5mA; the plug-in inrush test also needs specified sampling and current integration, so checking one capacitor's nominal value is not enough. These are requirements for the **whole USB device**, which now includes the C6, the encoders and the charger. [USB-IF electrical compliance updates](https://compliance.usb.org/index.asp?UpdateFile=Electrical).

## Software fixes and remaining hardware conditions

The C6 startup flow first provides native USB CDC and the ADC, and only calls `transport.begin(...)` after consecutive valid battery readings meet the conditions. A failed reading or out-of-range power first revokes control, then requests a software reset, and the next boot judges the power again. The RTC marker can be read over USB after the software reset. `rails` reports `usb_power` (IO8) and `charge` (`"external_power"` or `"battery"`); the IP2326 charge status is not connected to the MCU. For the specific thresholds, startup blocking, failure behavior and evidence, see the [wireless power policy](WIRELESS_POWER.md).

The firmware cannot see the USB configuration, the suspend state or the actual current drawn, and cannot turn off the charger. The wireless policy cannot prove that the MCU, encoder and charger loads meet the pre-enumeration or suspend current limits; the detection and execution delay of the auto reset cannot replace plug/unplug inrush measurement either. Wi-Fi/BLE and USB auto recovery on battery power are both kept; the actual links and currents still need testing.

This wireless software update did not change the power circuit or enable motion. The input protection and power switching plan still need to be decided; new parts on the board must also be counted in the cost of the five finished top boards.

## History: 2026-09 KiCad CH343G version (superseded)

At that time, USB `VBUS_RAW` through D1 and `VBAT_PROTECTED` through D2 entered `LOGIC_VIN_OR`, and U4 then bucked it to `MAIN_3V3`. The CH343's VDD5/VBUS connected to USB, VIO to `MAIN_3V3`, and ACT# was not connected; U5's EN was controlled by GPIO14, and UART0 was provided first at boot. WCH v2.0 §5.2 and §5.4 say the CH343 monitors both V3/VIO for reset, and that its ACT# shows the configured state and is inactive during suspend. So the conclusion at that time was that ACT# could not be used to first turn on its own VIO, and that ACT# also does not carry the current value declared in `bMaxPower`. [WCH datasheet](https://files.waveshare.com/wiki/ESP32-S3-LCD-1.3/CH343DS1_EN.pdf).

[Netlist/code source binding](verification/usb_power_modes.json) · [DTR startup bias simulation](usb_startup_review/README.md) (both for the 2026-09 KiCad version)
