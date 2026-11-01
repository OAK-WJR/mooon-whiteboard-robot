# Hardware

Hardware design files are licensed under [CERN-OHL-P-2.0](LICENSE).

## Boards in use on the robot

The robot is a stack of three boards:

1. A Waveshare ESP32-C6-LCD-1.47 module (bought, not designed here).
2. The custom **adapter v2** PCB.
3. The custom **base board** with three RZ7889 H-bridges, the motor and encoder
   connectors, and the 2S battery input.

The adapter and the base board are drawn in EasyEDA Pro. The project is
[`top_board_v2/motor_controller.epro2`](top_board_v2/motor_controller.epro2),
which holds nine boards.

[`v1_base_board/`](v1_base_board) holds the netlist of the first base board
(April 2026). That board had a socket for a 38-pin ESP32 DevKit.

## Next controller: `top_board_v2_logic` (in progress)

The goal is one board that replaces the module + adapter stack. It is board
**top_board_v2_logic** in `motor_controller.epro2`.

**Schematic** (one A3 sheet, as of 2026-10-29):
- ESP32-C6-MINI-1 on native USB-C (the C6's USB Serial/JTAG, 303A:1001).
  The CH343G USB-UART bridge and its auto-download transistors were removed
  on 2026-10-29. Manual download uses the BOOT/GND pads.
- 3.3 V buck, switched by SW1, and a 6 V motor supply enabled by GPIO0
  (pulled down).
- Battery-voltage sensing (120k/10k divider on IO4).
- Total motor current: an INA180A1 on a 0.1 Ω shunt, read on IO5. It
  replaced the INA3221 drawn on 2026-10-28.
- USB-power sense on IO8: VBUS → R52 1k → `USB_DIV` → R10 2.2k → GND. A
  2N7002 (Q1, gate at 3.3 V) passes `USB_DIV` to IO8, which has a 100k
  pull-up (R53) to 3.3 V. With USB plugged in, `USB_DIV` is about 3.4 V
  (3.0 V at a 4.4 V sag), Q1 is off and IO8 reads high. Without USB, VBUS only
  carries D1's reverse leakage. The stiff divider keeps IO8 at or below about
  0.72 V for up to ~0.3 mA of leakage, under the C6's 0.825 V low-level limit
  (0.25 × VDD). When 3.3 V is off, Q1 blocks the divider, so it cannot
  back-feed the C6 while the pack charges. Still to measure on the real board:
  IO8 on battery alone. See
  [`FIRMWARE.md`](top_board_v2/jlc_budget_6v/controller_core/logic/FIRMWARE.md#io8-usb-供电检测).
- On-board 2S USB-C charging (IP2326, 0.6 A into the pack, R40 150k). It
  draws about 1.2 A at 5 V (1.3 A worst case).
- USB-C current gate (added 2026-10-30): the IP2326 EN pin only goes high
  when the source advertises at least 1.5 A. A CJ431 (U9) makes a 2.5 V
  reference, which R57/R58 divide to a 0.65 V threshold (`CC_VTH`). An LM393
  (U8) compares each CC pin, filtered by 10k/1 µF (R59/C31, R60/C32), against
  that threshold. Both outputs are wired together onto `CHG_OFF`, which has a
  100k pull-up (R55) and drives the gate of Q2 (2N7002).
  - CC above 0.65 V means a 1.5 A or 3 A source. The comparator pulls
    `CHG_OFF` low, Q2 turns off, and EN rises through R54 (100k to VBUS) with
    C35 (470 nF), about 15 ms later.
  - A Default-USB source or a USB-A cable leaves both CC pins below the
    threshold. Q2 then holds EN low: the board still runs from USB but does
    not charge.
  - VBUS capacitance is C26 4.7 µF + C27 1 µF + C34 100 nF, under the 10 µF
    Type-C sink limit.

**Board:**
- IMU, ADC and IR emitters moved to a separate sensor-head board, connected
  through a 5-pin JST-SH (J1).
- Routing is converted to tangent arcs.
- The charger and the CC gate are placed and routed on the PCB. Not
  fabricated yet.
- The IP2326 exposed pad has 9 GND vias (0.3 mm drill, tented on the bottom).
- C5 is 22 µF.
- R49 is a 1 W alloy shunt (FRM121W, same 1206 land).
- R40/R41 sit 1 mm from U6.

**Firmware:** the `top_board_v2_kiss_c6` PlatformIO environment, a review
build with motion disabled. Flashing, the host commands and the rails report
are described in
[`FIRMWARE.md`](top_board_v2/jlc_budget_6v/controller_core/logic/FIRMWARE.md),
which sits in an older folder. Its pins follow the netlist export
[`top_board_v2/easyeda/logic_netlist.json`](top_board_v2/easyeda/logic_netlist.json).

In this snapshot, `motor_controller.epro2` is the EasyEDA backup saved on
2026-10-30 16:13. It includes the 2026-10-29 cut (CH343G removed, INA180,
native USB, IO8 sense, I2C/IR remap) and the 2026-10-30 USB-C charge gate.
It opens in EasyEDA Pro (desktop or web). The step-by-step
history of this file (one commit per design step up to 2026-10-28) is in the
full-history repository, not in this snapshot.

## Earlier top-board directions (not fabricated)

These designs were never ordered. To keep this snapshot small, only the
`jlc_budget_6v/` notes and the September KiCad sources of
`top_board_v2_logic` are included here; the other folders below are in the
full-history repository. Most notes are in Chinese.

| Folder | Direction | Why it stopped |
|---|---|---|
| `kicad/`, `validation/`, `R2_VERIFICATION.md` | R2: 4-layer board, 49 parts, verified KiCad ↔ EasyEDA round trip | Superseded by R3 |
| `kicad/r3_*`, `R3_STATUS.md` | R3: 147 parts with USB recovery, watchdog/fault latch and charger | Too complex |
| `kiss_6v/`, `KISS_DECISIONS.md` | "KISS" 6V: one motor voltage, ready-made modules, reuse the base board | Paused for cost |
| `jlc_budget_6v/` | Budget board for JLC assembly, target about 30 CNY each | Budget not met |

Vendor datasheets, saved vendor web pages and 3D models were used during these
reviews but are not redistributed here. The `sources/` and `inspection/` links in
the notes point to files that are not included.
