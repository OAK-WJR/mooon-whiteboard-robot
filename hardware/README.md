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

**Schematic** (one A3 sheet):
- ESP32-C6-MINI-1.
- CH343G USB-UART with auto-download.
- 3.3 V buck and 6 V motor supply.
- Battery and motor-voltage sensing.
- 2S USB-C charging (IP2326) and an INA3221 monitor, added on 2026-10-28.

**Board:**
- IMU, ADC and IR emitters moved to a separate sensor-head board, connected
  through a 5-pin JST-SH (J1).
- Routing is converted to tangent arcs.
- Not fabricated yet. The charger is not on the PCB yet.

Every design step is a commit of `motor_controller.epro2`:

```bash
git log --follow -- hardware/top_board_v2/motor_controller.epro2
```

## Earlier top-board directions (not fabricated)

All are under [`top_board_v2/`](top_board_v2). Most notes there are in Chinese.

| Folder | Direction | Why it stopped |
|---|---|---|
| `kicad/`, `validation/`, `R2_VERIFICATION.md` | R2: 4-layer board, 49 parts, verified KiCad ↔ EasyEDA round trip | Superseded by R3 |
| `kicad/r3_*`, `R3_STATUS.md` | R3: 147 parts with USB recovery, watchdog/fault latch and charger | Too complex |
| `kiss_6v/`, `KISS_DECISIONS.md` | "KISS" 6V: one motor voltage, ready-made modules, reuse the base board | Paused for cost |
| `jlc_budget_6v/` | Budget board for JLC assembly, target about 30 CNY each | Budget not met |

Vendor datasheets, saved vendor web pages and 3D models were used during these
reviews but are not redistributed here. The `sources/` and `inspection/` links in
the notes point to files that are not included.
