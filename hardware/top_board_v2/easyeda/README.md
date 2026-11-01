# KISS C6 logic board: EasyEDA netlist

`logic_netlist.json` is the connectivity of the current KISS C6 logic board
(ESP32-C6-MINI-1, native USB, on-board IP2326 2S charger), as one flat JSON
object: `"<ref>.<pin>": "<net>"`, keys in natural order. Example:
`"U3.12": "MOT_EN"` means pad 12 of the C6 module is on net `MOT_EN`.

How it was exported (2026-10-30, after the USB-C CC charge gate was added):

1. EasyEDA Pro, schematic page `047e235f224c2688`.
2. `sch_ManufactureData.getNetlistFile` in the Protel2 format.
3. The Protel2 nets were flattened into `ref.pin -> net` JSON.

The file has no part values or footprints, only references, pins and net
names. Pins that are not connected to anything are left out. Unnamed nets
keep EasyEDA's generated names (`$22N...`).

`control_tests/verify_kiss_c6.py` checks the compiled firmware against this
file. It maps each ESP32-C6-MINI-1 module pad to its GPIO using the datasheet
pin table (v1.5, Table 3-1). Re-export this file whenever the schematic
changes, then run the verifier again.
