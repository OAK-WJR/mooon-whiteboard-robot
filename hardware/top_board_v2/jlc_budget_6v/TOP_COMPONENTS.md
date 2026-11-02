# List of components drawn on the top board

Exported from the current six-page KiCad netlist: 70 components per board, 350 for five boards.

Of these, 63 references have store part numbers and 7 are not yet assigned. This is a list of drawn components, not a full purchasing BOM or a finished-board quote. Having a part number also does not mean assembly and electrical verification are done.

**Scope is only the five top PCBs, the components on them and soldering.** The base board, motors, battery, charger and external accessories are not counted. The final input connector and protection are not drawn yet; undecided USB and ADC behavior may also affect the final list.

| Reference | Value in drawing | JLCPCB part number filled in | Per board | Five boards |
| --- | --- | --- | ---: | ---: |
| C1, C2, C3, C6, C8, C15, C16, C17, C20, C21 | 100n 50V X7R 10% | C14663 | 10 | 50 |
| C4 | 1u 10V X7R | TBD | 1 | 5 |
| C5 | 10u 10V X7R | TBD | 1 | 5 |
| C7, C11, C14, C18, C19 | 22u 25V X5R 10% | C12891 | 5 | 25 |
| C9 | 33p 50V C0G 5% | C1663 | 1 | 5 |
| D1, D2 | SS14 / REVIEW | C2480 | 2 | 10 |
| L1 | 2.2u 20% | C408445 | 1 | 5 |
| L2 | 4.7u 20% | C408447 | 1 | 5 |
| M0, M1, M2 | PZ254V-11-05P | C492404 | 3 | 15 |
| PWR1 | PZ254V-11-04P | C2691448 | 1 | 5 |
| Q1, Q2 | MMBT3904 | TBD | 2 | 10 |
| R1, R2 | 5.1k 1% | C23186 | 2 | 10 |
| R3, R4, R9, R10, R14, R15, R18, R19, R22, R23, R24, R25, R29, R31, R32, R34, R36 | 10k 1% | C25804 | 17 | 85 |
| R5 | 680 1% | C23228 | 1 | 5 |
| R6, R7, R12, R13, R16, R17, R20, R21 | 4.7k 1% | C23162 | 8 | 40 |
| R8 | 22k 1% | C31850 | 1 | 5 |
| R11 | 499 1% | TBD | 1 | 5 |
| R26 | 135k 1% | TBD | 1 | 5 |
| R27 | 30k 1% | C22984 | 1 | 5 |
| R28 | 64.9k 1% | C14891 | 1 | 5 |
| R30 | 470 1% 0.25W | TBD | 1 | 5 |
| R33, R35 | 120k 1% | C25808 | 2 | 10 |
| U1 | CH343G | C2844153 | 1 | 5 |
| U2 | USBLC6-2SC6 | C7519 | 1 | 5 |
| U3 | ESP32-C6-MINI-1-N4 | C5736265 | 1 | 5 |
| U4 | TPS563252DRLR | C19188399 | 1 | 5 |
| U5 | TPS62933DRLR | C3200405 | 1 | 5 |
| USB1 | TYPE-C-31-M-12 | C165948 | 1 | 5 |

Quantities are placement use, without retail minimum order quantities, loss or spares. All unquoted items stay unknown and are not counted as zero in the total.

[Footprints, fields and source file hashes per reference](top_component_inventory.json) · [Cost review](COST_REVIEW.md) · [Six-page schematic](controller_core/logic/top_board_v2_review.pdf)
