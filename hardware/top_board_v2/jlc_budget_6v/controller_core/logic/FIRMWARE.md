# KISS C6 firmware and USB control (top_board_v2_logic)

2026-10-29. Environment **`top_board_v2_kiss_c6`** targets the current EasyEDA board `top_board_v2_logic`: ESP32-C6-MINI-1, native USB, on-board IP2326 2S charging. Pins follow [`include/board_profile_kiss_c6.h`](../../../../../include/board_profile_kiss_c6.h) and the EasyEDA netlist [`hardware/top_board_v2/easyeda/logic_netlist.json`](../../../easyeda/logic_netlist.json). The PCB has not been manufactured, and motion stays disabled; there has been no flashing or real-board testing. The six-page KiCad drawing in this folder and the 2026-09-09 CH343G version notes are superseded; see the [history](#history-2026-09-09-kicadch343g-version-superseded) at the end.

## Board-level interface

The firmware identity is `top_board_v2_kiss_c6_review`. The console is the C6's built-in USB Serial/JTAG (GPIO12/13, through R50/R51 22Ω): Arduino HWCDC, `ARDUINO_USB_CDC_ON_BOOT=1`, `Serial.begin(115200)` with no pins given. The USB identity is VID 303A, PID 1001, and the serial number is the chip MAC. There is no USB-UART bridge on the board (the CH343G and its auto-download transistors were removed on 2026-10-29). The build rejects other board types, S3, a UART console (`CDC_ON_BOOT≠1`) and the motor test entry point.

The motor power request ARM (MOT_EN → U5 EN, R24 pull-down) is **GPIO0**, held LOW during boot and while running. The six FI/BI lines are written LOW first, then the outputs are enabled; the review version does not attach the PWM peripheral, and the low-level driver also rejects nonzero speeds.

| Physical motor | Header | FI / BI GPIO | Encoder A / B GPIO |
| --- | --- | --- | --- |
| Motor1 | M0 | 22 / 23 | 20 / 21 |
| Motor2 | M2 | 14 / 7 | 19 / 18 |
| Motor3 | M1 | 1 / 6 | 2 / 3 |

| GPIO | Use |
| --- | --- |
| 4 | VBAT sensing, 120k/10k divider |
| 5 | INA180A1 total motor current, 2V/A |
| 8 | USB power detection input: high when USB/charger 5V is present (for the low level without USB, see below; the current values are not guaranteed) |
| 9 | BOOT: R9 10k pull-up, plus BOOT/GND recovery pads |
| 12 / 13 | USB D− / D+ |
| 15 / 16 / 17 | Sensor head J1: I2C SDA / SCL / IR_EN_N (R25 pull-up) |

GPIO15/16/17 are application pins; the motor and GPIO safety services do not drive them.

### IO8 USB power detection

Netlist connection: VBUS goes through the R52 1k/R10 2.2k divider to give `USB_DIV`; Q1 (2N7002, pin 1 G to 3V3, pin 2 S to IO8, pin 3 D to `USB_DIV`) connects the divider point to IO8; IO8 also has R53 100k pull-up to 3V3. With USB, the divider point is about 3.4V (about 3.0V when VBUS sags to 4.4V), the gate is at most about 0.3V higher than either side of Q1, so Q1 is off, and R53 pulls IO8 to about 3.3V, which reads high (VIH is 0.75 × VDD ≈ 2.48V). When SW1 turns off 3V3 while charging continues, Q1's gate is at 0V, the channel is off and the body diode is reverse-biased, so the divider cannot back-feed the unpowered C6.

**Low level without USB (values changed 2026-10-29).** Here nothing drives VBUS (D1 is reverse-biased; the IP2326 is a load on VBUS, not a source). Only two currents flow into `USB_DIV`, both to ground through R10 2.2k: one is the reverse leakage of D1 (SS14), entering through R52; the other is about 26µA from R53 flowing down through Q1. With 0.3mA of leakage, the divider point is about (0.3 + 0.026)mA × 2.2k ≈ 0.72V. Q1's Vgs is then about 2.6V, so it conducts and IO8 is about 0.72V, below the VIL max of 0.25 × VDD ≈ 0.825V in the ESP32-C6-MINI-1 datasheet v1.5. At room temperature the leakage at about 7V reverse voltage is far below 0.3mA, and IO8 is close to 0V. But the SS14's reverse leakage limit is already on the order of 0.2–0.5mA at 40V and 25°C, and it rises with temperature ([JLC C2480 SS14](https://jlcpcb.com/partdetail/Mdd-SS14/C2480)), so the margin shrinks when D1 is very hot.

With the original 22k/33k, R53's current alone lifted IO8 to about 0.82V (the VIL limit), and about 25µA of leakage would falsely report `external_power`. After the change to 1k/2.2k, the divider current with USB is about 1.6mA (about 8mW), negligible compared with the charge current. The new values are all JLC basic parts: R52 C21190, R10 C4190, R53 C25803, Q1 C8545.

To measure: IO8's voltage on battery alone (including after D1 heats up), and IO8's high level with USB. The netlist only has connections and no component values, so `verify_kiss_c6.py` can only check topology and cannot catch value problems.

The firmware can report raw encoder counts. Gear ratio, encoder direction and wheel diameter are not calibrated, so speed and position return null, `odometry_qualified=false`.

## Flashing

While the app runs normally, USB Serial/JTAG can enter download mode automatically:

```sh
.venv/bin/pio run -e top_board_v2_kiss_c6 -t upload --upload-port PORT
```

For the first flash, or when the app does not respond on USB, download manually:

1. Set SW1 to OFF. SW1 controls the EN of the 3V3 buck U4.
2. Plug in USB. After 3V3 powers up, IO8 is high, which meets the download condition GPIO8=1, GPIO9=0.
3. Short the BOOT pads (GPIO9 to ground).
4. Set SW1 to ON and run the flash command above.
5. Remove the short, then turn SW1 off once and on once.

USB Serial/JTAG treats RTS as a core reset, which does not re-sample GPIO9. After entering download through the BOOT pads, neither `reset` nor esptool's hard-reset gets back to the app; you must power-cycle as in step 5. Sources: [Espressif USB Serial/JTAG console](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c6/api-guides/usb-serial-jtag-console.html), [esptool reset options](https://docs.espressif.com/projects/esptool/en/latest/esp32c6/esptool/advanced-options.html), [boot mode](https://docs.espressif.com/projects/esptool/en/latest/esp32c6/advanced-topics/boot-mode-selection.html).

## Host entry points

Replace `PORT` with the real serial port, for example `/dev/cu.usbmodemXXXX` on macOS. These hardware commands were not run this time.

```sh
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind c6usb status
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind c6usb rails
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind c6usb reboot
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind c6usb reset
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind c6usb bootloader
```

`c6usb` only accepts a 303A:1001 device on an explicit path, and binds to `top_board_v2_kiss_c6_review`; `ch343g` has been removed. `reboot` requests an app restart and checks for the same device, a different boot ID, a software reset reason and idle/zero output. `reset` enters the C6 ROM through esptool `--before usb-reset` to read the MAC, then resets and checks that the same USB device and the new board firmware come back; if the app was not observed before the reset, it does not claim to have compared the old and new boot IDs. If the app does not come back, the result carries `manual_power_cycle_required=true`, meaning you must power-cycle as in step 5 above. `bootloader` stays in ROM and does not write or erase Flash.

Wi-Fi/BLE use the same authentication protocol. The key can only be written through USB `provision` (rejected in the GPIO fault state); for the `wifi.local` format and the key file, see the [control notes](../../../../../control_tests/README.md). Over USB, the KISS board must use `--usb-kind c6usb`; over wireless it must use `--profile top_board_v2_kiss_c6_review`, otherwise `hello` rejects with a board type mismatch:

```sh
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind c6usb provision --wifi-config wifi.local --key-file control-key.local
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind c6usb status
.venv/bin/python scripts/robot_control.py --wifi HOST --key-file control-key.local --profile top_board_v2_kiss_c6_review status
.venv/bin/python scripts/robot_control.py --wifi HOST --key-file control-key.local --profile top_board_v2_kiss_c6_review rails
.venv/bin/python scripts/robot_control.py --ble BLE_ID --key-file control-key.local --profile top_board_v2_kiss_c6_review status
```

`HOST` is the `wifi_ip` in USB `status`; wireless only starts once the battery qualifies (see below).

## Voltage, current and charging

In normal `status`, `motor_voltage_mv`, `motor_current_ma` and `rail_3v3_mv` stay null; ADC readings are returned through the separate `rails` (firmware command `rail_status`):

| Field | Meaning |
| --- | --- |
| `battery_voltage_mv` | IO4, nominal ratio 13, not calibrated per board (`divider_calibrated=false`) |
| `motor_current_ma` | IO5 INA180 output, equal to `adc_pin_mv[1] // 2`; linear up to about 1.4A, hits the rail at about 1.6A |
| `motor_current_saturated` | Only present when the reading is valid; true when any sample is ≥3150mV after calibration or has raw value 4095, meaning the actual current is at least the reported value |
| `motor_voltage_mv` | Always null; there is no VMOT divider |
| `usb_power` | IO8 |
| `charge` | `"external_power"` when `usb_power` is true, otherwise `"battery"`; never null, `"charging"` or `"full"` |
| `adc_raw`, `adc_pin_mv`, `fresh`, `state`, `divider_gain` | Raw values, calibrated pin voltages, validity, ADC state, ratio 13 |
| `wireless_power_state`, `wireless_start_attempted`, `previous_radio_power_reset` | Wireless power policy state |

The IP2326 charge status is not connected to the MCU, so the firmware only reports the power source and does not judge whether it is charging or full. For the conversion, sampling and invalidation rules, see [ADC readout](ADC_READOUT.md).

The IP2326 charges on its own, and the firmware cannot switch it (`charge_enabled=false`): about 0.6A into the battery (R40 150k), about 1.2A on the 5V side (about 1.3A worst case). Since 2026-10-30 the IP2326's EN is gated by USB-C CC: U8 (LM393) compares the two CC lines (10k/1µF filtered) against 0.65V divided from the U9 (CJ431) reference, and Q2 releases EN only when the source advertises ≥1.5A (Rp for 1.5A or 3A) (R54 100k to VBUS, C35 470nF, goes high after about 15ms). Default USB power and USB-A cables only power the board and do not charge. Charging continues as usual when SW1 turns off 3V3.

## GPIO fault

When GPIO0 init or a later write-low fails, the program latches `gpio_fault` with reason `GPIO0_SAFE_LOW_FAILED`. Motion is then forbidden, control is cleared, it tries to release the outputs, and the initial error and the release error are kept separately. Writing low and the status log cannot prove that the real motor power is cut. A fault latched at boot does not show up on USB by itself: HWCDC does not count as connected until the host first sends or receives a USB packet, so fault replies in `setup()` are dropped (counted in `tx_dropped`); after the host connects, read it with `hello`/`status`.

KISS stays read-only in the fault state and does not reset because of the fault itself:

- Voltage sampling continues.
- Wireless still starts under the same battery condition: VBAT 6.5–8.8V for 1 second, and the task watchdog must be available. On power failure it auto-resets as usual.
- After wireless starts, Wi-Fi/BLE only answer `hello`, `status`, `rail_status` and `stop`; other commands return a fault error, and `reboot` is only possible over USB.
- Fault status messages include `wireless_status`, `wireless_power_state`, `wifi_connected`, `wifi_ip`, `wifi_rssi_dbm`, `ble_ready`, `ble_connected` and `authenticated_wireless`.

The fault loop re-samples right after the blocking `transport.begin()` returns, so the few hundred milliseconds of SDK init do not trigger a `RADIO_POWER_LOST` reset because of an expired sample. A GPIO fault found during `RADIO_STARTING` also no longer triggers a power reset; the next iteration enters the read-only fault loop. R3/Pico fault behavior is unchanged and still goes only through USB.

Wireless starts based on voltage conditions; see [wireless power policy and limits](WIRELESS_POWER.md). The watchdog is registered before wireless init; an init failure is reported over USB and is not allocated again in the same boot cycle.

**Known issue (2026-10-29, firmware fix pending):** when wireless starts on the normal (non-fault) path, `transport.begin()` blocks in the SDK for about 300ms, and the control period timer `lastControlAt` is not refreshed afterward. The control period check in the same iteration treats this block as a timeout: `control_overruns` goes up by 1, `control_timing_fault=true` stays until reboot, and `reason` is overwritten with `CONTROL_OVERRUN`; on init failure it also overwrites `TRANSPORT_INIT_FAILED`. So on every boot where the battery qualifies and wireless starts, `status` shows this timing fault; the [firmware_verification/status.json](firmware_verification/status.json) sample is like this (`control_gap_ms` 310). Motion is disabled anyway and outputs are not affected, but `reason` and `control_timing_fault` after that cannot be used for diagnosis; to see whether init failed, check `wireless_power_state` (`init_failed`) in `rails`. The fix is to refresh `lastControlAt` after `transport.begin()` returns on both startup paths.

## Verification

`control_tests/verify_kiss_c6.py --build` builds KISS and the kept R3 environment, runs native scenarios and host regressions, and compares the compiled GPIO operations with the EasyEDA netlist `hardware/top_board_v2/easyeda/logic_netlist.json`. For IO8 it only accepts the isolation topology above (Q1 gate to 3V3, IO8 pull-up, no other parts on IO8 or the divider point); the verifier also confirms that this check rejects five netlist changes, including connecting the divider directly to IO8. Verification runs in a temporary copy of the public source, in which `config.h` is set up as a compile-error sentinel, so the real private config is never read. Results are written to [firmware_verification/](firmware_verification/report.json) in this folder: the report records each check and the netlist and source file hashes, and the `all_passed` and `cross_compiled` fields say whether everything passed and whether cross-compiling was included; the status, fault_status and rails samples in the same folder come from simulated inputs, not real-board readings.

Prepare the environment before the first run. `.pio/libdeps` is not in the repository, so the ArduinoJson headers must first be fetched by PlatformIO:

```sh
python3 -m venv .venv && .venv/bin/pip install platformio -r scripts/requirements-control.txt
.venv/bin/pio pkg install -e top_board_v2_kiss_c6
.venv/bin/python control_tests/verify_kiss_c6.py --build
```

If these two are missing, the verifier exits before starting, prints the commands above, and does not change the report folder.

The 2026-10-29 run with `--build` passed in full (`all_passed=true`, `cross_compiled=true`). These are all offline checks; the SDK, GPIO, wireless and USB are stand-ins.

Not yet done: measured IO8 voltage on battery alone (the values have changed, see above), power and default-stop hardware, ADC power-off handling and external divider calibration, hardware verification of wireless power switching, real USB enumeration and recovery, coordination between charge current and USB power, base board interface load, wireless coexistence/range, and a full cost count. The software review version is not a manufacturing or motion release.

## History: 2026-09-09 KiCad/CH343G version (superseded)

The results below are for this folder's six-page KiCad drawing: external charging, a CH343G USB-UART bridge (`--usb-kind ch343g`, DTR/RTS auto download, ROM subprocess at 115200 baud, `reset_delay=0.05`), UART0 TX16/RX17, ARM GPIO14, IO5 as the motor voltage divider. They do not represent the current board; the reports and samples in `firmware_verification/` from that time have been overwritten by the verification above, and the old versions can be seen in git history.

- At that time 73 offline checks and the two C6/R3 cross-compiles passed (the report has 75 lines), bound to the KiCad netlist.
- 39 new-board runtime scenarios pass: they run the production main loop, drivers, encoder ISR and communication implementation; the SDK, GPIO, clock, UART, wireless and NVS use stand-ins.
- 9 ADC boundary test groups and 6 wireless power policy boundary test groups pass.
- Compiled GPIO operations were compared with the KiCad-exported netlist of that time; four incompatible builds were rejected by the compiler.
- 131 host tests pass, including 8 CH343G tests and 9 voltage report tests; the real serial port and ROM subprocess are stand-ins.
- Status payload 1185 bytes, 1399 bytes with the authentication envelope.
