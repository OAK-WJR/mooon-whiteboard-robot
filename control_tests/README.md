# Controlling the integrated board over USB, Wi-Fi and BLE

**2026-09-08 shared runtime firmware fix:** adds the stop call that was missed when the lease expires during the signature check; the boundary regression on the real command handler, the related restart regressions and the C6 build pass. See [deadline boundary fix and verification](LEASE_DEADLINE_FIX.md). This is a new targeted verification; the historical full-suite report below must not be taken as having been re-run this time.

**R3 separate USB bridge version:** uses `top_board_v2_r3`, UART TX16/RX17, ENC0_B=GPIO6; on the computer side, pass `--usb PORT --usb-kind cp2102n` explicitly. See [R3 firmware](../hardware/top_board_v2/firmware_r3/README.md) and [CP2102N host tool notes](../hardware/top_board_v2/firmware_r3/host_cp2102n/README.md). These two versions have still not passed power and real-board acceptance. What is kept below is the native-USB R2/old integrated board flow, and it must not be used for R3 without telling them apart.

`src/top_v2_main.cpp` is the main firmware for the new integrated board, PlatformIO environment `top_board_v2`. It and the dedicated diagnostic firmware in `usb_diagnostics` are two different programs; run one of them at a time. The main project `c6` is still the old adapter board configuration and must not be mixed in: the new board has ENC0_A=GPIO7, the old board GPIO16.

The current power supply has not passed, so both the main firmware and the new board diagnostic firmware stay at `BOARD_POWER_UNREVIEWED`, with no software switch to bypass it. USB, wireless, encoders and the stop/reset flow can be checked; loaded running cannot be accepted on this basis. Not yet connected to a real board.

Install the local host tools from the project root and build:

```sh
.venv/bin/pip install -r scripts/requirements-control.txt
.venv/bin/pio run -e top_board_v2
.venv/bin/python scripts/pcb_control.py list
```

If `include/config.h` is missing, copy it from `include/config.example.h`; keep any existing config. The new board's wireless credentials are set over USB; the Wi-Fi credentials in the old config file are not used. The build is 4MB Flash with a 3MB app partition; this version has no OTA partition and updates over USB. `PORT`, `HOST` and `BLE_ID` below must be replaced with the real devices.

The PlatformIO platform is pinned to the pioarduino `55.03.39` release package, using Arduino-ESP32 `3.3.9` / ESP-IDF `5.5.4`; ArduinoJson is pinned to `7.4.3`. The earlier bare `espressif32` name depended on what was already installed on this machine, and could not guarantee that another computer picks the same platform. The release package was downloaded and checked against the publisher's SHA256; this round built with the existing tool cache, and no fresh-computer install test was done. [Platform release](https://github.com/pioarduino/platform-espressif32/releases/tag/55.03.39), [PlatformIO version config](https://docs.platformio.org/en/latest/projectconf/sections/env/options/platform/platform.html)

Flash only after the power path is confirmed to power the MCU stably; the existing BAT 1S must not be connected to 12V:

```sh
.venv/bin/pio run -e top_board_v2 -t upload --upload-port PORT
.venv/bin/python scripts/robot_control.py --usb PORT status --report diagnostic-reports/control-status.json
.venv/bin/python scripts/robot_control.py --usb PORT stop
.venv/bin/python scripts/robot_control.py --usb PORT reboot --report diagnostic-reports/reboot.json
```

`reboot` stops first. The host sends the reboot command only once and records `reboot_ack_received`; it still tries to reconnect if the acknowledgment is lost. It prints `APP_RETURN_VERIFIED` only after checking the new boot ID, the same board identity, a software reset reason, idle and zero PWM. If the device explicitly rejects the command, it reports failure directly and does not treat it as a lost acknowledgment. Wireless reconnects must still pass the HMAC check.

The firmware waits until the app send queue for that BLE connection drains, then waits about 100ms more; under sustained congestion it waits at most 2500ms. A drained queue does not prove the phone/computer received the notification; the final evidence is still the host's reply and the new-boot verification. `reset` calls esptool's native USB ROM sync and hard reset; `bootloader` syncs the ROM, reads the MAC and stays in ROM, without writing or erasing Flash:

```sh
.venv/bin/python scripts/robot_control.py --usb PORT reset
.venv/bin/python scripts/robot_control.py --usb PORT bootloader
```

If what is actually running is the USB **diagnostic firmware**, use the other host protocol:

```sh
.venv/bin/python scripts/pcb_control.py reboot --port PORT --profile top_board_v2
.venv/bin/python scripts/pcb_control.py reset --port PORT --profile top_board_v2
```

These commands cannot guarantee recovery when native USB fails, the MCU has no power, EN is pulled low, and so on. Full unattended recovery also needs separate EN/BOOT control and power circuits; see the [hardware report](../hardware/top_board_v2/VERIFICATION.md).

**First wireless setup.** Create `wifi.local` locally with only `ssid` and `password`, as in the placeholder values below; replace them, then configure over USB:

```json
{"ssid":"YOUR_2_4_GHZ_SSID","password":"YOUR_WIFI_PASSWORD"}
```

```sh
.venv/bin/python scripts/robot_control.py --usb PORT provision --wifi-config wifi.local --key-file control-key.local
.venv/bin/python scripts/robot_control.py --usb PORT status
```

The script generates a 32-byte random device authentication key with file permissions 0600; an existing key is not overwritten. Credentials and key are written to the same NVS config record, and the app restarts. `.local` files are ignored by the repository; do not share the key or the Wi-Fi config. This is a local device key; no external search/cloud API key is needed. `wifi_ip` in USB status is the HOST below.

```sh
.venv/bin/python scripts/robot_control.py --wifi HOST --key-file control-key.local status
.venv/bin/python scripts/robot_control.py --wifi HOST --key-file control-key.local watch --seconds 30 --report diagnostic-reports/wifi.json
.venv/bin/python scripts/robot_control.py --scan-ble --seconds 5
.venv/bin/python scripts/robot_control.py --ble BLE_ID --key-file control-key.local status
.venv/bin/python scripts/robot_control.py --ble BLE_ID --key-file control-key.local reboot
```

Wi-Fi uses STA on UDP 4243 and needs a reachable LAN; SoftAP is not opened. BLE uses GATT: service `6e400001-b5a3-f393-e0a9-e50e24dcca9e`, RX/TX are `...0002...` / `...0003...`. The script splits data into 20-byte fragments to work with the default MTU; BLE_ID is the address from the scan results, which is a UUID on macOS. Scan results are not yet authenticated; the later signed handshake is what counts. [Bleak scanning API](https://bleak.readthedocs.io/en/latest/api/scanner.html)

When a BLE SDK notification send fails, the firmware records the error, clears the app send queue and drops that connection, so the client discards the incomplete message; the control lease is revoked on disconnect. A new connection does not get send or receive fragments left over from the previous connection. In the current Arduino-ESP32 3.3.9, `SUCCESS_NOTIFY` is also used for events such as the SDK accepting a send, so it cannot be taken as the app having received the data; errors are recorded from the `onStatus` callback. [Espressif implementation](https://github.com/espressif/arduino-esp32/blob/3.3.9/libraries/BLE/src/BLECharacteristic.cpp)

**Protocol and observability.** `hello` returns the boot ID and capabilities; wireless requests and responses are both `{payload,mac}` envelopes, and the MAC covers the exact payload string. Data has integrity authentication but no application-layer encryption. Configuration is only accepted over USB; wireless uses Wi-Fi's own security settings plus the device key.

Run-type commands need an exclusive claim, a lease and an increasing sequence number. With no valid control heartbeat for 750ms it stops; an out-of-order sequence or a disconnect revokes control; a reconnect must acquire control again, and old tasks do not resume. USB/BLE framing overflow triggers a stop immediately; Wi-Fi messages are split at packet boundaries. In this version all start/goto/wheel/arm/pulse commands are rejected; even after a future hardware review passes, the supervision and stop mechanisms must stay.

A message must be a complete object, with only spaces, tabs, CR and LF allowed after it; both the signed envelope and the inner payload are checked for full consumption. Key strings reject embedded NUL, so that `reboot\u0000ignored` cannot be truncated to reboot. Parsing allows at most 8 levels of nesting; this is a command format limit, not a general JSON compliance certification. The implementation uses ArduinoJson's public [custom Reader interface](https://arduinojson.org/v7/api/json/deserializejson/).

The control loop computes wheel speed from the actual elapsed time; if the interval exceeds 100ms it stops, revokes the lease and records a timing fault that stays visible until reboot. If a PWM write fails, it records a fault, tries to detach the PWM peripheral and pull both pins low, and stops the other two channels at the same time. Arduino-ESP32 3.x returns boolean results for write/detach operations; the old 2.x write gives no such feedback. [LEDC API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html) A normal stop or run command does not clear a PWM fault; explicit re-initialization or a reboot still requires checking the hardware again.

| Field | Meaning |
|---|---|
| boot / reset_reason / uptime_ms | Boot session and reset evidence |
| pwm_ready / motor_pwm_ready | Usable state after PWM init and runtime faults; not a pad voltage measurement |
| motor_pwm_fault | Fault bits per channel: 1=init failed, 2=write failed, 4=peripheral detach failed, can be added together; nonzero must be handled |
| encoder_counts / wheel_speed_mps | Direction-corrected signed 32-bit wrapping counts for the three channels and wheel speed computed from the config; encoder parameters need calibration |
| pwm_command / physical_motors / header_order | The main runtime firmware's arrays are in physical Motor1, Motor2, Motor3 order, i.e. header M0, M2, M1; the diagnostic firmware uses header order; the two differ on purpose |
| wifi_connected / wifi_rssi_dbm / ble_connected | Current connection observations, not wireless reliability acceptance |
| watchdog_ready / tx_dropped / reason | Watchdog init, dropped sends, latest stop/limit reason |
| ble_tx_failures | Running count of SDK notification error callbacks; not the actual packet loss rate |
| control_gap_ms / max_control_gap_ms | Latest control update interval, and the longest since this boot |
| control_overruns / control_timing_fault | Number of intervals over 100ms, and the fault flag that stays set |
| motor_voltage_mv / motor_current_ma / rail_3v3_mv | null in `status`. The KISS C6's motor current (INA180) and battery voltage are only reported through `rails`, and motor voltage has no sensor; other board types have no matching onboard sensors |
| power_disconnect | Currently false; a software stop does not mean a physical power cut |

All CLI commands return JSON, and failures return a nonzero exit code; AI should save the report, and must not automatically retry startup after an error. Wireless watch only collects status and does not hold a motion lease. BLE scanning needs the system to allow Bluetooth access; this round did not actually scan for or connect to a wireless board.

**Local verification.** On macOS, after installing dependencies and building the main firmware, run `.venv/bin/python control_tests/verify.py`. The local suite includes:

- 196 control math assertions, 12 control client tests, 13 diagnostic/recovery client tests.
- 29 fault scenarios on the real MotorDriver source: six pins, switching in both directions, stop failure, partial init and PWM detach failure.
- The real RobotController/path planning/odometry/driver running together, verifying six fault propagation paths and two stop behaviors on path replacement; interrupts and GPIO are simulated.
- Forward/reverse count wraparound, abnormal time values and 120,000 GPIO edges on the real encoder ISR / odometry; three-channel stall cooldown across timer wraparound is also tested, see [long-run boundary fixes](LONG_RUN_VERIFICATION.md).
- 20 scenarios on the real main command handler/loop, including 10,000 fixed-seed malformed inputs; plus a real time-interval regression. In this group, RobotController and the transport layer use stand-ins.
- Connect/send/receive/error callbacks of the real ControlTransport, covering queue capacity, fragmentation, old-connection isolation, UDP boundaries, Wi-Fi reconnect timing and notification failures. The queue and BLE/Wi-Fi SDK are test stand-ins.
- The real transport layer and main loop running six cases together: normal backlog, ACK enqueue failure, sustained congestion, send failure, clock wraparound, and control revocation after a send failure. Host reconnect after a lost acknowledgment is tested separately.
- Native lease/framing, the real parsers/loops of the two diagnostic sources, and six firmware cross-compile environments: main project esp32/c6/top_board_v2/motor_test, diagnostics adapter/top_board_v2.

Native C++ uses ASAN/UBSAN. The command handler's HMAC is adapted to macOS CommonCrypto; it does not simulate the C6's mbedTLS. The signed UDP loopback happens on this machine; although this project's callback source has run, SDK callback scheduling, FreeRTOS concurrency, low-level wireless buffers and RF coexistence are still not measured. The driver test deliberately keeps the case "simulated PWM is still nonzero when detach fails" to confirm the software reports failure; a GPIO-low call cannot be taken as a real power cut.

Evidence is written to `hardware/top_board_v2/validation/reverification/`; the report records hashes of the firmware, tests, host and the key SDK/JSON dependencies that were checked, and checks that they did not change during the tests. Real USB ROM reset, power, electrical waveforms and RF acceptance still have to be done on a real board.

# KISS C6 current board (top_board_v2_logic, onboard IP2326 charging)

The current board uses its own environment, `top_board_v2_kiss_c6`. USB is the C6's native USB Serial/JTAG (303A:1001, no USB-UART bridge; the CH343G was removed on 2026-10-29); pass `--usb PORT --usb-kind c6usb`; for Wi-Fi/BLE pass `--profile top_board_v2_kiss_c6_review`. When the "First wireless setup" commands above are used with the KISS board, add these arguments, otherwise `hello` rejects with a board type mismatch:

```sh
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind c6usb provision --wifi-config wifi.local --key-file control-key.local
.venv/bin/python scripts/robot_control.py --wifi HOST --key-file control-key.local --profile top_board_v2_kiss_c6_review status
.venv/bin/python scripts/robot_control.py --ble BLE_ID --key-file control-key.local --profile top_board_v2_kiss_c6_review status
```

Pins follow `include/board_profile_kiss_c6.h` and the EasyEDA netlist `hardware/top_board_v2/easyeda/logic_netlist.json`; the ARM request GPIO0 stays LOW, and motion is still disabled. Battery voltage, motor current (INA180, IO5), `usb_power` and `charge` are only reported in `rails`. The first flash needs the BOOT pads shorted; after a manual download through the pads, power-cycle with SW1 to get back to the app. Raw counts are usable; speed/position await calibration; the old R3/Pico pins cannot be reused directly. For flashing steps, commands and verification, see the [firmware notes](../hardware/top_board_v2/jlc_budget_6v/controller_core/logic/FIRMWARE.md). Everything above this section describes the historical board types.
