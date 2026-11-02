# Wireless power policy of the top board

The C6 review firmware ties wireless startup to the battery interface voltage, while keeping USB diagnostics, download and auto recovery. Since 2026-10-29 it applies to the current EasyEDA board `top_board_v2_logic`: native USB (no USB-UART bridge), on-board IP2326 charging. The voltage is a diagnostic reading from the IO4 divider; whole-board power and real wireless performance are still not qualified.

## How it runs

| Condition | Firmware behavior |
| --- | --- |
| After power-up/reset | Native USB CDC and voltage sampling work first; Bluetooth and Wi-Fi are not initialized. USB `status`, `rails` and reboot are available |
| Valid battery readings meet the condition continuously | Wireless starts only when readings are within 6.5–8.8V, the continuous sampling span is at least 1 second, adjacent sample start times are at most 150ms apart, the watchdog is available and no reboot is pending |
| Starting wireless | First revokes any existing control lease; BLE is initialized only once per boot cycle, and Wi-Fi also starts when there is a valid key and SSID; clients acquire the control lease again. Sampling is re-checked right after init returns, so expired samples are not mistaken for power loss. Known issue: the normal path does not refresh the control period timer after init, so the block of about 300ms is recorded as one `CONTROL_OVERRUN`, and `control_timing_fault` stays until reboot (see below) |
| Wireless started | It keeps running while valid readings are within 6.0–8.8V; the start and keep thresholds are separate, to avoid switching back and forth at the boundary |
| Battery reading out of range while running, ADC failure/expiry | Before handling the next batch of communication data, it revokes control, requests safe outputs and performs a software reset; the next boot judges the power conditions again |
| GPIO safe output failure (`gpio_fault` latched) | Does not reset because of the fault itself; stays read-only: sampling continues, wireless starts under the same battery condition, and it resets as usual on power failure. Wi-Fi/BLE only answer `hello`, `status`, `rail_status`, `stop`; `reboot` is USB only. The fault loop re-samples right after `transport.begin()` returns; a GPIO fault found during `RADIO_STARTING` also does not trigger a power reset |
| Wireless init failure | Reports `init_failed`, and does not re-init repeatedly in the same boot cycle; USB diagnostics and the user/AI reboot entry are kept. Part of the SDK init may have finished, so power is still monitored |
| App reboot waiting to send its reply | Voltage sampling continues; with normal power, the original reply-wait flow is kept; on power failure, auto reset takes priority, and the reply may not be fully sent |

A software reset re-runs the startup program and resets specified peripherals such as Wi-Fi and BT; it does not cut PCB power. USB is the C6's built-in USB Serial/JTAG with no separate bridge, so how the host serial port disconnects and re-enumerates on `esp_restart()` and on SW1 power-off depends on the MCU itself, and must be measured on a real board. [Espressif `esp_restart()` notes](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32c6/api-reference/system/misc_system_api.html#_CPPv411esp_restartv).

The BLE library installed in this project waits for host sync during init, with an extra 200ms delay. Here a single init plus software reset is used to switch modes; `BLEDevice::deinit` is not called repeatedly to rebuild the async objects. The task watchdog is registered before wireless init; sampling is re-checked right after init returns. There is no continuous voltage sampling while the SDK blocks, so this cannot guarantee the max shutoff delay after the battery is pulled or the USB transient current.

**Known issue (2026-10-29, firmware fix pending):** after wireless starts on the normal path, `lastControlAt` is not refreshed, so the control period check in the same iteration sees a gap of about 300ms: `control_overruns` goes up by 1, `control_timing_fault=true` stays until reboot, and `reason` is overwritten with `CONTROL_OVERRUN` (on init failure it also overwrites `TRANSPORT_INIT_FAILED`). Every `status` after the battery qualifies and wireless starts looks like this; the [status.json sample](firmware_verification/status.json) is this result (`control_gap_ms` 310). Motion is disabled anyway and outputs are not affected; the init result is given by `wireless_power_state` in `rails`. The fix is to refresh `lastControlAt` after `transport.begin()` returns. The fault loop does not run the control period and is not affected. [Espressif BLE implementation](https://github.com/espressif/arduino-esp32/blob/master/libraries/BLE/src/BLEDevice.cpp). The locally compiled version and hash are in the verification record below.

## Diagnostics readable over USB

```sh
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind c6usb rails
```

The original voltage report adds `wireless_power_state`, `wireless_start_attempted` and `previous_radio_power_reset`. The first two tell apart waiting for voltage, confirming, starting, running and init failed; "start attempted" does not mean the wireless connection succeeded. The last one uses an RTC no-init marker and its complement; it is accepted only when the software reset reason matches, and is consumed and cleared on this boot, so that AI can identify the previous automatic power reset. It does not write Flash, is not guaranteed to survive a full power loss, and is not a detailed fault waveform record.

## Verification and open items

The 2026-09-09 production main loop tests cover USB-only power, missing ADC calibration, an unavailable watchdog, re-timing after voltage fluctuation, power loss/ADC fault/expiry/GPIO fault while running, power loss during init, four SDK init failures, and acceptance and rejection of the reset marker; GPIO faults were handled as resets at that time and have now changed to the read-only behavior in the table above. Pure policy tests also cover time wraparound and reuse of old samples. The original USB/Wi-Fi/BLE reboot, authentication, fragmentation and shared R3 regressions are kept; for the sources and scope of that time, see the [record](verification/wireless_power.json). All fault inputs and resets are test stand-ins; real RTC retention and wireless link performance have not been proven. For the current verification method, see the [firmware notes](FIRMWARE.md#verification).

6.5V/6.0V/8.8V are **software reading thresholds** for the current 2S battery interface; the external divider is not calibrated per board, and these are not cell protection thresholds, a USB current budget or a 6V regulation guarantee. A single ADC stuck at a plausible value may still go undetected; input protection, ADC power-off behavior, USB enumeration and suspend, plug/unplug and inrush, RF/power transients, full-board routing and a full cost count are still to be done. Motion stays disabled, and this cannot be called a passed whole-board stability verification.
