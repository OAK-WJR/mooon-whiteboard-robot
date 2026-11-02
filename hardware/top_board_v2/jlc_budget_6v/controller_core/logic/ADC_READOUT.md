# Top board ADC readout: battery voltage and motor current

2026-10-29, the current EasyEDA board `top_board_v2_logic`. The review firmware reports two ADC1 readings through `rail_status`:

| Channel | Net | Circuit | Attenuation/range | Report field |
| --- | --- | --- | --- | --- |
| GPIO4/U3.9 | VBAT_SENSE | R33 120k/R34 10k, nominal ratio 13 | 0dB, 0–1000mV | `battery_voltage_mv` |
| GPIO5/U3.10 | MOT_ISENSE | INA180A1 (U7, gain 20) measures R49 0.1Ω, through R36 1k/C21, 2V/A | 12dB, 0–3300mV | `motor_current_ma` = `adc_pin_mv[1] // 2` |

Since 2026-10-28, IO5 is motor current and no longer a motor voltage divider; `motor_voltage_mv` is always null. Real-board accuracy and power-off behavior are not verified, and motion stays disabled.

The host entry point is below; replace `PORT` with the real serial port. No device was connected and this hardware command was not run this time:

```sh
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind c6usb rails
```

Wi-Fi/BLE use the same read-only command; you must explicitly select `--profile top_board_v2_kiss_c6_review` and pass the existing authentication; no motion control lease is needed. In normal `status`, motor voltage and current are still null; the detailed readings come back separately through `rails`, to keep the existing status message length.

## Conversion and sampling

Conversion uses ADC1, 12 bits and the per-channel chip calibration interface; VBAT uses 0dB, current uses 12dB. If calibration init fails, an error is reported; the nominal reference voltage is not used instead. The external dividers are not calibrated per board, so the report keeps `divider_calibrated=false`, `hardware_qualified=false`. [Espressif ADC sampling docs](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32c6/api-reference/peripherals/adc_oneshot.html), [calibration docs](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32c6/api-reference/peripherals/adc_calibration.html).

After init it waits at least 10ms; samples are at least 100ms apart; each channel takes 8 readings, and both channels are published together when done. Any of the following makes the whole pair unavailable: a read or conversion failure, out of range, VBAT saturation, or a sample span over 25ms. Readings are valid for 250ms after sampling starts. When unavailable, the voltage, current, raw value and sample time fields are all null, `motor_current_saturated` does not appear, and the fault stage and SDK error code are kept. The 25ms check happens after the SDK call returns; it cannot interrupt the low-level call and is not a hard real-time guarantee.

When the current channel hits the rail, the pair is not discarded. If any sample is ≥3150mV after calibration, or its raw value is 4095, `motor_current_saturated=true` is set, meaning the actual current is at least the reported value; samples with raw value 4095 or above 3300mV count as 3300mV. The INA180 output tops out at about 3V3 minus 0.03V; the gain error is only specified up to about 1.4A (VS−0.5V), and it hits the rail at about 1.6A.

## Power source

`usb_power` reads IO8 (high when USB/charger 5V is present; without USB, R10 pulls it through Q1 to below about 0.72V, see the [firmware notes](FIRMWARE.md#io8-usb-power-detection)). `charge` is `"external_power"` when `usb_power` is true, otherwise `"battery"`. The IP2326 status pins are not connected to the MCU; VBAT is uncalibrated, with an error of about ±150mV, while the CV voltage is about 8.3V. So the firmware does not infer charging or full; it only reports the battery voltage.

## Faults and reset

The control lease is checked both before and after sampling. After a GPIO fault latches, sampling continues, and the read-only status stays complete over both USB and wireless. Voltage monitoring also continues while an app restart waits to send its reply. Before wireless has started, an ADC error keeps USB diagnostics and user restart; after wireless has started, an ADC failure or expiry triggers an auto reset, and the next boot judges the power again; see [wireless power policy](WIRELESS_POWER.md).

Still to do: external divider calibration, ADC power-off handling, USB power and suspend, full-board routing and hardware verification. Software thresholds and the auto reset cannot prove USB power is qualified.

## History: 2026-09 KiCad version

In the six-page KiCad drawing in this folder, IO5 is a 120k/10k motor supply divider, both channels use 0dB, and the readings are two voltages. At that time the schematic only updated comments and the ADC implementation status; the values, footprints and connections of the 70 components did not change; for how the old inputs map to the commented version, see the [evidence link record](verification/adc_revision_evidence.json). The software report of that time (75 entries) and the test voltage samples (simulated inputs 7.8V/5.98V) have been overwritten by the current verification; see git history for the old versions. The current [rails sample](firmware_verification/rails.json) also comes from simulated inputs, not PCB measurements.
