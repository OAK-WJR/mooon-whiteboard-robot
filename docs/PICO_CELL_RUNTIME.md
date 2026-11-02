# Pico two-cell monitoring and computer-side diagnostics

2026-09-08: the BQ76905 is connected to the real main loop and shares the project's I²C worker task with the INA3221.
GPIO39 does one open-drain wake, then the firmware checks the chip identity and config read-back and polls the two cells.
This is review firmware verified offline; it has not been flashed or connected to a device, and no real readings have been taken.
**Motion, charging and power cycling are still forbidden, and battery-specific protection thresholds are not yet approved.**

## Startup and bus scheduling

After GPIO/UART/wireless/task watchdog init succeeds and the bus is ready, GPIO39 is pulled low for about 100ms,
then released, followed by a 100ms wait, and then the BQ check starts at the fixed address 0x08 for the default non-CRC model.
If the pulse release is delayed to 150ms, a timeout is reported; a GPIO API failure stops the diagnostic service.
These times are engineering choices; the actual waveforms of the isolator, PhotoMOS, regulator and TS must be measured.
`wake_requested` only means a software request; it is not TS voltage feedback.

The wake and configuration flow is based on the kept [TI technical reference manual](https://www.ti.com/lit/ug/sluuci8/sluuci8.pdf)
and the [BQ76905 datasheet](https://www.ti.com/lit/ds/symlink/bq76905.pdf);
the engineering wait values are not a vendor-guaranteed startup limit for the whole isolation circuit. There is no automatic wake after an error,
no switching of CRC mode, and no repeat of uncertain config writes. NAKs on the initial read-only Battery Status transaction
may be retried within a 2-second window; the backend cannot tell whether the NAK came at the address byte or the register byte.

Each main loop iteration advances at most one I²C submit or poll. An INA acquisition that has already started finishes first;
the next INA sample group starts only when the BQ physical transaction is done and the buffer has been returned after a cancel.
BQ configuration and long messages advance between these acquisition groups, so INA readings still get a chance during initialization.
The INA's 4ms transfer/5ms operation budget was not enlarged. A fake-device test with 7 deferred polls checked the interleaving,
but cannot prove scheduling headroom or hard response times under real wireless load.

## USB, Wi-Fi and BLE diagnostics

Use the updated `scripts/robot_control.py`. After connecting and checking the real board, a USB query example is:

```sh
python3 scripts/robot_control.py --usb <PORT> --usb-kind ch343p cells
```

`cells` sends a read-only `cell_status` and does not request motion control; wireless still needs the correct profile and HMAC.
Normal `status` only adds a `cell_monitor`/`cell_error` summary; the full report is requested separately, which keeps the existing
1792-byte JSON payload and 2304-byte line buffer limits.

| Report field | Meaning and limits |
| --- | --- |
| `cell_mv` | Lower/upper cell, matching BQ physical inputs 1 and 5; not inputs 1 and 2 |
| `stack_mv`, `cell_pair_valid`, `stack_consistent` | Total voltage and validity/250mV consistency screening; not a guarantee of measurement accuracy |
| `seq`, `generation` | Sample sequence number and config/invalidation generation; cannot stand in for device session identity across reboots |
| `age_ms`, `read_span_ms` | Age since the publish point when the message is built, and the read span, which is under 1500ms; not synchronized ADC timestamps |
| `ts_counts`, `ts_nominal_uv` | TS raw value and nominal conversion; **not battery temperature** |
| `battery_current_ma`, `pack_temperature_mc` | The current hardware has no such measurement; always `null` |
| `configuration_verified` | The specified read-only monitoring config has been written and read back |
| `voltage_profile_verified`, `system_protection_qualified` | Both currently `false`; a completed config cannot be used to approve charging or discharging |
| `fets_off_observed` | BQ turn-off control observed in fresh status; not physical proof that the whole robot is unpowered |

When the report is built, the sample age must be under 3000ms; transmission and host queueing time are not included.
After the whole group expires, a config change, POR, a disconnect or a reboot, the readings and related metadata are revoked and output as `null`.
A fresh status can still contain an invalid single-cell measurement; when the lower cell is below 2V, the upper cell reading is not accepted.
The host strictly checks type, range, status, config generation, device identity and HMAC, but passing parsing does not mean
the message was still fresh when it arrived. The current diagnostics are not used to approve motion, charging or restoring power.

## Verification and remaining work

The [verification record for this round](PICO_CELL_RUNTIME_VERIFICATION.json) saves the actual commands, public source digests and scope.
14 new main loop scenarios, 74 Pico host tests, 27 GPIO scenarios, 18 BQ driver test groups,
the shared transport layer cancel race check, and the Pico/R3 public config builds pass. The build check confirms the actual application keeps
the BQ, INA and this project's backend symbols; forced linking was not used in place of proof of integration.

The GNB8502S80A's specific charging temperature range, normal low-voltage shutdown and protection coordination still await reliable specs and measurement.
The current monitoring config has no protection thresholds enabled, and CHG/DSG and external requests stay off.
If configuration fails partway, the BQ may stay in CONFIG_UPDATE; an MCU reboot alone or another wake cannot guarantee
recovery. A verified BQ reset/recovery transaction, charge/discharge permission, power dynamics, the real wire order and the final PCB
are still to be done. The existing record of the MOS transient not converging still fails, and has not been replaced by software tests.

The earlier `PICO_I2C_BACKEND_VERIFICATION.json`, `PICO_RAIL_RUNTIME_VERIFICATION.json`
and hardware GPIO checkpoints are kept as historical records; their source digests do not pose as the current file digests.
