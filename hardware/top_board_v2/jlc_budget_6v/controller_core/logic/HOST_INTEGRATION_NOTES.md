# C6 / CH343G host integration to-dos (2026-09, superseded)

> **Superseded (2026-10-29).** The CH343G has been removed from the board, and the host's `ch343g` branch was deleted with it (`--usb-kind ch343g` now errors out directly). The current KISS board uses the C6's native USB Serial/JTAG with `--usb-kind c6usb`; see the [firmware notes](FIRMWARE.md).

**This is the check record from before integration. The code locations and gap descriptions below are only a historical basis: the `ch343g` branch implemented from them at the time passed offline tests but was never checked on hardware, and was deleted on 2026-10-29.**

2026-09-08, read-only review of the code and manufacturer material. The current schematic's USB bridge is connected to the C6, but the new board's firmware identity and host commands have not been adapted; no serial port was opened, no ROM recovery was run, and nothing was flashed.

## Where the existing implementation does not match the new board

- `usb_chip("ch343p")` in [pcb_control.py](../../../../../scripts/pcb_control.py) returns `esp32s3`. This option belongs to the old Pico plan, and cannot be used for the new C6 just because the VID/PID are the same.
- `USB_PROFILES` in [robot_control.py](../../../../../scripts/robot_control.py) binds `ch343p` to `top_board_v2_pico_review`. It currently only accepts native USB, the old R3/CP2102N and the old Pico/CH343P; there is no identity for this C6/CH343G version yet.
- The old R3 firmware's GPIO assignment is not the same as [this version's pin table](c6_pin_plan.json). It needs separate firmware config, status identity and host checks; the checks must not be bypassed by disguising the new board as the old R3 or Pico.

## Verified external interface facts

WCH's official driver V2.2 lists `1A86:55D3` as the CH343 family, and further reads the chip type to tell G/P, J, K and so on apart. **This VID/PID cannot prove that the backend is a C6, nor that it is this project's PCB.** Later work must still explicitly select the local serial port, keep the device identity anchor, specify the C6 ROM, and check the new app identity. [WCH official driver](https://github.com/WCHSoftGroup/ch343ser_linux/blob/main/driver/ch343.c), `ch343_configure` and `ch343_ids`.

The project's [requirements-control.txt](../../../../../scripts/requirements-control.txt) pins esptool 5.3.0. In that version, `ClassicReset` first holds EN in reset for 100ms, then releases EN and keeps holding BOOT until `reset_delay`; the limited simulation of the current circuit uses 50ms after that. The host's boolean DTR/RTS are active-low control semantics; `True` cannot be taken directly as a high level in the schematic. [Espressif 5.3.0 reset.py](https://github.com/espressif/esptool/blob/v5.3.0/esptool/reset.py), [C6 auto download notes](https://docs.espressif.com/projects/esptool/en/latest/esp32c6/advanced-topics/boot-mode-selection.html).

esptool supports `ESPTOOL_CFGFILE` and custom reset sequences. The existing `robot_control.py` creates a temporary empty config for UART recovery, so that the user's global config cannot replace the policy. The new branch should keep this isolation and explicitly pin the reset/ROM-exit timing that matches this circuit version; changing only `--chip` and declaring compatibility is not enough. [Espressif config file notes](https://docs.espressif.com/projects/esptool/en/latest/esp32/esptool/configuration-file.html).

## Completion conditions

The later implementation must check all of these together: the new firmware pins and identity, the CH343G option, C6 ROM commands, reset timing, a new boot ID on the same device after restart, rejection of wrong board types, and zero output after reset. Offline tests can check command construction, simulated serial interaction and status validation; the OS driver's real DTR/RTS delays, enumeration, plugging/unplugging and recovery still need real-board verification. Keep motion disabled as it is now until the new power supply and whole-robot acceptance are done.
