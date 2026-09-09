#!/usr/bin/env python3
"""AI-callable native USB recovery. JSON output; never writes/erases flash.

reboot: cooperative diagnostic-firmware restart, checked against a new session.
reset: Espressif native USB reset, including recovery from a responsive USB ROM.
bootloader: enter ROM and read the C6 MAC, leave ROM ready for a separate upload.
"""
import argparse
from datetime import datetime, timezone
import io
import json
from pathlib import Path
import subprocess
import sys
import time

from pcb_diagnostics import Client, DiagnosticError, FrameDecoder, validate_status

NATIVE_USB = (0x303A, 0x1001)
CP2102N_USB = (0x10C4, 0xEA60)  # Default bridge identity; not proof of an R3 PCB.
CH343P_USB = (0x1A86, 0x55D3)  # WCH's CH343 family ID, not proof of a Pico PCB.
CH343G_USB = CH343P_USB  # Shared family identity; downstream MCU is selected separately.
UART_USB_KINDS = ("cp2102n", "ch343p", "ch343g")


def usb_identity(kind):
    if kind == "native":
        return NATIVE_USB
    if kind == "cp2102n":
        return CP2102N_USB
    if kind in ("ch343p", "ch343g"):
        return CH343P_USB
    raise DiagnosticError("Unknown USB kind; expected native, cp2102n, ch343p or ch343g")


def usb_chip(kind):
    usb_identity(kind)
    return "esp32s3" if kind == "ch343p" else "esp32c6"


def rom_config_text(kind):
    usb_identity(kind)
    # esptool 5.3 Classic/UnixTight: 100ms EN hold then this BOOT hold.
    # Keep its OS-specific line-control implementation and suppress user overrides.
    return "[esptool]\nreset_delay = 0.05\n" if kind == "ch343g" else "[esptool]\n"


def identity(port):
    return {"port": port.device, "vid": port.vid, "pid": port.pid,
            "serial": port.serial_number, "location": port.location}


def select_port(ports, requested=None, kind="native"):
    expected = usb_identity(kind)
    if kind in UART_USB_KINDS and (not isinstance(requested, str) or not requested.strip() or "://" in requested):
        raise DiagnosticError(f"{kind.upper()} requires an explicit local --usb port path")
    ports = list(ports)
    found = [p for p in ports if (p.vid, p.pid) == expected
             and (requested is None or p.device == requested)]
    if len(found) != 1:
        if kind in UART_USB_KINDS:
            raise DiagnosticError(f"Select exactly one {kind.upper()} bridge ({expected[0]:04X}:{expected[1]:04X}) at the explicit --usb path")
        raise DiagnosticError("Select exactly one Espressif native USB device (303A:1001) with --port")
    if kind in UART_USB_KINDS:
        # Require an anchor before opening/resetting, including duplicate-serial rejection.
        # Reconnection may rename the port, but must retain serial or location identity.
        match_port(ports, identity(found[0]))
    return found[0]


def match_port(ports, target):
    found = [p for p in ports if (p.vid, p.pid) == (target["vid"], target["pid"])]
    if target["serial"]:
        found = [p for p in found if p.serial_number == target["serial"]]
    elif target["location"]:
        found = [p for p in found if p.location == target["location"]]
    else:
        # A path alone can be reassigned to a different USB device after reset.
        raise DiagnosticError("Device has no serial or location; cannot verify reconnect identity")
    if len(found) > 1:
        raise DiagnosticError("Ambiguous device identity after reset")
    return found[0].device if found else None


def open_serial(path):
    import serial
    port = serial.Serial(port=None, baudrate=115200, timeout=0.05, write_timeout=0.2,
                         rtscts=False, dsrdtr=False, xonxoff=False)
    port.dtr = False
    port.rts = False
    port.port = path
    port.open()
    return port


def assert_idle(frame, profile):
    validate_status(frame)
    if frame["profile"] != profile:
        raise DiagnosticError("Firmware board profile does not match --profile")
    if frame["armed"] or frame["running"] or any(m["command"] for m in frame["motors"]):
        raise DiagnosticError("Motor outputs are not reported idle")


def verify_restart(before, after, profile, software=False):
    assert_idle(after, profile)
    if before is not None and before["session"] == after["session"]:
        raise DiagnosticError("Old MCU session received; restart was not verified")
    if software and after.get("reset_reason") != 3:  # ESP_RST_SW in esp_system.h
        raise DiagnosticError("Restart reason is not ESP_RST_SW")
    if after["uptime_ms"] > 30000:
        raise DiagnosticError("Post-reset uptime is too old")


def wait_for_app(target, before, profile, software, timeout, ports_fn, opener=open_serial,
                 clock=time.monotonic, sleep=time.sleep):
    deadline = clock() + timeout
    last_error = "No matching USB device"
    while clock() < deadline:
        path = match_port(ports_fn(), target)
        if not path:
            sleep(0.1)
            continue
        try:
            with opener(path) as port:
                decoder = FrameDecoder()
                # The new application emits status every 100ms. Do not send an
                # unversioned command while the port might still be the ROM.
                while clock() < deadline:
                    for frame in decoder.feed(port.read(min(port.in_waiting or 1, 4096))):
                        if frame.get("type") != "status":
                            continue
                        try:
                            verify_restart(before, frame, profile, software)
                        except DiagnosticError as exc:
                            last_error = str(exc)
                            continue
                        return frame
        except (OSError, DiagnosticError) as exc:
            last_error = str(exc)
        sleep(0.1)
    raise DiagnosticError(f"Restart sent but application return NOT verified: {last_error}")


def rom_command(path, stay, kind="native"):
    usb_identity(kind)
    if kind in UART_USB_KINDS and (not isinstance(path, str) or not path.strip() or "://" in path):
        raise DiagnosticError(f"{kind.upper()} ROM recovery requires an explicit local port path")
    # esptool 5.3: default-reset selects UART DTR/RTS; usb-reset is native JTAG only.
    before = "default-reset" if kind in UART_USB_KINDS else "usb-reset"
    uart_options = ["--baud", "115200", "--connect-attempts", "3"] if kind in UART_USB_KINDS else []
    return [sys.executable, "-m", "esptool", "--chip", usb_chip(kind), "--port", path,
            *uart_options, "--before", before, "--after", "no-reset" if stay else "hard-reset",
            "--no-stub", "read-mac"]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["list", "status", "stop", "reboot", "reset", "bootloader"])
    parser.add_argument("--port")
    parser.add_argument("--profile", choices=["adapter", "top_board_v2"], default="top_board_v2")
    parser.add_argument("--timeout", type=float, default=15)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args(argv)
    if not 2 <= args.timeout <= 30:
        parser.error("--timeout must be 2..30 seconds")
    from serial.tools import list_ports
    report = {"action": args.action, "time": datetime.now(timezone.utc).isoformat(),
              "result": "FAIL", "physical_power_cycle": False, "flash_modified": False}
    try:
        ports = list(list_ports.comports())
        if args.action == "list":
            report.update(result="LISTED", devices=[identity(p) for p in ports])
        else:
            selected = select_port(ports, args.port)
            target = identity(selected)
            report["device"] = target
            before = None
            if args.action in ("status", "stop", "reboot"):
                with open_serial(selected.device) as port:
                    client = Client(port, io.StringIO())
                    client.request("hello")
                    before = client.fresh_status()
                    if before["profile"] != args.profile:
                        raise DiagnosticError("Wrong firmware profile")
                    report["before"] = before
                    if args.action in ("stop", "reboot"):
                        client.request("stop")
                        assert_idle(client.fresh_status(), args.profile)
                    if args.action == "reboot":
                        if before.get("usb_reboot") is not True:
                            raise DiagnosticError("Firmware does not advertise USB reboot support")
                        # The ACK means accepted, not successfully rebooted.
                        client.request("reboot")
                    else:
                        report.update(result="STATUS_READ" if args.action == "status" else "STOP_REPORTED",
                                      after=client.latest)
            else:
                # ROM is independent of the diagnostic command parser. This
                # path can recover a wedged app while native USB still exists.
                completed = subprocess.run(rom_command(selected.device, args.action == "bootloader"),
                                           capture_output=True, text=True, timeout=args.timeout, check=False)
                report.update(esptool_stdout=completed.stdout, esptool_stderr=completed.stderr,
                              esptool_exit_code=completed.returncode)
                if completed.returncode != 0:
                    raise DiagnosticError("Native USB ROM synchronization failed")
                if args.action == "bootloader":
                    report.update(result="C6_ROM_VERIFIED", application_running=False)
            if args.action in ("reboot", "reset"):
                report["after"] = wait_for_app(target, before, args.profile, args.action == "reboot",
                                               args.timeout, list_ports.comports)
                report["result"] = "REBOOT_VERIFIED" if before else "APP_RETURN_VERIFIED"
    except Exception as exc:
        report["error"] = str(exc)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 1 if report["result"] == "FAIL" else 0


if __name__ == "__main__":
    raise SystemExit(main())
