#!/usr/bin/env python3
"""Focused R2/R3 compile and native checks; never opens a device or uploads."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "hardware/top_board_v2/firmware_r3"
REPORT.mkdir(parents=True, exist_ok=True)
parser = argparse.ArgumentParser(description=__doc__)
mode = parser.add_mutually_exclusive_group()
mode.add_argument("--builds-only", action="store_true")
mode.add_argument("--skip-builds", action="store_true")
args = parser.parse_args()
rows = []

# Compilation may read an old local R2 config. Never persist its credentials in logs.
private = ROOT / "include/config.h"
secrets = re.findall(r'^\s*#define\s+WIFI_(?:SSID|PASS)\s+"([^"\n]+)"',
                     private.read_text() if private.exists() else "", re.M)
def redact(text):
    for value in secrets:
        text = text.replace(value, "<redacted>").replace(json.dumps(value)[1:-1], "<redacted>")
    return text

def hashes():
    paths = set()
    for folder in ("src", "control_tests", "usb_diagnostics/src", "usb_diagnostics/include", "usb_diagnostics/tests"):
        paths.update(p for p in (ROOT / folder).rglob("*") if p.suffix in (".cpp", ".h", ".py"))
    paths.update(p for p in (ROOT / "include").glob("*.h") if p.name != "config.h")
    paths.update(ROOT / p for p in ("platformio.ini", "c6_selftest/src/main.cpp", "c6_selftest/platformio.ini",
        "usb_diagnostics/platformio.ini", "scripts/robot_control.py", "scripts/pcb_diagnostics.py", "scripts/pcb_control.py",
        "hardware/top_board_v2/kicad/r3_system/core_contract.json", "hardware/top_board_v2/kicad/r3_system/controller.kicad_sch",
        ".platformio/packages/framework-arduinoespressif32/package.json",
        ".platformio/packages/framework-arduinoespressif32/cores/esp32/HardwareSerial.h",
        ".platformio/packages/framework-arduinoespressif32/cores/esp32/HardwareSerial.cpp",
        ".platformio/packages/framework-arduinoespressif32/cores/esp32/HWCDC.h"))
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}

before = hashes()
def run(name, command, cwd=ROOT, reject=None):
    env = dict(os.environ, PLATFORMIO_CORE_DIR=str(ROOT / ".platformio"))
    try:
        result = subprocess.run([str(x) for x in command], cwd=cwd, env=env,
                                capture_output=True, text=True, timeout=900)
        code, output = result.returncode, result.stdout + result.stderr
    except (OSError, subprocess.TimeoutExpired) as error:
        code, output = 124, str(error)
    output = redact(output)
    (REPORT / (name + ".log")).write_text(output)
    passed = code == 0 if reject is None else code != 0 and reject in output
    rows.append(dict(name=name, command=[str(x) for x in command], cwd=str(cwd),
                     exit_code=code, expected_rejection=reject, passed=passed))
    print(name, "PASS" if passed else "FAIL", flush=True)
    if not passed:
        print(output[-2400:], flush=True)
    return output if passed else None

def contract_check(pin_output, main_output, diag_output):
    contract = json.loads((ROOT / "hardware/top_board_v2/kicad/r3_system/core_contract.json").read_text())
    symbols = (ROOT / "hardware/top_board_v2/kicad/r3_system/controller.kicad_sch").read_text()
    unit = symbols.split('(symbol "R3_Core:R3_U1"', 1)[1].split('(symbol "R3_Core:', 1)[0]
    pads = re.findall(r'\(name "(IO\d+|TXD0|RXD0)".*?\(number "(\d+)"', unit, re.S)
    gpios = {pad: int(name[2:]) if name.startswith("IO") else {"TXD0": 16, "RXD0": 17}[name]
             for name, pad in pads}
    mcu = contract["components"]["U1"]["pins"]
    nets = {mcu[pad]: gpio for pad, gpio in gpios.items() if mcu[pad] is not None}
    assert set(contract["components"]["R13"]["pins"].values()) == {"ENC0_A", "ENC0_A_MCU"}
    nets["ENC0_A"] = nets["ENC0_A_MCU"]
    def header(ref):
        pins = contract["components"][ref]["pins"]
        return [nets[pins[p]] for p in ("3", "2", "5", "4")]
    observed = json.loads(pin_output)
    assert observed["physical_order"] == [header(h) for h in ("M0", "M2", "M1")]
    assert observed["diagnostic_order"] == [header(h) for h in ("M0", "M1", "M2")]
    assert nets["ESP_RXD_GPIO17"] == 17 and nets["ESP_TXD_GPIO16"] == 16
    assert mcu["17"] == "WD_HEARTBEAT_OD" and mcu["18"] is None and gpios["17"] == 12 and gpios["18"] == 13
    assert observed["watchdog_heartbeat_gpio"] == nets["WD_HEARTBEAT_OD"] == 12
    assert nets["I2C_SDA"] == 14 and nets["I2C_SCL"] == 22 and nets["MOTOR_ARM_REQUEST"] == 21
    assert contract["components"]["J2"]["pins"]["3"] == "ESP_RXD_GPIO17"
    assert set(contract["components"]["R50"]["pins"].values()) == {"MOTOR_ARM_REQUEST", "GND"}
    assert contract["components"]["R50"]["value"].startswith("10k")
    sys.path.insert(0, str(ROOT / "scripts"))
    import robot_control
    import pcb_diagnostics
    full = json.loads(next(line.removeprefix("R3_STATUS ") for line in main_output.splitlines() if line.startswith("R3_STATUS ")))
    diagnostic = json.loads(next(line.removeprefix("R3_DIAG_STATUS ") for line in diag_output.splitlines() if line.startswith("R3_DIAG_STATUS ")))
    robot_control.check_status(full)
    for i, motor in enumerate(diagnostic["motors"]):
        assert [motor[k] for k in ("fi", "bi", "enc_a", "enc_b")] == observed["diagnostic_order"][i]
    # The independent read-only diagnostic must never masquerade as either old protocol.
    for check in (robot_control.check_status, pcb_diagnostics.validate_status):
        try:
            check(diagnostic)
        except pcb_diagnostics.DiagnosticError:
            pass
        else:
            raise AssertionError("Legacy host accepted the independent R3 diagnostic")
    rows.append(dict(name="compiled_consumers_match_kicad_and_host_identity", passed=True))
    print(rows[-1]["name"], "PASS", flush=True)

if not args.builds_only:
    with tempfile.TemporaryDirectory(prefix="wbb-r3-checks-") as temp:
        temp = Path(temp)
        cc = ["c++", "-std=c++17", "-fsanitize=address,undefined", "-fno-sanitize-recover=all"]
        inc = ["-Icontrol_tests/mock", "-Iinclude", "-I.pio/libdeps/top_board_v2/ArduinoJson/src"]
        r3 = ["-DWBB_TOP_BOARD_R3=1", "-DCONFIG_IDF_TARGET_ESP32C6=1", "-DARDUINO_USB_CDC_ON_BOOT=0"]
        outputs = {}
        cases = [
            ("r2_commands", "control_tests/test_runtime_commands.cpp", inc),
            ("r3_commands", "control_tests/test_runtime_commands.cpp", inc + r3),
            ("r3_pins", "control_tests/test_r3_pins.cpp", inc + r3 + ["-DWBB_TOP_BOARD_V2=1"]),
            ("r3_diagnostic", "control_tests/test_r3_diagnostic.cpp", inc + r3 + ["-Iusb_diagnostics/tests/mock"]),
            ("session_guards", "control_tests/test_session.cpp", ["-Iinclude"]),
            ("r3_motor_faults", "control_tests/test_motor_driver.cpp", inc + r3 + ["-DWBB_TOP_BOARD_V2=1", "-include", "include/config.example.h"]),
            ("r2_runtime_timing", "control_tests/test_runtime_timing.cpp", inc),
            ("r2_transport", "control_tests/test_transport.cpp", inc),
            ("r2_reboot_transport", "control_tests/test_transport_reboot.cpp", inc),
            ("r2_diagnostic_safety", "usb_diagnostics/tests/test_safety.cpp", ["-Iusb_diagnostics/include"]),
            ("r2_diagnostic_adapter", "usb_diagnostics/tests/test_firmware.cpp", ["-Iusb_diagnostics/tests/mock", "-Iusb_diagnostics/include"]),
            ("r2_diagnostic_board", "usb_diagnostics/tests/test_firmware_top_v2.cpp", ["-Iusb_diagnostics/tests/mock", "-Iusb_diagnostics/include"]),
        ]
        for name, source, flags in cases:
            binary = temp / name
            if run(name + "_compile", cc + flags + [source, "-o", binary]) is not None:
                outputs[name] = run(name, [binary])
        if all(outputs.get(k) is not None for k in ("r3_pins", "r3_commands", "r3_diagnostic")):
            try:
                contract_check(outputs["r3_pins"], outputs["r3_commands"], outputs["r3_diagnostic"])
            except Exception as error:
                rows.append(dict(name="compiled_consumers_match_kicad_and_host_identity", passed=False, error=str(error)))
                print(rows[-1], flush=True)
        # Verify the actual forced-include strategy against a stale private-config fixture.
        (temp / "sdkconfig.h").write_text("#pragma once\n")
        (temp / "config.h").write_text("#ifndef CONFIG_H\n#define CONFIG_H\n#define ENC0_B_PIN 17\n#define STALE_CONFIG_READ 1\n#endif\n")
        source = temp / "profile.cpp"
        source.write_text('#include "config.h"\n#ifdef STALE_CONFIG_READ\n#error "stale config loaded"\n#endif\nstatic_assert(ENC0_B_PIN==6);\nint main(){}\n')
        flags = ["c++", "-std=c++17", "-Iinclude", "-I" + str(temp), "-include", "include/r3_build_config.h"]
        for name, defines, rejection in (
            ("public_profile_precedes_private_config", r3 + ["-DWBB_TOP_BOARD_V2=1"], None),
            ("reject_native_usb_r3", r3[:-1] + ["-DARDUINO_USB_CDC_ON_BOOT=1", "-DWBB_TOP_BOARD_V2=1"], "requires UART0 Serial"),
            ("reject_wrong_mcu_r3", [r3[0], r3[2], "-DWBB_TOP_BOARD_V2=1"], "R3 requires ESP32-C6"),
            ("reject_ungated_r3_entry", r3, "must use the gated integrated runtime"),
            ("reject_r3_motor_test", r3 + ["-DWBB_TOP_BOARD_V2=1", "-DMOTOR_TEST_MODE"], "motor test entry point is forbidden"),
        ):
            run(name, flags + defines + [source, "-o", temp / name], reject=rejection)
        for name, folder in (("control_host", "control_tests"), ("diagnostic_host", "usb_diagnostics/tests")):
            run(name, [sys.executable, "-m", "unittest", "discover", "-s", folder, "-p", "test_*.py", "-v"])

if not args.skip_builds:
    pio = ROOT / ".venv/bin/pio"
    for environment in ("top_board_v2_r3", "top_board_v2_r3_diag", "top_board_v2", "c6", "esp32", "motor_test"):
        run("build_" + environment, [pio, "run", "-e", environment])
    for environment in ("adapter", "top_board_v2"):
        run("build_usb_diagnostic_" + environment, [pio, "run", "-e", environment], ROOT / "usb_diagnostics")
    run("build_legacy_c6_selftest", [pio, "run", "-e", "c6selftest"], ROOT / "c6_selftest")

after = hashes()
unchanged = before == after
result = dict(time_utc=datetime.now(timezone.utc).isoformat(), checks=rows,
              all_passed=unchanged and all(r["passed"] for r in rows), sources_unchanged_during_checks=unchanged,
              source_sha256=after, firmware_flashed=False, usb_hardware_tested=False, radio_tested=False,
              motor_executed=False, charging_enabled=False,
              scope="Compilation and ASAN/UBSAN native code; GPIO, UART, clocks, reset, RTOS and radio SDK are test doubles.")
filename = "build_verification.json" if args.builds_only else "host_verification.json" if args.skip_builds else "software_verification.json"
(REPORT / filename).write_text(json.dumps(result, indent=2) + "\n")
raise SystemExit(0 if result["all_passed"] else 1)
