#!/usr/bin/env python3
"""Only the R3 GPIO/UART fault regression and two R3 builds; no hardware access."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "hardware/top_board_v2/kicad/r3_watchdog/gpio_heartbeat_review/gpio_init_fix"
parser = argparse.ArgumentParser()
group = parser.add_mutually_exclusive_group()
group.add_argument("--tests-only", action="store_true")
group.add_argument("--builds-only", action="store_true")
args = parser.parse_args()
REPORT.mkdir(parents=True, exist_ok=True)
(REPORT / "bin").mkdir(exist_ok=True)
report_path = REPORT / "verification.json"
report = json.loads(report_path.read_text()) if report_path.exists() else {"phases": {}}
paths = ["src/top_v2_main.cpp", "c6_selftest/src/main.cpp", "include/r3_gpio_safe.h",
         "include/board_profile_r3.h", "include/r3_build_config.h", "platformio.ini",
         "include/config.example.h", "control_tests/mock/driver/gpio.h",
         "control_tests/test_r3_gpio_init.cpp", "control_tests/verify_r3_gpio_init.py"]

def hashes():
    return {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in paths}

def run(name, command, expected_failure=None):
    print(name + " START", flush=True)
    env = dict(os.environ, PLATFORMIO_CORE_DIR=str(ROOT / ".platformio"))
    try:
        result = subprocess.run([str(x) for x in command], cwd=ROOT, env=env,
                                capture_output=True, text=True, timeout=900)
        code, output = result.returncode, result.stdout + result.stderr
    except (OSError, subprocess.TimeoutExpired) as error:
        code, output = 124, str(error)
    (REPORT / (name + ".log")).write_text(output)
    passed = code == 0 if expected_failure is None else code != 0 and expected_failure in output
    row = {"name": name, "command": [str(x) for x in command], "exit_code": code,
           "expected_failure": expected_failure, "passed": passed, "log": name + ".log"}
    print(name + (" PASS" if passed else " FAIL"), flush=True)
    if not passed:
        print(output[-3000:], flush=True)
    return row

def save():
    report["time_utc"] = datetime.now(timezone.utc).isoformat()
    report["firmware_flashed"] = report["hardware_or_usb_access"] = False
    report["scope"] = "Two real R3 entry points with strict GPIO test doubles; only two R3 cross-builds."
    report["all_passed"] = len(report["phases"]) == 2 and all(
        p["sources_unchanged"] and all(c["passed"] for c in p["checks"])
        for p in report["phases"].values()) and (
            report["phases"]["tests"]["source_sha256_after"] ==
            report["phases"]["builds"]["source_sha256_before"])
    report_path.write_text(json.dumps(report, indent=2) + "\n")

if not args.builds_only:
    before = hashes()
    phase = report["phases"]["tests"] = {"checks": [], "source_sha256_before": before}
    cc = ["c++", "-std=c++17", "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
          "-Icontrol_tests/mock", "-Iinclude", "-Isrc",
          "-I.pio/libdeps/top_board_v2/ArduinoJson/src", "-Iusb_diagnostics/tests/mock"]
    test = ROOT / "control_tests/test_r3_gpio_init.cpp"
    for entry in ("runtime", "diagnostic"):
        defines = ["-DR3_GPIO_TEST_DIAG=1"] if entry == "diagnostic" else []
        for variant in ("baseline", "pre_uart_visibility", "fixed"):
            baseline = variant != "fixed"
            name = variant + "_" + entry
            binary = REPORT / "bin" / name
            flags = defines + ([f'-DR3_GPIO_ENTRYPOINT="{REPORT / variant / (entry + ".cpp")}"'] if baseline else [])
            row = run(name + "_compile", cc + flags + [test, "-o", binary])
            phase["checks"].append(row)
            if row["passed"]:
                scenarios = ["cold"] if variant == "baseline" else ["preload_failure"] if baseline else ["cold", "warm", "preload_failure", "config_failure", "write_failure"]
                if not baseline and entry == "diagnostic":
                    scenarios += [f"motor_{kind}_{pin}" for kind in ("preload", "config", "write") for pin in range(6)]
                if not baseline and entry == "runtime":
                    scenarios += ["provision_failure", "overrun_failure", "wireless_reboot_failure"]
                for scenario in scenarios:
                    rejection = ("safe-low write rejected before Arduino ownership" if variant == "baseline"
                                 else "fault recovery UART not initialized" if baseline else None)
                    phase["checks"].append(run(name + "_" + scenario, [binary, scenario], rejection))
    phase["source_sha256_after"] = hashes()
    phase["sources_unchanged"] = before == phase["source_sha256_after"]
    save()
    if not all(c["passed"] for c in phase["checks"]) or not phase["sources_unchanged"]:
        raise SystemExit(1)

if not args.tests_only:
    before = hashes()
    phase = report["phases"]["builds"] = {"checks": [], "source_sha256_before": before}
    for environment in ("top_board_v2_r3", "top_board_v2_r3_diag"):
        phase["checks"].append(run("build_" + environment,
            [ROOT / ".venv/bin/pio", "run", "-e", environment]))
    phase["source_sha256_after"] = hashes()
    phase["sources_unchanged"] = before == phase["source_sha256_after"]
    save()
    if not all(c["passed"] for c in phase["checks"]) or not phase["sources_unchanged"]:
        raise SystemExit(1)

print("Report: " + str(report_path), flush=True)
