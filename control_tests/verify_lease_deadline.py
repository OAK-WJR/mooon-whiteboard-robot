#!/usr/bin/env python3
"""Offline regression for a control lease expiring during authentication.

Uses the existing public-only staging helper; never opens hardware or private
configuration. --build also cross-compiles the shared runtime for the C6 R3
review profile. This is software evidence, not PCB or motor qualification.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

from verify_pico_profile import ROOT, public_copy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", action="store_true")
    args = parser.parse_args()
    evidence = ROOT / "control_tests/lease_deadline_results"
    evidence.mkdir(exist_ok=True)
    rows = []
    watched = ("src/top_v2_main.cpp", "include/control_session.h",
               "control_tests/test_runtime_commands.cpp", "control_tests/runtime_stubs.h",
               "control_tests/verify_lease_deadline.py")

    def hashes():
        return {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in watched}

    before = hashes()
    with tempfile.TemporaryDirectory(prefix="wbb-lease-public-") as temporary:
        stage = Path(temporary)
        public_copy(stage)

        def run(name, command):
            print(name + " START", flush=True)
            result = subprocess.run([str(a) for a in command], cwd=stage,
                                    capture_output=True, text=True, timeout=900)
            (evidence / (name + ".log")).write_text(result.stdout + result.stderr)
            rows.append({"name": name, "exit_code": result.returncode,
                         "command": [str(a).replace(str(stage), "$STAGE") for a in command]})
            print(name + (" PASS" if result.returncode == 0 else " FAIL"), flush=True)
            return result.returncode == 0

        common = ["c++", "-std=c++17", "-fsanitize=address,undefined",
                  "-fno-sanitize-recover=all", "-Icontrol_tests/mock", "-Iinclude",
                  "-I" + str(ROOT / ".pio/libdeps/top_board_v2/ArduinoJson/src")]
        r3 = ["-DWBB_TOP_BOARD_R3=1", "-DCONFIG_IDF_TARGET_ESP32C6=1",
              "-DARDUINO_USB_CDC_ON_BOOT=0"]
        cases = [
            ("commands_native_usb", "test_runtime_commands.cpp", [], [""]),
            ("commands_r3_uart", "test_runtime_commands.cpp", r3, [""]),
            ("lease_framing", "test_session.cpp", [], [""]),
            ("runtime_timing", "test_runtime_timing.cpp", [], [""]),
            ("transport_reboot", "test_transport_reboot.cpp", [],
             ["backlog", "full", "flood", "tx-failure", "wrap", "owner-failure"]),
        ]
        for name, source, flags, scenarios in cases:
            executable = stage / name
            if run(name + "_compile", common + flags + ["control_tests/" + source, "-o", executable]):
                for scenario in scenarios:
                    run(name + ("_" + scenario if scenario else ""),
                        [executable] + ([scenario] if scenario else []))
        if args.build:
            run("c6_r3_cross_build", [ROOT / ".venv/bin/pio", "run", "-e", "top_board_v2_r3"])

    after = hashes()
    passed = before == after and all(row["exit_code"] == 0 for row in rows)
    report = {"time_utc": datetime.now(timezone.utc).isoformat(), "all_passed": passed,
              "checks": rows, "source_hashes": after,
              "watched_sources_unchanged_during_tests": before == after,
              "pre_fix_reproduction": "New assertion failed: controller still moving after LEASE_EXPIRED; exit -6",
              "native_scope": "Production command, lease and loop; mock controller, clocks, SDK and transport boundaries. Transport reboot regressions use production transport with SDK doubles.",
              "hardware_tested": False, "firmware_uploaded": False,
              "pcb_qualified": False, "private_configuration_read": False}
    (evidence / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
