#!/usr/bin/env python3
"""Public-only native/profile checks; --build also cross-compiles Pico and R3.

Never opens serial/USB or reads the private include/config.h. All compilation
runs in a disposable copy with a deliberately failing replacement config.h.
No hardware report paths or shared hardware design files are written.
"""
import argparse
import configparser
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / "hardware/top_board_v2/kiss_6v/power_integration"


def run(name, args, cwd, rejection=None):
    print(name + " START", flush=True)
    result = subprocess.run([str(a) for a in args], cwd=cwd, capture_output=True,
                            text=True, timeout=900)
    output = result.stdout + result.stderr
    passed = result.returncode == 0 if rejection is None else result.returncode != 0 and rejection in output
    if not passed:
        print(output[-6000:], flush=True)
        raise RuntimeError(name + " failed")
    print(name + " PASS", flush=True)
    return output


def public_copy(destination):
    for folder in ("include", "src", "control_tests"):
        for source in (ROOT / folder).rglob("*"):
            # Skip by path before opening/reading/hashing/copying anything.
            if source == ROOT / "include/config.h" or source.suffix not in (".h", ".cpp", ".c"):
                continue
            target = destination / source.relative_to(ROOT)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
    (destination / "include/config.h").write_text('#error "PRIVATE_CONFIG_MUST_NOT_BE_USED"\n')
    # Exactly the project-owned driver sources needed by pico_i2c_native.c.
    vendor = Path("third_party/esp_idf_i2c_no_clear")
    for name in ("i2c_master.c", "i2c_private.h", "namespace.h"):
        target = destination / vendor / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / vendor / name, target)
    ini = (ROOT / "platformio.ini").read_text()
    ini = ini.replace("core_dir = .platformio", "core_dir = " + str(ROOT / ".platformio"))
    (destination / "platformio.ini").write_text(ini)


def export_native(stage, name):
    cli = shutil.which("kicad-cli") or "/Applications/KiCad.app/Contents/MacOS/kicad-cli"
    if not Path(cli).is_file():
        raise RuntimeError("Installed kicad-cli is required for the actual 14-sheet netlist check")
    output = stage / (name + ".xml")
    run(name, [cli, "sch", "export", "netlist", "--format", "kicadxml", "--output", output,
               CONTRACT / "power_modules.kicad_sch"], stage)
    return ET.parse(output).getroot()


def contract_check(observed, contract, recovery, native):
    # Derive GPIO identity from all 40 actual module contacts. gpio_assignments
    # alone omits FLT18, PG38 and cycle42 in the current adopted contract.
    contacts = contract["module"]["physical_contacts"]
    pins = contract["pin_nets"]["MCU1"]
    nets = {net: int(contacts[contact][4:]) for contact, net in pins.items()
            if net and contacts[contact].startswith("GPIO")}
    assert len(contacts) == 40 and len(nets) == 21
    assert len(native.findall("design/sheet")) == 14, "main sheet count changed; review required"
    native_pins = {}
    native_nets = {}
    for net in native.findall("nets/net"):
        members = {(node.attrib["ref"], node.attrib["pin"]) for node in net.findall("node")}
        native_nets[net.attrib["name"]] = members
        for node in net.findall("node"):
            if node.attrib["ref"] == "MCU1":
                assert node.attrib["pin"] not in native_pins, "module contact on multiple nets"
                native_pins[node.attrib["pin"]] = (net.attrib["name"], members)
                if contacts[node.attrib["pin"]].startswith("GPIO"):
                    assert node.attrib["pinfunction"].split("_")[0] == contacts[node.attrib["pin"]]
    for contact, expected in pins.items():
        actual, members = native_pins[contact]
        if expected is None:
            assert members == {("MCU1", contact)} and actual.startswith("unconnected-"), "NC contact connected"
        else:
            assert actual == expected, "contract disagrees with actual MCU1 net at contact " + contact
    assert pins["19"] == "WAKE_REQUEST_N" and pins["22"] == "CHARGE_REQUEST" and pins["20"] is None
    assert {("MCU1", "19"), ("U94", "2"), ("R101", "2")} <= native_nets["WAKE_REQUEST_N"]
    assert ("R101", "1") in native_nets["MAIN_3V3"]
    assert {("MCU1", "22"), ("R112", "1")} <= native_nets["CHARGE_REQUEST"]
    sense = next(members for members in native_nets.values() if ("R112", "2") in members)
    assert {("U111", "6"), ("R113", "1")} <= sense
    assert ("R113", "2") in native_nets["GND"]
    values = {c.attrib["ref"]: c.findtext("value") for c in native.findall("components/comp")}
    assert [values[ref] for ref in ("R101", "R112", "R113")] == ["10k", "330", "10k"]
    assert "LOGIC_CYCLE_REQUEST" in recovery["pin_nets"]["R50"].values()
    assert recovery["pin_nets"]["Q40"]["3"] == "LOGIC_BUCK_SHDN"
    headers = sorted(contract["bottom_mates"], key=lambda h: int(contract["bottom_mates"][h].split("Motor")[-1])
                     if "Motor" in contract["bottom_mates"][h] else 999)[:3]
    motor_pins = [nets[contract["pin_nets"][h][contact]] for h in headers for contact in ("3", "2")]
    encoders = [[nets[contract["pin_nets"][h][contact]] for contact in ("5", "4")] for h in headers]
    assert observed["motor_config_groups"] == motor_pins * 2, "real driver initialized wrong physical order"
    assert set(observed["encoders"]) == {p for pair in encoders for p in pair}
    assert set(observed["outputs"]) == set(motor_pins) | {nets[n] for n in (
        "MOTOR_ARM_REQUEST", "WD_HEARTBEAT_OD", "LOGIC_CYCLE_REQUEST", "WAKE_REQUEST_N", "CHARGE_REQUEST")}
    assert set(observed["status_inputs"]) == {nets["LOGIC_FAULT_N"], nets["LOGIC_PGOOD"]}
    assert observed["wake_request_n"] == nets["WAKE_REQUEST_N"]
    assert observed["charge_request"] == nets["CHARGE_REQUEST"]
    for pin, *deltas in observed["edge_responses"]:
        wheel = next(i for i, pair in enumerate(encoders) if pin in pair)
        assert abs(deltas[wheel]) == 1 and sum(abs(d) for d in deltas) == 1, "ISR updated wrong physical wheel"
    assert observed["uart"] == [43, 44]  # Factory UART is internal, not a header contact.
    assert [observed["i2c_sda"], observed["i2c_scl"]] == [nets["I2C_SDA"], nets["I2C_SCL"]]
    consumers = set(observed["outputs"] + observed["encoders"] + observed["status_inputs"] +
                    [observed["i2c_sda"], observed["i2c_scl"]])
    assert consumers == set(nets.values()) and len(consumers) == 21, "actual consumers do not cover 21 GPIOs"
    # Only worker calls are mocked; GPIO consumers must not claim I2C through
    # Arduino. The compiled production adapter statically binds SDK pins1/2.
    assert not observed["arduino_i2c_pins_claimed"]
    assert observed["rail_worker_open_requests"] == 1
    assert observed["profile"] == "top_board_v2_pico_review"
    print("21 GPIO compiled consumers match fresh 14-sheet MCU40 netlist + contracts PASS", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", action="store_true", help="also cross-compile public-only Pico and R3 firmware")
    args = parser.parse_args()
    contract = json.loads((CONTRACT / "controller_contract.json").read_text())
    recovery = json.loads((CONTRACT / "logic_recovery_contract.json").read_text())
    watchdog = json.loads((CONTRACT / "watchdog_contract.json").read_text())
    # Native export was independently checked for this handoff. Prevent a later
    # pin-contract change from silently retaining the boot/recovery assumptions.
    assert watchdog["watchdog_timeout_ms"] == [170, 200, 230]
    assert watchdog["pin_nets"]["U20"] == {
        "1": "MAIN_3V3", "2": "WD_CWD", "3": "EN_CHIP_PU", "4": "GND",
        "5": "MAIN_3V3", "6": "WD_HEARTBEAT_OD", "7": "SAFETY_CLEAR_N", "8": "SAFETY_CLEAR_N", "9": "GND"}
    config = configparser.ConfigParser(interpolation=None)
    config.read(ROOT / "platformio.ini")
    selected = config["env:top_board_v2_pico"]
    assert selected["board"] == "esp32-s3-devkitc-1"
    assert selected["board_upload.flash_size"] == "16MB"
    assert "top_board_v2_pico" not in config["platformio"]["default_envs"]
    pico_flags = shlex.split(selected["build_flags"]) + ["-DCONFIG_IDF_TARGET_ESP32S3=1"]
    json_include = next((p for p in (ROOT / ".pio/libdeps").glob("*/ArduinoJson/src") if p.is_dir()), None)
    if json_include is None:
        raise RuntimeError("An installed ArduinoJson 7.4.3 is required for native checks")
    with tempfile.TemporaryDirectory(prefix="wbb-pico-public-checks-") as temp:
        stage = Path(temp)
        public_copy(stage)
        native = export_native(stage, "native_netlist_before")
        common = ["c++", "-std=c++17", "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
                  "-Icontrol_tests/mock", "-Iinclude", "-Isrc", "-I" + str(json_include)]
        pico_common = common[:1] + ["-Icontrol_tests/pico_mock"] + common[1:]
        gpio_binary = stage / "pico-gpio-safe"
        run("pico_gpio_safe_compile", pico_common + pico_flags +
            ["-MMD", "-MF", "gpio_safe.d", "control_tests/test_pico_gpio_safe.cpp", "-o", gpio_binary], stage)
        assert "include/config.h" not in (stage / "gpio_safe.d").read_text()
        print(run("pico_gpio_safe", [gpio_binary], stage).strip(), flush=True)
        binary = stage / "pico-native"
        run("pico_native_compile", pico_common + pico_flags +
            ["-MMD", "-MF", "pico.d", "control_tests/test_pico_profile.cpp", "-o", binary], stage)
        assert "include/config.h" not in (stage / "pico.d").read_text()
        modes = ["startup", "status_budget", "fail_level_16", "fail_level_42", "fail_config_16", "fail_config_42",
                 "fail_direction_17", "fail_level_17", "fail_config_17", "fail_config_18", "fail_config_38",
                 "release_failure", "hold_16", "hold_42", "reboot_usb", "reboot_wifi", "reboot_ble"]
        modes += ["fail_direction_39", "fail_level_39", "fail_config_39", "fail_level_41", "fail_config_41",
                  "hold_39", "hold_41", "release_failure_39", "release_failure_41"]
        modes += [f"motor_{op}_{pin}" for op in ("level", "config") for pin in range(4, 10)]
        for mode in modes:
            output = run("pico_" + mode, [binary, mode], stage)
            if mode == "startup":
                observed = json.loads(next(line.removeprefix("PICO_OBSERVED ") for line in output.splitlines()
                                           if line.startswith("PICO_OBSERVED ")))
                contract_check(observed, contract, recovery, native)

        watchdog_binary = stage / "pico-watchdog"
        run("pico_watchdog_compile", pico_common + pico_flags +
            ["-MMD", "-MF", "watchdog.d", "control_tests/test_pico_watchdog.cpp", "-o", watchdog_binary], stage)
        assert "include/config.h" not in (stage / "watchdog.d").read_text()
        for mode in ("healthy", "startup_timeout", "startup_wdt_failure", "boot_rom_margin", "wrap", "stale",
                     "lost_control", "slow_control", "slow_transport", "task_feed_failure", "pulse_low_failure",
                     "pulse_release_failure", "pulse_cleanup_failure", "reboot_progress"):
            run("watchdog_" + mode, [watchdog_binary, mode], stage)

        # Compile failures exercise real selection/entry-point guards with flags
        # taken from platformio.ini, rather than a second generated pin table.
        selection = stage / "selection.cpp"
        selection.write_text('#include "firmware_config.h"\nint main() { return 0; }\n')
        guards = [
            ("wrong_soc", [f for f in pico_flags if "CONFIG_IDF_TARGET_ESP32S3" not in f] + ["-DCONFIG_IDF_TARGET_ESP32C6=1"], "Pico requires ESP32-S3"),
            ("cdc_console", [f for f in pico_flags if "ARDUINO_USB_CDC_ON_BOOT" not in f] + ["-DARDUINO_USB_CDC_ON_BOOT=1"], "Pico factory CH343P requires UART0"),
            ("mixed_r3", pico_flags + ["-DWBB_TOP_BOARD_R3=1"], "Pico and R3 profiles are mutually exclusive"),
            ("stale_config", pico_flags + ["-DCONFIG_H=1"], "Pico public defaults must be loaded before"),
            ("motor_test", pico_flags + ["-DMOTOR_TEST_MODE=1"], "Pico requires the gated integrated runtime"),
            ("r3_diagnostic", pico_flags + ["-DWBB_R3_DIAGNOSTIC=1"], "Pico requires the gated integrated runtime"),
            ("missing_runtime", [f for f in pico_flags if "WBB_TOP_BOARD_V2" not in f], "Pico requires the gated integrated runtime"),
            ("implicit_pico", [f for f in pico_flags if "WBB_TOP_BOARD_PICO" not in f], "Select the explicit top_board_v2_pico environment"),
            ("unselected_s3", ["-DWBB_TOP_BOARD_V2=1", "-DCONFIG_IDF_TARGET_ESP32S3=1"], "requires explicit WBB_TOP_BOARD_PICO selection"),
        ]
        for name, flags, rejection in guards:
            run(name, pico_common + flags + [selection, "-o", stage / "selection"], stage, rejection)
        (stage / "selection.c").write_text("int c_preinclude_is_valid;\n")
        run("framework_c_preinclude", ["cc", "-c", "-Iinclude"] + pico_flags + ["selection.c", "-o", "selection.o"], stage)

        # Small relevant regressions for the shared runtime/driver/transport.
        r3_flags = ["-DWBB_TOP_BOARD_R3=1", "-DCONFIG_IDF_TARGET_ESP32C6=1", "-DARDUINO_USB_CDC_ON_BOOT=0"]
        cases = [
            ("legacy_commands", "test_runtime_commands.cpp", [], [""]),
            ("r3_commands", "test_runtime_commands.cpp", r3_flags, [""]),
            ("r3_motor_faults", "test_motor_driver.cpp", r3_flags + ["-DWBB_TOP_BOARD_V2=1", "-include", "include/config.example.h"], [""]),
            ("r3_gpio_startup", "test_r3_gpio_init.cpp", [], ["cold", "warm", "preload_failure", "config_failure", "write_failure", "provision_failure", "overrun_failure", "wireless_reboot_failure"]),
            ("legacy_transport", "test_transport.cpp", [], [""]),
            ("legacy_reboot", "test_transport_reboot.cpp", [], ["backlog", "full", "flood", "tx-failure", "wrap", "owner-failure"]),
        ]
        for name, source, flags, scenarios in cases:
            executable = stage / name
            run(name + "_compile", common + flags + ["control_tests/" + source, "-o", executable], stage)
            for scenario in scenarios:
                run(name + ("_" + scenario if scenario else ""), [executable] + ([scenario] if scenario else []), stage)

        if args.build:
            for environment in ("top_board_v2_pico", "top_board_v2_r3"):
                output = run("cross_build_" + environment, [ROOT / ".venv/bin/pio", "run", "-e", environment], stage)
                for line in output.splitlines():
                    if "RAM:" in line or "Flash:" in line:
                        print(line, flush=True)
                dependency_files = list((stage / ".pio/build" / environment / "src").glob("*.d"))
                assert dependency_files, "no compiler dependency evidence"
                for dependency in dependency_files:
                    assert "include/config.h" not in dependency.read_text(), "private-config dependency escaped selection"
        # Check current identity again if the hardware owner edited during the run.
        contract_check(observed, json.loads((CONTRACT / "controller_contract.json").read_text()),
                       json.loads((CONTRACT / "logic_recovery_contract.json").read_text()),
                       export_native(stage, "native_netlist_after"))
        assert watchdog == json.loads((CONTRACT / "watchdog_contract.json").read_text()), "watchdog contract changed during verification"
    print("Public-only checks complete; no hardware/serial/USB access or firmware upload.", flush=True)


if __name__ == "__main__":
    main()
