"""Offline KISS C6 checks over its native USB Serial/JTAG (c6usb, 303A:1001).

The 2026-10-29 board has no USB-UART bridge (the CH343G and its auto-download
transistors were removed). Serial, network and ROM subprocesses are fakes.
"""
import configparser
import contextlib
import io
from pathlib import Path
import subprocess
import unittest
from unittest.mock import patch

from test_cp2102n_host import SerialFixture, args as old_args, port as old_port, r3
from test_host import BOOT, KEY, FakeWire, envelope
import pcb_control as recovery
import robot_control as host

PATH = "/dev/cu.usbmodem1101"
MAC = "54:32:04:AA:BB:CC"  # USB Serial/JTAG reports the chip MAC as its USB serial number.


def port(**changes):
    fields = dict(path=PATH, serial=MAC, location="20-1", vid=0x303A, pid=0x1001)
    fields.update(changes)
    return old_port(**fields)


def status(**changes):
    result = r3(profile=host.KISS_C6_PROFILE, pwm_ready=False, odometry_qualified=False,
                x=None, y=None, theta=None, wheel_speed_mps=[None]*3)
    result.update(changes)
    return result


def fault(**changes):
    """Mirrors src/top_v2_main.cpp fillStatus() with the latched GPIO fault (KISS branch)."""
    result = dict(id=0, ok=True, error="", boot=BOOT, lease=0, protocol=2, firmware="wbb-control-2",
                  profile=host.KISS_C6_PROFILE, device="WhiteBoardBot-123456", uptime_ms=200, reset_reason=1,
                  state="gpio_fault", reason="GPIO0_SAFE_LOW_FAILED", gpio_fault_latched=True, gpio_error=258,
                  gpio_release_error=0, read_only=True, motion_enabled=False, charge_enabled=False,
                  power_disconnect=False, pwm_ready=False, watchdog_ready=True, wireless_control_enabled=False,
                  telemetry_available=False, x=None, y=None, theta=None, motor_voltage_mv=None,
                  motor_current_ma=None, rail_3v3_mv=None, encoder_counts=[None]*3, wheel_speed_mps=[None]*3,
                  pwm_command=[0, 0, 0], physical_motors=[1, 2, 3], header_order=["M0", "M2", "M1"], owner=0,
                  lease_ms=500, usb_reboot=True, reboot_pending=False, tx_dropped=0, odometry_qualified=False,
                  wireless_status=False, wireless_power_state="waiting_voltage", wifi_connected=False,
                  wifi_ip="0.0.0.0", wifi_rssi_dbm=None, ble_ready=False, ble_connected=False,
                  authenticated_wireless=False)
    result.update(changes)
    return result


def online_fault(**changes):
    return fault(**(dict(wireless_status=True, wireless_power_state="online", wifi_connected=True,
                         wifi_ip="192.0.2.7", wifi_rssi_dbm=-61, ble_ready=True,
                         authenticated_wireless=True) | changes))


def args(action="status", **changes):
    return old_args(action, **(dict(usb=PATH, usb_kind="c6usb", profile=host.KISS_C6_PROFILE) | changes))


class SelectionTests(unittest.TestCase):
    def test_c6usb_binds_native_identity_and_kiss_profile(self):
        self.assertEqual(recovery.usb_identity("c6usb"), (0x303A, 0x1001))
        self.assertEqual(recovery.usb_identity("c6usb"), recovery.NATIVE_USB)
        self.assertEqual(recovery.usb_chip("c6usb"), "esp32c6")
        self.assertEqual(host.USB_PROFILES["c6usb"], host.KISS_C6_PROFILE)
        self.assertEqual(host.selected_profile(args()), host.KISS_C6_PROFILE)
        self.assertEqual(host.Wire(args(profile=None)).expected_profile, host.KISS_C6_PROFILE)
        for changes in (dict(profile=host.PICO_PROFILE), dict(profile=host.R3_PROFILE), dict(profile="top_board_v2"),
                        dict(usb_kind="native"), dict(usb_kind="ch343p"), dict(usb=None), dict(usb=""),
                        dict(usb="rfc2217://test"), dict(usb=None, wifi="192.0.2.1")):
            with self.subTest(changes=changes), self.assertRaises(host.DiagnosticError):
                host.selected_profile(args(**changes))

    def test_ch343g_kind_is_removed(self):
        self.assertNotIn("ch343g", host.USB_PROFILES)
        self.assertNotIn("ch343g", recovery.USB_KINDS)
        self.assertFalse(hasattr(recovery, "CH343G_USB"))
        with self.assertRaisesRegex(host.DiagnosticError, "c6usb"):
            recovery.usb_identity("ch343g")
        for call in (lambda: host.selected_profile(args(usb_kind="ch343g")),
                     lambda: recovery.select_port([port()], PATH, "ch343g"),
                     lambda: recovery.rom_command(PATH, True, "ch343g"),
                     lambda: recovery.rom_config_text("ch343g")):
            with self.assertRaises(host.DiagnosticError):
                call()

    def test_port_selection_requires_one_explicit_c6_usb_jtag_device(self):
        self.assertEqual(recovery.select_port([port(), old_port()], PATH, "c6usb").device, PATH)
        for requested, ports in ((None, [port()]), ("", [port()]), ("rfc2217://fixture", [port()]),
                                 (PATH, [port(vid=0x1A86, pid=0x55D3)]),  # the removed CH343 bridge
                                 (PATH, [port(vid=0x10C4, pid=0xEA60)]), (PATH, [port(pid=0x1002)]),
                                 ("/dev/other", [port()]), (PATH, [port(), port()]),
                                 (PATH, [port(), port(path="/dev/clone")]),  # same MAC twice
                                 (PATH, [port(serial=None, location=None)])):
            with self.subTest(requested=requested, ports=ports), self.assertRaises(host.DiagnosticError):
                recovery.select_port(ports, requested, "c6usb")
        with self.assertRaisesRegex(host.DiagnosticError, "303A:1001"):
            recovery.select_port([], PATH, "c6usb")
        target = recovery.identity(recovery.select_port([port()], PATH, "c6usb"))
        self.assertEqual(target["serial"], MAC)
        # Re-enumeration may rename the CDC path; the MAC serial must survive.
        self.assertEqual(recovery.match_port([port(path="/dev/cu.usbmodem1102", location="20-2")], target),
                         "/dev/cu.usbmodem1102")
        self.assertIsNone(recovery.match_port([port(serial="54:32:04:00:00:01")], target))

    def test_explicit_profile_capability_and_message(self):
        for profile in (None, host.R3_PROFILE, host.PICO_PROFILE):
            with self.assertRaises(host.DiagnosticError):
                host.check_status(status(), profile)
        with self.assertRaisesRegex(host.DiagnosticError, "c6usb"):
            host.check_status(status(), None)
        host.check_status(status(), host.KISS_C6_PROFILE)
        host.check_status(status(encoder_counts=[0, -5, 2**31-1]), host.KISS_C6_PROFILE)
        for field, value in (("motion_enabled", True), ("pwm_ready", True), ("charge_enabled", True),
                             ("odometry_qualified", True), ("motor_voltage_mv", 6000), ("x", 0),
                             ("wheel_speed_mps", [0, 0, 0]), ("gpio_fault_latched", True),
                             ("encoder_counts", [None]*3), ("encoder_counts", [0, 0]), ("encoder_counts", [0, 0, 2**31])):
            with self.subTest(field=field, value=value), self.assertRaises(host.DiagnosticError):
                host.check_status(status(**{field: value}), host.KISS_C6_PROFILE)

    def test_latched_gpio_fault_status_is_validated(self):
        for frame in (fault(), online_fault(), online_fault(wifi_connected=False, wifi_rssi_dbm=None, ble_connected=True),
                      fault(wireless_status=True, wireless_power_state="init_failed"),
                      fault(wireless_power_state="qualifying"), fault(wireless_power_state="inhibited"),
                      fault(gpio_release_error=259, watchdog_ready=False)):
            host.check_status(frame, host.KISS_C6_PROFILE)
        bad = [dict(reason="GPIO14_SAFE_LOW_FAILED"), dict(reason="x"), dict(gpio_fault_latched=False),
               dict(read_only=False), dict(wireless_control_enabled=True), dict(telemetry_available=True),
               dict(usb_reboot=False), dict(gpio_error=0), dict(gpio_error=None), dict(gpio_release_error=None),
               dict(encoder_counts=[5, 6, 7]), dict(pwm_command=[1, 0, 0]), dict(owner=1), dict(motion_enabled=True),
               dict(wireless_status=True), dict(wireless_power_state="online"),
               dict(wireless_power_state="guaranteed"), dict(wireless_status=1), dict(wifi_connected=None),
               dict(wifi_rssi_dbm=-50), dict(wifi_ip=None), dict(ble_ready=True), dict(ble_connected=True),
               dict(wifi_connected=True, wifi_rssi_dbm=-60), dict(authenticated_wireless="yes"),
               dict(state="idle")]
        for changes in bad:
            with self.subTest(changes=changes), self.assertRaises(host.DiagnosticError):
                host.check_status(fault(**changes), host.KISS_C6_PROFILE)
        for changes in (dict(wifi_rssi_dbm=None), dict(wifi_rssi_dbm=5), dict(wifi_rssi_dbm=-61.0)):
            with self.subTest(changes=changes), self.assertRaises(host.DiagnosticError):
                host.check_status(online_fault(**changes), host.KISS_C6_PROFILE)
        for field in ("reason", "gpio_error", "gpio_release_error", "wireless_status", "wireless_power_state",
                      "wifi_connected", "wifi_ip", "wifi_rssi_dbm", "ble_ready", "ble_connected",
                      "authenticated_wireless", "owner", "encoder_counts", "telemetry_available"):
            frame = fault()
            del frame[field]
            with self.subTest(missing=field), self.assertRaises(host.DiagnosticError):
                host.check_status(frame, host.KISS_C6_PROFILE)

    def test_rom_command_is_usb_jtag_reset_without_uart_options(self):
        for stay in (True, False):
            command = recovery.rom_command(PATH, stay, "c6usb")
            self.assertEqual(command[command.index("--chip") + 1], "esp32c6")
            self.assertEqual(command[command.index("--port") + 1], PATH)
            self.assertEqual(command[command.index("--before") + 1], "usb-reset")
            self.assertEqual(command[command.index("--after") + 1], "no-reset" if stay else "hard-reset")
            self.assertEqual(command[-2:], ["--no-stub", "read-mac"])
            for option in ("--baud", "--connect-attempts", "default-reset"):
                self.assertNotIn(option, command)
            self.assertFalse(any("flash" in word or "erase" in word for word in command))
        for path in (None, "", "rfc2217://example"):
            with self.assertRaises(host.DiagnosticError):
                recovery.rom_command(path, False, "c6usb")
        for kind in recovery.USB_KINDS:
            self.assertEqual(recovery.rom_config_text(kind), "[esptool]\n")

    def test_installed_esptool_uses_usb_jtag_reset_under_isolated_config(self):
        import esptool.loader as loader
        from esptool.reset import CustomReset, USBJTAGSerialReset
        from esptool.targets.esp32c6 import ESP32C6ROM
        chip = object.__new__(ESP32C6ROM)
        chip._port = type("Port", (), {"name": PATH})()
        chip.get_usb_vid_pid = lambda: recovery.usb_identity("c6usb")
        isolated = configparser.ConfigParser()
        isolated.read_string(recovery.rom_config_text("c6usb"))
        with patch.object(loader, "cfg", isolated["esptool"]):
            for mode in ("usb-reset", "default-reset"):
                strategies = chip._construct_reset_strategy_sequence(mode)
                self.assertEqual(len(strategies), 1)
                self.assertIsInstance(strategies[0], USBJTAGSerialReset)
        # Why the host isolates esptool.cfg: a user custom_reset_sequence replaces even usb-reset.
        user = configparser.ConfigParser()
        user.read_string("[esptool]\ncustom_reset_sequence = R0|D1|W0.1\n")
        with patch.object(loader, "cfg", user["esptool"]):
            self.assertIsInstance(chip._construct_reset_strategy_sequence("usb-reset")[0], CustomReset)

    def test_cli_selection_errors_and_help(self):
        for flags in (["--usb", PATH, "--usb-kind", "ch343g"], ["--wifi", "192.0.2.1", "--usb-kind", "c6usb"],
                      ["--usb", "", "--usb-kind", "c6usb"], ["--usb", PATH, "--profile", host.KISS_C6_PROFILE],
                      ["--usb", PATH, "--usb-kind", "c6usb", "--profile", host.PICO_PROFILE]):
            with self.subTest(flags=flags), patch.object(host, "run") as run, \
                    contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                host.main(flags)
            self.assertEqual(error.exception.code, 2)
            run.assert_not_called()
        out = io.StringIO()
        with contextlib.redirect_stdout(out), self.assertRaises(SystemExit):
            host.main(["--help"])
        text = " ".join(out.getvalue().split())
        self.assertIn("c6usb", text)
        self.assertNotIn("ch343g", text)
        for words in ("SW1 off", "short the BOOT pads", "SW1 on", "remove the short"):
            self.assertIn(words, text)


class IntegrationTests(unittest.IsolatedAsyncioTestCase):
    async def test_wrong_s3_or_r3_app_is_rejected_before_any_command(self):
        for discovery in (status(), fault(), r3(), status(profile=host.PICO_PROFILE)):
            stream = SerialFixture([discovery, status(id=1)])
            with patch("serial.tools.list_ports.comports", return_value=[port()]), \
                    patch.object(host, "open_serial", return_value=stream) as opener:
                result = await host.run(args())
            if discovery["profile"] == host.KISS_C6_PROFILE:
                self.assertEqual(result["result"], "ACKNOWLEDGED")
                self.assertTrue(result["app_profile_verified"])
                self.assertEqual(result["usb_device"]["serial"], MAC)
                opener.assert_called_once_with(PATH)
            else:
                self.assertEqual(result["result"], "FAIL")
                self.assertEqual(stream.writes, [b"hello\n"])
            self.assertEqual(result["usb_kind"], "c6usb")
            self.assertTrue(stream.closed)

    async def test_rom_uses_isolated_config_and_polls_identity_after_reenumeration(self):
        renamed = port(path="/dev/cu.usbmodem1102")
        cases = (("present", [[port()], [port()]], "C6_ROM_VERIFIED", PATH),
                 ("reenumerated", [[port()], [], [old_port()], [renamed]], "C6_ROM_VERIFIED", renamed.device),
                 ("swapped", [[port()]] + [[port(serial="54:32:04:00:00:01")]] * 50, "FAIL", None),
                 ("ambiguous", [[port()], [port(), renamed]], "FAIL", None))
        for name, inventories, expected, after in cases:
            calls = []

            def probe(command, **kwargs):
                calls.append(command)
                self.assertEqual(command[command.index("--before") + 1], "usb-reset")
                self.assertEqual(command[command.index("--after") + 1], "no-reset")
                self.assertNotIn("--baud", command)
                self.assertEqual(Path(kwargs["env"]["ESPTOOL_CFGFILE"]).read_text(), "[esptool]\n")
                self.assertEqual(kwargs["env"]["ESPTOOL_OPEN_PORT_ATTEMPTS"], "1")
                return subprocess.CompletedProcess(command, 0, "C6 ROM fixture", "")
            sequence = iter(inventories)
            with self.subTest(case=name), patch.object(host, "C6USB_REENUMERATION_S", .3), \
                    patch("serial.tools.list_ports.comports", side_effect=lambda: next(sequence, inventories[-1])), \
                    patch.object(host.subprocess, "run", side_effect=probe), patch.object(host, "open_serial") as opener:
                result = await host.run(args("bootloader"))
                self.assertEqual(result["result"], expected, result.get("error"))
                self.assertEqual(len(calls), 1)
                opener.assert_not_called()
                for field in ("custom_pcb_identity_verified", "physical_power_cycle", "flash_modified"):
                    self.assertIs(result[field], False)
                self.assertEqual(result["usb_kind"], "c6usb")
                self.assertEqual(result["usb_device"]["serial"], MAC)
                if after:
                    self.assertEqual(result["usb_port_after_rom"], after)
                    self.assertFalse(result["application_running"])
                    self.assertIn("SW1", result["note"])
                elif name == "swapped":
                    self.assertIn("identity lost", result["error"])

    async def test_reset_checks_new_profile_and_idle_return(self):
        real = host.wait_for_restart

        async def quick(*a, **kw):
            return await real(*a, **kw, timeout=.01, interval=0)
        for after in (status(), r3(), status(pwm_command=[1, 0, 0]), status(uptime_ms=40000), fault()):
            with self.subTest(after=after), patch("serial.tools.list_ports.comports", return_value=[port()]), \
                    patch.object(host.subprocess, "run", return_value=subprocess.CompletedProcess([], 0, "C6", "")), \
                    patch.object(host, "open_serial", side_effect=lambda _: SerialFixture([after])), \
                    patch.object(host, "wait_for_restart", side_effect=quick):
                result = await host.run(args("reset"))
            self.assertEqual(result["result"], "APP_RETURN_VERIFIED" if after == status() else "FAIL")
            # An application answered every time: no BOOT-pad power-cycle hint.
            self.assertIs(result["manual_power_cycle_required"], False)
            self.assertNotIn("manual_power_cycle_note", result)
            if result["result"] != "FAIL":
                self.assertFalse(result["new_boot_verified"])
                self.assertTrue(result["app_profile_verified"])

    async def test_silent_port_after_reset_requests_power_cycle(self):
        # After a BOOT-pad download the RTS core reset leaves the chip in ROM: the port stays, nothing answers.
        real_wait, real_recv = host.wait_for_restart, host.Wire.recv

        async def quick(*a, **kw):
            return await real_wait(*a, **kw, timeout=.15, interval=0)

        async def short_recv(self, timeout=3):
            return await real_recv(self, .02)
        streams = []
        with patch("serial.tools.list_ports.comports", return_value=[port()]), \
                patch.object(host.subprocess, "run", return_value=subprocess.CompletedProcess([], 0, "C6", "")), \
                patch.object(host, "open_serial", side_effect=lambda _: streams.append(SerialFixture([])) or streams[-1]), \
                patch.object(host, "wait_for_restart", side_effect=quick), patch.object(host.Wire, "recv", short_recv):
            result = await host.run(args("reset"))
        self.assertEqual(result["result"], "FAIL")
        self.assertIn("NOT verified", result["error"])
        self.assertIs(result["manual_power_cycle_required"], True)
        for words in ("BOOT", "GPIO9", "SW1"):
            self.assertIn(words, result["manual_power_cycle_note"])
        self.assertTrue(streams and all(s.closed for s in streams))

    async def test_restart_wait_records_whether_any_application_answered(self):
        class Silent(FakeWire):
            wireless = False
            async def open(self): pass
            async def close(self): pass
            async def recv(self, timeout=3): raise host.ReplyTimeout("Control reply timed out")

        class Answering(Silent):
            async def recv(self, timeout=3): return envelope(r3())
        for wire, answered in ((Silent([]), False), (Answering([]), True)):
            with self.assertRaises(host.RestartNotVerified) as error:
                await host.wait_for_restart(wire, None, None, software=False, timeout=.01, interval=0,
                                            expected_profile=host.KISS_C6_PROFILE)
            self.assertIs(error.exception.application_answered, answered)

    async def test_software_restart_rejects_old_boot_wrong_device_and_wrong_reason(self):
        for changes in ({}, dict(boot=BOOT), dict(device="other"), dict(reset_reason=1), dict(state="running")):
            class Wire(FakeWire):
                wireless = False
                async def open(self): self.replies = [envelope(status(boot="a"*16, **changes))] if "boot" not in changes else [envelope(status(**changes))]
                async def close(self): pass
            if changes:
                with self.assertRaises(host.DiagnosticError):
                    await host.wait_for_restart(Wire([]), None, status(), timeout=.01, interval=0, expected_profile=host.KISS_C6_PROFILE)
            else:
                result = await host.wait_for_restart(Wire([]), None, status(), timeout=.01, interval=0, expected_profile=host.KISS_C6_PROFILE)
                self.assertEqual(result["boot"], "a"*16)

    async def test_fault_is_read_only_over_usb_and_authenticated_wireless(self):
        for wireless in (False, True):
            frame = online_fault() if wireless else fault()
            wire = FakeWire([envelope(frame), envelope(dict(frame, id=1)),
                             envelope(dict(frame, id=2, ok=False, error="GPIO0_SAFE_LOW_FAILED"))])
            wire.wireless = wireless
            client = host.Controller(wire, KEY, host.KISS_C6_PROFILE)
            self.assertEqual((await client.hello())["state"], "gpio_fault")
            self.assertEqual((await client.request("status"))["reason"], "GPIO0_SAFE_LOW_FAILED")
            with self.assertRaisesRegex(host.CommandRejected, "GPIO0_SAFE_LOW_FAILED"):
                await client.request("claim")
            self.assertEqual(client.lease, 0)


if __name__ == "__main__":
    unittest.main()
