"""Offline CP2102N host integration; ports, subprocesses and resets are fakes."""
import asyncio
import configparser
import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import pcb_control as recovery
import robot_control as host
from test_host import BOOT, FakeWire, envelope, status


def port(path="/dev/cp", serial="bridge-A", location="1-2", vid=0x10C4, pid=0xEA60):
    return SimpleNamespace(device=path, serial_number=serial, location=location, vid=vid, pid=pid)


def r3(**changes):
    frame = status(profile=host.R3_PROFILE, charge_enabled=False, power_disconnect=False,
                   motor_voltage_mv=None, motor_current_ma=None, rail_3v3_mv=None, usb_reboot=True)
    frame.update(changes)
    return frame


def args(action="status", **changes):
    fields = dict(action=action, usb="/dev/cp", usb_kind="cp2102n", wifi=None, ble=None,
                  scan_ble=False, key_file=None, wifi_config=None, seconds=.1, params="{}")
    fields.update(changes)
    return SimpleNamespace(**fields)


class SerialFixture:
    def __init__(self, replies):
        self.replies = list(replies)
        self.pending = bytearray()
        self.writes = []
        self.closed = False

    def write(self, data):
        self.writes.append(data)
        if self.replies:
            self.pending.extend((json.dumps(envelope(self.replies.pop(0))) + "\n").encode())
        return len(data)

    @property
    def in_waiting(self):
        return len(self.pending)

    def read(self, count):
        data = bytes(self.pending[:count])
        del self.pending[:count]
        return data

    def close(self):
        self.closed = True


class SelectionTests(unittest.TestCase):
    def test_native_defaults_and_explicit_cp_allowlist(self):
        native = port("/dev/native", vid=0x303A, pid=0x1001)
        self.assertIs(recovery.select_port([port(), native]), native)
        self.assertEqual(recovery.select_port([port(), native], "/dev/cp", "cp2102n").device, "/dev/cp")
        for kind, path, ports in (
                ("native", "/dev/cp", [port()]), ("cp2102n", None, [port()]),
                ("cp2102n", "", [port()]), ("cp2102n", "/dev/cp", [port(vid=0x303A, pid=0x1001)]),
                ("cp2102n", "/dev/cp", [port(pid=0xEA61)]), ("cp2102n", "/dev/other", [port()]),
                ("cp2102n", "/dev/cp", [port(), port()]), ("auto", "/dev/cp", [port()])):
            with self.subTest(kind=kind, path=path), self.assertRaises(recovery.DiagnosticError):
                recovery.select_port(ports, path, kind)

    def test_cp_requires_unique_serial_or_location_before_any_open(self):
        for ports in ([port(serial=None, location=None)], [port(), port("/dev/clone")],
                      [port(serial=None), port("/dev/clone", serial=None)]):
            with self.assertRaises(recovery.DiagnosticError):
                recovery.select_port(ports, "/dev/cp", "cp2102n")
        target = recovery.identity(recovery.select_port([port()], "/dev/cp", "cp2102n"))
        self.assertEqual(recovery.match_port([port("/dev/renamed")], target), "/dev/renamed")
        self.assertIsNone(recovery.match_port([port(serial="other")], target))
        self.assertIsNone(recovery.match_port([port(pid=0xEA61)], target))
        target = recovery.identity(recovery.select_port([port(serial=None)], "/dev/cp", "cp2102n"))
        self.assertEqual(recovery.match_port([port("/dev/renamed", serial=None)], target), "/dev/renamed")
        self.assertIsNone(recovery.match_port([port(serial=None, location="other")], target))

    def test_rom_command_uses_uart_strategy_and_only_c6_read_mac_without_stub(self):
        for kind in ("native", "cp2102n"):
            for stay in (True, False):
                command = recovery.rom_command("/dev/cp", stay, kind)
                self.assertEqual(command[command.index("--chip") + 1], "esp32c6")
                self.assertEqual(command[command.index("--port") + 1], "/dev/cp")
                self.assertEqual(command[command.index("--before") + 1],
                                 "default-reset" if kind == "cp2102n" else "usb-reset")
                self.assertEqual(command[command.index("--after") + 1], "no-reset" if stay else "hard-reset")
                self.assertEqual(command[-2:], ["--no-stub", "read-mac"])
                self.assertFalse(any("flash" in word for word in command))
                if kind == "cp2102n":
                    self.assertEqual(command[command.index("--baud") + 1], "115200")
                    self.assertEqual(command[command.index("--connect-attempts") + 1], "3")
        self.assertEqual(recovery.rom_command("/dev/native", True), recovery.rom_command("/dev/native", True, "native"))
        for path, kind in ((None, "cp2102n"), ("rfc2217://example", "cp2102n"), ("/dev/cp", "auto")):
            with self.assertRaises(recovery.DiagnosticError):
                recovery.rom_command(path, False, kind)

    def test_installed_esptool_selects_uart_not_usb_jtag_for_cp(self):
        # No ROM constructor/serial open: execute installed strategy selection only.
        import esptool.loader as loader
        from esptool.targets.esp32c6 import ESP32C6ROM
        from esptool.reset import ClassicReset, UnixTightReset, USBJTAGSerialReset
        config = configparser.ConfigParser(); config["esptool"] = {}
        chip = object.__new__(ESP32C6ROM)
        chip._port = SimpleNamespace(name="/dev/cp")
        chip.get_usb_vid_pid = lambda: recovery.CP2102N_USB
        with patch.object(loader, "cfg", config["esptool"]):
            strategies = chip._construct_reset_strategy_sequence("default-reset")
            self.assertTrue(all(isinstance(s, (ClassicReset, UnixTightReset)) for s in strategies))
            self.assertTrue(any(isinstance(s, ClassicReset) for s in strategies))
            self.assertIsInstance(chip._construct_reset_strategy_sequence("usb-reset")[0], USBJTAGSerialReset)
            chip.get_usb_vid_pid = lambda: recovery.NATIVE_USB
            self.assertIsInstance(chip._construct_reset_strategy_sequence("usb-reset")[0], USBJTAGSerialReset)

    def test_cli_rejects_cp_without_explicit_usb_before_running(self):
        for flags in (["--wifi", "192.0.2.1"], ["--ble", "fixture"], ["--scan-ble"], ["--usb", ""]):
            with patch.object(host, "run") as run, contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    host.main(flags + ["--usb-kind", "cp2102n"])
                self.assertEqual(error.exception.code, 2)
                run.assert_not_called()


class IntegrationTests(unittest.IsolatedAsyncioTestCase):
    async def test_normal_cp_flow_requires_r3_before_command(self):
        for discovery in (r3(), status(), r3(profile="unknown"),
                          dict(firmware="wbb-r3-review-diag", profile=host.R3_PROFILE)):
            stream = SerialFixture([discovery, r3(id=1)])
            with patch("serial.tools.list_ports.comports", return_value=[port()]), \
                    patch.object(host, "open_serial", return_value=stream):
                result = await host.run(args())
            if discovery.get("firmware") == "wbb-control-2" and discovery.get("profile") == host.R3_PROFILE:
                self.assertEqual(result["result"], "ACKNOWLEDGED")
                self.assertTrue(result["app_profile_verified"])
                self.assertFalse(result["custom_pcb_identity_verified"])
                self.assertEqual(json.loads(stream.writes[1])["cmd"], "status")
            else:
                self.assertEqual(result["result"], "FAIL")
                self.assertEqual(stream.writes, [b"hello\n"])
            self.assertTrue(stream.closed)

    async def test_cp_controller_refuses_commands_before_profile_discovery(self):
        wire = FakeWire([])
        client = host.Controller(wire, None, host.R3_PROFILE)
        with self.assertRaisesRegex(host.DiagnosticError, "hello"):
            await client.request("provision")
        self.assertEqual(wire.sent, [])

    async def test_reopen_follows_identity_not_reused_path(self):
        wire = host.Wire(args())
        with patch("serial.tools.list_ports.comports", side_effect=[[port()], [port("/dev/new")], [port(serial="other")]]), \
                patch.object(host, "open_serial", side_effect=lambda _: SerialFixture([])) as opener:
            await wire.open(); await wire.close(); await wire.open(); await wire.close()
            self.assertEqual([call.args[0] for call in opener.call_args_list], ["/dev/cp", "/dev/new"])
            with self.assertRaises(host.DiagnosticError):
                await wire.open()
            self.assertEqual(opener.call_count, 2)

    async def test_rom_without_firmware_reports_chip_not_custom_board(self):
        done = subprocess.CompletedProcess([], 0, "Connected to ESP32-C6\nMAC: 12:34:56:78:9a:bc\n", "")
        def probe_rom(command, **kwargs):
            self.assertEqual(Path(kwargs["env"]["ESPTOOL_CFGFILE"]).read_text(), "[esptool]\n")
            self.assertEqual(kwargs["env"]["ESPTOOL_OPEN_PORT_ATTEMPTS"], "1")
            return done
        with patch("serial.tools.list_ports.comports", return_value=[port()]), \
                patch.object(host, "open_serial") as opener, patch.object(host.subprocess, "run", side_effect=probe_rom) as probe:
            result = await host.run(args("bootloader"))
        self.assertEqual(result["result"], "C6_ROM_VERIFIED")
        self.assertTrue(result["rom_chip_verified"])
        self.assertFalse(result["custom_pcb_identity_verified"])
        self.assertFalse(result["application_running"])
        self.assertNotIn("app_profile_verified", result)
        opener.assert_not_called()
        self.assertEqual(probe.call_args.args[0], recovery.rom_command("/dev/cp", True, "cp2102n"))
        self.assertEqual(probe.call_args.kwargs["timeout"], 15)

    async def test_rom_rejects_wrong_chip_failed_probe_and_bridge_swap(self):
        for code, inventories in ((2, [[port()]]), (0, [[port()], [port(serial="other")]]),
                                  (0, [[port(vid=0x303A, pid=0x1001)]])):
            done = subprocess.CompletedProcess([], code, "", "wrong chip" if code else "")
            with patch("serial.tools.list_ports.comports", side_effect=inventories), \
                    patch.object(host, "open_serial") as opener, patch.object(host.subprocess, "run", return_value=done) as probe:
                result = await host.run(args("bootloader"))
            self.assertEqual(result["result"], "FAIL")
            self.assertNotIn("rom_chip_verified", result)
            opener.assert_not_called()
            if inventories[0][0].vid == 0x303A:
                probe.assert_not_called()

    async def test_reset_requires_r3_idle_app_return_but_does_not_claim_new_boot(self):
        real_wait = host.wait_for_restart
        async def quick_wait(*positional, **kw):
            return await real_wait(*positional, **kw, timeout=.01, interval=0)
        for after in (r3(boot="a" * 16, reset_reason=1), status(boot="a" * 16),
                      r3(uptime_ms=40000), r3(reboot_pending=True), r3(pwm_command=[0, 0, 1])):
            done = subprocess.CompletedProcess([], 0, "C6 ROM", "")
            with patch("serial.tools.list_ports.comports", return_value=[port()]), \
                    patch.object(host.subprocess, "run", return_value=done) as probe, \
                    patch.object(host, "open_serial", side_effect=lambda _: SerialFixture([after])), \
                    patch.object(host, "wait_for_restart", side_effect=quick_wait):
                result = await host.run(args("reset"))
            self.assertEqual(probe.call_args.args[0], recovery.rom_command("/dev/cp", False, "cp2102n"))
            if after["boot"] == "a" * 16 and after["profile"] == host.R3_PROFILE:
                self.assertEqual(result["result"], "APP_RETURN_VERIFIED")
                self.assertFalse(result["new_boot_verified"])
                self.assertFalse(result["prior_app_observed"])
                self.assertFalse(result["custom_pcb_identity_verified"])
            else:
                self.assertEqual(result["result"], "FAIL")

    async def test_software_reboot_verifies_new_boot_device_profile_idle_and_reset_reason(self):
        class RebootWire(FakeWire):
            wireless = False
            target = recovery.identity(port())
            opens = 0
            async def open(self):
                self.opens += 1
                self.replies = ([envelope(r3()), envelope(r3(id=1))] if self.opens == 1
                                else [envelope(r3(boot="a" * 16))])
            async def close(self):
                pass
        wire = RebootWire([])
        with patch.object(host, "Wire", return_value=wire), patch.object(host.asyncio, "sleep", new=AsyncMock()):
            result = await host.run(args("reboot"))
        self.assertEqual(result["result"], "APP_RETURN_VERIFIED")
        self.assertTrue(result["new_boot_verified"])
        commands = [json.loads(data)["cmd"] for data in wire.sent if data != b"hello\n"]
        self.assertEqual(commands, ["reboot"])
        for change in (dict(boot=BOOT), dict(device="another"), dict(profile="top_board_v2"),
                       dict(state="running"), dict(pwm_command=[1, 0, 0]), dict(reset_reason=1)):
            class ReturnWire(FakeWire):
                wireless = False
                async def open(self):
                    self.replies = [envelope(r3(boot="a" * 16) | change)]
                async def close(self):
                    pass
            with self.assertRaisesRegex(host.DiagnosticError, "return NOT verified"):
                await host.wait_for_restart(ReturnWire([]), None, r3(), timeout=.01, interval=0,
                                            expected_profile=host.R3_PROFILE)


if __name__ == "__main__":
    unittest.main()
