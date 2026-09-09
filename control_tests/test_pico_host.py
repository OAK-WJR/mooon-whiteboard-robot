"""Offline Pico CLI/schema tests: fake ports, ROM subprocesses and radio links.

The schema fixture executes the production firmware serializer with its native SDK
boundary in a public-only temporary copy. No serial/USB hardware is accessed.
"""
import configparser
import contextlib
from functools import lru_cache
import io
import json
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import pcb_control as recovery
import robot_control as host
from test_host import FakeWire, KEY, envelope, status
from test_cp2102n_host import SerialFixture, r3
from verify_pico_profile import public_copy


@lru_cache(maxsize=1)
def firmware_frames():
    with tempfile.TemporaryDirectory(prefix="wbb-pico-host-schema-") as folder:
        stage = Path(folder)
        public_copy(stage)  # Real private config is skipped before any read/copy.
        config = configparser.ConfigParser(interpolation=None)
        config.read(stage / "platformio.ini")
        flags = shlex.split(config["env:top_board_v2_pico"]["build_flags"])
        json_include = next((ROOT / ".pio/libdeps").glob("*/ArduinoJson/src"))
        command = ["c++", "-std=c++17", "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
                   "-Icontrol_tests/pico_mock", "-Icontrol_tests/mock", "-Iinclude", "-Isrc", "-I" + str(json_include),
                   "-DCONFIG_IDF_TARGET_ESP32S3=1", *flags, "-MMD", "-MF", "fixture.d",
                   "control_tests/pico_host_status_fixture.cpp", "-o", "fixture"]
        compiled = subprocess.run(command, cwd=stage, capture_output=True, text=True, timeout=60)
        if compiled.returncode:
            raise AssertionError(compiled.stdout + compiled.stderr)
        assert "include/config.h" not in (stage / "fixture.d").read_text()
        frames = {}
        for mode in ("normal", "large", "fault", "acquiring", "stale", "init_failed",
                     "rails_ready", "rails_large", "rails_expired", "rails_sampling", "rails_fault", "rails_cleanup"):
            result = subprocess.run([stage / "fixture", mode], cwd=stage, capture_output=True, text=True, timeout=10, check=True)
            if not result.stdout.endswith("\n") or len(result.stdout.encode("utf-8")) >= 2304:
                raise AssertionError(f"{mode}: firmware response exceeds Pico line bound or lacks newline")
            frames[mode] = json.loads(result.stdout)
        return frames


def pico(mode="normal", **changes):
    frame = json.loads(firmware_frames()[mode]["payload"])
    frame.update(changes)
    return frame


def port(path="/dev/pico", serial="pico-bridge-A", location="1-2.3", **changes):
    fields = dict(device=path, serial_number=serial, location=location, vid=0x1A86, pid=0x55D3)
    fields.update(changes)
    return SimpleNamespace(**fields)


def args(action="status", **changes):
    fields = dict(action=action, usb="/dev/pico", usb_kind="ch343p", profile=host.PICO_PROFILE,
                  wifi=None, ble=None, scan_ble=False, key_file=None, wifi_config=None, seconds=.1, params="{}")
    fields.update(changes)
    return SimpleNamespace(**fields)


class SchemaTests(unittest.TestCase):
    def test_accepts_actual_normal_large_gpio_fault_and_watchdog_envelopes(self):
        for mode, wire in firmware_frames().items():
            frame = host.unpack(wire, wireless=False, expected_profile=host.PICO_PROFILE)
            self.assertIs(host.check_status(frame, host.PICO_PROFILE), frame)
            self.assertFalse(frame["motion_enabled"])
            self.assertFalse(frame["source_cycle_enabled"])
            signed = envelope(frame)
            self.assertLess(len((json.dumps(signed, separators=(",", ":")) + "\n").encode()), 2304)
            self.assertEqual(host.unpack(signed, KEY, expected_profile=host.PICO_PROFILE), frame)
        self.assertGreater(len(firmware_frames()["large"]["payload"].encode()), 1200)

    def test_actual_monitor_samples_map_channels_and_preserve_null_unmeasured_rails(self):
        for mode in ("rails_ready", "rails_large"):
            with self.subTest(mode=mode):
                frame = host.check_status(pico(mode), host.PICO_PROFILE)
                self.assertEqual([frame[field] for field in ("raw_logic_5v_mv", "usb_vbus_mv", "motor_voltage_mv")],
                                 [5000, 5000, 0])
                self.assertEqual(frame["rail_monitor"]["state"], "ready")
                self.assertEqual(frame["rail_monitor"]["bus"], "ready")
                self.assertEqual(frame["rail_monitor"]["error"], "none")
                self.assertEqual(frame["rail_monitor"]["seq"], 1)
                self.assertIsNone(frame["motor_current_ma"])
                self.assertIsNone(frame["rail_3v3_mv"])
                self.assertEqual(host.unpack(envelope(frame), KEY, expected_profile=host.PICO_PROFILE), frame)

    def test_actual_expiration_sampling_and_faults_invalidate_complete_generation(self):
        for mode, state, error in (("rails_expired", "expired", "none"), ("rails_sampling", "sampling", "none"),
                                   ("rails_fault", "fault", "nack"), ("rails_cleanup", "fault", "cleanup_failed")):
            with self.subTest(mode=mode):
                frame = host.check_status(pico(mode), host.PICO_PROFILE)
                self.assertEqual(frame["rail_monitor"]["state"], state)
                self.assertEqual(frame["rail_monitor"]["error"], error)
                for field in ("motor_voltage_mv", "raw_logic_5v_mv", "usb_vbus_mv"):
                    self.assertIsNone(frame[field])
                for field in ("seq", "age_us", "span_us"):
                    self.assertIsNone(frame["rail_monitor"][field])
        self.assertEqual(pico("rails_cleanup")["rail_monitor"]["bus"], "cleanup_fault")

    def test_pico_selection_rejects_r3_legacy_diagnostic_and_implicit_pico(self):
        for frame in (r3(), status(), pico(profile="top_board_v2_pico"), pico(firmware="wbb-r3-review-diag")):
            with self.subTest(frame=frame.get("profile")), self.assertRaises(host.DiagnosticError):
                host.check_status(frame, host.PICO_PROFILE)
        for expected in (None, host.R3_PROFILE, "top_board_v2"):
            with self.assertRaises(host.DiagnosticError):
                host.check_status(pico(), expected)

    def test_unqualified_capabilities_cannot_be_omitted_coerced_or_enabled(self):
        fields = ("motion_enabled", "charge_enabled", "power_disconnect", "pwm_ready", "motor_arm_requested",
                  "source_cycle_enabled", "odometry_qualified")
        for field in fields:
            for value in (True, 0, 1, None, "false"):
                with self.subTest(field=field, value=value), self.assertRaises(host.DiagnosticError):
                    host.check_status(pico(**{field: value}), host.PICO_PROFILE)
            missing=pico(); del missing[field]
            with self.assertRaises(host.DiagnosticError):
                host.check_status(missing, host.PICO_PROFILE)

    def test_watchdog_service_readiness_is_explicit_and_never_qualifies_motion(self):
        for mode in ("normal", "large", "fault", "acquiring", "stale", "init_failed"):
            frame = host.check_status(pico(mode), host.PICO_PROFILE)
            self.assertEqual(frame["external_watchdog_ready"], mode in ("normal", "large"))
            self.assertFalse(frame["motion_enabled"])
            self.assertFalse(frame["source_cycle_enabled"])
        for changes in (dict(external_watchdog_ready=False), dict(external_watchdog_ready=1),
                        dict(external_watchdog_state="stale"), dict(external_watchdog_state=None),
                        dict(external_watchdog_state="gpio_fault"), dict(watchdog_ready=False),
                        dict(control_timing_fault=True)):
            with self.subTest(changes=changes), self.assertRaises(host.DiagnosticError):
                host.check_status(pico(**changes), host.PICO_PROFILE)
        for mode in ("acquiring", "stale", "init_failed"):
            with self.assertRaises(host.DiagnosticError):
                host.check_status(pico(mode, external_watchdog_ready=True), host.PICO_PROFILE)
            with self.assertRaises(host.DiagnosticError):
                host.verify_restart(pico(), pico(mode, boot="a"*16, reset_reason=3), expected_profile=host.PICO_PROFILE)

    def test_unavailable_telemetry_must_remain_present_and_null(self):
        for field in ("motor_current_ma", "rail_3v3_mv", "x", "y", "theta"):
            for value in (0, 5000, False, "unknown"):
                with self.subTest(field=field, value=value), self.assertRaises(host.DiagnosticError):
                    host.check_status(pico(**{field: value}), host.PICO_PROFILE)
            missing=pico();del missing[field]
            with self.assertRaises(host.DiagnosticError):
                host.check_status(missing, host.PICO_PROFILE)
        for changes in (dict(wheel_speed_mps=[0,0,0]), dict(motor_pwm_ready=[False,False,True]),
                        dict(motor_pwm_ready=[0,0,0]), dict(pwm_command=[1,0,0]), dict(pwm_command=[False,0,0]),
                        dict(physical_motors=[True,2,3]), dict(encoder_counts=[0,0,0x80000000]),
                        dict(encoder_counts=[None,None,None]), dict(state="running"), dict(usb_reboot=False)):
            with self.subTest(changes=changes), self.assertRaises(host.DiagnosticError):
                host.check_status(pico(**changes), host.PICO_PROFILE)

    def test_raw_logic_bits_are_not_power_qualification(self):
        for flt in (0,1):
            for pg in (0,1):
                frame=host.check_status(pico(logic_flt_n=flt,logic_pgood=pg),host.PICO_PROFILE)
                self.assertFalse(frame["source_cycle_enabled"])
                self.assertFalse(frame["power_disconnect"])
        for field in ("logic_flt_n","logic_pgood"):
            for value in (True,False,None,2,"1"):
                with self.assertRaises(host.DiagnosticError):
                    host.check_status(pico(**{field:value}),host.PICO_PROFILE)
        for changes in (dict(logic_pgood=1),dict(encoder_counts=[0,0,0]),dict(read_only=False),
                        dict(gpio_fault_latched=False),dict(gpio_error=0),dict(gpio_release_error=None)):
            with self.assertRaises(host.DiagnosticError):
                host.check_status(pico("fault",**changes),host.PICO_PROFILE)

    def test_larger_payload_is_profile_bound_byte_limited_and_authenticated(self):
        frame=pico("large")
        self.assertEqual(host.unpack(envelope(frame),KEY,expected_profile=host.PICO_PROFILE),frame)
        for expected in (None,host.R3_PROFILE,"top_board_v2"):
            with self.assertRaises(host.DiagnosticError):
                host.unpack(envelope(frame),KEY,expected_profile=expected)
        with self.assertRaises(host.DiagnosticError):
            host.unpack(dict(envelope(frame),mac="0"*64),KEY,expected_profile=host.PICO_PROFILE)
        for length in (1792,1793):
            text=firmware_frames()["large"]["payload"]
            text += " "*(length-len(text.encode()))
            with self.assertRaisesRegex(host.DiagnosticError,"Oversized"):
                host.unpack(dict(payload=text,mac=""),wireless=False,expected_profile=host.PICO_PROFILE)
        with self.assertRaisesRegex(host.DiagnosticError,"Oversized"):
            host.unpack(dict(payload="é"*896,mac=""),wireless=False,expected_profile=host.PICO_PROFILE)


class SelectionTests(unittest.TestCase):
    def test_ch343_requires_exact_explicit_unique_identity(self):
        self.assertIsNotNone(recovery.select_port([port()],"/dev/pico","ch343p"))
        for requested, inventory in ((None,[port()]),("",[port()]),("rfc2217://fixture",[port()]),
                ("/dev/pico",[port(pid=0x55D4)]),("/dev/pico",[port(vid=0x10C4,pid=0xEA60)]),
                ("/dev/pico",[port(vid=0x303A,pid=0x1001)]),("/dev/pico",[port(),port("/dev/clone")]),
                ("/dev/pico",[port(serial=None,location=None)])):
            with self.subTest(requested=requested,inventory=inventory), self.assertRaises(host.DiagnosticError):
                recovery.select_port(inventory,requested,"ch343p")

    def test_selection_binds_pico_for_usb_or_explicit_wireless(self):
        self.assertEqual(host.Wire(args(profile=None)).expected_profile,host.PICO_PROFILE)
        for link in (dict(wifi="192.0.2.1"),dict(ble="fixture")):
            self.assertEqual(host.Wire(args(usb=None,usb_kind="native",**link)).expected_profile,host.PICO_PROFILE)
        for changes in (dict(profile=host.R3_PROFILE),dict(profile="top_board_v2"),dict(usb_kind="native"),
                        dict(usb_kind="cp2102n"),dict(usb=None,wifi="192.0.2.1")):
            with self.assertRaises(host.DiagnosticError):
                host.Wire(args(**changes))

    def test_cli_mismatch_and_motion_rejected_before_run(self):
        bad = (["--usb","/dev/pico","--usb-kind","ch343p","--profile",host.R3_PROFILE],
               ["--wifi","192.0.2.1","--usb-kind","ch343p"],
               ["--usb","/dev/pico","--profile",host.PICO_PROFILE],
               ["--usb","/dev/pico","--usb-kind","ch343p","start"])
        for flags in bad:
            with patch.object(host,"run") as run,contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                host.main(flags)
            run.assert_not_called()

    def test_rom_recipe_is_s3_uart_no_flash_and_installed_strategy_is_uart(self):
        for stay in (True,False):
            command=recovery.rom_command("/dev/pico",stay,"ch343p")
            self.assertEqual(command[command.index("--chip")+1],"esp32s3")
            self.assertEqual(command[command.index("--before")+1],"default-reset")
            self.assertEqual(command[command.index("--after")+1],"no-reset" if stay else "hard-reset")
            self.assertEqual(command[-2:],["--no-stub","read-mac"])
            self.assertFalse(any("flash" in part for part in command))
        import esptool.loader as loader
        from esptool.targets.esp32s3 import ESP32S3ROM
        from esptool.reset import ClassicReset,UnixTightReset
        chip=object.__new__(ESP32S3ROM);chip._port=SimpleNamespace(name="/dev/pico")
        chip.get_usb_vid_pid=lambda:recovery.CH343P_USB
        config=configparser.ConfigParser();config["esptool"]={}
        with patch.object(loader,"cfg",config["esptool"]):
            self.assertTrue(all(isinstance(s,(ClassicReset,UnixTightReset)) for s in chip._construct_reset_strategy_sequence("default-reset")))

    def test_serial_opens_with_flow_control_and_reset_lines_disabled(self):
        fake=SimpleNamespace(open=lambda:None)
        with patch("serial.Serial",return_value=fake) as create:
            self.assertIs(recovery.open_serial("/dev/pico"),fake)
        self.assertFalse(fake.dtr);self.assertFalse(fake.rts)
        for field in ("rtscts","dsrdtr","xonxoff"):
            self.assertIs(create.call_args.kwargs[field],False)


class IntegrationTests(unittest.IsolatedAsyncioTestCase):
    async def test_real_wire_validates_actual_rail_samples_and_invalidations(self):
        modes = ("rails_ready", "rails_large", "rails_expired", "rails_sampling", "rails_fault", "rails_cleanup")
        frames = [pico(mode, id=index) for index, mode in enumerate(modes)]
        stream = SerialFixture(frames)
        wire = host.Wire(args())
        with patch("serial.tools.list_ports.comports", return_value=[port()]), patch.object(host, "open_serial", return_value=stream):
            await wire.open()
        try:
            client = host.Controller(wire, None)
            self.assertEqual(await client.hello(), frames[0])
            for frame in frames[1:]:
                self.assertEqual(await client.request("status"), frame)
        finally:
            await wire.close()
        self.assertTrue(stream.closed)

    async def test_real_wire_status_stop_watch_accept_full_pico_frames_only(self):
        for action in ("status","stop","watch"):
            stream=SerialFixture([pico(),pico("large",id=1)])
            with patch("serial.tools.list_ports.comports",return_value=[port()]),patch.object(host,"open_serial",return_value=stream):
                result=await host.run(args(action))
            self.assertEqual(result["result"],"CAPTURED" if action=="watch" else "ACKNOWLEDGED")
            self.assertTrue(result["app_profile_verified"])
            self.assertFalse(result["custom_pcb_identity_verified"])
            for field in ("motion_enabled","charge_enabled","source_cycle_enabled","physical_power_cycle","hardware_power_measured"):
                self.assertFalse(result[field])
            self.assertTrue(stream.closed)
        for frame in (r3(),status(),pico(source_cycle_enabled=True)):
            stream=SerialFixture([frame])
            with patch("serial.tools.list_ports.comports",return_value=[port()]),patch.object(host,"open_serial",return_value=stream):
                result=await host.run(args())
            self.assertEqual(result["result"],"FAIL")
            self.assertEqual(stream.writes,[b"hello\n"])

    async def test_gpio_fault_is_diagnosable_without_claiming_readiness(self):
        stream=SerialFixture([pico("fault"),pico("fault",id=1)])
        with patch("serial.tools.list_ports.comports",return_value=[port()]),patch.object(host,"open_serial",return_value=stream):
            result=await host.run(args())
        self.assertEqual(result["result"],"ACKNOWLEDGED")
        self.assertEqual(result["after"]["state"],"gpio_fault")
        self.assertFalse(result["source_cycle_enabled"])

    async def test_no_command_before_hello_and_no_profile_override_or_motion(self):
        wire=FakeWire([envelope(pico())]);wire.expected_profile=host.PICO_PROFILE
        with self.assertRaises(host.DiagnosticError):
            host.Controller(wire,KEY,host.R3_PROFILE)
        client=host.Controller(wire,KEY)
        with self.assertRaises(host.DiagnosticError):
            await client.request("provision")
        self.assertFalse(wire.sent)
        await client.hello()
        for cmd in host.PICO_BLOCKED_COMMANDS:
            with self.assertRaises(host.CommandRejected):
                await client.request(cmd,{"usb_vbus_mv":5000,"raw_logic_5v_mv":0})
        self.assertEqual(wire.sent,[b"hello\n"])

    async def test_wrong_profile_status_and_reconnect_never_pass_for_pico(self):
        boot=pico()["boot"]
        for wrong in (r3(id=1,boot=boot),status(id=1,boot=boot),pico(id=1,device="other")):
            wire=FakeWire([envelope(pico()),envelope(wrong)])
            client=host.Controller(wire,KEY,host.PICO_PROFILE);await client.hello()
            with self.assertRaises(host.DiagnosticError):await client.request("status")
        for wrong in (r3(boot="a"*16),status(boot="a"*16),pico(),pico(boot="a"*16,device="other"),
                      pico("fault",boot="a"*16),pico(boot="a"*16,reset_reason=1)):
            class ReturnWire(FakeWire):
                async def open(self):self.replies=[envelope(wrong)]
                async def close(self):pass
            with self.assertRaisesRegex(host.DiagnosticError,"return NOT verified"):
                await host.wait_for_restart(ReturnWire([]),KEY,pico(),timeout=.01,interval=0,expected_profile=host.PICO_PROFILE)

    async def test_real_wire_reconnect_follows_bridge_identity_not_reused_path(self):
        wire=host.Wire(args())
        with patch("serial.tools.list_ports.comports",side_effect=[[port()],[port("/dev/renamed")],[port(serial="other")]]), \
                patch.object(host,"open_serial",side_effect=lambda _:SerialFixture([])) as opener:
            await wire.open();await wire.close();await wire.open();await wire.close()
            with self.assertRaises(host.DiagnosticError):await wire.open()
        self.assertEqual([call.args[0] for call in opener.call_args_list],["/dev/pico","/dev/renamed"])

    async def test_software_reboot_lost_ack_and_provision_return_require_pico(self):
        for action,lost in (("reboot",False),("reboot",True),("provision",False)):
            class RebootWire(FakeWire):
                wireless=False
                expected_profile=host.PICO_PROFILE
                target=recovery.identity(port())
                opens=0
                async def open(self):
                    self.opens+=1
                    if self.opens==1:
                        self.replies=[envelope(pico())]
                        if action=="provision":self.replies.append(envelope(pico(id=1)))
                        self.replies.append(host.DiagnosticError("fixture lost ACK") if lost else envelope(pico(id=2 if action=="provision" else 1)))
                    else:self.replies=[envelope(pico(boot="a"*16,reset_reason=3))]
                async def recv(self,timeout=3):
                    value=await super().recv(timeout)
                    if isinstance(value,Exception):raise value
                    return value
                async def close(self):pass
            wire=RebootWire([])
            with tempfile.TemporaryDirectory() as folder:
                key_file=Path(folder)/"new-control-key";wifi_file=Path(folder)/"wifi.json"
                wifi_file.write_text(json.dumps(dict(ssid="public-fixture",password="fixture-password")))
                with patch.object(host,"Wire",return_value=wire),patch.object(host.asyncio,"sleep",new=AsyncMock()):
                    result=await host.run(args(action,key_file=key_file,wifi_config=wifi_file))
                if action=="provision":
                    self.assertEqual(key_file.stat().st_mode & 0o777,0o600)
                    self.assertEqual(len(bytes.fromhex(key_file.read_text().strip())),32)
            self.assertEqual(result["result"],"APP_RETURN_VERIFIED")
            self.assertTrue(result["new_boot_verified"])
            self.assertEqual(result["reboot_ack_received"],not lost)
            self.assertFalse(result["physical_power_cycle"]);self.assertFalse(result["source_cycle_enabled"])
            commands=[json.loads(line)["cmd"] for line in wire.sent if line!=b"hello\n"]
            self.assertEqual(commands,["provision","reboot"] if action=="provision" else ["reboot"])

    async def test_wrong_profile_provision_creates_no_key_and_sends_no_secret(self):
        stream=SerialFixture([r3()])
        with tempfile.TemporaryDirectory() as folder:
            key=Path(folder)/"not-created";wifi=Path(folder)/"wifi.json"
            wifi.write_text(json.dumps(dict(ssid="fixture",password="fixture-password")))
            with patch("serial.tools.list_ports.comports",return_value=[port()]),patch.object(host,"open_serial",return_value=stream):
                result=await host.run(args("provision",key_file=key,wifi_config=wifi))
            self.assertFalse(key.exists())
        self.assertEqual(result["result"],"FAIL");self.assertEqual(stream.writes,[b"hello\n"])

    async def test_s3_rom_recipe_reports_no_app_or_physical_power_proof(self):
        done=subprocess.CompletedProcess([],0,"ESP32-S3 fixture ROM"," ")
        def rom(command,**kwargs):
            self.assertEqual(command,recovery.rom_command("/dev/pico",True,"ch343p"))
            self.assertEqual(Path(kwargs["env"]["ESPTOOL_CFGFILE"]).read_text(),"[esptool]\n")
            return done
        with patch("serial.tools.list_ports.comports",return_value=[port()]),patch.object(host.subprocess,"run",side_effect=rom), \
                patch.object(host,"open_serial") as opener:
            result=await host.run(args("bootloader"))
        self.assertEqual(result["result"],"S3_ROM_VERIFIED");self.assertEqual(result["rom_chip"],"esp32s3")
        self.assertFalse(result["app_profile_verified"]);self.assertFalse(result["custom_pcb_identity_verified"])
        self.assertFalse(result["physical_power_cycle"]);self.assertFalse(result["source_cycle_enabled"])
        opener.assert_not_called()


if __name__ == "__main__":
    unittest.main()
