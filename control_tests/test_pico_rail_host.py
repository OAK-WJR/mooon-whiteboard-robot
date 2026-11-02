"""INA host acceptance with in-memory frames/transports; no native build or I/O.

The separate test_pico_host suite checks the actual firmware serializer. These
explicit contract examples keep adversarial checks independent of that fixture.
"""
import hashlib
import hmac
import itertools
import json
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from test_host import KEY, FakeWire, envelope, status
from test_cp2102n_host import SerialFixture, r3
from test_pico_host import args, host, port


VOLTAGES = ("motor_voltage_mv", "raw_logic_5v_mv", "usb_vbus_mv")
METADATA = ("seq", "age_us", "span_us")
NONREADY = ("not_started", "starting", "initializing", "sampling", "expired", "fault", "stopped")
ERRORS = ("start_failed", "setup_timeout", "bus_fault", "cleanup_failed", "invalid_address",
          "wrong_manufacturer", "wrong_die", "nack", "bus_error", "short_read", "bus_busy_timeout",
          "transfer_timeout", "conversion_timeout", "acquisition_timeout", "configuration_changed", "invalid_voltage")
BUS_STATES = ("stopped", "starting", "ready", "fault", "stopping", "cleanup_fault")


def rail_status(rail_state="ready", **changes):
    ready = rail_state == "ready"
    bus = {"not_started": "stopped", "starting": "starting", "initializing": "ready",
           "sampling": "ready", "ready": "ready", "expired": "ready", "fault": "fault", "stopped": "stopped"}
    frame = status(
        profile=host.PICO_PROFILE, charge_enabled=False, power_disconnect=False,
        cell_monitor="not_started", cell_error="none",
        pwm_ready=False, motor_pwm_ready=[False, False, False], motor_arm_requested=False,
        source_cycle_enabled=False, odometry_qualified=False, motor_current_ma=None, rail_3v3_mv=None,
        motor_voltage_mv=6016 if ready else None, raw_logic_5v_mv=5000 if ready else None,
        usb_vbus_mv=4992 if ready else None, x=None, y=None, theta=None,
        wheel_speed_mps=[None, None, None], encoder_counts=[0, 0, 0], logic_flt_n=1, logic_pgood=1,
        usb_reboot=True, external_watchdog_ready=True, external_watchdog_state="active", control_timing_fault=False,
        rail_monitor=dict(state=rail_state, error="nack" if rail_state == "fault" else "none", bus=bus[rail_state],
                          seq=7 if ready else None, age_us=12000 if ready else None, span_us=3000 if ready else None))
    frame.update(changes)
    return frame


def gpio_fault_status():
    return rail_status("stopped", state="gpio_fault", external_watchdog_state="gpio_fault",
                       external_watchdog_ready=False, watchdog_ready=False, gpio_fault_latched=True,
                       read_only=True, wireless_control_enabled=False, telemetry_available=False,
                       reason="PICO_SAFE_DEFAULTS_FAILED", gpio_error=-42, gpio_release_error=0,
                       logic_flt_n=None, logic_pgood=None, encoder_counts=[None, None, None])


def padded_envelope(frame, length, *, unicode=False):
    value = dict(frame)
    if unicode:
        value["note"] = "\u7535\u538b\u00e9"
    payload = json.dumps(value, ensure_ascii=not unicode, separators=(",", ":"))
    size = len(payload.encode("utf-8"))
    if size > length:
        raise AssertionError(f"Fixture payload {size} exceeds requested {length}")
    payload += " " * (length - size)
    return dict(payload=payload, mac=hmac.new(KEY, payload.encode("utf-8"), hashlib.sha256).hexdigest())


class EncodedSerial(SerialFixture):
    """Feed exact wire bytes, including whitespace and deliberately bad MACs."""
    def __init__(self, replies, chunk_size=17):
        super().__init__(replies)
        self.chunk_size = chunk_size

    def write(self, data):
        self.writes.append(data)
        if self.replies:
            self.pending.extend(self.replies.pop(0))
        return len(data)

    def read(self, count):
        return super().read(min(count, self.chunk_size))


def encoded(wire):
    return (json.dumps(wire, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")


class RailSchemaTests(unittest.TestCase):
    def accept(self, frame):
        self.assertIs(host.check_status(frame, host.PICO_PROFILE), frame)

    def reject(self, frame):
        with self.assertRaises(host.DiagnosticError):
            host.check_status(frame, host.PICO_PROFILE)

    def test_accepts_voltage_endpoints_and_independent_channel_values(self):
        for values in ((0, 0, 0), (8, 4992, 26000), (26000, 26000, 26000), (6016, 5000, 4992)):
            with self.subTest(values=values):
                self.accept(rail_status(**dict(zip(VOLTAGES, values))))

    def test_accepts_sequence_and_timing_boundaries(self):
        for seq, (age, span) in itertools.product((1, 0xffffffff), ((0, 0), (1, 1), (29999, 29999), (49999, 29999))):
            frame = rail_status()
            frame["rail_monitor"].update(seq=seq, age_us=age, span_us=span)
            with self.subTest(seq=seq, age=age, span=span):
                self.accept(frame)

    def test_accepts_all_nonready_states_and_known_errors(self):
        for state in NONREADY:
            with self.subTest(state=state):
                self.accept(rail_status(state))
        for error, bus in itertools.product(ERRORS, BUS_STATES):
            frame = rail_status("fault")
            frame["rail_monitor"].update(error=error, bus=bus)
            with self.subTest(error=error, bus=bus):
                self.accept(frame)

    def test_requires_nested_monitor_object(self):
        missing = rail_status()
        del missing["rail_monitor"]
        self.reject(missing)
        for value in (None, False, True, 0, 1, "ready", [], [rail_status()["rail_monitor"]], {}):
            with self.subTest(value=value):
                self.reject(rail_status(rail_monitor=value))

    def test_every_monitor_field_is_required_even_when_null(self):
        for state, field in itertools.product(("ready", *NONREADY), ("state", "error", "bus", *METADATA)):
            frame = rail_status(state)
            del frame["rail_monitor"][field]
            with self.subTest(state=state, field=field):
                self.reject(frame)

    def test_enums_are_exact_known_strings(self):
        for field, value in itertools.product(("state", "error", "bus"), (None, True, False, 0, 1, [], {}, "", "READY", "unknown")):
            frame = rail_status()
            frame["rail_monitor"][field] = value
            with self.subTest(field=field, value=value):
                self.reject(frame)

    def test_voltage_fields_are_required_in_every_state(self):
        for state, field in itertools.product(("ready", *NONREADY), VOLTAGES):
            frame = rail_status(state)
            del frame[field]
            with self.subTest(state=state, field=field):
                self.reject(frame)

    def test_no_partial_mixed_or_all_null_ready_snapshot(self):
        for present in itertools.product((False, True), repeat=3):
            if all(present):
                continue
            frame = rail_status()
            for field, available in zip(VOLTAGES, present):
                if not available:
                    frame[field] = None
            with self.subTest(present=present):
                self.reject(frame)
        for field in METADATA:
            frame = rail_status()
            frame["rail_monitor"][field] = None
            with self.subTest(field=field):
                self.reject(frame)

    def test_voltage_types_ranges_and_eight_mv_quantization(self):
        invalid = (True, False, -8, -1, 1, 7, 9, 4999, 26001, 26008, 32760, 0x100000000,
                   0.0, 5000.0, "5000", [], {}, float("nan"), float("inf"))
        for field, value in itertools.product(VOLTAGES, invalid):
            with self.subTest(field=field, value=value):
                self.reject(rail_status(**{field: value}))

    def test_sample_metadata_rejects_bool_coercion_and_wrong_types(self):
        for field, value in itertools.product(METADATA, (True, False, 1.0, "1", [], {}, float("nan"), float("inf"))):
            frame = rail_status()
            frame["rail_monitor"][field] = value
            with self.subTest(field=field, value=value):
                self.reject(frame)

    def test_sample_metadata_range_and_expiration_are_exclusive(self):
        for field, values in (("seq", (-1, 0, 0x100000000)), ("age_us", (-1, 50000, 50001, 0xffffffff)),
                              ("span_us", (-1, 30000, 30001, 0xffffffff))):
            for value in values:
                frame = rail_status()
                frame["rail_monitor"].update(age_us=49999)
                frame["rail_monitor"][field] = value
                with self.subTest(field=field, value=value):
                    self.reject(frame)

    def test_acquisition_span_cannot_exceed_trigger_age(self):
        for age, span in ((0, 1), (100, 101), (29998, 29999)):
            frame = rail_status()
            frame["rail_monitor"].update(age_us=age, span_us=span)
            with self.subTest(age=age, span=span):
                self.reject(frame)

    def test_nonready_rejects_each_leaked_value_or_metadata(self):
        for state, field, value in itertools.product(NONREADY, (*VOLTAGES, *METADATA), (0, 8, True, False, "0")):
            frame = rail_status(state)
            (frame if field in VOLTAGES else frame["rail_monitor"])[field] = value
            with self.subTest(state=state, field=field, value=value):
                self.reject(frame)

    def test_expired_or_resampling_cannot_keep_previous_complete_snapshot(self):
        for state in NONREADY:
            frame = rail_status()
            frame["rail_monitor"]["state"] = state
            frame["rail_monitor"]["error"] = "nack" if state == "fault" else "none"
            with self.subTest(state=state):
                self.reject(frame)

    def test_fault_requires_error_and_nonfault_including_stopped_forbids_it(self):
        frame = rail_status("fault")
        frame["rail_monitor"]["error"] = "none"
        self.reject(frame)
        for state, error in itertools.product(("ready", *NONREADY), ERRORS):
            if state == "fault":
                continue
            frame = rail_status(state)
            frame["rail_monitor"]["error"] = error
            with self.subTest(state=state, error=error):
                self.reject(frame)

    def test_ready_requires_ready_bus(self):
        for bus in BUS_STATES:
            if bus == "ready":
                continue
            frame = rail_status()
            frame["rail_monitor"]["bus"] = bus
            with self.subTest(bus=bus):
                self.reject(frame)

    def test_watchdog_failures_and_gpio_fault_forbid_ready_rails(self):
        for watchdog in ("stale", "init_failed"):
            changes = dict(external_watchdog_state=watchdog, external_watchdog_ready=False)
            with self.subTest(watchdog=watchdog):
                self.accept(rail_status("stopped", **changes))
                self.reject(rail_status(**changes))
        frame = gpio_fault_status()
        self.accept(frame)
        ready = rail_status()
        frame.update({field: ready[field] for field in (*VOLTAGES, "rail_monitor")})
        self.reject(frame)
        self.reject(rail_status(gpio_fault_latched=True))

    def test_reboot_cannot_report_ready_and_requires_explicit_false(self):
        self.accept(rail_status("stopped", reboot_pending=True))
        for value in (True, None, 0, 1, "false", [], {}):
            with self.subTest(value=value):
                self.reject(rail_status(reboot_pending=value))
        frame = rail_status()
        del frame["reboot_pending"]
        self.reject(frame)

    def test_watchdog_acquisition_is_distinct_from_monitor_readiness(self):
        for state in ("startup", "acquiring"):
            self.accept(rail_status(external_watchdog_state=state, external_watchdog_ready=False))

    def test_measured_voltages_never_relax_capabilities_or_other_measurements(self):
        for field in ("motion_enabled", "charge_enabled", "power_disconnect", "pwm_ready", "motor_arm_requested",
                      "source_cycle_enabled", "odometry_qualified"):
            for value in (True, 0, 1, None, "false"):
                with self.subTest(field=field, value=value):
                    self.reject(rail_status(**{field: value}))
        for field, value in itertools.product(("motor_current_ma", "rail_3v3_mv", "x", "y", "theta"), (0, 5000, False, "unknown")):
            with self.subTest(field=field, value=value):
                self.reject(rail_status(**{field: value}))
        for changes in (dict(pwm_command=[1, 0, 0]), dict(motor_pwm_ready=[True, False, False]),
                        dict(wheel_speed_mps=[0, 0, 0])):
            self.reject(rail_status(**changes))

    def test_r3_and_legacy_keep_their_existing_contract(self):
        for frame in (r3(), status()):
            self.assertNotIn("rail_monitor", frame)
            self.assertIs(host.check_status(frame, frame["profile"]), frame)
        for value in (0, 8, 26000, True):
            with self.subTest(value=value), self.assertRaises(host.DiagnosticError):
                host.check_status(r3(motor_voltage_mv=value), host.R3_PROFILE)
        with self.assertRaises(host.DiagnosticError):
            host.check_status(rail_status())


class RailEnvelopeTests(unittest.TestCase):
    def test_new_payload_capacity_accepts_exact_bytes_on_usb_and_hmac(self):
        for length, wireless, unicode in itertools.product((1535, 1536, 1537, 1791), (False, True), (False, True)):
            wire = padded_envelope(rail_status(), length, unicode=unicode)
            with self.subTest(length=length, wireless=wireless, unicode=unicode):
                self.assertEqual(len(wire["payload"].encode("utf-8")), length)
                result = host.unpack(wire, KEY, wireless=wireless, expected_profile=host.PICO_PROFILE)
                self.assertEqual(result["rail_monitor"], rail_status()["rail_monitor"])
                host.check_status(result, host.PICO_PROFILE)

    def test_payload_bound_is_exclusive_and_counts_utf8_bytes(self):
        for length, wireless, unicode in itertools.product((1792, 1793), (False, True), (False, True)):
            wire = padded_envelope(rail_status(), length, unicode=unicode)
            with self.subTest(length=length, wireless=wireless, unicode=unicode), self.assertRaisesRegex(host.DiagnosticError, "Oversized"):
                host.unpack(wire, KEY, wireless=wireless, expected_profile=host.PICO_PROFILE)

    def test_larger_frames_cannot_self_select_pico_limits(self):
        wire = padded_envelope(rail_status(), 1536)
        for expected, wireless in itertools.product((None, host.R3_PROFILE, "top_board_v2"), (False, True)):
            with self.subTest(expected=expected, wireless=wireless), self.assertRaisesRegex(host.DiagnosticError, "Oversized"):
                host.unpack(wire, KEY, wireless=wireless, expected_profile=expected)

    def test_r3_and_legacy_payload_limits_remain_1200(self):
        for frame, length, wireless in itertools.product((r3(), status()), (1199, 1200, 1201), (False, True)):
            wire = padded_envelope(frame, length)
            with self.subTest(profile=frame["profile"], length=length, wireless=wireless):
                if length < 1200:
                    self.assertEqual(host.unpack(wire, KEY, wireless, frame["profile"]), frame)
                else:
                    with self.assertRaisesRegex(host.DiagnosticError, "Oversized"):
                        host.unpack(wire, KEY, wireless, frame["profile"])

    def test_hmac_covers_voltage_metadata_and_exact_serialized_whitespace(self):
        wire = padded_envelope(rail_status(), 1791)
        changes = (("6016", "6024"), ('"seq":7', '"seq":8'), ('"age_us":12000', '"age_us":12001'))
        bad = [dict(wire, payload=wire["payload"].replace(old, new)) for old, new in changes]
        bad += [dict(wire, payload=wire["payload"].rstrip()), dict(wire, mac="0" * 64),
                dict(wire, mac=None), dict(wire, mac=True), dict(payload=wire["payload"])]
        for altered in bad:
            with self.subTest(altered=altered.get("mac")), self.assertRaisesRegex(host.DiagnosticError, "authentication"):
                host.unpack(altered, KEY, expected_profile=host.PICO_PROFILE)
        for key in (None, b"x" * 32):
            with self.assertRaisesRegex(host.DiagnosticError, "authentication"):
                host.unpack(wire, key, expected_profile=host.PICO_PROFILE)

    def test_usb_envelope_does_not_require_a_key_but_still_requires_schema(self):
        frame = rail_status()
        wire = dict(payload=envelope(frame)["payload"], mac="")
        self.assertEqual(host.unpack(wire, wireless=False, expected_profile=host.PICO_PROFILE), frame)
        for malformed in (frame, None, [], dict(payload=frame), dict(payload=True)):
            with self.subTest(malformed=type(malformed)), self.assertRaises(host.DiagnosticError):
                host.unpack(malformed, wireless=False, expected_profile=host.PICO_PROFILE)
        for changes in (dict(id=True), dict(ok=1)):
            with self.assertRaises(host.DiagnosticError):
                host.unpack(envelope(rail_status(**changes)), wireless=False, expected_profile=host.PICO_PROFILE)


class RailIntegrationTests(unittest.IsolatedAsyncioTestCase):
    async def test_fragmented_usb_hello_status_stop_pass_through_real_wire_and_controller(self):
        frames = [rail_status(), rail_status(id=1), rail_status("stopped", id=2)]
        replies = [encoded(padded_envelope(dict(frame, quoted='"' * 64), 1791, unicode=True)) for frame in frames]
        self.assertTrue(all(2048 < len(line) < 2304 for line in replies))
        stream = EncodedSerial(replies)
        wire = host.Wire(args())
        with patch("serial.tools.list_ports.comports", return_value=[port()]), patch.object(host, "open_serial", return_value=stream):
            await wire.open()
        try:
            client = host.Controller(wire, None)
            results = [await client.hello(), await client.request("status"), await client.request("stop")]
            self.assertEqual([frame["rail_monitor"]["state"] for frame in results], ["ready", "ready", "stopped"])
            self.assertFalse(any(frame["motion_enabled"] or frame["source_cycle_enabled"] for frame in results))
            self.assertEqual(stream.writes[0], b"hello\n")
            self.assertEqual([json.loads(line)["cmd"] for line in stream.writes[1:]], ["status", "stop"])
        finally:
            await wire.close()
        self.assertTrue(stream.closed)

    async def test_delayed_fragmented_ble_preserves_serialization_time_freshness(self):
        # Advance only the host clock: receipt latency is not an MCU age field.
        elapsed = [0.0]
        frames = [rail_status(), rail_status(id=1)]
        for frame in frames:
            frame["rail_monitor"].update(age_us=49999, span_us=29999)
        replies = [encoded(envelope(frame)) for frame in frames]
        wire = host.Wire(args(usb=None, usb_kind="native", ble="fixture"))
        notifications = wire.notifications
        command = bytearray()
        commands = []

        async def write(_uuid, data, response=True):
            command.extend(data)
            if command.endswith(b"\n"):
                commands.append(bytes(command))
                command.clear()
                reply = replies.pop(0)
                for offset in range(0, len(reply), 20):
                    notifications.put_nowait(reply[offset:offset + 20])

        async def receive():
            elapsed[0] += .002  # Many fragments make each reply take >50ms.
            return await notifications.get()

        wire.ble = SimpleNamespace(is_connected=True, write_gatt_char=write)
        wire.notifications = SimpleNamespace(get=receive)
        client = host.Controller(wire, KEY)
        with patch.object(host, "time", SimpleNamespace(monotonic=lambda: elapsed[0])):
            self.assertEqual((await client.hello())["rail_monitor"]["age_us"], 49999)
            reply = await client.request("status")
        self.assertEqual(reply["rail_monitor"]["age_us"], 49999)
        self.assertGreater(client.latencies[0], 50)
        self.assertFalse(reply["source_cycle_enabled"])
        self.assertEqual(commands[0], b"hello\n")
        request = json.loads(commands[1])
        self.assertEqual(request["mac"], hmac.new(KEY, request["payload"].encode(), hashlib.sha256).hexdigest())

    async def test_usb_and_signed_status_reject_freshness_and_partial_contradictions(self):
        bad = [rail_status(id=1, usb_vbus_mv=None), rail_status(id=1, reboot_pending=True),
               rail_status(id=1, external_watchdog_state="stale", external_watchdog_ready=False)]
        expired = rail_status(id=1)
        expired["rail_monitor"]["age_us"] = 50000
        bad.append(expired)
        missing = rail_status(id=1)
        del missing["rail_monitor"]
        bad.append(missing)
        for frame, wireless, cmd in itertools.product(bad, (False, True), ("status", "stop")):
            with self.subTest(wireless=wireless, cmd=cmd, monitor=frame.get("rail_monitor")):
                wire = FakeWire([envelope(rail_status()), envelope(frame)])
                wire.wireless = wireless
                client = host.Controller(wire, KEY if wireless else None, host.PICO_PROFILE)
                await client.hello()
                with self.assertRaises(host.DiagnosticError):
                    await client.request(cmd)

    async def test_signed_hello_and_status_reject_tamper_and_wrong_key(self):
        good = envelope(rail_status())
        bad = dict(good, payload=good["payload"].replace("6016", "6024"))
        for wire, key in ((FakeWire([bad]), KEY), (FakeWire([good]), b"x" * 32)):
            client = host.Controller(wire, key, host.PICO_PROFILE)
            with self.assertRaisesRegex(host.DiagnosticError, "authentication"):
                await client.hello()
            self.assertIsNone(client.profile)
        reply = envelope(rail_status(id=1))
        wire = FakeWire([good, dict(reply, payload=reply["payload"].replace('"seq":7', '"seq":8'))])
        client = host.Controller(wire, KEY, host.PICO_PROFILE)
        await client.hello()
        with self.assertRaisesRegex(host.DiagnosticError, "authentication"):
            await client.request("status")

    async def test_status_payload_limits_apply_after_discovery_on_both_transports(self):
        for wireless, length in itertools.product((False, True), (1791, 1792)):
            with self.subTest(wireless=wireless, length=length):
                wire = FakeWire([envelope(rail_status()), padded_envelope(rail_status(id=1), length)])
                wire.wireless = wireless
                client = host.Controller(wire, KEY if wireless else None, host.PICO_PROFILE)
                await client.hello()
                if length == 1791:
                    self.assertEqual((await client.request("status"))["motor_voltage_mv"], 6016)
                else:
                    with self.assertRaisesRegex(host.DiagnosticError, "Oversized"):
                        await client.request("status")

    async def test_sequence_wrap_and_new_sampling_do_not_reuse_cached_voltage(self):
        frames = [rail_status() for _ in range(4)]
        frames[0]["rail_monitor"]["seq"] = 0xffffffff
        frames[1] = rail_status("sampling", id=1)
        frames[2] = rail_status(id=2, motor_voltage_mv=0)
        frames[2]["rail_monitor"]["seq"] = 1
        frames[3] = rail_status("expired", id=3)
        client = host.Controller(FakeWire([envelope(frame) for frame in frames]), KEY, host.PICO_PROFILE)
        self.assertEqual((await client.hello())["rail_monitor"]["seq"], 0xffffffff)
        self.assertIsNone((await client.request("status"))["motor_voltage_mv"])
        ready = await client.request("status")
        self.assertEqual(ready["rail_monitor"]["seq"], 1)
        self.assertEqual(ready["motor_voltage_mv"], 0)
        self.assertIsNone((await client.request("status"))["motor_voltage_mv"])

    async def test_ready_voltages_do_not_enable_blocked_commands(self):
        wire = FakeWire([envelope(rail_status())])
        client = host.Controller(wire, KEY, host.PICO_PROFILE)
        await client.hello()
        for cmd in ("start", "goto", "wheel", "arm", "pulse", "source_cycle", "logic_cycle"):
            with self.subTest(cmd=cmd), self.assertRaises(host.CommandRejected):
                await client.request(cmd)
        self.assertEqual(wire.sent, [b"hello\n"])

    async def test_status_run_keeps_measurements_and_does_not_claim_power_qualification(self):
        stream = SerialFixture([rail_status(), rail_status(id=1)])
        with patch("serial.tools.list_ports.comports", return_value=[port()]), patch.object(host, "open_serial", return_value=stream):
            result = await host.run(args())
        self.assertEqual(result["result"], "ACKNOWLEDGED")
        self.assertEqual(result["after"]["motor_voltage_mv"], 6016)
        for field in ("hardware_power_measured", "motion_enabled", "charge_enabled", "source_cycle_enabled", "physical_power_cycle"):
            self.assertIs(result[field], False)
        self.assertTrue(stream.closed)

    async def test_pico_line_limit_grows_without_changing_r3_or_legacy(self):
        for profile, kind, path, bound in ((host.PICO_PROFILE, "ch343p", "/dev/pico", 2304),
                                           (host.R3_PROFILE, "cp2102n", "/dev/cp", 1536),
                                           ("top_board_v2", "native", "/dev/native", 1536)):
            wire = host.Wire(args(profile=profile, usb_kind=kind, usb=path))
            stream = SerialFixture([])
            wire.usb = stream
            try:
                for length in (bound - 1, bound, bound + 1):
                    # Valid JSON plus padding; the complete command includes LF.
                    data = b"{}" + b" " * (length - 3) + b"\n"
                    with self.subTest(profile=profile, length=length):
                        if length < bound:
                            await wire.send(data)
                        else:
                            with self.assertRaisesRegex(host.DiagnosticError, "frame limit"):
                                await wire.send(data)
                self.assertEqual([len(data) for data in stream.writes], [bound - 1])
            finally:
                await wire.close()


if __name__ == "__main__":
    unittest.main()
