"""Fault tests for the real host client; loopback is not a physical radio test."""
import asyncio
import hashlib
import hmac
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, patch
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import robot_control as host

KEY = bytes(range(32))
BOOT = "1234567890abcdef"


def status(**changes):
    result = dict(id=0, ok=True, error="", protocol=2, firmware="wbb-control-2",
                  profile="top_board_v2", boot=BOOT, device="WhiteBoardBot-123456",
                  lease=0, motion_enabled=False, watchdog_ready=True, pwm_ready=True,
                  uptime_ms=200, state="idle", reset_reason=3, reboot_pending=False,
                  physical_motors=[1,2,3], header_order=["M0","M2","M1"], pwm_command=[0,0,0])
    result.update(changes)
    return result


def envelope(payload, key=KEY):
    # The firmware always replies with an envelope, including over USB.
    text = json.dumps(payload, separators=(",", ":"))
    return dict(payload=text, mac=hmac.new(key, text.encode(), hashlib.sha256).hexdigest())


class FakeWire:
    wireless = True
    def __init__(self, replies):
        self.replies, self.sent = list(replies), []
    async def send(self, data):
        self.sent.append(data)
    async def recv(self, timeout=3):
        if not self.replies:
            raise host.DiagnosticError("Test link disconnected")
        return self.replies.pop(0)


class AuthenticationTests(unittest.TestCase):
    def test_r3_is_explicitly_unqualified_and_cannot_change_revision_on_reset(self):
        r3 = status(profile="top_board_v2_r3_review", charge_enabled=False, power_disconnect=False,
                    motor_voltage_mv=None, motor_current_ma=None, rail_3v3_mv=None)
        host.check_status(r3)
        for field in ("motion_enabled", "charge_enabled", "power_disconnect", "motor_voltage_mv",
                      "motor_current_ma", "rail_3v3_mv"):
            missing = dict(r3); del missing[field]
            for bad in (missing, dict(r3, **{field: True}), dict(r3, **{field: 0})):
                with self.assertRaises(host.DiagnosticError):
                    host.check_status(bad)
        for revision in ("top_board_v2_r3", "r3", "released", None):
            with self.assertRaises(host.DiagnosticError):
                host.check_status(dict(r3, profile=revision))
        host.verify_restart(r3, dict(r3, boot="a" * 16))
        for before, after in ((status(), dict(r3, boot="a" * 16)), (r3, status(boot="a" * 16))):
            with self.assertRaises(host.DiagnosticError):
                host.verify_restart(before, after)

    def test_authentication_rejects_tamper_wrong_key_missing_mac(self):
        original = envelope(status())
        self.assertEqual(host.unpack(original, KEY), status())
        cases = [dict(original, payload=original["payload"].replace('"ok":true', '"ok":false')),
                 dict(original, mac="0"*64), dict(payload=original["payload"])]
        for bad in cases:
            with self.assertRaises(host.DiagnosticError):
                host.unpack(bad, KEY)
        with self.assertRaises(host.DiagnosticError):
            host.unpack(original, b"x"*32)

    def test_exact_payload_bytes_are_authenticated(self):
        data = dict(id=1, boot=BOOT, cmd="status", lease=0)
        self.assertEqual(json.loads(host.pack(data)), data)
        packed = json.loads(host.pack(data, KEY))
        self.assertEqual(json.loads(packed["payload"]), data)
        self.assertEqual(packed["mac"], hmac.new(KEY, packed["payload"].encode(), hashlib.sha256).hexdigest())
        spaced = dict(packed, payload=packed["payload"]+" ")
        with self.assertRaises(host.DiagnosticError):
            host.unpack(spaced, KEY)

    def test_invalid_fields_and_nonfinite_request(self):
        for changes in (dict(ok="true"), dict(id=True)):
            with self.assertRaises(host.DiagnosticError):
                host.unpack(envelope(status(**changes)), KEY)
        for changes in (dict(boot="old"), dict(profile="adapter"), dict(pwm_ready="true"), dict(device=None),
                        dict(physical_motors=[1,3,2]),dict(header_order=["M0","M1","M2"]),dict(pwm_command=[0,0])):
            with self.assertRaises(host.DiagnosticError):
                host.check_status(status(**changes))
        with self.assertRaises(ValueError):
            host.pack(dict(v=float("nan")), KEY)

    def test_restart_requires_new_boot_same_board_idle_and_reset_reason(self):
        before = status()
        after = status(boot="a"*16)
        host.verify_restart(before, after)
        for changes in (dict(boot=BOOT), dict(device="another"), dict(state="running"),
                        dict(uptime_ms=40000), dict(uptime_ms=True), dict(reset_reason=1),
                        dict(reboot_pending=True), dict(pwm_command=[0,0,1])):
            with self.assertRaises(host.DiagnosticError):
                host.verify_restart(before, dict(after, **changes))
        host.verify_restart(None, dict(after, reset_reason=1), software=False)


class ClientTests(unittest.IsolatedAsyncioTestCase):
    async def test_profile_or_device_cannot_change_within_a_live_session(self):
        r3 = status(profile="top_board_v2_r3_review", charge_enabled=False, power_disconnect=False,
                    motor_voltage_mv=None, motor_current_ma=None, rail_3v3_mv=None)
        for bad in (r3, status(device="different")):
            wire = FakeWire([envelope(status()), envelope(dict(bad, id=1))])
            client = host.Controller(wire, KEY)
            await client.hello()
            with self.assertRaisesRegex(host.DiagnosticError, "Board/profile changed"):
                await client.request("status")
            wire.replies.append(envelope(bad))
            with self.assertRaisesRegex(host.DiagnosticError, "Board/profile changed"):
                await client.hello()

    async def test_reboot_return_verified_with_or_without_ack_and_sent_once(self):
        for lost in (False, True):
            class RebootWire(FakeWire):
                opens = 0
                async def open(self):
                    self.opens += 1
                    if self.opens == 1:
                        self.replies = [envelope(status()), envelope(status(id=1, lease=42)),
                            host.DiagnosticError("Test disconnect before ACK") if lost else envelope(status(id=2))]
                    else:
                        self.replies = [envelope(status(boot="a"*16))]
                async def recv(self, timeout=3):
                    value = await super().recv(timeout)
                    if isinstance(value, Exception):
                        raise value
                    return value
                async def close(self):
                    pass
            wire = RebootWire([])
            with tempfile.TemporaryDirectory() as folder:
                keyfile = Path(folder)/"fixture-key.local"
                keyfile.write_text(KEY.hex())
                args = SimpleNamespace(scan_ble=False, key_file=keyfile, wifi=None, usb=None, ble="fixture", action="reboot")
                with patch.object(host, "Wire", return_value=wire), patch.object(host.asyncio, "sleep", new=AsyncMock()):
                    result = await host.run(args)
            self.assertEqual(result["result"], "APP_RETURN_VERIFIED")
            self.assertEqual(result["reboot_ack_received"], not lost)
            self.assertEqual(result["after"]["boot"], "a"*16)
            commands = [json.loads(json.loads(data)["payload"])["cmd"] for data in wire.sent if data != b"hello\n"]
            self.assertEqual(commands.count("reboot"), 1)
            self.assertEqual(wire.opens, 2)

    async def test_reboot_rejection_is_not_treated_as_ack_loss(self):
        wire = FakeWire([envelope(status(id=1, ok=False, error="NOT_OWNER"))])
        client = host.Controller(wire, KEY)
        client.boot = BOOT
        result = {}
        with self.assertRaises(host.CommandRejected):
            await host.request_reboot(client, result)
        self.assertNotIn("reboot_ack_received", result)

    async def test_restart_reconnect_rejects_old_wrong_or_unauthenticated_board(self):
        for bad in (envelope(status()), envelope(status(boot="b"*16, device="different")),
                    envelope(status(boot="b"*16, reset_reason=1)),
                    dict(envelope(status(boot="b"*16)), mac="0"*64)):
            class ReturnWire(FakeWire):
                async def open(self):
                    self.replies = [bad]
                async def close(self):
                    pass
            with self.assertRaisesRegex(host.DiagnosticError, "return NOT verified"):
                await host.wait_for_restart(ReturnWire([]), KEY, status(), timeout=.01, interval=0)

    async def test_claim_heartbeat_stop_and_sequence(self):
        wire = FakeWire([envelope(status()), envelope(status(id=1, lease=42)),
                         envelope(status(id=2, lease=42)), envelope(status(id=3))])
        client = host.Controller(wire, KEY)
        await client.hello()
        await client.request("claim")
        self.assertEqual(client.lease, 42)
        await client.request("heartbeat")
        await client.request("stop")
        self.assertEqual(client.lease, 0)
        requests = [json.loads(json.loads(x)["payload"]) for x in wire.sent[1:]]
        self.assertEqual([r["id"] for r in requests], [1, 2, 3])
        self.assertEqual([r["lease"] for r in requests], [0, 42, 42])

    async def test_old_ack_ignored_new_boot_rejected(self):
        wire = FakeWire([envelope(status()), envelope(status(id=0)), envelope(status(id=1))])
        client = host.Controller(wire, KEY)
        await client.hello()
        await client.request("status")
        wire.replies.append(envelope(status(id=2, boot="b"*16)))
        with self.assertRaisesRegex(host.DiagnosticError, "rebooted"):
            await client.request("status")

    async def test_device_rejection_disconnect_and_parameter_injection(self):
        wire = FakeWire([envelope(status()), envelope(status(id=1, ok=False, error="BOARD_POWER_UNREVIEWED"))])
        client = host.Controller(wire, KEY)
        await client.hello()
        with self.assertRaisesRegex(host.DiagnosticError, "BOARD_POWER_UNREVIEWED"):
            await client.request("start")
        with self.assertRaisesRegex(host.DiagnosticError, "disconnected"):
            await client.request("status")
        count = len(wire.sent)
        with self.assertRaises(host.DiagnosticError):
            await client.request("status", {"boot":"injected"})
        self.assertEqual(len(wire.sent), count)

    async def test_unsigned_discovery_rejected_on_wireless(self):
        wire = FakeWire([dict(envelope(status()), mac="")])
        with self.assertRaises(host.DiagnosticError):
            await host.Controller(wire, KEY).hello()

    async def test_real_udp_loopback_status(self):
        class Peer(asyncio.DatagramProtocol):
            errors = []
            def connection_made(self, transport):
                self.transport = transport
            def datagram_received(self, data, address):
                try:
                    if data == b"hello\n":
                        reply = status()
                    else:
                        request = json.loads(data)
                        text = request["payload"]
                        assert hmac.compare_digest(request["mac"], hmac.new(KEY, text.encode(), hashlib.sha256).hexdigest())
                        command = json.loads(text)
                        assert command["boot"] == BOOT and command["cmd"] == "status"
                        reply = status(id=command["id"])
                    self.transport.sendto((json.dumps(envelope(reply))+"\n").encode(), address)
                except Exception as exc:
                    self.errors.append(str(exc))
        transport, peer = await asyncio.get_running_loop().create_datagram_endpoint(Peer, local_addr=("127.0.0.1",4243))
        wire = host.Wire(SimpleNamespace(usb=None, wifi="127.0.0.1", ble=None))
        try:
            await wire.open()
            client = host.Controller(wire, KEY)
            await client.hello()
            self.assertFalse((await client.request("status"))["motion_enabled"])
            self.assertFalse(peer.errors)
        finally:
            await wire.close()
            transport.close()


if __name__ == "__main__":
    unittest.main()
