#!/usr/bin/env python3
"""Capture WhiteBoardBot USB diagnostics; default operation never runs motors."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time


class DiagnosticError(RuntimeError):
    pass


class FrameDecoder:
    """Newline framing survives fragmented USB reads and discards overlong lines."""
    def __init__(self):
        self.pending = bytearray()
        self.discard = False
        self.invalid = 0

    def feed(self, chunk):
        frames = []
        for byte in chunk:
            if byte == 10:
                if self.discard:
                    self.discard = False
                elif self.pending.strip():
                    try:
                        obj = json.loads(self.pending.decode("utf-8"))
                        if not isinstance(obj, dict):
                            raise ValueError("not an object")
                        frames.append(obj)
                    except (UnicodeError, ValueError):
                        self.invalid += 1
                self.pending.clear()
            elif not self.discard:
                if len(self.pending) >= 4096:
                    self.pending.clear()
                    self.discard = True
                    self.invalid += 1
                else:
                    self.pending.append(byte)
        return frames


def validate_status(frame):
    if frame.get("protocol") != 1 or frame.get("firmware") != "wbb-usb-diag-1":
        raise DiagnosticError("Unexpected firmware/protocol; no motion command sent")
    for key in ("session", "seq", "uptime_ms"):
        if type(frame.get(key)) is not int or not 0 <= frame[key] <= 0xFFFFFFFF:
            raise DiagnosticError(f"Malformed status field: {key}")
    if frame.get("profile") not in ("adapter", "top_board_v2") or not isinstance(frame.get("reason"), str):
        raise DiagnosticError("Missing/invalid board profile or stop reason")
    for key in ("pwm_ready", "armed", "running", "voltage_feedback", "current_feedback", "power_disconnect"):
        if type(frame.get(key)) is not bool:
            raise DiagnosticError(f"Malformed boolean field: {key}")
    if type(frame.get("tx_dropped")) is not int or not 0 <= frame["tx_dropped"] <= 0xFFFFFFFF:
        raise DiagnosticError("Malformed dropped-frame counter")
    motors = frame.get("motors")
    if not isinstance(motors, list) or len(motors) != 3:
        raise DiagnosticError("Status must describe all three motors")
    expected = ((0,1,7 if frame["profile"] == "top_board_v2" else 16,17,1),
                (2,3,18,19,3), (4,5,20,23,2))
    for i, motor in enumerate(motors):
        if not isinstance(motor, dict):
            raise DiagnosticError("Malformed motor record")
        if motor.get("index") != i:
            raise DiagnosticError("Motor index mismatch")
        for key in ("count", "edges_a", "edges_b", "invalid", "command", "physical"):
            if type(motor.get(key)) is not int:
                raise DiagnosticError(f"Malformed motor field: {key}")
        if tuple(motor.get(key) for key in ("fi","bi","enc_a","enc_b","physical")) != expected[i]:
            raise DiagnosticError("Firmware pin mapping does not match the board profile")


class Client:
    def __init__(self, port, logfile, clock=time.monotonic):
        self.port, self.logfile, self.clock = port, logfile, clock
        self.decoder = FrameDecoder()
        self.latest = None
        self.session = None
        self.last_sequence = None
        self.sequence_gaps = 0
        self.status_count = 0
        self.request_id = 0
        self.acks = {}
        self.last_ping = clock()
        self.last_status = clock()

    def send(self, command):
        self.request_id += 1
        line = f"{self.request_id} {command}\n".encode("ascii")
        if self.port.write(line) != len(line):
            raise DiagnosticError("USB write incomplete")
        return self.request_id

    def ingest(self, obj):
        self.logfile.write(json.dumps({"host_monotonic": self.clock(), "frame": obj}) + "\n")
        if obj.get("type") == "ack":
            if type(obj.get("ok")) is not bool or type(obj.get("session")) is not int:
                raise DiagnosticError("Malformed acknowledgement")
            if self.session is not None and obj["session"] != self.session:
                raise DiagnosticError("MCU session changed during command")
            if type(obj.get("id")) is int:
                self.acks[obj["id"]] = obj
                if len(self.acks) > 64:
                    del self.acks[min(self.acks)]
            return
        if obj.get("type") != "status":
            return
        validate_status(obj)
        if self.session is not None and obj["session"] != self.session:
            raise DiagnosticError("MCU reset/reboot detected; motion will not resume")
        self.session = obj["session"]
        if self.last_sequence is not None:
            delta = (obj["seq"] - self.last_sequence) & 0xFFFFFFFF
            if delta == 0 or delta > 0x7FFFFFFF:
                raise DiagnosticError("Duplicate/reordered telemetry frame")
            self.sequence_gaps += delta - 1
        self.last_sequence = obj["seq"]
        self.latest = obj
        self.last_status = self.clock()
        self.status_count += 1

    def pump(self, seconds, heartbeat=False):
        end = self.clock() + seconds
        while self.clock() < end:
            if heartbeat and self.clock() - self.last_ping >= 0.10:
                self.send("ping")
                self.last_ping = self.clock()
            data = self.port.read(min(self.port.in_waiting or 1, 4096))
            for obj in self.decoder.feed(data):
                self.ingest(obj)
            if self.clock() - self.last_status > 2.0:
                raise DiagnosticError("USB telemetry timeout; check cable, power, boot mode or MCU")

    def request(self, command, heartbeat=False):
        ident = self.send(command)
        deadline = self.clock() + 1.0
        while ident not in self.acks and self.clock() < deadline:
            self.pump(0.02, heartbeat=heartbeat)
        result = self.acks.pop(ident, None)
        if result is None:
            raise DiagnosticError(f"No acknowledgement for {command.split()[0]}")
        if not result.get("ok"):
            raise DiagnosticError(f"Device rejected command: {result.get('error')}")
        return result

    def fresh_status(self):
        old = self.status_count
        self.request("status")
        deadline = self.clock() + 0.5
        while self.status_count == old and self.clock() < deadline:
            self.pump(0.02)
        if self.status_count == old:
            raise DiagnosticError("Status acknowledgement without telemetry")
        return self.latest


def motor_result(before, after, selected):
    results = []
    for i, (a, b) in enumerate(zip(before["motors"], after["motors"])):
        da = (b["edges_a"] - a["edges_a"]) & 0xFFFFFFFF
        db = (b["edges_b"] - a["edges_b"]) & 0xFFFFFFFF
        bad = (b["invalid"] - a["invalid"]) & 0xFFFFFFFF
        delta = ((b["count"] - a["count"] + 0x80000000) & 0xFFFFFFFF) - 0x80000000
        active = selected == 3 or i == selected
        state = "PASS"
        finding = "Both encoder phases responded"
        if active and (da < 4 or db < 4 or abs(delta) < 4):
            state, finding = "REVIEW", "Insufficient encoder response; power, duty, load, wiring, driver or encoder may be responsible"
        elif not active and da + db > 4:
            state, finding = "REVIEW", "Uncommanded channel moved or received transitions; check settling, mapping and crosstalk"
        elif not active:
            finding = "No significant uncommanded encoder transitions"
        if bad:
            state, finding = "REVIEW", "Invalid quadrature transitions; inspect signal quality or sampling limits"
        results.append({"index": i, "physical_motor": b["physical"], "state": state,
                        "finding": finding, "delta": delta, "edges_a": da,
                        "edges_b": db, "invalid": bad})
    return results


def settle(client):
    """Require a quiet encoder interval before another pulse/reversal."""
    for _ in range(6):
        before = client.fresh_status()
        client.pump(0.5)
        after = client.fresh_status()
        if all(a["edges_a"] == b["edges_a"] and a["edges_b"] == b["edges_b"]
               for a, b in zip(before["motors"], after["motors"])):
            return after
    raise DiagnosticError("Wheels have not settled; subsequent motor tests cancelled")


def run_pulse(client, args, motor, direction, watchdog=False):
    before = settle(client)
    client.request(f"armbench {int(args.motor_voltage*1000)} {round(args.supply_voltage*1000)} {args.duty}")
    client.request(f"pulse {motor} {direction} {args.duty} {1000 if watchdog else args.pulse_ms}", heartbeat=True)
    client.pump(0.7 if watchdog else args.pulse_ms/1000 + 0.15, heartbeat=not watchdog)
    after = client.fresh_status()
    wanted = "HEARTBEAT_TIMEOUT" if watchdog else "PULSE_DONE"
    if after["running"] or any(m["command"] for m in after["motors"]) or after["reason"] != wanted:
        raise DiagnosticError(f"Stop condition failed: expected {wanted}, received {after['reason']}")
    return {"selected_index": motor, "direction": direction, "duty": args.duty,
            "stop_reason": after["reason"], "software_stop": "PASS",
            "physical_power_off": "NOT_TESTED", "motors": motor_result(before, after, motor)}


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port")
    p.add_argument("--list", action="store_true", help="list serial ports and exit")
    p.add_argument("--profile", choices=("adapter", "top_board_v2"), default="adapter")
    p.add_argument("--seconds", type=float, default=10)
    p.add_argument("--report", type=Path, default=Path("diagnostic-reports") / datetime.now().strftime("%Y%m%d-%H%M%S"))
    p.add_argument("--run-motors", action="store_true", help="opt in to unloaded bench motor tests")
    p.add_argument("--motor-voltage", type=int, choices=(6,12))
    p.add_argument("--supply-voltage", type=float, help="voltage actually measured at the motor rail")
    p.add_argument("--bench-current-limit", type=float, help="external supply current limit in A, selected for the actual motors/PCB")
    p.add_argument("--duty", type=int, default=64)
    p.add_argument("--pulse-ms", type=int, default=500)
    p.add_argument("--all-at-once", action="store_true", help="also test simultaneous startup after individual tests")
    return p


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    if not 0.1 <= args.seconds <= 3600 or not 1 <= args.duty <= 255 or not 1 <= args.pulse_ms <= 1000:
        p.error("seconds must be 0.1..3600, duty 1..255 and pulse-ms 1..1000")
    if args.all_at_once and not args.run_motors:
        p.error("--all-at-once requires --run-motors")
    if args.run_motors:
        if args.profile == "top_board_v2":
            p.error("top_board_v2 power design is not released; only passive diagnostics are available")
        if args.motor_voltage is None or args.supply_voltage is None or args.bench_current_limit is None:
            p.error("motor tests require --motor-voltage, --supply-voltage and --bench-current-limit")
        if not 0 < args.bench_current_limit <= 100 or not args.motor_voltage*.95 <= args.supply_voltage <= args.motor_voltage*1.05:
            p.error("measured supply must match motor rating within 5%; supply current limit must be positive")
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        p.error("pyserial is required: .venv/bin/python -m pip install -r requirements.txt")
    ports = list(list_ports.comports())
    if args.list:
        for port in ports:
            print(f"{port.device}\t{port.description}\t{port.hwid}")
        return 0
    if not args.port:
        candidates = [port.device for port in ports if port.vid == 0x303A]
        if len(candidates) != 1:
            p.error("select one device with --port (use --list)")
        args.port = candidates[0]
    args.report.mkdir(parents=True, exist_ok=False)
    report = {"created_utc": datetime.now(timezone.utc).isoformat(), "port": args.port,
              "profile": args.profile, "state": "INCOMPLETE", "motion_tests": [],
              "bench_declarations": {"motor_voltage": args.motor_voltage, "measured_supply_voltage": args.supply_voltage,
                                     "external_current_limit_a": args.bench_current_limit},
              "not_tested": ["actual rail voltages", "motor currents", "physical disconnect", "all PCB faults",
                             "thermal performance", "stall-current capability", "USB signal integrity"]}
    port = None
    client = None
    try:
        port = serial.Serial(port=None, baudrate=115200, timeout=0.02, write_timeout=0.2)
        port.dtr = False
        port.rts = False
        port.port = args.port
        port.open()
        with (args.report / "telemetry.jsonl").open("w") as log:
            client = Client(port, log)
            client.request("stop")
            client.request("hello")
            first = client.fresh_status()
            if first.get("profile") != args.profile:
                raise DiagnosticError("Firmware pin profile does not match selected board")
            report["first_status"] = first
            client.pump(args.seconds)
            if not first.get("pwm_ready"):
                raise DiagnosticError("One or more PWM channels failed to initialize")
            if args.run_motors:
                for motor in range(3):
                    for direction in (1,-1):
                        result = run_pulse(client,args,motor,direction)
                        report["motion_tests"].append(result)
                        if any(m["state"] != "PASS" for m in result["motors"]):
                            raise DiagnosticError("Motor test needs inspection; later motion tests cancelled")
                    forward, reverse = report["motion_tests"][-2:]
                    if forward["motors"][motor]["delta"] * reverse["motors"][motor]["delta"] >= 0:
                        raise DiagnosticError("Forward/reverse encoder signs did not reverse")
                report["heartbeat_stop_test"] = run_pulse(client,args,0,1,watchdog=True)
                if args.all_at_once:
                    result = run_pulse(client,args,3,1)
                    report["motion_tests"].append(result)
                    if any(m["state"] != "PASS" for m in result["motors"]):
                        raise DiagnosticError("Simultaneous startup test needs inspection")
            client.request("stop")
            report["last_status"] = client.fresh_status()
            if client.sequence_gaps or client.decoder.invalid or client.latest["tx_dropped"] != first["tx_dropped"]:
                raise DiagnosticError("USB stream contained lost/invalid frames; inspect report")
            report["state"] = "BENCH_CHECKS_PASSED" if args.run_motors else "USB_CAPTURE_PASSED"
    except (DiagnosticError, OSError, serial.SerialException, KeyboardInterrupt) as exc:
        report["state"] = "INCOMPLETE_OR_FAILED"
        report["error"] = str(exc) or "Interrupted"
    finally:
        if client:
            # This is also attempted after a malformed frame, timeout or reboot.
            try:
                client.send("stop")
            except (OSError, serial.SerialException):
                pass
            report["transport"] = {"status_frames": client.status_count, "sequence_gaps": client.sequence_gaps,
                                   "invalid_frames": client.decoder.invalid}
        if port:
            port.close()
        (args.report / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(f"{report['state']}: {args.report / 'report.json'}")
    return 0 if report["state"] in ("USB_CAPTURE_PASSED", "BENCH_CHECKS_PASSED") else 1


if __name__ == "__main__":
    raise SystemExit(main())
