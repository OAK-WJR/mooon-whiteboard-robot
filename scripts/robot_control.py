#!/usr/bin/env python3
"""Control over native USB, R3/CP2102N, Pico/CH343P, KISS C6/CH343G, UDP or BLE.

The current board build rejects all motion. JSON results never imply measured
motor power or verified radio range. Reconnection never resumes an old lease.
"""
import argparse
import asyncio
from collections import deque
import hashlib
import hmac
import json
import os
from pathlib import Path
import secrets
import socket
import subprocess
import tempfile
import time

from pcb_control import (UART_USB_KINDS, identity, match_port, open_serial,
                         rom_command, rom_config_text, select_port, usb_chip, usb_identity)
from pcb_diagnostics import FrameDecoder, DiagnosticError

RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
R3_PROFILE = "top_board_v2_r3_review"
PICO_PROFILE = "top_board_v2_pico_review"
KISS_C6_PROFILE = "top_board_v2_kiss_c6_review"
PROFILES = ("top_board_v2", R3_PROFILE, PICO_PROFILE, KISS_C6_PROFILE)
USB_PROFILES = dict(native="top_board_v2", cp2102n=R3_PROFILE, ch343p=PICO_PROFILE, ch343g=KISS_C6_PROFILE)
PICO_BLOCKED_COMMANDS = frozenset(("start", "goto", "wheel", "arm", "pulse", "source_cycle", "logic_cycle"))
PICO_RESPONSE_PAYLOAD_LIMIT = 1792
PICO_LINE_LIMIT = 2304
PICO_RAIL_STATES = ("not_started", "starting", "initializing", "sampling", "ready", "expired", "fault", "stopped")
PICO_RAIL_ERRORS = ("none", "start_failed", "setup_timeout", "bus_fault", "cleanup_failed",
                    "invalid_address", "wrong_manufacturer", "wrong_die", "nack", "bus_error",
                    "short_read", "bus_busy_timeout", "transfer_timeout", "conversion_timeout",
                    "acquisition_timeout", "configuration_changed", "invalid_voltage")
PICO_RAIL_BUS_STATES = ("stopped", "starting", "ready", "fault", "stopping", "cleanup_fault")
PICO_RAIL_VOLTAGES = ("motor_voltage_mv", "raw_logic_5v_mv", "usb_vbus_mv")
PICO_CELL_STATES = ("not_started", "waking", "settling", "detecting", "configuring", "awaiting_scan",
                    "monitoring", "stale", "fault", "stopped")
PICO_CELL_ERRORS = ("none", "wake_gpio_fault", "wake_aborted", "shared_bus_fault", "invalid_options",
                    "incomplete_profile", "nack", "bus_error", "short_transfer", "io_timeout", "wake_timeout",
                    "initialization_timeout", "mode_timeout", "frame_timeout", "crc_mismatch", "crc_mode_mismatch",
                    "echo_mismatch", "checksum_mismatch", "invalid_length", "wrong_device", "invalid_status",
                    "not_full_access", "fets_not_off", "balancing_active", "config_not_active", "readback_mismatch",
                    "unexpected_por", "unexpected_mode", "external_reset")

class CommandRejected(DiagnosticError):
    """The validated profile or an authenticated peer explicitly rejected a command."""


def selected_profile(args):
    """Bind bridge and application identity before opening a port or issuing ROM reset."""
    kind = getattr(args, "usb_kind", "native")
    usb_identity(kind)
    requested = getattr(args, "profile", None)
    if requested is not None and requested not in PROFILES:
        raise DiagnosticError("Unknown application profile")
    if kind in UART_USB_KINDS and (not isinstance(args.usb, str) or not args.usb.strip() or "://" in args.usb):
        raise DiagnosticError(f"--usb-kind {kind} requires an explicit local --usb path")
    if args.usb:
        if requested is not None and requested != USB_PROFILES[kind]:
            raise DiagnosticError("--profile does not match --usb-kind; Pico uses ch343p, R3 cp2102n, KISS C6 ch343g")
        # Preserve legacy native discovery; bridge selection explicitly pins the app.
        return USB_PROFILES[kind] if kind in UART_USB_KINDS else requested
    return requested


def pack(payload, key=None):
    text = json.dumps(payload, ensure_ascii=True, separators=(",", ":"), allow_nan=False)
    if not key:
        return (text + "\n").encode()
    mac = hmac.new(key, text.encode(), hashlib.sha256).hexdigest()
    return (json.dumps({"payload": text, "mac": mac}, separators=(",", ":")) + "\n").encode()


def unpack(envelope, key=None, wireless=True, expected_profile=None):
    if not isinstance(envelope, dict) or not isinstance(envelope.get("payload"), str):
        raise DiagnosticError("Malformed control response")
    payload = envelope["payload"]
    payload_limit = PICO_RESPONSE_PAYLOAD_LIMIT if expected_profile == PICO_PROFILE else 1200
    if len(payload.encode("utf-8")) >= payload_limit:
        raise DiagnosticError("Oversized response")
    if wireless:
        signature = envelope.get("mac")
        if not key or not isinstance(signature, str) or not hmac.compare_digest(
                signature, hmac.new(key, payload.encode(), hashlib.sha256).hexdigest()):
            raise DiagnosticError("Response authentication failed")
    result = json.loads(payload)
    if not isinstance(result, dict) or type(result.get("ok")) is not bool or type(result.get("id")) is not int:
        raise DiagnosticError("Malformed response fields")
    return result


def check_status(frame, expected_profile=None):
    if (not isinstance(frame, dict) or (frame.get("protocol"), frame.get("firmware")) != (2, "wbb-control-2")
            or frame.get("profile") not in PROFILES):
        raise DiagnosticError("Not the integrated-board control firmware")
    if expected_profile is not None and frame["profile"] != expected_profile:
        raise DiagnosticError("Firmware board profile does not match the selected application profile/USB kind")
    if frame["profile"] == PICO_PROFILE and expected_profile != PICO_PROFILE:
        raise DiagnosticError("Pico requires explicit --profile top_board_v2_pico_review or --usb-kind ch343p")
    if frame["profile"] == KISS_C6_PROFILE and expected_profile != KISS_C6_PROFILE:
        raise DiagnosticError("KISS C6 requires explicit --profile top_board_v2_kiss_c6_review or --usb-kind ch343g")
    if frame["profile"] in (R3_PROFILE, PICO_PROFILE, KISS_C6_PROFILE):
        for field, expected in (("motion_enabled", False), ("charge_enabled", False),
                                ("power_disconnect", False),
                                ("motor_current_ma", None), ("rail_3v3_mv", None)):
            if field not in frame or frame[field] is not expected:
                raise DiagnosticError(f"Unqualified {frame['profile']} capability mismatch: {field}")
        if frame["profile"] in (R3_PROFILE, KISS_C6_PROFILE) and ("motor_voltage_mv" not in frame or frame["motor_voltage_mv"] is not None):
            raise DiagnosticError("Unqualified profile capability mismatch: motor_voltage_mv")
    if frame["profile"] == KISS_C6_PROFILE:
        for field, expected in (("odometry_qualified", False), ("pwm_ready", False),
                                ("x", None), ("y", None), ("theta", None)):
            if field not in frame or frame[field] is not expected:
                raise DiagnosticError(f"Unqualified KISS C6 field: {field}")
        if frame.get("wheel_speed_mps") != [None, None, None]:
            raise DiagnosticError("KISS C6 speed must remain uncalibrated/null")
    boot = frame.get("boot", "")
    if not isinstance(boot, str) or len(boot) != 16 or any(c not in "0123456789abcdef" for c in boot):
        raise DiagnosticError("Invalid boot identifier")
    for field in ("motion_enabled", "watchdog_ready", "pwm_ready"):
        if type(frame.get(field)) is not bool:
            raise DiagnosticError(f"Missing capability: {field}")
    if not isinstance(frame.get("device"), str) or not frame["device"]:
        raise DiagnosticError("Missing board identity")
    physical = frame.get("physical_motors")
    if (physical != [1, 2, 3] or any(type(v) is not int for v in physical)
            or frame.get("header_order") != ["M0", "M2", "M1"]):
        raise DiagnosticError("Unexpected motor/header ordering")
    pwm = frame.get("pwm_command")
    if not isinstance(pwm, list) or len(pwm) != 3 or any(type(v) is not int or abs(v) > 255 for v in pwm):
        raise DiagnosticError("Invalid three-channel PWM report")
    if frame["profile"] == PICO_PROFILE:
        check_pico_status(frame)
    return frame


def check_kiss_rail_report(frame, expected_profile=None):
    """ADC-calibrated pin measurements; not board/divider calibration."""
    if (expected_profile != KISS_C6_PROFILE or not isinstance(frame, dict)
            or (frame.get("protocol"), frame.get("firmware"), frame.get("profile"), frame.get("kind"))
            != (2, "wbb-control-2", KISS_C6_PROFILE, "rails")):
        raise DiagnosticError("Rail diagnostics require the selected KISS C6 profile")
    for field, value in (("read_only", True), ("motion_enabled", False),
                         ("hardware_qualified", False), ("divider_calibrated", False)):
        if frame.get(field) is not value:
            raise DiagnosticError(f"Invalid rail capability: {field}")
    # Optional as a pair for earlier read-only firmware reports of this profile.
    if "wireless_power_state" in frame or "wireless_start_attempted" in frame:
        phase = frame.get("wireless_power_state")
        waiting = {"waiting_voltage", "qualifying", "inhibited"}
        attempted = {"starting", "online", "init_failed", "restart"}
        if (not isinstance(phase, str) or phase not in waiting | attempted
                or type(frame.get("wireless_start_attempted")) is not bool
                or frame["wireless_start_attempted"] != (phase in attempted)):
            raise DiagnosticError("Invalid wireless supply policy state")
    if "previous_radio_power_reset" in frame and type(frame["previous_radio_power_reset"]) is not bool:
        raise DiagnosticError("Invalid previous radio power reset marker")
    states = {"not_started", "init", "settling", "ready", "stale", "stopped", "adc_release",
              "adc_mapping", "adc_unit", "adc_config", "adc_calibration", "adc_read",
              "adc_saturated", "adc_convert", "adc_range", "adc_deadline"}
    if not isinstance(frame.get("state"), str) or frame["state"] not in states:
        raise DiagnosticError("Invalid ADC state")
    if type(frame.get("fresh")) is not bool or type(frame.get("divider_gain")) is not int or frame["divider_gain"] != 13:
        raise DiagnosticError("Invalid ADC freshness/divider")
    for field in ("adc_error", "adc_cleanup_error", "failing_channel"):
        if type(frame.get(field)) is not int or not -(2**31) <= frame[field] < 2**31:
            raise DiagnosticError(f"Invalid ADC error metadata: {field}")
    if frame["failing_channel"] not in (-1, 0, 1):
        raise DiagnosticError("Invalid ADC channel")
    vectors = (frame.get("adc_raw"), frame.get("adc_pin_mv"))
    if any(not isinstance(v, list) or len(v) != 2 for v in vectors):
        raise DiagnosticError("Incomplete paired ADC measurements")
    numeric = ("battery_voltage_mv", "motor_voltage_mv", "seq", "age_ms", "span_ms")
    if not frame["fresh"]:
        if (frame["state"] == "ready" or any(k not in frame or frame[k] is not None for k in numeric)
                or any(v is not None for a in vectors for v in a)):
            raise DiagnosticError("Unavailable ADC measurements must be null")
        return frame
    if frame["state"] != "ready" or frame["adc_error"] or frame["adc_cleanup_error"] or frame["failing_channel"] != -1:
        raise DiagnosticError("Fresh ADC report has a fault")
    bounds = {"battery_voltage_mv": (0, 13000), "motor_voltage_mv": (0, 13000),
              "seq": (1, 2**32-1), "age_ms": (0, 250), "span_ms": (0, 25)}
    for field, (low, high) in bounds.items():
        if type(frame.get(field)) is not int or not low <= frame[field] <= high:
            raise DiagnosticError(f"Invalid ADC measurement: {field}")
    if frame["span_ms"] > frame["age_ms"]:
        raise DiagnosticError("ADC sample age precedes acquisition")
    for vector, maximum in zip(vectors, (4094, 1000)):
        if any(type(v) is not int or not 0 <= v <= maximum for v in vector):
            raise DiagnosticError("Invalid raw/calibrated ADC pair")
    for field, mv in zip(numeric[:2], vectors[1]):
        if abs(frame[field]-13*mv) > 6:
            raise DiagnosticError("ADC divider scaling mismatch")
    return frame


def check_pico_rail_status(frame):
    """Validate one complete INA snapshot; freshness is measured from its trigger."""
    monitor = frame.get("rail_monitor")
    if not isinstance(monitor, dict):
        raise DiagnosticError("Missing/invalid Pico rail_monitor")
    for field, choices in (("state", PICO_RAIL_STATES), ("error", PICO_RAIL_ERRORS),
                           ("bus", PICO_RAIL_BUS_STATES)):
        if not isinstance(monitor.get(field), str) or monitor[field] not in choices:
            raise DiagnosticError(f"Invalid Pico rail monitor {field}")
    for field in ("seq", "age_us", "span_us"):
        if field not in monitor:
            raise DiagnosticError(f"Missing Pico rail monitor {field}")
    for field in PICO_RAIL_VOLTAGES:
        if field not in frame:
            raise DiagnosticError(f"Missing Pico rail voltage: {field}")
    if (monitor["state"] == "fault") != (monitor["error"] != "none"):
        raise DiagnosticError("Pico rail monitor state/error mismatch")
    if monitor["state"] != "ready":
        if (any(frame[field] is not None for field in PICO_RAIL_VOLTAGES)
                or any(monitor[field] is not None for field in ("seq", "age_us", "span_us"))):
            raise DiagnosticError("Unavailable Pico rail snapshot must be null")
        return
    if (monitor["bus"] != "ready" or frame.get("state") == "gpio_fault"
            or frame.get("reboot_pending") is not False
            or frame.get("external_watchdog_state") in ("stale", "init_failed", "gpio_fault")):
        raise DiagnosticError("Pico rail monitor cannot be ready in this bus/control state")
    for field in PICO_RAIL_VOLTAGES:
        value = frame[field]
        if type(value) is not int or not 0 <= value <= 26000 or value % 8:
            raise DiagnosticError(f"Invalid Pico INA bus voltage: {field}")
    # Mirrors include/ina3221_monitor.h and src/ina3221_monitor.cpp: uint32
    # sequence skips zero, sample lifetime <50ms, acquisition interval <30ms.
    for field, low, high in (("seq", 1, 0xffffffff), ("age_us", 0, 49999), ("span_us", 0, 29999)):
        value = monitor[field]
        if type(value) is not int or not low <= value <= high:
            raise DiagnosticError(f"Invalid Pico rail monitor {field}")
    if monitor["span_us"] > monitor["age_us"]:
        raise DiagnosticError("Pico rail acquisition span exceeds sample age")


def check_pico_status(frame):
    # Match the review firmware, including its readable GPIO-fault path.
    # FLT/PGOOD are raw digital levels, never USB sustain or battery evidence.
    for field in ("pwm_ready", "motor_arm_requested", "source_cycle_enabled", "odometry_qualified"):
        if frame.get(field) is not False:
            raise DiagnosticError(f"Unqualified Pico capability mismatch: {field}")
    watchdog_state = frame.get("external_watchdog_state")
    if (watchdog_state not in ("startup", "acquiring", "active", "stale", "init_failed", "gpio_fault")
            or type(frame.get("external_watchdog_ready")) is not bool
            or frame["external_watchdog_ready"] != (watchdog_state == "active")):
        raise DiagnosticError("Inconsistent Pico external watchdog service status")
    for field in ("x", "y", "theta"):
        if field not in frame or frame[field] is not None:
            raise DiagnosticError(f"Unavailable Pico measurement must be null: {field}")
    if frame.get("usb_reboot") is not True or frame["pwm_command"] != [0, 0, 0]:
        raise DiagnosticError("Pico must advertise USB reboot and inhibited PWM")
    if frame.get("wheel_speed_mps") != [None, None, None]:
        raise DiagnosticError("Unqualified Pico wheel speeds must be null")
    fault = frame.get("state") == "gpio_fault"
    if (watchdog_state == "gpio_fault") != fault:
        raise DiagnosticError("Pico watchdog/GPIO fault state mismatch")
    if watchdog_state == "active" and (frame.get("watchdog_ready") is not True
                                      or frame.get("control_timing_fault") is not False):
        raise DiagnosticError("Pico watchdog cannot be active without healthy task/control progress")
    if frame.get("state") not in ("idle", "planning", "ready", "done", "gpio_fault"):
        raise DiagnosticError("Unexpected Pico review state")
    for field in ("logic_flt_n", "logic_pgood"):
        if field not in frame or (frame[field] is not None if fault else
                                  type(frame[field]) is not int or frame[field] not in (0, 1)):
            raise DiagnosticError(f"Invalid Pico raw digital level: {field}")
    counts = frame.get("encoder_counts")
    if not isinstance(counts, list) or len(counts) != 3 or any(
            count is not None if fault else type(count) is not int or not -0x80000000 <= count <= 0x7fffffff
            for count in counts):
        raise DiagnosticError("Invalid Pico raw encoder counts")
    if fault:
        for field, expected in (("gpio_fault_latched", True), ("read_only", True),
                                ("wireless_control_enabled", False), ("telemetry_available", False),
                                ("reason", "PICO_SAFE_DEFAULTS_FAILED")):
            if field not in frame or (frame[field] != expected if isinstance(expected, str) else frame[field] is not expected):
                raise DiagnosticError(f"Invalid Pico latched-fault status: {field}")
        if type(frame.get("gpio_error")) is not int or frame["gpio_error"] == 0 or type(frame.get("gpio_release_error")) is not int:
            raise DiagnosticError("Pico GPIO fault must retain its SDK errors")
    elif frame.get("gpio_fault_latched", False) is not False:
        raise DiagnosticError("Pico GPIO fault contradicts the reported state")
    ready = frame.get("motor_pwm_ready")
    if not fault or "motor_pwm_ready" in frame:
        if not isinstance(ready, list) or len(ready) != 3 or any(v is not False for v in ready):
            raise DiagnosticError("Pico PWM channels must remain unqualified")
    check_pico_rail_status(frame)
    check_cell_state(frame)


def check_cell_state(frame):
    state, error = frame.get("cell_monitor"), frame.get("cell_error")
    if (not isinstance(state, str) or state not in PICO_CELL_STATES
            or not isinstance(error, str) or error not in PICO_CELL_ERRORS
            or (state == "fault") != (error != "none")):
        raise DiagnosticError("Invalid Pico cell monitor state/error")


def check_cell_report(frame, expected_profile):
    if (expected_profile != PICO_PROFILE or not isinstance(frame, dict)
            or (frame.get("protocol"), frame.get("firmware"), frame.get("profile"), frame.get("kind"))
            != (2, "wbb-control-2", PICO_PROFILE, "cells")):
        raise DiagnosticError("Not a selected Pico cell report")
    boot = frame.get("boot")
    if (not isinstance(boot, str) or len(boot) != 16 or any(c not in "0123456789abcdef" for c in boot)
            or not isinstance(frame.get("device"), str) or not frame["device"]):
        raise DiagnosticError("Invalid cell report identity")
    check_cell_state(frame)
    for field, expected in (("read_only", True), ("motion_enabled", False), ("charge_enabled", False),
                            ("system_protection_qualified", False), ("voltage_profile_verified", False),
                            ("battery_current_ma", None), ("pack_temperature_mc", None)):
        if field not in frame or frame[field] is not expected:
            raise DiagnosticError("Unqualified cell capability: " + field)
    for field in ("fresh", "wake_requested", "configuration_verified", "cell_pair_valid", "stack_consistent"):
        if type(frame.get(field)) is not bool:
            raise DiagnosticError("Invalid cell flag: " + field)
    state = frame["cell_monitor"]
    if (frame["wake_requested"] != (state == "waking")
            or frame["fresh"] != (state == "monitoring")
            or frame["configuration_verified"] != (state in ("monitoring", "stale"))):
        raise DiagnosticError("Cell state contradicts acquisition/configuration")
    def integer(field, low, high, nullable=False):
        if field not in frame or (frame[field] is not None or not nullable) and (
                type(frame[field]) is not int or not low <= frame[field] <= high):
            raise DiagnosticError("Invalid cell field: " + field)
    integer("generation", 0, 0xffffffff)
    integer("uptime_ms", 0, 0xffffffff)
    integer("device_number", 0x7605, 0x7605, True)
    integer("failing_address", 0, 0xffff, True)
    arrays = ("cell_mv", "safety_alert", "safety_status")
    for field in arrays:
        if not isinstance(frame.get(field), list) or len(frame[field]) != 2:
            raise DiagnosticError("Invalid cell pair field: " + field)
    measured = ("stack_mv", "ts_counts", "ts_nominal_uv", "seq", "age_ms", "read_span_ms",
                "battery_status", "alarm_status", "alarm_raw", "balance_mask", "fets_off_observed")
    if not frame["fresh"]:
        if (any(field not in frame or frame[field] is not None for field in measured)
                or any(v is not None for field in arrays for v in frame[field])
                or frame["cell_pair_valid"] or frame["stack_consistent"]):
            raise DiagnosticError("Unavailable cell report leaked values")
        return frame
    if frame["device_number"] != 0x7605 or frame.get("fets_off_observed") is not True:
        raise DiagnosticError("Fresh cells lack identity or OFF observation")
    for field, low, high, nullable in (("seq", 0, 0xffffffff, False), ("age_ms", 0, 2999, False),
            ("read_span_ms", 0, 1499, False), ("stack_mv", 1, 11000, True), ("ts_counts", 0, 32767, True),
            ("ts_nominal_uv", 0, 1800000, True), ("battery_status", 0, 0xffff, False),
            ("alarm_status", 0, 0xffff, False), ("alarm_raw", 0, 0xffff, False), ("balance_mask", 0, 0, False)):
        integer(field, low, high, nullable)
    lower, upper = frame["cell_mv"]
    if any(v is not None and (type(v) is not int or not 1 <= v <= 5500) for v in (lower, upper)):
        raise DiagnosticError("Invalid per-cell voltage")
    if (lower is None or lower < 2000) and upper is not None:
        raise DiagnosticError("Upper2S measurement requires valid lower cell >=2V")
    pair = (lower is not None and upper is not None and frame["stack_mv"] is not None
            and abs(lower + upper - frame["stack_mv"]) <= 250)
    if frame["cell_pair_valid"] != pair or frame["stack_consistent"] != pair:
        raise DiagnosticError("Cell/stack validity contradicts readings")
    if any(type(v) is not int or not 0 <= v <= 255 for field in arrays[1:] for v in frame[field]):
        raise DiagnosticError("Invalid cell safety bytes")
    if (frame["ts_counts"] is None) != (frame["ts_nominal_uv"] is None):
        raise DiagnosticError("Partial TS sample")
    if frame["ts_counts"] is not None and frame["ts_nominal_uv"] != (frame["ts_counts"] * 5981000 + 49152) // 98304:
        raise DiagnosticError("TS nominal conversion mismatch")
    return frame


def verify_restart(before, after, software=True, expected_profile=None):
    check_status(after, expected_profile or (before.get("profile") if before else None))
    if before and (after["boot"] == before["boot"] or after["device"] != before["device"]
                   or after["profile"] != before["profile"]):
        raise DiagnosticError("Old session or different board")
    if (type(after.get("uptime_ms")) is not int or not 0 <= after["uptime_ms"] <= 30000
            or after.get("state") != "idle" or after.get("reboot_pending") is not False
            or any(after["pwm_command"])):
        raise DiagnosticError("Application did not return to idle")
    if software and after.get("reset_reason") != 3:
        raise DiagnosticError("Expected software reset reason")
    if after["profile"] == PICO_PROFILE and after["external_watchdog_ready"] is not True:
        raise DiagnosticError("Pico external watchdog has not acquired healthy control progress")


class Wire:
    def __init__(self, args):
        self.args = args
        self.usb_kind = getattr(args, "usb_kind", "native")
        self.expected_profile = selected_profile(args)
        self.usb = self.sock = self.ble = None
        self.pending = deque()
        self.decoder = FrameDecoder()
        self.overflow = False
        self.target = None
        self.notifications = asyncio.Queue(maxsize=128)

    @property
    def wireless(self):
        return not self.args.usb

    async def open(self):
        self.decoder = FrameDecoder()
        self.pending.clear()
        if self.args.usb:
            from serial.tools import list_ports
            ports = list(list_ports.comports())
            if self.target:
                path = match_port(ports, self.target)
                if not path:
                    raise DiagnosticError("Target USB device not present")
            else:
                p = select_port(ports, self.args.usb, self.usb_kind)
                self.target = identity(p)
                path = p.device
            self.usb = open_serial(path)
        elif self.args.wifi:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.connect((self.args.wifi, 4243))
            self.sock.setblocking(False)
        else:
            from bleak import BleakClient
            self.notifications = asyncio.Queue(maxsize=128)
            self.overflow = False
            self.ble = BleakClient(self.args.ble, timeout=10)
            await self.ble.connect()

            def notified(_characteristic, data):
                try:
                    self.notifications.put_nowait(bytes(data))
                except asyncio.QueueFull:
                    self.overflow = True

            await self.ble.start_notify(TX_UUID, notified)

    async def close(self):
        if self.usb:
            self.usb.close()
            self.usb = None
        if self.sock:
            self.sock.close()
            self.sock = None
        if self.ble:
            await self.ble.disconnect()
            self.ble = None

    async def send(self, data):
        if len(data) >= (PICO_LINE_LIMIT if self.expected_profile == PICO_PROFILE else 1536):
            raise DiagnosticError("Command exceeds firmware frame limit")
        if self.usb:
            if self.usb.write(data) != len(data):
                raise DiagnosticError("Incomplete USB write")
        elif self.sock:
            await asyncio.get_running_loop().sock_sendall(self.sock, data)
        elif self.ble:
            for i in range(0, len(data), 20):
                await self.ble.write_gatt_char(RX_UUID, data[i:i+20], response=True)
        else:
            raise DiagnosticError("Transport is closed")

    async def recv(self, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.pending:
                return self.pending.popleft()
            if self.usb:
                data = await asyncio.to_thread(self.usb.read, min(self.usb.in_waiting or 1, 4096))
            elif self.sock:
                data = await asyncio.wait_for(asyncio.get_running_loop().sock_recv(self.sock, 4096),
                                              max(.01, deadline-time.monotonic()))
            elif self.ble:
                if self.overflow or not self.ble.is_connected:
                    raise DiagnosticError("BLE disconnected or receive queue overflowed")
                data = await asyncio.wait_for(self.notifications.get(), max(.01, deadline-time.monotonic()))
            else:
                raise DiagnosticError("Transport is closed")
            for frame in self.decoder.feed(data):
                if "payload" in frame:
                    self.pending.append(frame)
        raise DiagnosticError("Control reply timed out")


class Controller:
    def __init__(self, wire, key, expected_profile=None):
        self.wire, self.key = wire, key
        bound = getattr(wire, "expected_profile", None)
        if expected_profile is not None and bound is not None and expected_profile != bound:
            raise DiagnosticError("Controller profile conflicts with transport selection")
        self.expected_profile = expected_profile or bound
        if self.expected_profile is not None and self.expected_profile not in PROFILES:
            raise DiagnosticError("Unknown application profile")
        self.boot = None
        self.profile = self.device = None
        self.sequence, self.lease = 0, 0
        self.latencies = []

    async def hello(self):
        await self.wire.send(b"hello\n")
        result = unpack(await self.wire.recv(), self.key, self.wire.wireless, self.expected_profile)
        check_status(result, self.expected_profile)
        if result["id"] != 0 or not result["ok"]:
            raise DiagnosticError("Unexpected discovery response")
        if self.boot and self.boot != result["boot"]:
            raise DiagnosticError("Unexpected MCU reset; old control lease is invalid")
        if self.profile and (result["profile"] != self.profile or result["device"] != self.device):
            raise DiagnosticError("Board/profile changed during discovery")
        self.boot = result["boot"]
        self.profile, self.device = result["profile"], result["device"]
        return result

    async def request(self, cmd, params=None):
        if self.expected_profile and self.profile != self.expected_profile:
            raise DiagnosticError("Verify the selected board profile with hello before sending commands")
        if self.profile == PICO_PROFILE and cmd in PICO_BLOCKED_COMMANDS:
            raise CommandRejected("Pico review forbids motion and source-cycle commands")
        if cmd == "cell_status" and (self.profile != PICO_PROFILE or self.expected_profile != PICO_PROFILE):
            raise CommandRejected("Cell diagnostics require the selected Pico profile")
        if cmd == "rail_status" and (self.profile != KISS_C6_PROFILE or self.expected_profile != KISS_C6_PROFILE):
            raise CommandRejected("Rail diagnostics require the selected KISS C6 profile")
        self.sequence += 1
        fields = dict(params or {})
        if any(k in fields for k in ("id", "boot", "lease", "cmd", "payload", "mac")):
            raise DiagnosticError("Command parameters cannot override protocol fields")
        fields.update(id=self.sequence, boot=self.boot, lease=self.lease, cmd=cmd)
        began = time.monotonic()
        await self.wire.send(pack(fields, self.key if self.wire.wireless else None))
        while True:
            result = unpack(await self.wire.recv(max(.01, 3-(time.monotonic()-began))), self.key, self.wire.wireless,
                            self.expected_profile)
            if result.get("boot") != self.boot:
                raise DiagnosticError("MCU rebooted during command; motion will not resume")
            if result["id"] != self.sequence:
                if time.monotonic()-began >= 3:
                    raise DiagnosticError("No matching acknowledgement")
                continue
            self.latencies.append((time.monotonic()-began)*1000)
            if not result["ok"]:
                raise CommandRejected(f"Device rejected command: {result.get('error')}")
            if cmd in ("status", "stop"):
                check_status(result, self.expected_profile)
                if self.profile and (result["profile"] != self.profile or result["device"] != self.device):
                    raise DiagnosticError("Board/profile changed during command")
            if cmd == "cell_status":
                check_cell_report(result, self.expected_profile)
                if result["device"] != self.device:
                    raise DiagnosticError("Board changed during cell readout")
            if cmd == "rail_status":
                check_kiss_rail_report(result, self.expected_profile)
                if result.get("device") != self.device:
                    raise DiagnosticError("Board changed during rail readout")
            if cmd == "claim":
                if type(result.get("lease")) is not int or not result["lease"]:
                    raise DiagnosticError("Claim acknowledged without a control lease")
                self.lease = result["lease"]
            if cmd in ("stop", "release", "reboot"):
                self.lease = 0
            return result


async def request_reboot(client, result):
    """Send once; loss of the ACK still requires a new, authenticated boot."""
    try:
        await client.request("reboot")
        result["reboot_ack_received"] = True
    except CommandRejected:
        raise
    except (DiagnosticError, TimeoutError, OSError) as exc:
        result["reboot_ack_received"] = False
        result["reboot_ack_error"] = str(exc) or type(exc).__name__


async def wait_for_restart(wire, key, before, software=True, timeout=20, interval=.3, expected_profile=None):
    expected_profile = expected_profile or getattr(wire, "expected_profile", None) or (before.get("profile") if before else None)
    deadline = time.monotonic()+timeout
    error = "Application not returned"
    while time.monotonic() < deadline:
        await asyncio.sleep(interval)
        try:
            await wire.open()
            after = await Controller(wire, key, expected_profile).hello()
            verify_restart(before, after, software=software, expected_profile=expected_profile)
            return after
        except Exception as exc:
            error = str(exc)
            await wire.close()
    raise DiagnosticError(f"Reset sent, return NOT verified: {error}")


async def run(args):
    usb_kind = getattr(args, "usb_kind", "native")
    expected_profile = selected_profile(args)
    if args.action == "cells" and expected_profile != PICO_PROFILE:
        raise DiagnosticError("Cell diagnostics require explicit Pico selection")
    if args.action == "rails" and expected_profile != KISS_C6_PROFILE:
        raise DiagnosticError("Rail diagnostics require explicit KISS C6 selection")
    if expected_profile == PICO_PROFILE and args.action in PICO_BLOCKED_COMMANDS:
        raise CommandRejected("Pico review forbids motion and source-cycle commands")
    if args.scan_ble:
        from bleak import BleakScanner
        found = await BleakScanner.discover(timeout=min(args.seconds, 30), return_adv=True,
                                           service_uuids=[SERVICE_UUID])
        return {"action":"scan_ble", "result":"DISCOVERED", "authenticated":False,
                "devices":[{"address":device.address, "name":adv.local_name or device.name,
                            "rssi_dbm":adv.rssi} for device,adv in found.values()]}
    key = None
    if args.key_file and args.key_file.exists():
        key = bytes.fromhex(args.key_file.read_text().strip())
        if len(key) != 32:
            raise DiagnosticError("Key file must contain 64 hexadecimal characters")
    if (args.wifi or args.ble) and not key:
        raise DiagnosticError("Wireless control requires a USB-provisioned --key-file")
    wire = Wire(args)
    result = {"action": args.action, "result": "FAIL", "hardware_power_measured": False}
    if usb_kind in UART_USB_KINDS:
        result.update(usb_kind=usb_kind, custom_pcb_identity_verified=False,
                      physical_power_cycle=False, flash_modified=False)
    if expected_profile == PICO_PROFILE:
        result.update(expected_profile=PICO_PROFILE, app_profile_verified=False,
                      motion_enabled=False, charge_enabled=False, source_cycle_enabled=False,
                      physical_power_cycle=False)
    client = None
    try:
        if args.action in ("reset", "bootloader"):
            if not args.usb:
                raise DiagnosticError("ROM recovery requires an explicit --usb path")
            from serial.tools import list_ports
            selected = select_port(list_ports.comports(), args.usb, usb_kind)
            wire.target = identity(selected)
            result["usb_device"] = wire.target
            command = rom_command(selected.device, args.action == "bootloader", usb_kind)
            result["rom_command"] = command
            if usb_kind in UART_USB_KINDS:
                # esptool custom_reset_sequence overrides --before. Keep bridge recovery
                # on its declared UART strategy, independent of user esptool.cfg.
                with tempfile.TemporaryDirectory(prefix="wbb-uart-rom-") as folder:
                    config_path = Path(folder) / "esptool.cfg"
                    config_path.write_text(rom_config_text(usb_kind))
                    rom_env = dict(os.environ, ESPTOOL_CFGFILE=str(config_path), ESPTOOL_OPEN_PORT_ATTEMPTS="1")
                    done = await asyncio.to_thread(subprocess.run, command, env=rom_env,
                                                   capture_output=True, text=True, timeout=15)
            else:
                done = await asyncio.to_thread(subprocess.run, command,
                                               capture_output=True, text=True, timeout=15)
            result.update(esptool_stdout=done.stdout, esptool_stderr=done.stderr, esptool_exit_code=done.returncode)
            if done.returncode:
                raise DiagnosticError(f"{usb_chip(usb_kind)} ROM recovery failed")
            if usb_kind in UART_USB_KINDS:
                result["usb_port_after_rom"] = match_port(list_ports.comports(), wire.target)
                if not result["usb_port_after_rom"]:
                    raise DiagnosticError(f"{usb_kind.upper()} bridge identity lost after ROM recovery")
            result.update(rom_chip=usb_chip(usb_kind), rom_chip_verified=True,
                          custom_pcb_identity_verified=False)
            if args.action == "bootloader":
                chip_label = "S3" if usb_kind == "ch343p" else "C6"
                result.update(result=f"{chip_label}_ROM_VERIFIED", application_running=False,
                              identity_scope=f"{chip_label} ROM and USB interface only; custom PCB identity is unproven")
                return result
            result["result"] = "RESET_SENT"
            before = None
        else:
            await wire.open()
            client = Controller(wire, key, expected_profile)
            before = await client.hello()
            result["before"] = before
            if usb_kind in UART_USB_KINDS:
                result.update(usb_device=wire.target, app_profile_verified=True)
            if expected_profile == PICO_PROFILE:
                result["app_profile_verified"] = True
            if args.action == "provision":
                if not args.usb or not args.wifi_config or not args.key_file:
                    raise DiagnosticError("Provision requires --usb, --wifi-config and --key-file")
                config = json.loads(args.wifi_config.read_text())
                if set(config) != {"ssid", "password"}:
                    raise DiagnosticError("Wi-Fi config must contain only ssid and password")
                if key is None:
                    key = secrets.token_bytes(32)
                    fd = os.open(args.key_file, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
                    with os.fdopen(fd, "w") as stream:
                        stream.write(key.hex()+"\n")
                await client.request("provision", dict(config, key=key.hex()))
                await request_reboot(client, result)
                result["result"] = "CONFIG_SAVED_REBOOT_SENT"
            elif args.action == "reboot":
                if wire.wireless:
                    await client.request("claim")
                await request_reboot(client, result)
                result["result"] = "REBOOT_SENT"
            elif args.action == "watch":
                deadline = time.monotonic()+args.seconds
                samples = []
                while time.monotonic() < deadline:
                    samples.append(check_status(await client.request("status"), expected_profile))
                    await asyncio.sleep(.5)
                result.update(result="CAPTURED", samples=samples)
            elif args.action in ("status", "stop", "cells", "rails"):
                operation = {"cells": "cell_status", "rails": "rail_status"}.get(args.action, args.action)
                result.update(result="ACKNOWLEDGED", after=await client.request(operation))
            else:
                await client.request("claim")
                params = json.loads(args.params)
                if not isinstance(params, dict):
                    raise DiagnosticError("--params must be a JSON object")
                result["after"] = await client.request(args.action, params)
                # Future qualified builds remain supervised for the whole move.
                deadline = time.monotonic()+args.seconds
                if args.action in ("start", "goto", "wheel"):
                    while time.monotonic() < deadline:
                        await client.request("heartbeat")
                        await asyncio.sleep(.1)
                await client.request("stop")
                result["result"] = "ACKNOWLEDGED_AND_STOPPED"
        if args.action in ("reset", "reboot", "provision"):
            await wire.close()
            client = None
            after = await wait_for_restart(wire, key, before, software=args.action != "reset",
                                           expected_profile=expected_profile)
            result.update(result="APP_RETURN_VERIFIED", after=after,
                          new_boot_verified=before is not None, prior_app_observed=before is not None)
            if usb_kind in UART_USB_KINDS or expected_profile == PICO_PROFILE:
                result["app_profile_verified"] = True
        if client:
            result["round_trip_ms"] = client.latencies
        return result
    except Exception as exc:
        result.update(result="FAIL", error=str(exc) or type(exc).__name__)
        if client:
            result["round_trip_ms"] = client.latencies
        return result
    finally:
        if client and client.lease:
            try:
                await client.request("stop")
            except Exception:
                pass  # Board's lease timeout still applies; never resume automatically.
        await wire.close()


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument("--usb", metavar="PORT")
    group.add_argument("--wifi", metavar="HOST")
    group.add_argument("--ble", metavar="ADDRESS_OR_MACOS_UUID")
    group.add_argument("--scan-ble", action="store_true", help="List advertised control services; discovery is unauthenticated")
    p.add_argument("--usb-kind", choices=["native", "cp2102n", "ch343p", "ch343g"], default="native",
                   help="USB interface: cp2102n/R3, ch343p/Pico, ch343g/KISS C6; bridges require explicit --usb PORT")
    p.add_argument("--profile", choices=PROFILES,
                   help="Expected application identity; required for Pico/KISS C6 over Wi-Fi/BLE, checked against USB kind")
    p.add_argument("action", nargs="?", default="status", choices=["status", "cells", "rails", "stop", "watch", "reboot", "reset", "bootloader", "provision",
                                      "area", "pos", "set_speed", "start", "goto", "wheel"])
    p.add_argument("--key-file", type=Path)
    p.add_argument("--wifi-config", type=Path)
    p.add_argument("--params", default="{}", help="JSON command parameters")
    p.add_argument("--seconds", type=float, default=5)
    p.add_argument("--report", type=Path)
    args = p.parse_args(argv)
    try:
        expected_profile = selected_profile(args)
    except DiagnosticError as exc:
        p.error(str(exc))
    if expected_profile == PICO_PROFILE and args.action in PICO_BLOCKED_COMMANDS:
        p.error("Pico review forbids motion and source-cycle commands")
    if args.scan_ble and args.action != "status":
        p.error("--scan-ble cannot execute a control action")
    if not .1 <= args.seconds <= 3600:
        p.error("--seconds must be 0.1..3600")
    try:
        result = asyncio.run(run(args))
    except Exception as exc:
        result = {"result": "FAIL", "action": args.action, "error": str(exc)}
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 1 if result["result"] == "FAIL" else 0


if __name__ == "__main__":
    raise SystemExit(main())
