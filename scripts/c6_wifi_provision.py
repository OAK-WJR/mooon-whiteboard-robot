#!/usr/bin/env python3
"""Provision personal Wi-Fi over USB without changing the computer's network.

Password input is hidden and never included in reports or command-line arguments.
The only robot operations are STOP, wifi_config and connection-info readback.
"""
import argparse
from datetime import datetime, timezone
import getpass
import json
from pathlib import Path
import statistics
import sys
import time


def validate_input(ssid, password):
    if not 1 <= len(ssid.encode('utf-8')) <= 32 or '\0' in ssid:
        raise ValueError('SSID must be 1–32 bytes, without NUL')
    size = len(password.encode('utf-8'))
    if '\0' in password or not (size == 0 or 8 <= size <= 63 or
            (size == 64 and all(c in '0123456789abcdefABCDEF' for c in password))):
        raise ValueError('Invalid personal Wi-Fi password length/format')
    payload = dict(id=99999999, op='wifi_config', ssid=ssid, password=password, auth='personal')
    if len(json.dumps(payload).encode()) > 512:
        raise ValueError('Credentials exceed USB protocol size limit; use the AP form')


def provision(client, ssid, password, report, timeout=45):
    from c6_drive_sysid import stopped, stop_and_wait
    validate_input(ssid, password)
    stop_and_wait(client)
    initial = client.latest
    if initial.get('usb_wifi_revision', 0) < 1:
        raise RuntimeError('Connected firmware does not support USB Wi-Fi provisioning')
    if not initial.get('ready') or not stopped(initial):
        raise RuntimeError('Outputs are not verified stopped')
    report.update(session=initial['session'], ap=initial['ap'], firmware=initial['firmware'])
    # A previously saved misspelled SSID may currently be retrying. Wait for
    # its bounded attempt to finish; do not bypass the firmware's busy guard.
    idle_deadline = time.monotonic() + 35
    while client.latest.get('network', {}).get('busy'):
        if time.monotonic() >= idle_deadline:
            raise RuntimeError('Previous Wi-Fi attempt remained busy')
        client.pump(.07)
        if (time.monotonic() - client.last_status > .25 or
                client.latest['session'] != initial['session'] or
                not client.latest.get('ready') or not stopped(client.latest)):
            raise RuntimeError('Unsafe or stale status while waiting for Wi-Fi idle')
    client.request('wifi_config', ssid=ssid, password=password, auth='personal')
    report['saved'] = True
    deadline = time.monotonic() + timeout
    last_state = None
    verified, previous_ms, previous_ip = 0, None, None
    while time.monotonic() < deadline:
        client.pump(.07)
        s = client.latest
        if time.monotonic() - client.last_status > .25:
            raise RuntimeError('USB telemetry stale during provisioning')
        if s['session'] != initial['session']:
            raise RuntimeError('Board rebooted during provisioning')
        if not s.get('ready') or not stopped(s):
            raise RuntimeError('Stopped-output verification failed')
        network = s.get('network', {})
        state = (network.get('state'), network.get('disconnect_reason'))
        if state != last_state:
            print(json.dumps({'network_state': state[0], 'disconnect_reason': state[1]}), flush=True)
            last_state = state
        ip = network.get('station_ip')
        connected = (network.get('connected') and network.get('ssid') == ssid
                     and ip not in (None, '', '0.0.0.0') and not network.get('busy'))
        if s['ms'] != previous_ms:
            verified = verified + 1 if connected and previous_ip == ip else int(bool(connected))
            previous_ms, previous_ip = s['ms'], ip
        report['network'] = network
        if verified >= 20:
            report.update(connected=True, verified_stopped_statuses=verified,
                          gains_unchanged=[m['gains'] for m in s['motors']] ==
                                          [m['gains'] for m in initial['motors']],
                          calibrated=s.get('calibrated'))
            return network['station_url']
    raise RuntimeError('Wi-Fi connection not verified within timeout; saved network retained')


def verify_lan(client, url, report):
    from c6_wifi_client import WifiClient
    from c6_drive_sysid import stopped
    # Read the existing device login key through its documented local USB API.
    # Keep it in memory; it is unrelated to the supplied Wi-Fi network password.
    info = client.request('info')
    key = info['password']
    times = []
    remote = None
    try:
        for _ in range(30):
            before = time.monotonic()
            if remote is None:
                remote = WifiClient(url, device_key=key)
            else:
                remote._status()
            times.append((time.monotonic() - before) * 1000)
            f = remote.latest
            if f['session'] != client.session or not f.get('ready') or not stopped(f):
                raise RuntimeError('LAN device identity/stopped state mismatch')
            client.pump(.05)
        report['lan'] = dict(reachable=True, status_reads=len(times),
                             median_rtt_ms=statistics.median(times), max_rtt_ms=max(times))
    except Exception as error:
        report['lan'] = dict(reachable=False, status_reads=len(times), error=type(error).__name__)
    finally:
        if remote:
            remote.close()  # Read-only client: no command is sent.
        key = None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/cu.usbmodem1101')
    parser.add_argument('--ssid', required=True)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--password-stdin', action='store_true')
    args = parser.parse_args()
    if args.report.exists():
        parser.error('Use a new report path')
    password = sys.stdin.readline().rstrip('\r\n') if args.password_stdin else getpass.getpass('Wi-Fi password (hidden): ')
    validate_input(args.ssid, password)
    from c6_drive_tune import Client
    from c6_drive_sysid import write_json, stop_and_wait
    report = dict(schema='wbb-usb-wifi-provision-1', started_utc=datetime.now(timezone.utc).isoformat(),
                  ssid=args.ssid, saved=False, connected=False, motion_commands_sent=False)
    client = None
    try:
        client = Client(args.port)  # No raw logfile: no credential ACK is saved.
        url = provision(client, args.ssid, password, report)
        password = None
        print('Wi-Fi connected; checking read-only LAN access', flush=True)
        verify_lan(client, url, report)
    except Exception as error:
        report['error'] = str(error)
    finally:
        password = None
        if client:
            try:
                stop_and_wait(client)
                report['stopped'] = True
            except Exception as error:
                report.update(stopped=False, stop_error=str(error))
            finally:
                client.s.close()
        write_json(args.report, report)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if not report.get('connected') or not report.get('stopped'):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
