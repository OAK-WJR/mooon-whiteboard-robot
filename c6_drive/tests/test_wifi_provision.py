"""Offline provisioning checks. Credentials are fixtures, never real passwords."""
import copy
import io
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
import c6_wifi_provision as provision


class Client:
    def __init__(self):
        self.now = 0.
        self.last_status = 0.
        self.calls = []
        self.latest = dict(ms=0, session=12, usb_wifi_revision=1, ready=True, armed=False,
                           ap='test-ap', firmware='wbb-c6-drive-1', calibrated=False,
                           motors=[dict(pwm=0, fi_duty=0, bi_duty=0, fi_level=0, bi_level=0,
                                        speed=0, gains=[1, 2]) for _ in range(3)], network={})

    def request(self, op, **kwargs):
        self.calls.append((op, kwargs))
        return dict(ok=True, saved=True)

    def pump(self, seconds):
        self.now += seconds
        self.last_status = self.now
        self.latest['ms'] = round(self.now*1000)
        self.latest['network'] = dict(connected=True, ssid='test-home', busy=False,
                                     state='connected', station_ip='192.0.2.4',
                                     station_url='http://192.0.2.4/')


class ProvisionTests(unittest.TestCase):
    def test_password_not_in_report_and_no_motion_ops(self):
        c = Client(); result = {}
        with patch('c6_drive_sysid.stop_and_wait'), patch('time.monotonic', side_effect=lambda:c.now), \
                patch('sys.stdout', new=io.StringIO()) as out:
            url = provision.provision(c, 'test-home', 'fixture-secret', result)
            self.assertNotIn('fixture-secret', out.getvalue())
        self.assertEqual(url, 'http://192.0.2.4/')
        self.assertEqual([op for op, _ in c.calls], ['wifi_config'])
        self.assertTrue(result['connected'])
        self.assertNotIn('fixture-secret', json.dumps(result))
        self.assertTrue(result['gains_unchanged'])

    def test_old_firmware_or_output_fault_does_not_receive_password(self):
        for field, value in [('usb_wifi_revision', 0), ('ready', False), ('armed', True)]:
            c = Client();c.latest[field] = value
            with patch('c6_drive_sysid.stop_and_wait'), self.assertRaises(RuntimeError):
                provision.provision(c, 'test-home', 'fixture-secret', {})
            self.assertEqual(c.calls, [])

    def test_timeout_preserves_failure_instead_of_claiming_connection(self):
        c = Client()
        pump = c.pump
        def no_connection(seconds):
            pump(seconds)
            c.latest['network']['connected'] = False
        c.pump = no_connection
        result = {}
        with patch('c6_drive_sysid.stop_and_wait'), patch('time.monotonic', side_effect=lambda:c.now), \
                patch('sys.stdout', new=io.StringIO()), self.assertRaisesRegex(RuntimeError, 'timeout'):
            provision.provision(c, 'test-home', 'fixture-secret', result, timeout=.3)
        self.assertTrue(result['saved'])
        self.assertFalse(result.get('connected', False))

    def test_protocol_byte_limits_and_embedded_nul(self):
        for ssid, password in [('x'*33, 'fixture-secret'), ('home', 'short'),
                               ('bad\0ssid', 'fixture-secret'), ('home', 'g'*64)]:
            with self.assertRaises(ValueError):
                provision.validate_input(ssid, password)
        provision.validate_input('test-home', '')
        provision.validate_input('test-home', 'a'*64)


if __name__ == '__main__':
    unittest.main()
