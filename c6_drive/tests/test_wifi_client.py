"""Offline transport contract checks; no socket or serial connection."""
import io
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from c6_wifi_client import WifiClient,NoRedirect


class Reply(io.BytesIO):
    pass


class FakeHTTP:
    def __init__(self):
        self.calls=[];self.session=12;self.error=None;self.ms=100
    def open(self,request,timeout):
        self.calls.append(request)
        assert timeout<=.2
        if request.full_url.endswith('/status'):
            return Reply(json.dumps(dict(type='status',firmware='wbb-c6-drive-1',session=self.session,ms=self.ms)).encode())
        if request.data is None:return Reply(b"const token='0123456789ABCDEF'")
        command=json.loads(request.data)
        return Reply(json.dumps(dict(ok=not self.error,error=self.error,id=command['id'],session=self.session,lease=7)).encode())


class WifiTests(unittest.TestCase):
    def client(self,backend):
        with patch('c6_wifi_client.build_opener',return_value=backend):return WifiClient('http://wbb-test.local/')
    def test_observer_never_sends_stop_or_fetches_token(self):
        backend=FakeHTTP();client=self.client(backend);client.close()
        self.assertEqual(len(backend.calls),1)
        self.assertIsNone(backend.calls[0].data)
        self.assertTrue(backend.calls[0].full_url.endswith('/status'))
    def test_commands_have_token_and_close_stops(self):
        backend=FakeHTTP();client=self.client(backend)
        self.assertEqual(client.request('arm')['lease'],7);client.close()
        posts=[r for r in backend.calls if r.data]
        self.assertEqual([json.loads(r.data)['op'] for r in posts],['arm','stop'])
        self.assertTrue(all(r.get_header('X-control-token')=='0123456789ABCDEF' for r in posts))
    def test_validation_cannot_be_bypassed(self):
        backend=FakeHTTP();client=self.client(backend)
        for op in ('pwm','validate_robust','validate_hold','save_controller','gains'):
            with self.assertRaises(ValueError):client.request(op)
        self.assertEqual(len(backend.calls),1)
        backend.error='NEEDS_CALIBRATION'
        with self.assertRaisesRegex(RuntimeError,'NEEDS_CALIBRATION'):client.request('arm')
    def test_duplicate_status_and_reboot(self):
        backend=FakeHTTP();client=self.client(backend);client._status()
        self.assertEqual(len(client.frames),1)
        backend.ms+=1;client._status();self.assertEqual(len(client.frames),2)
        backend.session+=1
        with self.assertRaisesRegex(RuntimeError,'rebooted'):client._status()
    def test_url_and_redirect_boundaries(self):
        for url in ('https://test/','http://user:pass@test/','http://test/cmd','http://test/?token=secret'):
            with self.assertRaises(ValueError):WifiClient(url)
        with self.assertRaisesRegex(RuntimeError,'redirect'):NoRedirect().redirect_request(None)


if __name__=='__main__':unittest.main()
