"""Offline actual-serializer cell reports and adversarial host/client tests."""
import configparser
import copy
from functools import lru_cache
import json
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest
from unittest.mock import patch
from test_pico_host import ROOT, host, args, pico
from test_host import FakeWire, KEY, envelope
from verify_pico_profile import public_copy


@lru_cache(maxsize=1)
def frames():
    with tempfile.TemporaryDirectory(prefix='wbb-cell-host-') as folder:
        stage=Path(folder);public_copy(stage)
        config=configparser.ConfigParser(interpolation=None);config.read(stage/'platformio.ini')
        flags=shlex.split(config['env:top_board_v2_pico']['build_flags'])
        inc=next((ROOT/'.pio/libdeps').glob('*/ArduinoJson/src'))
        cmd=['c++','-std=c++17','-fsanitize=address,undefined','-fno-sanitize-recover=all',
             '-Icontrol_tests/pico_mock','-Icontrol_tests/mock','-Iinclude','-Isrc','-I'+str(inc),
             '-DCONFIG_IDF_TARGET_ESP32S3=1',*flags,'-MMD','-MF','fixture.d',
             'control_tests/pico_cell_host_fixture.cpp','-o','fixture']
        built=subprocess.run(cmd,cwd=stage,capture_output=True,text=True,timeout=90)
        if built.returncode:raise AssertionError(built.stdout+built.stderr)
        assert 'include/config.h' not in (stage/'fixture.d').read_text()
        result={}
        for mode in ('not_started','waking','settling','detecting','configuring','awaiting_scan',
                     'monitoring','fault','stopped','missing','lower_invalid','safety'):
            run=subprocess.run([stage/'fixture',mode],cwd=stage,capture_output=True,text=True,timeout=15,check=True)
            assert run.stdout.endswith('\n') and len(run.stdout.encode())<2304
            result[mode]=json.loads(run.stdout)
        return result


def report(mode='monitoring', **changes):
    result=json.loads(frames()[mode]['payload']);result.update(changes);return result


class CellSchemaTests(unittest.TestCase):
    def checked(self, frame):return host.check_cell_report(frame,host.PICO_PROFILE)
    def rejects(self, frame):
        with self.assertRaises(host.DiagnosticError):self.checked(frame)

    def test_actual_serializer_all_phases_and_failures(self):
        for mode,wire in frames().items():
            with self.subTest(mode=mode):
                frame=host.unpack(wire,bytes.fromhex('aa'*32),True,host.PICO_PROFILE)
                self.assertIs(self.checked(frame),frame)
                self.assertFalse(frame['system_protection_qualified'])
                self.assertIsNone(frame['pack_temperature_mc'])

    def test_actual_physical_cell1_and_cell5_readings(self):
        self.assertEqual(report()['cell_mv'],[3700,3800])
        self.assertEqual(report()['stack_mv'],7500)
        self.assertEqual(report('lower_invalid')['cell_mv'],[1800,None])
        self.assertFalse(report('lower_invalid')['cell_pair_valid'])
        self.assertEqual(report('safety')['safety_status'],[0x40,0])

    def test_required_identity_and_explicit_profile(self):
        for expected in (None,host.R3_PROFILE,'top_board_v2'):
            with self.assertRaises(host.DiagnosticError):host.check_cell_report(report(),expected)
        for changes in ({'profile':host.R3_PROFILE},{'kind':'status'},{'boot':'abc'},{'device':''},
                        {'protocol':1},{'firmware':'other'}):self.rejects(report(**changes))

    def test_required_measurements_cannot_disappear(self):
        for field in ('cell_mv','stack_mv','ts_counts','ts_nominal_uv','seq','age_ms','read_span_ms',
                      'battery_status','alarm_status','alarm_raw','balance_mask','fets_off_observed',
                      'safety_alert','safety_status','device_number','generation','uptime_ms','failing_address'):
            for mode in ('monitoring','stopped'):
                frame=report(mode);del frame[field]
                with self.subTest(field=field,mode=mode):self.rejects(frame)

    def test_capabilities_remain_inhibited(self):
        for field in ('motion_enabled','charge_enabled','system_protection_qualified','voltage_profile_verified'):
            for bad in (True,0,None,'false'):self.rejects(report(**{field:bad}))
        for field in ('battery_current_ma','pack_temperature_mc'):
            self.rejects(report(**{field:0}))

    def test_state_error_and_boolean_contradictions(self):
        for field in ('fresh','wake_requested','configuration_verified','cell_pair_valid','stack_consistent'):
            for bad in (0,1,None,'true'):self.rejects(report(**{field:bad}))
        for changes in ({'cell_monitor':'fault'},{'cell_error':'bus_error'},{'wake_requested':True},
                        {'fresh':False},{'configuration_verified':False},{'cell_error':'unknown'}):
            self.rejects(report(**changes))

    def test_integer_bounds_and_bool_rejection(self):
        for field,bad in (('age_ms',3000),('age_ms',-1),('read_span_ms',1500),('seq',2**32),
                          ('generation',-1),('device_number',0x7604),('stack_mv',11001),
                          ('battery_status',65536),('balance_mask',1),('alarm_raw',True)):
            self.rejects(report(**{field:bad}))
        for field in ('seq','age_ms','stack_mv','ts_counts'):
            for bad in (True,'3700',3700.0):self.rejects(report(**{field:bad}))

    def test_nonfresh_never_reuses_previous_values(self):
        source=report()
        for field in ('cell_mv','stack_mv','ts_counts','seq','age_ms','read_span_ms','battery_status',
                      'safety_alert','safety_status','fets_off_observed'):
            self.rejects(report('stopped',**{field:source[field]}))

    def test_partial_or_invalid_cells_and_stack_consistency(self):
        for cells in ([True,3800],[3700.0,3800],[1800,3800],[None,3800],[0,3800],[5501,3800],[3700]):
            self.rejects(report(cell_mv=cells))
        self.rejects(report(stack_mv=7000))
        self.rejects(report(cell_pair_valid=False))
        self.rejects(report(stack_consistent=False))
        self.checked(report(cell_mv=[3700,3800],stack_mv=7000,cell_pair_valid=False,stack_consistent=False))

    def test_ts_is_nominal_voltage_not_pack_temperature(self):
        self.rejects(report(ts_counts=None))
        self.rejects(report(ts_nominal_uv=1800001))
        self.rejects(report(ts_nominal_uv=0))
        self.checked(report(ts_counts=None,ts_nominal_uv=None))

    def test_hmac_covers_cell_values_and_faults(self):
        wire=copy.deepcopy(frames()['monitoring']);wire['payload']=wire['payload'].replace('3700','3600')
        with self.assertRaises(host.DiagnosticError):host.unpack(wire,bytes.fromhex('aa'*32),True,host.PICO_PROFILE)


class CellClientTests(unittest.IsolatedAsyncioTestCase):
    async def test_usb_and_authenticated_wireless_cell_status(self):
        for wireless in (False,True):
            hello=pico();hello.update(boot=report()['boot'],device=report()['device'])
            response=report(id=1)
            wire=FakeWire([envelope(hello),envelope(response)]);wire.wireless=wireless
            client=host.Controller(wire,KEY,host.PICO_PROFILE);await client.hello()
            self.assertEqual((await client.request('cell_status'))['cell_mv'],[3700,3800])
            command=json.loads(wire.sent[-1]);command=json.loads(command['payload']) if wireless else command
            self.assertEqual(command['cmd'],'cell_status');self.assertEqual(command['lease'],0)

    async def test_identity_change_is_rejected(self):
        hello=pico();response=report(id=1,boot=hello['boot'],device='other-board')
        wire=FakeWire([envelope(hello),envelope(response)])
        client=host.Controller(wire,KEY,host.PICO_PROFILE);await client.hello()
        with self.assertRaises(host.DiagnosticError):await client.request('cell_status')

    async def test_cli_wrong_profile_fails_before_io(self):
        with patch.object(host,'Wire') as wire:
            with self.assertRaises(host.DiagnosticError):await host.run(args('cells',usb_kind='native',profile=None))
            wire.assert_not_called()

    async def test_legacy_client_cannot_send_cell_command(self):
        wire=FakeWire([]);client=host.Controller(wire,KEY)
        with self.assertRaises(host.DiagnosticError):await client.request('cell_status')
        self.assertEqual(wire.sent,[])

if __name__=='__main__':unittest.main()
