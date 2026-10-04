"""Offline checks only. No serial import or device access."""
import copy
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('loaded_capture',ROOT/'scripts/c6_loaded_capture.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
sys.path.insert(0,str(ROOT/'scripts'))


class FakeClient:
    """Clock-driven firmware transport; never imports or opens serial."""
    def __init__(self, failure=None):
        self.now=10.;self.failure=failure;self.commands=[];self.frames=[]
        self.mode='encoder_hold';self.armed=False;self.moved=False
        self.command_ms=0;self.latest=None;self.last_status=self.now
        self.assist=False;self.direct=False;self.fast=False;self.nonzero=False
        self.pump(.01)

    def request(self,op,**args):
        self.commands.append((op,args))
        if op=='stop':self.armed=False
        elif op=='controller':self.mode=args['mode']
        elif op=='arm':self.armed=True
        elif op=='heading_assist':
            self.assist=args['enabled'];self.direct=args.get('direct',self.direct)
            self.fast=args.get('fast_release',self.fast)
        elif op=='wheels':
            self.moved=True;self.command_ms=round(self.now*1000)
            self.nonzero=any(args['values'])
        return dict(lease=7,command_ms=self.command_ms)

    def pump(self,seconds):
        for _ in range(max(1,round(seconds/.01))):
            self.now+=.01
            if self.failure=='stale' and self.moved and self.armed:continue
            armed=self.armed and not (self.failure=='firmware_fault' and self.moved)
            moving=armed and self.moved and self.nonzero
            speed=100 if moving or (self.failure=='stop_failure' and self.moved) else 0
            motors=[dict(pwm=160 if moving else 0,speed=speed,
                         fi_duty=0,bi_duty=0,fi_level=0,bi_level=0) for _ in range(3)]
            ms=round(self.now*1000)
            self.latest=dict(ms=ms,ready=True,armed=armed,lease=7,clients=0,
                reason='FEEDBACK_FAULT' if not armed and self.armed else 'READY',
                controller=self.mode,heading_assist=dict(enabled=self.assist,braking=False,direct_feedback=self.direct,fast_release=self.fast),control_age_ms=1,motors=motors,
                odometry=dict(valid=True,age_ms=1,x_m=0,y_m=0,yaw_rad=0),
                control_samples=[[ms,self.command_ms,-speed,0,speed,-800,0,800,-160,0,160]])
            if self.failure=='invalid_origin':self.latest['odometry']['x_m']=float('nan')
            self.frames.append(self.latest);self.last_status=self.now


class FloorCaptureTests(unittest.TestCase):
    def test_duty_reduction_restored_after_failure(self):
        c=FakeClient('firmware_fault');r=dict(frames=[],samples=[],capture_complete=False)
        p=module.plan();p.update(heading_assist=True,fast_release=True,controlled_release=True)
        with patch('time.monotonic',side_effect=lambda:c.now),self.assertRaises(RuntimeError):module.capture(c,p,r)
        self.assertFalse(c.armed);self.assertFalse(c.fast);self.assertFalse(c.assist)
        self.assertTrue(r['experiment_status']['heading_assist']['fast_release'])
    def test_direct_feedback_restored_after_trial_and_failure(self):
        for failure in (None,'firmware_fault'):
            c=FakeClient(failure);r=dict(frames=[],samples=[],capture_complete=False)
            p=module.plan();p.update(heading_assist=True,direct_heading=True,controlled_release=True)
            with patch('time.monotonic',side_effect=lambda:c.now):
                if failure:
                    with self.assertRaises(RuntimeError):module.capture(c,p,r)
                else:module.capture(c,p,r)
            self.assertFalse(c.armed);self.assertFalse(c.direct);self.assertFalse(c.assist)
            self.assertTrue(r['experiment_status']['heading_assist']['direct_feedback'])
    def test_plan_has_no_serial_import_and_small_target(self):
        self.assertNotIn('serial',sys.modules)
        for direction in module.DIRECTIONS:
            p=module.plan(direction)
            self.assertLessEqual(max(abs(v) for v in p['wheel_targets_cps']),1000)
            self.assertEqual(p['motion_seconds'],.6)
            self.assertFalse(p['firmware_or_gain_changes'])
        for seconds in (1.2,2.):
            p=module.plan(seconds=seconds)
            self.assertLess(p['motion_seconds']*p['body_request_mps'],p['encoder_excursion_limit_m'])
        with self.assertRaises(ValueError):module.plan(seconds=3)
        self.assertEqual(module.plan(seconds=1,speed=.08)['encoder_excursion_limit_m'],.10)
        for seconds,speed in [(2,.08),(1.2,.08),(1,.1),(1,float('nan'))]:
            with self.assertRaises(ValueError):module.plan(seconds=seconds,speed=speed)

    def test_guard_does_not_mistake_passive_wheel_motion_for_fault(self):
        f=dict(ready=True,armed=True,lease=7,clients=0,control_age_ms=1,controller='robust_wheels',
               odometry=dict(valid=True,age_ms=1,x_m=.01,y_m=0,yaw_rad=0),
               motors=[dict(pwm=0,speed=80),dict(pwm=160,speed=300),dict(pwm=-160,speed=-300)])
        module.check_frame(f,7,[0,0,0])
        for change in ('stopped','travel','heading','other_client','nan','mode','old_packet','bad_age'):
            bad=copy.deepcopy(f)
            if change=='stopped':bad['armed']=False
            if change=='travel':bad['odometry']['x_m']=.07
            if change=='heading':bad['odometry']['yaw_rad']=.3
            if change=='other_client':bad['clients']=1
            if change=='nan':bad['odometry']['x_m']=float('nan')
            if change=='mode':bad['controller']='encoder_hold'
            if change=='old_packet':bad['telemetry_build_ms']=25
            if change=='bad_age':bad['telemetry_build_ms']=float('nan')
            with self.assertRaises(RuntimeError):module.check_frame(bad,7,[0,0,0])

    def test_onset_uses_sustained_motion_and_deduplicates_packets(self):
        rows=[]
        for n in range(30):
            speed=100 if n==1 or n>=10 else 0
            rows.append([100+n*10,100,-speed,0,speed,-800,0,800,-160,0,160])
        result=module.analyze(rows+rows[10:],100,[-800,0,800])
        self.assertEqual(len(result['samples']),30)
        self.assertEqual(result['motors'][0]['motion_onset_s'],.1)
        self.assertIsNone(result['motors'][1]['motion_onset_s'])
        self.assertEqual(result['motors'][2]['peak_applied_pwm'],160)
        extended=rows+[[400+n*10,100,-100,0,100,-800,0,800,-160,0,160] for n in range(90)]
        self.assertGreater(len(module.analyze(extended,100,[-800,0,800],1.2)['samples']),100)

    def test_capture_stops_before_analysis_and_restores_existing_mode(self):
        client=FakeClient();record=dict(frames=[],samples=[],capture_complete=False)
        analyze=module.analyze
        def checked_analysis(*args):
            self.assertFalse(client.armed)
            self.assertEqual(client.mode,'encoder_hold')
            return analyze(*args)
        with patch('time.monotonic',side_effect=lambda:client.now),patch.object(module,'analyze',side_effect=checked_analysis):
            module.capture(client,module.plan(),record)
        self.assertTrue(record['capture_complete'])
        self.assertTrue(record['stop_frames'])
        self.assertTrue(all(not f['armed'] for f in record['stop_frames']))
        self.assertEqual(sum(op=='arm' for op,_ in client.commands),1)
        self.assertEqual(sum(op=='stop' for op,_ in client.commands),2)
        self.assertNotIn('serial',sys.modules)

    def test_fault_or_stale_telemetry_stops_without_rearming(self):
        for failure in ('firmware_fault','stale'):
            client=FakeClient(failure);record=dict(frames=[],samples=[],capture_complete=False)
            with patch('time.monotonic',side_effect=lambda:client.now),self.assertRaises(RuntimeError):
                module.capture(client,module.plan(),record)
            self.assertFalse(client.armed)
            self.assertFalse(record['capture_complete'])
            self.assertNotIn('analysis',record)
            self.assertEqual(sum(op=='arm' for op,_ in client.commands),1)
            self.assertEqual(sum(op=='stop' for op,_ in client.commands),2)

    def test_failed_stop_does_not_restore_mode_or_mark_complete(self):
        client=FakeClient('stop_failure');record=dict(frames=[],samples=[],capture_complete=False)
        with patch('time.monotonic',side_effect=lambda:client.now),self.assertRaisesRegex(RuntimeError,'did not stop'):
            module.capture(client,module.plan(),record)
        self.assertFalse(record['capture_complete'])
        self.assertNotIn('analysis',record)
        self.assertTrue(record['stop_frames'])
        self.assertGreater(record['final_status']['motors'][0]['speed'],30)
        self.assertEqual(client.mode,'robust_wheels')
        self.assertEqual(sum(op=='controller' for op,_ in client.commands),1)

    def test_invalid_origin_cannot_arm_or_move(self):
        client=FakeClient('invalid_origin');record=dict(frames=[],samples=[],capture_complete=False)
        with patch('time.monotonic',side_effect=lambda:client.now),self.assertRaisesRegex(RuntimeError,'pre-arm'):
            module.capture(client,module.plan(),record)
        self.assertFalse(any(op in ('arm','wheels') for op,_ in client.commands))
        self.assertFalse(record['capture_complete'])

    def test_assist_trial_is_explicit_and_restores_previous_setting(self):
        for failure in (None,'firmware_fault'):
            client=FakeClient(failure);record=dict(frames=[],samples=[],capture_complete=False)
            experiment=module.plan();experiment['heading_assist']=True
            with patch('time.monotonic',side_effect=lambda:client.now):
                if failure:
                    with self.assertRaises(RuntimeError):module.capture(client,experiment,record)
                else:module.capture(client,experiment,record)
            self.assertFalse(client.armed);self.assertFalse(client.assist)
            self.assertTrue(record['experiment_status']['heading_assist']['enabled'])
            self.assertEqual([a['enabled'] for op,a in client.commands if op=='heading_assist'],[True,False])

    def test_normal_release_is_recorded_before_final_hard_stop(self):
        client=FakeClient();record=dict(frames=[],samples=[],capture_complete=False)
        experiment=module.plan();experiment.update(heading_assist=True,controlled_release=True)
        with patch('time.monotonic',side_effect=lambda:client.now):module.capture(client,experiment,record)
        self.assertTrue(record['capture_complete']);self.assertTrue(record['controlled_release_complete'])
        self.assertGreater(record['release_command_ms'],record['command_ms'])
        zeros=[n for n,(op,a) in enumerate(client.commands) if op=='wheels' and not any(a['values'])]
        stops=[n for n,(op,a) in enumerate(client.commands) if op=='stop']
        self.assertTrue(zeros);self.assertLess(zeros[-1],stops[-1]);self.assertFalse(client.armed)


if __name__=='__main__':unittest.main()
