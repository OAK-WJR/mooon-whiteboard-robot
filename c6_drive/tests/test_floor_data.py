"""Offline checks: real-motion criteria must not confuse collection with success."""
import json
import hashlib
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from c6_floor_campaign import plan
from c6_floor_response_report import summarize,consecutive
from c6_floor_reversal import capture


class FloorDataTests(unittest.TestCase):
    def fixture(self,path,stall=False,gap=False):
        record=dict(stages=[dict(key='W',target_cps=[1000,0,0],accepted_ms=1000)],
                    initial=dict(session=123),final=dict(control_run=1,last_fault=None),
                    plan=dict(requested_speed_mps=.04),passed=True)
        path.write_text(json.dumps(record))
        rows=[]
        for t in range(1000,1900,10):
            active=t<1600
            speed=1000 if 1100<=t<1600 and not (stall and t>=1300) else 0
            rows.append([t,1000,speed,0,0,1000 if active else 0,0,0,120 if active else 0,0,0])
        if gap:rows=[r for r in rows if not 1400<=r[0]<1480]
        # Duplicate packets must not inflate the number of control samples.
        log=[dict(frame=dict(type='status',session=123,control_samples=rows)),
             dict(frame=dict(type='status',session=123,control_samples=rows[-20:]))]
        path.with_suffix('.jsonl').write_text('\n'.join(json.dumps(x) for x in log))

    def test_complete_capture_with_stall_fails_motion(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'trial.json';self.fixture(path,stall=True)
            r=summarize(path)
            self.assertTrue(r['capture_complete']);self.assertFalse(r['motion_quality_passed'])
            self.assertEqual(r['wheels'][0]['longest_low_speed_after_300ms'],300)

    def test_stop_tail_not_averaged_into_moving_tail(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'trial.json';self.fixture(path)
            r=summarize(path)
            self.assertTrue(r['motion_quality_passed'])
            self.assertEqual(r['sample_count'],90)
            self.assertEqual(r['stop_ms'],1600)
            self.assertEqual(r['wheels'][0]['tail_mean_target_percent'],100)
            self.assertEqual(r['wheels'][0]['onset_ms'],100)

    def test_new_board_format_and_aborted_capture_stay_distinct(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'trial.json';self.fixture(path)
            old=json.loads(path.read_text())
            current=dict(schema='wbb-loaded-capture-1',command_ms=1000,
                hardware_label='Replacement board',initial_status=old['initial'],final_status=old['final'],
                plan=dict(direction='forward',wheel_targets_cps=[1000,0,0],body_request_mps=.04),
                capture_complete=True)
            path.write_text(json.dumps(current))
            r=summarize(path)
            self.assertTrue(r['motion_quality_passed'])
            self.assertEqual(r['hardware_label'],'Replacement board')
            self.assertEqual(r['stop_ms'],1600)
            current.update(capture_complete=False,error='Invalid odometry')
            path.write_text(json.dumps(current))
            r=summarize(path)
            self.assertFalse(r['motion_quality_passed'])
            self.assertEqual(r['capture_error'],'Invalid odometry')
            self.assertEqual(r['sample_count'],90)

    def test_missing_trace_rejects_quality(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'trial.json';self.fixture(path,gap=True)
            self.assertFalse(summarize(path)['motion_quality_passed'])

    def test_controlled_release_not_scored_as_tracking_failure(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'trial.json';self.fixture(path)
            old=json.loads(path.read_text())
            old['release_command_ms']=1400
            # Keep the full deceleration/stop trace, but score requested travel
            # only up to the accepted zero-velocity command.
            path.write_text(json.dumps(old))
            r=summarize(path)
            self.assertTrue(r['controlled_release']);self.assertEqual(r['motion_end_ms'],1400)
            self.assertEqual(r['stop_ms'],1600);self.assertEqual(r['sample_count'],90)
            self.assertEqual(r['driven_samples'],40)

    def test_pre_arm_abort_retained_without_inventing_motion_samples(self):
        from c6_floor_trial_score import score
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'trial.json'
            p.write_text(json.dumps(dict(schema='wbb-loaded-capture-1',
                plan=dict(direction='right',body_request_mps=.04,motion_seconds=2),
                error='WIFI_CONNECTING',capture_complete=False)))
            p.with_suffix('.jsonl').write_text('')
            r=summarize(p)
            self.assertFalse(r['motion_started']);self.assertEqual(r['samples'],[])
            self.assertEqual(r['capture_error'],'WIFI_CONNECTING')
            self.assertFalse(score(p,{})['passed'])

    def test_contact_annotation_excludes_clean_pass_and_checks_source(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'trial.json';self.fixture(path)
            self.assertTrue(summarize(path)['motion_quality_passed'])
            annotation=dict(clean_performance_evidence=False,classification='user_reported_hand_contact',
                sources={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (path,path.with_suffix('.jsonl'))})
            path.with_suffix('.annotation.json').write_text(json.dumps(annotation))
            r=summarize(path)
            self.assertTrue(r['capture_complete']);self.assertFalse(r['clean_performance_evidence'])
            self.assertFalse(r['motion_quality_passed'])
            path.write_text(path.read_text()+'\n')
            with self.assertRaises(ValueError):summarize(path)

    def test_no_onset_across_packet_gap(self):
        self.assertIsNone(consecutive([[0,110],[10,120],[60,110]],lambda r:r[1]>100))

    def test_plan_keeps_independent_repeat_and_bounded_steps(self):
        p=plan()
        self.assertEqual(p['trial_count'],84)
        self.assertEqual(p['development_count'],56);self.assertEqual(p['validation_count'],28)
        self.assertEqual(len([t for t in p['trials'] if t['duration'] is not None]),60)
        for t in p['trials']:
            self.assertEqual(t['role']=='validation',t['repeat']==3)
            self.assertLessEqual(t['speed'],.12)
            if t['duration'] is not None:self.assertLessEqual(t['speed']*t['duration'],.060001)

    def test_invalid_motion_duration_rejected_before_device_access(self):
        for duration in [0,.1,2,float('nan')]:
            with self.assertRaises(ValueError):capture(None,{},'WW',.08,duration)
        with self.assertRaises(ValueError):capture(None,{},'WA',.08,.5)


if __name__=='__main__':unittest.main()
