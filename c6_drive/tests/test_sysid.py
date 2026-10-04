"""Offline model recovery and experiment-abort checks; no serial hardware."""
import copy
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
import c6_drive_sysid as sid


def synthetic(profile, role, tau=.24, gain=85., bias=-5100., delay=.05, sample_ms=50):
    # Independent fine-step Euler plant; do not generate with the fitting code.
    fine = .001
    elapsed = np.r_[0, np.cumsum([duration for _, duration in profile])]
    speed = gain * profile[0][0] + bias
    count = 0.; frames = []
    for n in range(int(elapsed[-1] / fine)):
        t = n * fine
        index = min(len(profile) - 1, np.searchsorted(elapsed, max(0, t - delay), side='right') - 1)
        speed += (gain * profile[index][0] + bias - speed) * fine / tau
        count += speed * fine
        if n % sample_ms:
            continue
        index = min(len(profile) - 1, np.searchsorted(elapsed, t, side='right') - 1)
        pwm = profile[index][0]
        wheel = {'pwm': pwm, 'speed': speed, 'count': round(count), 'edges': round(count),
                 'fi_duty': pwm, 'bi_duty': 0, 'fi_level': 0, 'bi_level': 0}
        idle = {key: 0 for key in wheel}
        frames.append({'ms': round(t * 1000), 'session': 1, 'lease': 1 if role == 'train' else 2,
                       'armed': True, 'ready': True, 'reason': 'BENCH', 'clients': 0,
                       'motors': [wheel, idle.copy(), idle.copy()]})
    return {'motor': 1, 'sign': 1, 'role': role, 'lease': 1 if role == 'train' else 2,
            'complete': True, 'frames': frames}


class IdentificationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.train = synthetic(sid.TRAIN, 'train')
        cls.validation = synthetic(sid.VALIDATE, 'validation')

    def test_recovers_known_model_and_predicts_independent_input(self):
        t, u, y, _ = sid.series(self.train)
        model = sid.fit_model(t, u, y)
        self.assertLess(abs(model['gain_cps_per_pwm'] / 85 - 1), .025)
        self.assertLess(abs(model['tau_s'] - .24), .04)
        vt, vu, vy, _ = sid.series(self.validation)
        predicted = sid.simulate(vt, vu, vy[0], model)
        self.assertGreater(sid.metrics(vy, predicted)['r_squared'], .98)
        # A changed load must produce a failed held-out prediction, not be refitted.
        loaded = synthetic(sid.VALIDATE, 'validation', gain=50., bias=-4000.)
        lt, lu, ly, _ = sid.series(loaded)
        self.assertLess(sid.metrics(ly, sid.simulate(lt, lu, ly[0], model))['r_squared'], .9)

    def test_rejects_bad_feedback_closed_loop_and_missing_samples(self):
        for mutation in ['feedback', 'closed_loop', 'missing', 'other_wheel', 'pwm_mismatch', 'nan']:
            run = copy.deepcopy(self.train)
            if mutation == 'feedback':
                for f in run['frames']:
                    f['motors'][0]['count'] = 0
            elif mutation == 'closed_loop':
                run['frames'][40]['reason'] = 'DRIVE'
            elif mutation == 'missing':
                del run['frames'][40:50]
            elif mutation == 'other_wheel':
                run['frames'][40]['motors'][1]['count'] = 100
            elif mutation == 'pwm_mismatch':
                run['frames'][40]['motors'][0]['fi_duty'] = 255
            else:
                run['frames'][40]['motors'][0]['speed'] = float('nan')
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                sid.series(run)

    def test_fault_always_attempts_stop_and_never_changes_gains(self):
        c = Mock()
        c.latest = copy.deepcopy(self.train['frames'][0])
        c.latest['motors'][0].update(speed=0, pwm=0)
        c.frames = []
        c.request.side_effect = [{'lease': 42}, RuntimeError('USB unplugged')]
        with patch.object(sid, 'stop_and_wait') as stop:
            with self.assertRaisesRegex(RuntimeError, 'USB unplugged'):
                sid.collect_run(c, 0, 1, 'train', [(140, .1)], {})
            self.assertEqual(stop.call_count, 2)
        self.assertEqual([call.args[0] for call in c.request.call_args_list], ['arm', 'pwm'])

    def test_preflight_and_counter_wrap(self):
        self.assertEqual(sid.delta32(-2147483640, 2147483640), 16)
        c = Mock()
        with self.assertRaises(ValueError):
            sid.collect_run(c, 0, 1, 'train', [(255, .1)], {})
        c.request.assert_not_called()
        frame = copy.deepcopy(self.train['frames'][0]); frame['ready'] = False
        with self.assertRaises(RuntimeError):
            sid.check_frame(frame, 0, 1, [0, 0, 0])

    def test_high_rate_ring_deduplication_and_independent_plant(self):
        dense=synthetic(sid.TRAIN,'train',sample_ms=10)
        run=copy.deepcopy(dense);run['frames']=[]
        for n in range(0,len(dense['frames']),5):
            f=copy.deepcopy(dense['frames'][n]);f['control_samples']=[]
            for source in dense['frames'][max(0,n-7):n+1]:
                f['control_samples'].append([source['ms'],0,*[m['speed'] for m in source['motors']],
                                              0,0,0,*[m['pwm'] for m in source['motors']]])
            run['frames'].append(f)
        t,u,y,q=sid.series_high_rate(run)
        self.assertAlmostEqual(q['median_sample_s'],.01)
        model=sid.fit_model(t,u,y)
        self.assertLess(abs(model['tau_s']-.24),.04)
        self.assertLess(abs(model['gain_cps_per_pwm']/85-1),.025)
        corrupt=copy.deepcopy(run);corrupt['frames'][20]['control_samples'][0][2]+=100
        with self.assertRaises(ValueError):sid.series_high_rate(corrupt)

    def test_collection_error_keeps_log_open_until_stop(self):
        import c6_drive_tune
        client = Mock(); client.latest = {}
        def make_client(port, log):
            client.logfile = log
            return client
        def final_stop(c):
            self.assertFalse(c.logfile.closed)
            c.logfile.write('stop confirmation\n')
        with tempfile.TemporaryDirectory() as tmp:
            args = SimpleNamespace(suspended=True, report=str(Path(tmp) / 'run.json'), port='fake', motors=[1])
            with patch.object(c6_drive_tune, 'Client', side_effect=make_client), \
                 patch.object(sid, 'collect_run', side_effect=RuntimeError('feedback fault')), \
                 patch.object(sid, 'stop_and_wait', side_effect=final_stop):
                with self.assertRaisesRegex(RuntimeError, 'feedback fault'):
                    sid.collect(args)
            self.assertTrue(client.logfile.closed)
            client.s.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
