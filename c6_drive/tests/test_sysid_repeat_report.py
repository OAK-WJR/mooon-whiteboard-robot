"""Checks that repeated observations and refitting cannot masquerade as holdouts."""
import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
from test_sysid import synthetic
import c6_drive_sysid as sid
import c6_sysid_repeat_report as report


def capture(session):
    seeds = {role: synthetic(profile, role, sample_ms=10)
             for role, profile in [('train', sid.TRAIN), ('validation', sid.VALIDATE)]}
    runs = []
    for motor in (1, 2, 3):
        for sign in (1, -1):
            for role in ('train', 'validation'):
                lease = len(runs) + 1
                dense = copy.deepcopy(seeds[role]['frames'])
                for f in dense:
                    wheel = f['motors'][0]
                    for key in ('pwm', 'speed', 'count'):
                        wheel[key] *= sign
                    wheel['fi_duty'] = abs(wheel['pwm']) if sign > 0 else 0
                    wheel['bi_duty'] = abs(wheel['pwm']) if sign < 0 else 0
                    f['motors'] = [{key: 0 for key in wheel} for _ in range(3)]
                    f['motors'][motor - 1] = wheel
                    f.update(ms=f['ms'] + lease*20000, session=session, lease=lease)
                frames = []
                for n in range(0, len(dense), 5):
                    f = copy.deepcopy(dense[n])
                    f['control_samples'] = [[s['ms'], 0, *[m['speed'] for m in s['motors']],
                                             0, 0, 0, *[m['pwm'] for m in s['motors']]]
                                            for s in dense[max(0, n-7):n+1]]
                    frames.append(f)
                runs.append(dict(motor=motor, sign=sign, role=role, lease=lease,
                                 session=session, complete=True, frames=frames))
    return dict(status='COLLECTED_NOT_TUNED', stopped=True, condition='suspended',
                hardware_id='test-board', runs=runs)


class RepeatAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.first, cls.second = capture(1), capture(2)
        cls.model = {'condition': 'suspended', 'models': [
            dict(motor=m, sign=s, tau_s=.24, delay_s=.05,
                 gain_cps_per_pwm=85., bias_cps=-5100.)
            for m in (1, 2, 3) for s in (1, -1)]}

    def test_frozen_prediction_and_changed_model_are_reported_honestly(self):
        original = copy.deepcopy(self.model)
        result, _ = report.audit(self.model, [self.first, self.second])
        self.assertEqual(self.model, original)
        self.assertEqual(result['heldout_runs'], 18)
        self.assertEqual(result['passed_runs'], 18)
        self.assertFalse(result['model_refitted_to_holdouts'])
        wrong = copy.deepcopy(self.model)
        for model in wrong['models']:
            model['gain_cps_per_pwm'] *= .7
        result, _ = report.audit(wrong, [self.first, self.second])
        self.assertEqual(result['passed_runs'], 0)

    def test_same_experiment_cannot_be_counted_twice(self):
        with self.assertRaisesRegex(ValueError, 'not independent'):
            report.audit(self.model, [self.first, self.first])

    def test_different_board_and_incomplete_capture_rejected(self):
        other = dict(self.second, hardware_id='other-board')
        with self.assertRaisesRegex(ValueError, 'different boards'):
            report.audit(self.model, [self.first, other])
        with self.assertRaises(ValueError):
            report.audit(self.model, [dict(self.first, status='ABORTED')])


if __name__ == '__main__':
    unittest.main()
