"""Prospective acceptance, plant parity, and saved-executable replay checks."""
from dataclasses import asdict
import json
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'sim/physics'))
from engine import Plant, ROOT
from robust_campaign import CampaignPlant, Case, nominal_cases, qualification_cases, uncertain_cases, run_case
from evaluate_robust import audit


class RobustCampaignTests(unittest.TestCase):
    def test_prospective_acceptance_and_failures_retained(self):
        self.assertGreater(audit()['acceptance_checks_passed'], 900)

    def test_distribution_and_seed_integrity(self):
        for suite, cases in [('holdout', qualification_cases(8100, 60)), ('stress', uncertain_cases(9100, 30))]:
            saved = json.loads((ROOT/f'reports/robust-{suite}.json').read_text())
            cases = {c.name: json.loads(json.dumps(asdict(c))) for c in cases}
            for r in saved['results']:
                self.assertEqual(cases[r['case']['name']], r['case'])
        self.assertFalse(set(range(1100,1112)) & set(range(8100,8160)))
        self.assertFalse(set(range(9100,9130)) & set(range(8100,8160)))

    def test_extended_plant_matches_previous_physics(self):
        a = Plant(incline=45)
        b = CampaignPlant(incline=45)
        c = Case(incline=45, adhesion=12, supply=.9, torque=(.8,1.,1.), push=(.1,-.1))
        for n in range(100):
            pwm = [130 if n<50 else -130, -90, 160]
            a.step(pwm, adhesion=12, supply=.9, weak=.8, push=c.push)
            b.step_case(pwm, c, 3+n*.01)
        np.testing.assert_allclose(a.d.qpos, b.d.qpos, atol=1e-12, rtol=1e-12)
        np.testing.assert_allclose(a.d.qvel, b.d.qvel, atol=1e-12, rtol=1e-12)

    def test_recorded_traces_match_current_native_controller(self):
        records = json.loads((ROOT/'reports/robust-nominal-traces.json').read_text())
        for name in ['outlier', 'stuck']:
            case = next(c for c in nominal_cases() if c.name == name)
            fresh = run_case(case, 'robust', save_trace=True)
            saved = next(r for r in records if r['case']['name'] == name and r['variant'] == 'robust')
            self.assertEqual(fresh['metrics']['faults'], saved['metrics']['faults'])
            self.assertAlmostEqual(fresh['metrics']['mean_task_error_m'], saved['metrics']['mean_task_error_m'], places=10)
            np.testing.assert_allclose([f['qpos'] for f in fresh['trace']], [f['qpos'] for f in saved['trace']], atol=1e-10)


if __name__ == '__main__':
    unittest.main(verbosity=2)
