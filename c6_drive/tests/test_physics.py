"""Independent physical invariants and controller/plant boundary checks."""
import json
import math
from pathlib import Path
import sys
import unittest
import numpy as np
import mujoco
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'sim/physics'))
from engine import Plant, Wheels, Camera, run, MPC, RAD_PER_COUNT, motor_constants, ROOT


class PhysicsTests(unittest.TestCase):
    def test_free_body_and_roller_structure(self):
        p = Plant()
        self.assertEqual(p.m.nv, 45)  # Six base, three hubs, 36 free rollers.
        self.assertEqual(p.m.nu, 3)
        self.assertAlmostEqual(sum(p.m.body_mass), .3, places=8)
        self.assertEqual(p.m.neq, 0)  # Chassis is not constrained to a plane.

    def test_static_normal_force_and_frictless_gravity(self):
        p = Plant()
        for _ in range(100):
            p.step([0, 0, 0])
        self.assertAlmostEqual(p.diagnostics()['normal_N'], .3*9.81, delta=.01)
        self.assertLess(np.linalg.norm(p.pose()[:2]), 1e-4)
        p = Plant(mu=0, incline=90)
        for _ in range(20):
            p.step([0, 0, 0], adhesion=12)
        self.assertAlmostEqual(p.pose()[1], -.5*9.81*.2**2, delta=.003)

    def test_freewheel_speed_uses_measured_gain(self):
        for i in range(3):
            for direction in (1, -1):
                p = Plant(fixed=True)
                p.m.geom_contype[:] = 0
                p.m.geom_conaffinity[:] = 0
                p.m.opt.gravity[:] = 0
                pwm = [0, 0, 0]
                pwm[i] = direction * 160
                for _ in range(250):
                    p.step(pwm)
                k, _, ks, _, _ = motor_constants(i, direction)
                expected = direction * k * (160-ks)
                measured = p.d.qvel[p.axles[i]] / RAD_PER_COUNT
                self.assertAlmostEqual(measured, expected, delta=abs(expected)*.012)

    def test_camera_delay_and_validity(self):
        cam = Camera(np.random.default_rng(4))
        self.assertIsNone(cam.read(np.zeros(3), 0))
        self.assertIsNone(cam.read(np.ones(3), .07, False))
        reading = cam.read(np.ones(3), .08, False)
        self.assertLess(max(abs(reading[1])), .002)
        self.assertIsNone(cam.read(np.ones(3), .31, False))

    def test_controller_boundary_and_reset(self):
        w = Wheels('eso')
        try:
            args = dict(targets=[5000, -2500, -2500], measured=[0,0,0],hold=True,learn=True)
            out = w.call(args)
            self.assertAlmostEqual(sum(out['targets']), 0)
            self.assertTrue(all(abs(x) <= 255 for x in out['pwm']))
            self.assertEqual(w.call({'stop':True})['pwm'], [0,0,0])
        finally:
            w.close()

    def test_time_step_sensitivity(self):
        a = run('floor', seconds=4, timestep=.001)
        b = run('floor', seconds=4, timestep=.0005)
        delta = np.linalg.norm(np.array(a['metrics']['final_pose'][:2])-b['metrics']['final_pose'][:2])
        self.assertLess(delta, .015, 'Reducing timestep must not materially alter the floor trajectory')

    def test_scenario_report(self):
        path = ROOT/'reports/mujoco-traces.json'
        data = json.loads(path.read_text())
        self.assertEqual(len(data), 24)
        for r in data:
            self.assertEqual(r['metrics']['solver_warnings'], 0, r['scenario'])
            for f in r['trace']:
                self.assertTrue(np.isfinite(f['qpos']).all())
                self.assertTrue(all(abs(x) <= 255 for x in f['pwm']))
                self.assertTrue(all(abs(x) <= 90.001 for x in f['comp']))
                if f['fault']:
                    self.assertEqual(f['pwm'], [0,0,0])
            if r['scenario'] in ('slip', 'magnet'):
                self.assertTrue(r['metrics']['ended_early'])
                self.assertIn('SLIP_DERATE', r['metrics']['statuses'])
            if r['scenario'] == 'encoder':
                self.assertEqual(r['metrics']['fault'], 'ENCODER_INVALID')
            if r['scenario'] == 'camera':
                self.assertEqual(r['metrics']['fault'], 'POSITION_FEEDBACK_LOST')
            if r['scenario'] == 'saturation':
                self.assertIn('SATURATION_DERATE', r['metrics']['statuses'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
