"""Physical invariants for the additional resistance fixture (no hardware)."""
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'sim/physics'))
import mujoco
import numpy as np
from engine import Plant, RW
from load_plant import LoadPlant, Resistance


class LoadPhysicsTests(unittest.TestCase):
    def test_no_added_load_preserves_archived_plant(self):
        old, new = Plant(mass=.6), LoadPlant(mass=.6)
        for _ in range(50):
            old.step([-110, 0, 110]); new.step([-110, 0, 110])
        np.testing.assert_allclose(new.d.qpos, old.d.qpos, atol=1e-12, rtol=0)

    def test_total_mass_and_contact_force_balance(self):
        for mass in (.5, 1., 2.):
            plant = LoadPlant(mass=mass, resistance=Resistance(pad_preload_N=2))
            self.assertAlmostEqual(sum(plant.m.body_mass), mass)
            for _ in range(200): plant.step([0, 0, 0])
            diag = plant.diagnostics()
            self.assertAlmostEqual(diag['normal_N'], mass * 9.81, delta=.05)
            self.assertGreater(diag['pad_normal_N'], 1)
            self.assertLess(sum(diag['wheel_normal_N']), mass * 9.81 - 1)
            self.assertEqual(sum(w.number for w in plant.d.warning), 0)

    def test_dry_friction_opposes_joint_motion(self):
        plant = LoadPlant(fixed=True, resistance=Resistance(axle_Nm=(.01,.02,.03), roller_Nm=.00001))
        plant.m.geom_contype[:] = 0; plant.m.geom_conaffinity[:] = 0
        plant.m.opt.gravity[:] = 0
        plant.d.qvel[:] = np.resize([.5, -.5], plant.m.nv)
        mujoco.mj_forward(plant.m, plant.d)
        # With contacts/limits disabled, these constraint forces are dry friction.
        self.assertLess(float(plant.d.qfrc_constraint @ plant.d.qvel), 0)
        # Tiny coupled rollers can reverse within the implicit step. Check the
        # system's dissipated power and constraint bounds, not a sign rule on
        # each roller's pre-step velocity.
        self.assertTrue(np.all(abs(plant.d.qfrc_constraint) <= plant.m.dof_frictionloss + 1e-12))
        self.assertTrue(np.all(plant.d.qfrc_constraint[plant.axles] * plant.d.qvel[plant.axles] < 0))

    def test_pad_does_not_apply_drag_in_air(self):
        plant = LoadPlant(mass=1, resistance=Resistance(pad_preload_N=3))
        plant.d.qpos[2] = RW + .1
        mujoco.mj_forward(plant.m, plant.d)
        diag = plant.diagnostics()
        self.assertEqual(diag['pad_normal_N'], 0)
        self.assertEqual(diag['pad_tangent_N'], 0)

    def test_reject_invalid_resistance(self):
        for bad in (-1, float('nan'), float('inf')):
            with self.assertRaises(ValueError): Resistance(roller_Nm=bad)


if __name__ == '__main__': unittest.main()
