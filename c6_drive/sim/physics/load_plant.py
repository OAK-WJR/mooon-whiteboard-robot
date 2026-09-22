"""Additive heavy-load fixture; leaves the archived physics model unchanged.

All payload, friction, pad geometry and torque values are assumed. Joint dry
friction is solved by MuJoCo, not approximated by a force that vanishes at rest.
The optional rubbing pad is a spring-loaded contact body, so its support force
also unloads the wheels. It is not a constant drag force applied in mid-air.
"""
from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from engine import Plant, make_xml, motor_constants, DT, RW


@dataclass(frozen=True)
class Resistance:
    axle_Nm: tuple = (0., 0., 0.)  # additional to identified motor losses
    roller_Nm: float = 0.
    pad_preload_N: float = 0.  # nominal spring force at q=0, not fixed normal force
    pad_mu: float = .6
    motor_force_N: float = 4.  # assumed zero-speed force at full duty, not a rating

    def __post_init__(self):
        values = (*self.axle_Nm, self.roller_Nm, self.pad_preload_N, self.pad_mu)
        if len(self.axle_Nm) != 3 or any(not math.isfinite(v) or v < 0 for v in values):
            raise ValueError('Resistance must be finite and nonnegative')
        if not math.isfinite(self.motor_force_N) or self.motor_force_N <= 0:
            raise ValueError('Assumed motor force must be positive')


def load_xml(*, resistance, mass=.30, mu=.65, incline=0., timestep=.001, fixed=False):
    if not math.isfinite(mass) or mass <= .0512:
        raise ValueError('Mass must exceed hub/roller/pad mass')
    root = ET.fromstring(make_xml(mass, mu, incline, timestep, fixed))
    for joint in root.iter('joint'):
        name = joint.get('name', '')
        if name.startswith('wheel'):
            joint.set('frictionloss', str(resistance.axle_Nm[int(name[5:])]))
            # Scale the assumed rotor inertia with torque and damping, preserving
            # measured unloaded K/tau when testing unknown torque authority.
            joint.set('armature', str(float(joint.get('armature')) * resistance.motor_force_N / 4))
        elif name.startswith('roller'):
            joint.set('frictionloss', str(resistance.roller_Nm))
    if resistance.pad_preload_N:
        chassis = root.find(".//body[@name='chassis']")
        inertial = chassis.find('inertial')
        inertial.set('mass', str(float(inertial.get('mass')) - .020))
        pad = ET.SubElement(chassis, 'body', name='rubbing_pad', pos='0 0 -.016')
        ET.SubElement(pad, 'joint', name='pad_spring', type='slide', axis='0 0 1',
                      limited='true', range='-.005 .005', damping='2',
                      stiffness=str(resistance.pad_preload_N / .003), springref='-.003')
        ET.SubElement(pad, 'geom', name='pad_contact', type='box', size='.010 .010 .003',
                      mass='.020', contype='2', conaffinity='1', condim='3',
                      priority='1', friction=f'{resistance.pad_mu} 0 0', rgba='.75 .45 .22 1')
    return ET.tostring(root, encoding='unicode')


class LoadPlant(Plant):
    def __init__(self, *, resistance=Resistance(), **kwargs):
        self.resistance = resistance
        self.xml = load_xml(resistance=resistance, **kwargs)
        self.m = mujoco.MjModel.from_xml_string(self.xml)
        self.d = mujoco.MjData(self.m)
        self.body = self.m.body('chassis').id
        self.axles = [int(self.m.joint(f'wheel{i}').dofadr[0]) for i in range(3)]
        self.angles = [int(self.m.joint(f'wheel{i}').qposadr[0]) for i in range(3)]
        self.tires = [self.m.geom(f'tire{i}_{j}').id for i in range(3) for j in range(12)]
        self.pad = self.m.geom('pad_contact').id if resistance.pad_preload_N else -1
        self.mu = kwargs.get('mu', .65)
        mujoco.mj_forward(self.m, self.d)

    def step(self, pwm, *, adhesion=0., supply=1., weak=1., push=(0., 0.), coast=False):
        for _ in range(round(DT / self.m.opt.timestep)):
            self.d.xfrc_applied[:] = 0
            gap = max(0., self.d.xpos[self.body, 2] - RW - .001)
            magnetic_force = adhesion * math.exp(-gap / .005)
            self.d.xfrc_applied[self.body, :3] = [*push, -magnetic_force]
            for i, u in enumerate(pwm):
                omega = self.d.qvel[self.axles[i]]
                _, _, ks, ct, damping = motor_constants(i, u or omega)
                scale = self.resistance.motor_force_N / 4
                effective = np.sign(u) * max(0., abs(u) * supply - ks)
                drag = .15 if coast and abs(u) < .5 else 1.
                torque = scale * (ct * effective * (weak if i == 0 else 1.) - drag * damping * omega)
                self.d.ctrl[i] = np.clip(torque, -.2, .2)
            mujoco.mj_step(self.m, self.d)
        mujoco.mj_forward(self.m, self.d)

    def diagnostics(self):
        result = super().diagnostics()
        wheel_normal = np.zeros(3)
        pad_normal = pad_tangent = 0.
        force = np.zeros(6)
        tire_wheel = {gid: i // 12 for i, gid in enumerate(self.tires)}
        for contact_index in range(self.d.ncon):
            contact = self.d.contact[contact_index]
            mujoco.mj_contactForce(self.m, self.d, contact_index, force)
            for geom in (contact.geom1, contact.geom2):
                if geom == self.pad:
                    pad_normal += max(0., force[0])
                    pad_tangent += np.linalg.norm(force[1:3])
                elif geom in tire_wheel:
                    wheel_normal[tire_wheel[geom]] += max(0., force[0])
        result.update(wheel_normal_N=wheel_normal.tolist(), pad_normal_N=float(pad_normal),
                      pad_tangent_N=float(pad_tangent), motor_torque_Nm=self.d.ctrl.tolist())
        return result
