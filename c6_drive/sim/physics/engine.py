"""Six-DOF MuJoCo robot with 3 driven hubs and 36 passive roller joints.

An original, approximate model: not a dimensionally verified robot CAD import.
Only the moving-range speed gains and effective time constants are measured.
All mass, torque, contact, roller geometry and magnetic field values are assumed.
No controller receives friction, contact force, injected force, or truth velocity.
"""
from __future__ import annotations
import json
import math
from pathlib import Path
import subprocess
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parents[2]
RW, R, CPR, DT = .019, .033738, 2800, .01
RAD_PER_COUNT = 2 * math.pi / CPR
MPC = 2 * math.pi * RW / CPR
PHI = np.array([math.pi / 3, math.pi, -math.pi / 3])
MODEL_DATA = json.loads((ROOT / 'reports/sysid-model-03.json').read_text())
MODELS = [[next(m for m in MODEL_DATA['models'] if m['motor'] == i and m['sign'] == s)
           for s in (1, -1)] for i in (1, 2, 3)]


def kin(yaw):
    angles = PHI + yaw
    return np.column_stack((-np.sin(angles), np.cos(angles), np.full(3, R)))


def motor_constants(i, direction):
    m = MODELS[i][int(direction < 0)]
    k, tau = m['gain_cps_per_pwm'], m['tau_s']
    ks = max(0., -m['bias_cps'] / k)
    # Chosen 4 N tangential force at maximum duty, NOT a torque measurement.
    ct = 4 * RW / (255 - ks)
    damping = ct / (k * RAD_PER_COUNT)
    return k, tau, ks, ct, damping


def make_xml(mass=.30, mu=.65, incline=0., timestep=.001, fixed=False):
    wheels = []
    colors = ('0.95 0.61 0.36', '0.39 0.65 0.97', '0.36 0.84 0.70')
    for i, phi in enumerate(PHI):
        # Armature represents an assumed effective reflected motor inertia.
        inertia = np.mean([motor_constants(i, s)[1] * motor_constants(i, s)[4] for s in (1, -1)])
        rollers = []
        for j in range(12):
            a = j * 2 * math.pi / 12
            tangent = f'0 {math.cos(a):.9f} {-math.sin(a):.9f}'
            rollers.append(f'''<body name="r{i}_{j}" pos="0 {.0152*math.sin(a):.9f} {.0152*math.cos(a):.9f}">
              <joint name="roller{i}_{j}" axis="{tangent}" damping="0.0000002"/>
              <geom name="tire{i}_{j}" type="ellipsoid" size=".0038 .0038 .0056" zaxis="{tangent}"
                    mass=".0002" rgba=".13 .17 .20 1" contype="2" conaffinity="1"/>
            </body>''')
        wheels.append(f'''<body name="hub{i}" pos="{R*math.cos(phi):.9f} {R*math.sin(phi):.9f} 0" euler="0 0 {phi:.9f}">
          <joint name="wheel{i}" axis="-1 0 0" armature="{inertia:.10f}"/>
          <geom type="cylinder" size=".012 .004" zaxis="1 0 0" mass=".008" rgba="{colors[i]} 1" contype="0" conaffinity="0"/>
          {''.join(rollers)}
        </body>''')
    g = math.radians(incline)
    return f'''<mujoco model="WhiteBoardBot roller contact study">
      <compiler angle="radian"/>
      <option timestep="{timestep}" gravity="0 {-9.81*math.sin(g):.9f} {-9.81*math.cos(g):.9f}"
              integrator="implicitfast" cone="elliptic" solver="Newton" iterations="80" tolerance="1e-10"/>
      <visual><global offwidth="1440" offheight="1080"/><headlight ambient=".4 .4 .4" diffuse=".7 .7 .7"/></visual>
      <default><joint type="hinge" limited="false"/><geom friction="{mu} .00001 .000001" condim="3"
                 solref=".004 1" solimp=".95 .99 .001"/></default>
      <asset><texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1=".16 .20 .24" rgb2=".20 .25 .29"/>
        <material name="surface" texture="grid" texrepeat="20 20" reflectance=".05"/>
        <mesh name="deck" vertex=".049 0 .002 -.0245 .0424 .002 -.0245 -.0424 .002 .049 0 .008 -.0245 .0424 .008 -.0245 -.0424 .008"/>
      </asset>
      <worldbody>
        <light pos="0 -.4 1" dir="0 .3 -1"/>
        <geom name="board" type="plane" size="1 1 .01" material="surface" contype="1" conaffinity="2"/>
        <body name="chassis" pos="0 0 {RW+.0001}" quat=".70710678 0 0 .70710678">
          {'' if fixed else '<freejoint name="base"/>'}
          <inertial pos="0 0 .006" mass="{mass-.0312}" diaginertia="{mass*.0006} {mass*.0006} {mass*.0010}"/>
          <geom type="mesh" mesh="deck" rgba=".18 .43 .37 1" contype="0" conaffinity="0"/>
          <geom type="box" pos="-.005 0 .013" size=".013 .009 .005" rgba=".22 .28 .34 1" contype="0" conaffinity="0"/>
          <geom type="box" pos=".025 0 .013" size=".008 .007 .003" rgba=".70 .78 .83 1" contype="0" conaffinity="0"/>
          {''.join(wheels)}
        </body>
      </worldbody>
      <actuator>{''.join(f'<motor name="drive{i}" joint="wheel{i}" gear="1" ctrllimited="true" ctrlrange="-.2 .2"/>' for i in range(3))}</actuator>
    </mujoco>'''


class Plant:
    def __init__(self, *, mass=.30, mu=.65, incline=0., timestep=.001, fixed=False):
        self.xml = make_xml(mass, mu, incline, timestep, fixed)
        self.m = mujoco.MjModel.from_xml_string(self.xml)
        self.d = mujoco.MjData(self.m)
        self.body = self.m.body('chassis').id
        self.axles = [int(self.m.joint(f'wheel{i}').dofadr[0]) for i in range(3)]
        self.angles = [int(self.m.joint(f'wheel{i}').qposadr[0]) for i in range(3)]
        self.tires = [self.m.geom(f'tire{i}_{j}').id for i in range(3) for j in range(12)]
        self.mu = mu
        mujoco.mj_forward(self.m, self.d)

    def set_mu(self, mu):
        self.m.geom_friction[self.tires, 0] = mu
        self.m.geom_friction[self.m.geom('board').id, 0] = mu
        self.mu = mu

    def pose(self):
        mat = self.d.xmat[self.body].reshape(3, 3)
        return np.array([*self.d.xpos[self.body, :2], math.atan2(mat[1, 0], mat[0, 0])])

    def step(self, pwm, *, adhesion=0., supply=1., weak=1., push=(0., 0.), coast=False):
        for _ in range(round(DT / self.m.opt.timestep)):
            self.d.xfrc_applied[:] = 0
            gap = max(0., self.d.xpos[self.body, 2] - RW - .001)
            magnetic_force = adhesion * math.exp(-gap / .005)
            self.d.xfrc_applied[self.body, :3] = [*push, -magnetic_force]
            for i, u in enumerate(pwm):
                omega = self.d.qvel[self.axles[i]]
                _, _, ks, ct, damping = motor_constants(i, u or omega)
                effective = np.sign(u) * max(0., abs(u) * supply - ks)
                drag = .15 if coast and abs(u) < .5 else 1.
                torque = ct * effective * (weak if i == 0 else 1.) - drag * damping * omega
                self.d.ctrl[i] = np.clip(torque, -.2, .2)
            mujoco.mj_step(self.m, self.d)
        mujoco.mj_forward(self.m, self.d)

    def diagnostics(self):
        normal = 0.
        force = np.zeros(6)
        for i in range(self.d.ncon):
            mujoco.mj_contactForce(self.m, self.d, i, force)
            normal += max(0., force[0])
        return {'contacts': self.d.ncon, 'normal_N': normal,
                'height_m': float(self.d.xpos[self.body, 2]),
                'tilt_deg': math.degrees(math.acos(np.clip(self.d.xmat[self.body].reshape(3,3)[2,2], -1, 1)))}


class Wheels:
    def __init__(self, variant):
        self.proc = subprocess.Popen(['node', str(Path(__file__).with_name('controller_bridge.mjs'))],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self.call({'reset': True, 'variant': variant})

    def call(self, value):
        self.proc.stdin.write(json.dumps(value, allow_nan=False) + '\n')
        self.proc.stdin.flush()
        line = self.proc.stdout.readline()
        if not line:
            raise RuntimeError('Wheel controller subprocess stopped')
        result = json.loads(line)
        if 'error' in result:
            raise RuntimeError(result['error'])
        return result

    def close(self):
        self.proc.stdin.close()
        self.proc.wait(timeout=5)
        self.proc.stdout.close()


class Camera:
    """20 Hz pose samples, 80 ms delay; no truth velocity in the controller."""
    def __init__(self, rng):
        self.queue, self.last, self.next = [], None, 0.
        self.rng = rng

    def read(self, truth_pose, t, available=True):
        if available and t + 1e-8 >= self.next:
            self.next = t + .05
            sample = truth_pose + self.rng.uniform(-1, 1, 3) * np.array([.0002, .0002, .001])
            self.queue.append((t, sample))
        while self.queue and self.queue[0][0] + .08 <= t + 1e-8:
            stamp, pose = self.queue.pop(0)
            velocity = (pose - self.last[1]) / (stamp - self.last[0]) if self.last else np.zeros(3)
            if self.last:
                velocity[2] = math.atan2(math.sin(pose[2]-self.last[1][2]), math.cos(pose[2]-self.last[1][2])) / (stamp-self.last[0])
            self.last = (stamp, pose, velocity)
        return self.last if self.last and t-self.last[0] < .30 else None


SCENARIOS = {
    'floor': {'title': 'Floor translation', 'mass': .30, 'incline': 0., 'adhesion': 0.},
    'load': {'title': 'Unknown external load', 'mass': .30, 'incline': 0., 'adhesion': 0.},
    'payload': {'title': 'Doubled payload', 'mass': .60, 'incline': 0., 'adhesion': 0.},
    'weak': {'title': 'Motor 1 torque loss', 'mass': .30, 'incline': 0., 'adhesion': 0.},
    'sag': {'title': 'Supply drops to 65%', 'mass': .30, 'incline': 0., 'adhesion': 0.},
    'saturation': {'title': 'Severe supply loss and output limits', 'mass': .30, 'incline': 0., 'adhesion': 12.},
    'slope': {'title': '45-degree inclined board', 'mass': .30, 'incline': 45., 'adhesion': 12.},
    'wall': {'title': 'Vertical board and hold', 'mass': .30, 'incline': 90., 'adhesion': 12.},
    'slip': {'title': 'Friction falls on a vertical board', 'mass': .30, 'incline': 90., 'adhesion': 12.},
    'camera': {'title': 'Body feedback lost on a wall', 'mass': .30, 'incline': 90., 'adhesion': 12.},
    'encoder': {'title': 'Invalid encoder feedback', 'mass': .30, 'incline': 0., 'adhesion': 0.},
    'magnet': {'title': 'Magnetic attraction becomes insufficient', 'mass': .30, 'incline': 90., 'adhesion': 12.},
}


def run(name, variant='eso', seconds=8., timestep=.001, capture=None):
    cfg = SCENARIOS[name]
    plant = Plant(mass=cfg['mass'], incline=cfg['incline'], timestep=timestep)
    wheels = Wheels(variant)
    rng = np.random.default_rng(42)
    cam = Camera(rng)
    goal = np.array([0., 0., math.pi/2])
    task_goal = goal.copy()
    speed_filter = np.zeros(3)
    previous_counts = np.round(plant.d.qpos[plant.angles] / RAD_PER_COUNT)
    pwm = np.zeros(3)
    trace, statuses = [], set()
    slip_time = saturation_time = 0.
    fault = None
    ended = False
    try:
        for n in range(round(seconds / DT)):
            t = n * DT
            command = np.array([0., .12 if 1 <= t < 3 else 0., 0.]) if cfg['incline'] else np.array([.14 if 1 <= t < 3 else -.14 if 4 <= t < 6 else 0., 0., 0.])
            if not ended:
                goal += command * DT
                task_goal += command * DT
            sensor = cam.read(plant.pose(), t, available=not(name == 'camera' and t >= 4))
            counts = np.round(plant.d.qpos[plant.angles] / RAD_PER_COUNT)
            speed_filter += .4 * ((counts - previous_counts) / DT - speed_filter)
            previous_counts = counts
            valid = not(name == 'encoder' and t >= 4)
            if not valid:
                fault = 'ENCODER_INVALID'
            if sensor is None and t > .35:
                fault = 'POSITION_FEEDBACK_LOST'
            yaw = sensor[1][2] if sensor else math.pi/2
            A = kin(yaw)
            residual = speed_filter*MPC - A@ sensor[2] if sensor else np.zeros(3)
            slip_time = slip_time + DT if sensor is not None and max(abs(residual)) > .08 else max(0., slip_time-DT*2)
            slip = slip_time > .2
            saturation_time = saturation_time + DT if max(abs(pwm)) >= 250 else max(0., saturation_time-DT)
            limited = saturation_time > .25
            desired = command.copy()
            if sensor:
                error = goal - sensor[1]
                error[2] = math.atan2(math.sin(error[2]), math.cos(error[2]))
                desired += 2.0 * error
            desired[:2] = np.clip(desired[:2], -.30, .30)
            desired[2] = np.clip(desired[2], -1.2, 1.2)
            if slip or limited:
                desired *= .35 if slip else .6
                # Do not accumulate an unreachable reference while derated.
                if sensor:
                    goal = sensor[1].copy()
            targets = A@desired/MPC
            targets *= min(1., 10000/max(1., max(abs(targets))))
            control = wheels.call({'targets': targets.tolist(), 'measured': speed_filter.tolist(),
                                  'hold': True, 'learn': not(slip or limited), 'rate': 3000 if slip else 12000,
                                  'stop': bool(fault or ended)})
            pwm = np.array(control['pwm'])
            status = 'TRIAL_ENDED' if ended else fault or ('SLIP_DERATE' if slip else 'SATURATION_DERATE' if limited else 'HOLD' if not np.any(command) else 'TRACKING')
            statuses.add(status)
            supply = .65 if name == 'sag' and t >= 3 else .30 if name == 'saturation' and t >= 3 else 1.
            weak = .65 if name == 'weak' else 1.
            mu = .06 if name == 'slip' and t >= 4 else .65
            adhesion = 1.5 if name == 'magnet' and t >= 4 else cfg['adhesion']
            push = (-.8, 0.) if name == 'load' and 3 <= t < 6 else (0., 0.)
            plant.set_mu(mu)
            if not ended:
                plant.step(pwm, adhesion=adhesion, supply=supply, weak=weak, push=push, coast=bool(fault))
            diag = plant.diagnostics()
            if not np.all(np.isfinite(plant.d.qpos)) or not np.all(np.isfinite(plant.d.qvel)):
                raise RuntimeError('Nonfinite physics state')
            frame = {'t': round(t+DT, 4), 'pose': plant.pose().tolist(), 'goal': goal.tolist(), 'task_goal': task_goal.tolist(),
                     'qpos': plant.d.qpos.tolist(), 'speed': speed_filter.tolist(), 'pwm': pwm.tolist(),
                     'comp': control['comp'], 'status': status, 'fault': fault, 'slip': slip,
                     'camera_fresh': sensor is not None, 'mu': mu, 'supply': supply, 'adhesion': adhesion,
                     'push': list(push), **diag}
            trace.append(frame)
            if capture is not None and n % 5 == 0:
                capture(plant, frame)
            if plant.pose()[1] < -.6 or diag['tilt_deg'] > 65 or diag['height_m'] > .12:
                ended = True
                pwm[:] = 0
                fault = fault or 'LEFT_TEST_ENVELOPE'
    finally:
        wheels.close()
    # Score against the original task, never the derated/reset reference.
    errors = [np.linalg.norm(np.array(f['pose'][:2])-f['task_goal'][:2]) for f in trace]
    post = [e for e, f in zip(errors, trace) if 3 <= f['t'] < 6]
    summary = {'max_tracking_error_m': max(errors), 'post_event_mean_error_m': float(np.mean(post)),
               'final_pose': trace[-1]['pose'], 'peak_pwm': max(abs(x) for f in trace for x in f['pwm']),
               'max_tilt_deg': max(f['tilt_deg'] for f in trace), 'ended_early': ended,
               'fault': fault, 'statuses': sorted(statuses), 'solver_warnings': int(sum(w.number for w in plant.d.warning))}
    return {'scenario': name, 'variant': variant, 'config': cfg, 'metrics': summary, 'trace': trace}
