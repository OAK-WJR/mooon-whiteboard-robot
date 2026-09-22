# Open-source references and response design

This page describes the first JavaScript model/ESO experiment. The subsequent [C++ controller and research basis](../../ROBUST_CONTROL.md) replace its fixed demand reduction with adaptive reduction, align slip observations by capture time, and add latched consistency faults. Results and failure boundaries are reported separately.

Research date: 2026-09-21. The local `hyperresearch search` command reported no initialized vault. Official documentation and project source were then read using built-in web search; no external search API keys were used.

## What was referenced

| Source | What informed this implementation | What was not assumed |
|---|---|---|
| [MuJoCo Python bindings](https://mujoco.readthedocs.io/en/stable/python.html), [modeling guide](https://mujoco.readthedocs.io/en/stable/modeling.html), [joint/actuator reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html) | Official `mujoco` package, free joints, passive hinge joints, torque actuators, reflected armature inertia and native rendering. | Choosing a physics engine does not calibrate friction, inertia or magnet forces. |
| [JunHeonYoon/mujoco_mecanum](https://github.com/JunHeonYoon/mujoco_mecanum), [wheel generator source](https://github.com/JunHeonYoon/mujoco_mecanum/blob/master/wheel_code_gen.py) | Explicit roller joints and force-based wheel actuation. The project specifically warns against substituting a velocity actuator for wheel dynamics. | Its Mecanum roller angle, dimensions, inertia and older `mujoco_py` setup do not describe this three-omniwheel robot. Our tangent-roller model was written independently; no code/assets copied. |
| [Omni3WD](https://github.com/mohmedatwa/Omni3WD) | Three wheels at 120 degrees and separation of chassis kinematics, wheel controllers and simulation. | This is a reference architecture, not validation of our wiring, loaded behavior or wall adhesion. No ROS/Gazebo dependency or project code was imported. |
| [WPILib SysId](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/index.html), [feedforward](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html) | Measure per-mechanism dynamics and distinguish feedforward from feedback. Real voltage-based feedforward can address battery sag when applied voltage is measured/controlled. | PWM duty is not motor voltage or measured torque. The current robot has no VMOT telemetry. |
| [MathWorks ESO example](https://www.mathworks.com/help/slcontrol/ug/apply-extended-state-observer-for-disturbance-rejection.html), [observer reference](https://www.mathworks.com/help/slcontrol/ug/extendedstateobserver.html) | A model-based disturbance estimate can supplement a speed controller; model quality and compensation limits matter. | The example does not prove our observer stable under arbitrary contact, delay, current limits or unknown surfaces. |
| [Modern Robotics: omnidirectional wheels](https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-2-omnidirectional-wheeled-mobile-robots-part-1-of-2/), [AprilTag](https://github.com/AprilRobotics/apriltag) | Wheel/body kinematics and a possible independent body-pose measurement for later hardware validation. | Wheel odometry alone is not ground truth under slip. A camera-like simulation is not an implemented AprilTag pipeline. |

MuJoCo permits anisotropic friction in explicit contact pairs, with attention to the contact frame. Instead of relying on an unverified friction-axis alignment, this model represents transverse rolling with actual passive roller hinges. Contact still uses an assumed Coulomb friction coefficient. See the [contact-parameter discussion](https://mujoco.readthedocs.io/en/stable/modeling.html#contact-parameters).

## Implemented responses, by condition

| Condition | Implemented in simulation | Remaining hardware requirement / limit |
|---|---|---|
| Different wheel responses | Six direction-specific speed models, bounded PI/ESO, one shared wheel-target ramp | Verify actual loaded response and wheel/body geometry; controller is not deployed. |
| Unknown payload / wipe force | PI rejects residual speed error; ESO estimates an additional bounded input-equivalent disturbance | Need measured loaded torque/current and noise behavior. ESO was worse in one external-force trial. |
| Battery sag | Plant reduces effective drive. Feedback and ESO react to the resulting response error | Not voltage feedforward: no injected supply value is supplied to the controller. Add VMOT sensing before voltage normalization. |
| Persistent PWM saturation | Reduce demand to 60%, freeze extra learning, retain output bounds; score against original requested task | Demand reduction cannot recover an infeasible task. Current and thermal protection are not modeled. |
| Slip evidence | Compare encoder speed with projected delayed body velocity; after a sustained discrepancy, reduce demand to 35%, lower ramp rate and freeze PI/ESO learning | Requires independent body sensing. Thresholds are provisional; not a certified slip detector. |
| Gravity and release-to-hold | Camera-like pose outer loop preserves a bounded hold reference and requests wheel torque | No identified gravity feedforward or real body sensor yet. Static heating and power budgets remain unknown. |
| Lost body feedback | Latch stale-data fault and clear outputs after 300 ms without fresh pose | On a wall this can cause sliding. It is a fault experiment, not a safe wall-stop design. |
| Explicitly invalid encoder | Clear outputs and latch fault | Injection uses a validity flag. Automatic diagnosis of stuck/noisy physical encoder signals is not proven by it. |
| Reduced magnetic attraction | Physics reduces normal force; slip response can trigger | Fixed magnets cannot gain attraction through PWM. Mechanical adhesion/fall prevention must be validated separately. |

The supervisor does not know which test condition was selected. It receives encoder samples, explicit validity and delayed pose measurements; it does not receive friction, injected force, mass, magnetic attraction, contact forces or true body velocity. Scenario injection and trial-envelope monitoring belong to the plant/test harness.

## Interpretation

Contact dynamics expose limitations hidden by a wheel-only model. Some failures are due to insufficient available force rather than poor PID gains. The present results justify further measured, restrained loaded experiments and an independent body-motion measurement; they do not justify claiming automatic adaptation to every board or load.
