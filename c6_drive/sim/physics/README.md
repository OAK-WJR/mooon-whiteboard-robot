# MuJoCo roller-wheel study

New: see [the C++ controller implementation and qualification limits](../../ROBUST_CONTROL.md) and its [English replay](../../portfolio/robust.html). `robust_campaign.py` executes the C++ core directly; its prospective acceptance and wider stress reports are separate from the earlier model/ESO comparison below.

Run from the repository root:

```sh
.venv/bin/python -m pip install -r c6_drive/sim/physics/requirements.txt
.venv/bin/python c6_drive/sim/physics/run_suite.py
.venv/bin/python c6_drive/tests/test_physics.py
.venv/bin/python c6_drive/sim/physics/render.py
```

`run_suite.py --scenario wall` runs a single condition and writes suffixed reports without replacing the complete matrix. `render.py --scenario wall --stills` renders a preview. On macOS, rendering requires access to the graphics session. No simulator process talks to the physical robot.

## Physics and data boundary

- MuJoCo 3.13.0, 1 ms physics, 10 ms control, Newton solver, elliptic friction cones, implicit-fast integration.
- Free six-DOF chassis, three driven wheel hinges, 36 free roller hinges: 45 generalized velocities total. No chassis-velocity command, planar joint, equality constraint or fixed attachment to the surface.
- Wheel radius 19 mm, hub-center radius 33.738 mm, nominal total mass 0.30 kg. Geometry is an original approximation; it is not a verified CAD import. Hub colors identify wheels 1/2/3 (orange/blue/green).
- Six measured moving-range motor gains/time constants map PWM to speed. An **assumed** 4 N maximum tangential drive sets torque scale. Reflected inertia is chosen to approximate measured response, not independently identified. No current, inductance, gear backlash or thermal model. The measured response includes filtering; inferred inertia must not be treated as a physical measurement.
- Passive roller hinges allow transverse motion. Only tire-plane collisions are enabled. Body collisions and neighboring roller collisions are omitted. Contact softness, roller friction/shape and their damping are assumptions.
- Board is the XY plane. Gravity is rotated to represent horizontal, inclined and vertical use. Thus vertical-board graphics are displayed in board coordinates, with gravity along -Y. The body remains free to tip and separate.
- Magnetic attraction is an assumed force toward the board, decreasing exponentially with excess separation (5 mm scale). This is not a characterized magnet model. Its magnitude is never provided to the controller.
- Encoders are quantized with the inherited 2800 counts/rev assumption and filtered at 100 Hz. A separate simulated camera supplies noisy pose at 20 Hz after 80 ms. The controller gets finite-difference camera velocity, not truth velocity. Current physical hardware has no such body-feedback pipeline.

`controller_bridge.mjs` runs the same tested JavaScript `ModelWheel` for both physics variants. Model PI and model PI+ESO share all other control settings. This isolates the incremental observer effect; it does not compare against the unchanged robot firmware.

The supervisor derates demand and freezes learning when saturation or slip evidence persists. It does not secretly use the test's friction or load. Fault injection and a test-envelope stop are distinct. Leaving Y < -0.6 m, tilting more than 65 degrees, or separating beyond 0.12 m ends a trial; playback freezes afterward. This is not simulated fall arrest.

## Results

`../../reports/mujoco-results.json` contains source hashes, assumptions and all 24 runs (12 conditions × 2 controllers). `mujoco-traces.json` keeps 20 Hz recorded states for rendering; scoring uses every 100 Hz sample. Position error is always measured against the original task reference, including during demand reduction. Do not report “all 24 passed” as a motion-performance claim: several tests intentionally fail or fault.

For the 3–6 s window, model PI → model PI+ESO mean task-position error:

| Condition | Error | Interpretation |
|---|---|---|
| Unknown 0.8 N external force | 18.9 → 20.8 mm | ESO slightly worse in this test |
| Motor 1 at 65% drive torque | 21.3 → 19.9 mm | Some improvement; heading still imperfect |
| Supply drops to 65% | 24.3 → 21.2 mm | Some improvement; more PWM used |
| Vertical hold with assumed 12 N attraction | 12.0 → 9.6 mm | Conditional simulated hold, not hardware proof |
| Severe supply loss | 104.7 → 105.8 mm | Both hit output limits and miss the task |
| Friction drop / magnet weakening | Both leave the test envelope | Compensation cannot replace traction |
| Camera loss on vertical board | Both fault and slide | Output-off is not a safe wall-hold policy |

The test suite checks free-body structure, static weight support, frictionless gravity, six freewheel gains, delayed sensor validity, controller reset, a halved-timestep comparison and scenario fault/output invariants. These checks test the implementation, not agreement with the actual loaded robot. See [source research and condition responses](RESEARCH.md).
