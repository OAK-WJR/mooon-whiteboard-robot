# Heavy-load and rubbing-contact study

The physical robot is still on the wall. This work used **software and simulation only**: no robot connection, commands, resets or uploads. The encoder servo and its output/protection limits were left unchanged.

The previous 24-case floor result covered only **0.25–0.45 kg** and had no rubbing pad. It cannot establish performance for a substantially heavier robot or an eraser dragging on the surface. Actual total mass, pull force, motor torque and pad pressure have not been measured. The new 0.5 / 1 / 2 kg points are assumed sensitivity cases, not estimates of the user's robot.

## Model additions

`sim/physics/load_plant.py` extends the existing 6-DOF body, three powered hubs and 36 passive rollers. It does not change the archived plant or its results.

- Additional dry axle friction uses MuJoCo joint `frictionloss`, including resistance before movement. Added passive roller friction represents bearing resistance. These are separate from the archived motor loss and sliding grip coefficient. [MuJoCo friction-loss model](https://mujoco.readthedocs.io/en/stable/computation/index.html#friction-loss).
- The optional pad is an assumed 20 × 20 mm, 20 g contact body on a damped vertical spring. Its nominal preload is specified at zero spring displacement; actual normal force is solved, not prescribed. A 2 N or 5 N preload does **not** mean 2 N or 5 N of drag. Pad friction is 0.6, tire friction 0.65. The pad supports part of the robot, reducing wheel normal force and available grip. It exerts no rubbing force when clear of the surface. [MuJoCo contact and spring reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint-springref).
- Pad mass is subtracted from chassis mass. Total modeled mass equals the declared mass. Geometry, center of mass, inertia and compliance remain approximate; this is not verified robot CAD.
- The archived motor authority is **an assumed 4 N per wheel at full duty and zero speed**, not a measured or continuous rating. Six additional cases halve it to 2 N. Motor damping and reflected inertia scale together, preserving the identified unloaded moving-range gain/time constant. Actual current, temperature and battery sag are not identified by this study.

| Profile | Added axle friction, M1 / M2 / M3 | Each passive roller | Nominal pad preload |
|---|---:|---:|---:|
| No added drag | 0 / 0 / 0 mN·m | 0 | 0 |
| Bearings | 8 / 6 / 6 mN·m | 10 µN·m | 0 |
| Bearings + pad | 8 / 6 / 6 mN·m | 10 µN·m | 2 N |
| High drag | 30 / 20 / 20 mN·m | 40 µN·m | 5 N |

“No added drag” still includes the earlier motor and contact model. More drag is not always worse in every score: damping can reduce settling oscillation, while added weight can increase wheel grip. Neither trend justifies adding ballast without a physical force budget.

## Declared sweep and results

The manifest, source hashes, cases and acceptance bounds were written before running the sweep. There was no controller tuning against these results. The 48 main cases combine three masses, four profiles, two directions and two requested speeds (0.04 and 0.08 m/s). Six 1 kg cases test the lower torque assumption. Each run requests travel for three seconds and then allows four seconds to settle. Per-wheel delay is 0 / 30 / 70 ms. Reverse travel, yaw commands and a loaded wall are not qualified by these floor trials.

The same C++ `drive::Control` and `EncoderServo` are used. Inputs are command, quantized encoder increments, filtered encoder speed and prior PWM register value. Mass, contact force, friction, motor torque and body truth never reach the controller. Only the scorer has true chassis position.

| Main group, 4 N/ wheel assumption | Within all motion bounds |
|---|---:|
| No added drag | 9 / 12 |
| Bearing drag | 12 / 12 |
| Bearings + 2 N pad | 8 / 12 |
| High drag + 5 N pad | 0 / 12 |
| Additional 2 N/wheel cases | 0 / 6 |
| Total | **29 / 54** |

Pass means final original-request error ≤10 mm, peak lateral error ≤5 mm, final heading error ≤5°, final-second displacement range ≤3 mm, and no fault, envelope departure or solver warning. All 54 had zero solver warnings. This is a deterministic sensitivity sweep, not a probability of real-world success. A small lateral error alone can mean the robot barely moved.

Two distinct failures matter:

1. **1 kg, high drag, X, 0.08 m/s:** original request 240 mm, actual final travel about 14.5 mm. Peak body-versus-odometry discrepancy is about 180 mm. The rubbing pad carries about 5.3 N, unloading the wheels. The controller sees wheel travel and cannot infer this large body shortfall from those counts alone. Increasing wheel PID gains does not add the missing measurement or grip.
2. **2 kg, high drag, X, 0.08 m/s:** actual travel about 149 mm versus the original 240 mm. M1 spends 1.30 s at |PWM| ≥250 during the move; the common reference scale falls to 13%. Encoder/body discrepancy stays below 2.6 mm. Here output authority and reference slowdown dominate, rather than a large hidden displacement. At 0.04 m/s, original request is 120 mm and final travel about 86.9 mm; lowering speed alone still fails the declared travel bound.

Two other cases trip `SERVO_START_NO_MOTION`; commanded outputs after fault are zero. The simulated heavy-load no-motion stop does not prove a broken motor, nor does it prove the real robot has sufficient torque. No thresholds were relaxed to make the sweep pass. The no-added-drag 2 kg failures are settling failures, not all traction failures.

The candidate is therefore **not established as a robust heavy-load controller**. Next physical inputs needed are total mass, start/running pull force with the rubbing parts engaged and disengaged, and per-wheel response under load. Only then can breakaway duration, feasible acceleration and output authority be calibrated meaningfully. Encoder-only compensation can respond to visible wheel slowdown within available torque; it cannot guarantee body position during slip, create extra torque, or mechanically arrest a falling robot.

## Verification and reproduction

Five physical checks pass: unchanged behavior with all additional resistance disabled, total mass/contact-force balance, dissipation and dry-friction bounds, no pad drag in air, and invalid-parameter rejection. The UI shows these trials on the existing simulator/gallery with selectable mass, resistance, speed and direction. English PNG/SVG figures are generated from the saved data.

```sh
.venv/bin/python c6_drive/tests/test_load_plant.py
.venv/bin/python c6_drive/sim/physics/load_campaign.py --report /private/tmp/wbb-heavy-new.json
MPLCONFIGDIR=/private/tmp/wbb-mpl-servo .venv/bin/python scripts/c6_load_report.py
```

The campaign refuses to overwrite reports/manifests; the publisher verifies source hashes. Raw evidence: `reports/heavy-load-01.json`, with its matching manifest. Summary/plots are generated outputs. Earlier 24/24 light-load results remain visible with their explicit mass/pad limitations.
