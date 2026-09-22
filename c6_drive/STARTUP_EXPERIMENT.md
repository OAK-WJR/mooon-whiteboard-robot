# Difficult startup under load — bounded ramp experiment

The user reports that the wheels themselves are difficult to start and turn slowly under load. Total robot mass has **not been measured**. This observation makes loaded breakaway, mechanical drag, available torque and power delivery the immediate diagnostic focus. It does not establish which cause is responsible. No hardware measurements or commands were made during this work.

The previous encoder-servo candidate uses fixed 160 / 140 / 140 PWM breakaway magnitudes inherited from suspended tests. In the assumed 2 kg high-drag Y-axis trial at 0.04 m/s, M1/M3 remain nearly stationary at those magnitudes and the recovery expires. That reproduces one possible mechanism; it is not a diagnosis of the physical robot.

## Isolated implementation

`sim/physics/startup_experiment.py` builds the same native C++ command/lease/fault wrapper with one checked source transformation in a temporary include directory. During demanded travel, the existing breakaway pulse rises from its fixed magnitude toward the existing 255 ceiling as its original 250 ms budget elapses. Expected-direction encoder motion ends recovery as before. Holding behavior, PWM slew, cooldown, no-motion deadlines, STOP, invalid-data and lease guards remain unchanged. This adds no mass input, current estimate or slip detector.

The published `include/encoder_servo.h`, firmware selection routes and deployed robot remain unchanged. The exact transformation, source hashes and both executable hashes are stored in `reports/startup-ramp-dev01.manifest.json` before the campaign. This is an experimental variant, not a released controller.

The rationale is empirical breakaway compensation and bounded feedback: [WPILib feedforward documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html) identifies static friction compensation as a quantity requiring measurement. The current anti-windup behavior is retained, consistent with [MathWorks saturation/tracking guidance](https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html). Neither source validates this robot-specific ramp or its thermal duty.

## Results, including remaining failures

The fixed and ramped versions each ran 54 load cases and 15 wheel-model regression cases, for 138 simulation runs. The transformed C++ passes the existing sanitizer-enabled safety test for retained bias, STOP/reset, leases, frozen feedback, invalid samples and held overload. All simulation runs have zero solver warnings and zero commanded output after a fault.

| Load-study measure | Fixed pulse | Ramped pulse |
|---|---:|---:|
| Within all motion bounds | 29 / 54 | 30 / 54 |
| Startup faults | 2 | 1 |
| High-drag main group passed | 0 / 12 | 0 / 12 |

In the illustrative 2 kg high-drag Y-axis case at 0.04 m/s, the fixed version faults 0.52 s after the travel command, with only about 2.2 mm final body travel. The ramped version starts M1/M2/M3 after about 460 / 350 / 450 ms and reaches about 84.5 mm, versus the original 120 mm request. Its final original-request error is still **35.5 mm**, exceeding the 10 mm bound. One additional pass elsewhere is a light-drag settling improvement, not a qualified high-drag case.

The remaining 2 N/wheel high-drag Y case still faults. Wheel-model jam, lost lease, invalid encoder and frozen encoder cases retain their latched stops. A successful start alone does not show adequate continuous torque, straight-line synchronization or thermal margin. This variant has not undergone a new independent validation set and was **not promoted or flashed**.

The UI shows the user-reported mass as unknown and separates offline simulated body/encoder discrepancy from observable wheel startup or near-full PWM. A physical loaded recording of PWM, wheel motion and supply voltage is still needed to distinguish insufficient startup compensation from electrical/mechanical limits. Weight would improve the force budget but is not required to inspect whether encoders respond to a bounded startup command.

## Reproduce (no hardware)

```sh
.venv/bin/python c6_drive/sim/physics/startup_experiment.py --report /private/tmp/wbb-startup-new.json
MPLCONFIGDIR=/private/tmp/wbb-mpl-servo .venv/bin/python scripts/c6_startup_report.py
```

Reports refuse overwrites. The publisher verifies source hashes before deriving `startup-ramp-summary.json` and the English PNG/SVG figure. The example plot is simulation, not physical footage.
