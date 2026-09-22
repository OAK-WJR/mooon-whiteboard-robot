# Robust controller candidate: implementation and measured limits

Hardware follow-up: the [switchable wheel-only adapter](ROBUST_HARDWARE.md) has now been flashed and tested on the actual C6 robot. Revision 4 passed 17 suspended speed fixtures and reversal/release/timeout checks, with retained earlier startup failures. It is distinct from the body-feedback controller qualified below. [Hardware results](reports/robust-hardware-summary.md).

The portable [C++ controller](include/robust_control.h) is implemented and executed directly inside the MuJoCo test loop through a native bridge. In 60 prospectively selected paired trials, mean task-position error decreased from **9.417 to 4.447 mm (52.8%)** relative to the previous model PI + ESO supervisor. All 60 trials improved position error and passed the declared limits. These are simulated results with independent body feedback, not results from the flashed robot.

The wider 30-trial stress set **did not qualify**: mean position error increased from **36.74 to 58.93 mm**, although mean heading error decreased from 11.53° to 6.33°. Seed 9129 stalled, latched zero output, then fell in the simulation. Reducing motion demand can preserve heading at the cost of task completion. This candidate does not solve arbitrary loads or surfaces.

- [English interactive replay and portfolio media](portfolio/robust.html)
- [Machine-readable audit](reports/robust-validation.json)
- [Acceptance criteria fixed before holdout](sim/physics/robust-acceptance.json)
- [Nominal](reports/robust-nominal.json), [holdout](reports/robust-holdout.json), [wider stress](reports/robust-stress.json) and [development](reports/robust-development.json) reports

## What was implemented

1. **Six motor models:** separate positive/negative gain, time constant and dead-zone estimates from `sysid-model-03.json`. Model feedforward and bounded acceleration feedforward reduce the different wheel responses. Feedback is PI, with no speed-derivative term.
2. **Bounded disturbance observer:** estimates an input-equivalent disturbance from filtered encoder speed and the actual previous drive duty. It receives no true payload, friction, force, supply scale or contact state. Compensation is limited to ±65 PWM and changes at no more than 100 PWM/s. Learning freezes at low speed, strong saturation or detected slip; the model does not identify static holding torque.
3. **Coordinated wheel demands:** a common target filter, common vector ramp and common scaling preserve commanded wheel ratios. PWM is limited to ±255 and 1200 PWM/s relative to the actual last applied duty. The integral tracks the final constrained output and remains within ±120.
4. **Body-position and heading feedback:** timestamped pose measurements pass freshness, sequence and jump checks. A filtered finite-difference velocity predicts pose through up to 200 ms of sensor age. This is a constant-velocity predictor, not a Kalman filter. The heading goal is retained; translation yields as heading error grows.
5. **Available-output response:** high actual duty plus persistent wheel-speed error reduces the common motion demand, potentially to zero. Recovery is slow and requires output margin. The controller limits accumulated translation tracking debt; it does not claim that a derated task has been completed. The evaluator always uses the original requested trajectory.
6. **Feedback consistency and fault states:** wheel-distance history aligns encoder observations with body capture timestamps. Persistent mismatch can reduce demand and freeze learning; a nearly stationary encoder inconsistent with observed body motion can latch `FEEDBACK_INCONSISTENT`. This is ambiguous feedback/traction evidence, not a definitive diagnosis of a broken encoder. Invalid input, stale commands, stale body feedback, late loops, stalls and emergency stops latch zero output until explicitly re-armed.

Zero output is an electrical stop request. It is **not a physical wall brake**. In the rapid friction-loss and magnet-loss fixtures, the robot falls before the new slip detector trips. The playback boundary then stops/freezes the test; it is not an implemented catch mechanism.

## Evidence and operating envelope

The 60 holdout seeds (8100–8159) were run only after tuning was frozen. The envelope was narrowed during development and declared before that holdout; it is not the wider stress distribution. Twelve development seeds (1100–1111), 30 stress seeds (9100–9129), and 16 named nominal/fault cases remain available. Each run lasts 8 seconds; there is no long-duration thermal or endurance validation.

| Parameter | Qualified simulated range | Wider stress range |
|---|---|---|
| Chassis mass | 0.22–0.30 kg | 0.22–0.48 kg |
| Contact friction coefficient | 0.60–0.85 | 0.50–0.85 |
| Board inclination | 0°, 45°, 90° | Same |
| Magnetic normal force when inclined | 12–15 N | 14–18 N |
| Effective supply scale | 0.90–1.05 | 0.80–1.05 |
| Per-wheel torque scale | 0.90–1.15 | 0.80–1.20 |
| Per-wheel damping scale | 0.90–1.10 | 0.85–1.15 |
| Reflected inertia scale | 0.80–1.30 | 0.75–1.35 |
| Pose delay at 20 Hz | 40–120 ms | 40–150 ms |
| Pose noise bound per position axis | 0.2–0.8 mm | Same |
| Encoder noise bound | 10–50 counts/s | Same |
| Independent pose-drop probability | 0–8% | Same |
| Actuation delay | 0 or 10 ms | Same |
| Translation command | 0.08–0.12 m/s | 0.09–0.17 m/s |
| Additional force per board axis | ±0.2 N | ±0.3 N |

These ranges are assumptions in a model, not a hardware operating specification. Both variants get identical fixtures and noise seeds, and the same simulated body-pose channel. Controller improvements combine wheel-law, filtering, estimator and supervisor changes; this comparison does not isolate the observer's contribution.

| Holdout metric | Previous ESO supervisor | C++ candidate |
|---|---:|---:|
| Mean task error across 60 runs | 9.417 mm | 4.447 mm |
| Worst individual run's mean error | 18.081 mm | 8.723 mm |
| Mean heading error | 2.279° | 1.794° |
| Maximum heading error across runs | 11.764° | 11.432° |
| Unexpected fault / envelope exit | 0 / 0 | 0 / 0 |

Position error is Euclidean distance from the **original task reference**, averaged over `1 <= t < 8` seconds, then equally across runs. Tests that fall remain in the score; the trial freezes at its boundary, so post-fall error is not a physical prediction. Acceptance limits were 35 mm mean position error, 8° mean / 30° peak heading error, bounded output/integral/compensation, no solver warnings, no unexpected faults/falls, and at least 25% aggregate improvement. The audit checks report hashes, complete seed sets, paired fixtures, numerical limits and retained failures.

The MuJoCo chassis has six free body DOFs, three driven wheel joints and 36 passive roller joints. It is force driven; body speed is not imposed. The extended randomized plant matches the previous physics implementation at identical parameters. Native replay of saved cases checks that media traces came from the current C++ executable. Mass, magnet strength, torque conversion, inertia, wheel geometry and independent pose sensing are still uncalibrated. The measured motor fit covers suspended motion at 120–200 PWM; extrapolation to static hold or near-zero speed remains unvalidated.

## Firmware integration contract

The robot now runs firmware containing the switchable wheel-only adapter. The full body-feedback controller remains a bounded-memory library plus a directly executable simulation bridge. The core has passed native address/undefined-behavior sanitizers and an ESP32-C6 cross-compilation probe with exceptions and RTTI disabled. The full controller object is 1372 bytes in the native build. The separate hardware report measures the adapter at a 10 ms loop interval; loaded tracking, physical slip detection and wall holding remain unvalidated.

```cpp
#include "robust_control.h"
wbb_robust::Controller controller;  // defaults to requiring independent body pose

// Explicit start of a new externally authorized control session:
controller.arm(0.0f);

// Each 100 Hz tick, after assembling a coherent sensor sample:
wbb_robust::Input in;
in.now = sessionSeconds;
in.dt = elapsedSeconds;
in.commandStamp = lastAcceptedCommandSeconds;
in.command = {vxInBoardFrame, vyInBoardFrame, yawRate};
for (int i = 0; i < 3; ++i) {
    in.speed[i] = filteredSignedEncoderCountsPerSecond[i];
    in.applied[i] = signedDutyActuallyAppliedDuringLastInterval[i];
}
in.encoderValid = allEncoderChecksPass;
in.bodyValid = poseAvailable;
in.bodyStamp = poseCaptureTimeInSessionSeconds; // not USB/Wi-Fi arrival time
in.bodyPose = {xMetres, yMetres, headingRadians};
in.estop = externalStopRequested;
const auto& out = controller.update(in);
// Adapter must apply all three out.pwm values, including immediate fault zeros,
// record what hardware actually applied, and preserve the existing output checks.
```

Use a monotonic, **session-relative** clock for these float timestamps, synchronized with pose capture. Do not convert a many-day absolute uptime directly to float; precision loss can invalidate timing. Supply actual measured `dt`; an accepted tick is 5–25 ms, a gap over 40 ms faults. Command heartbeat must remain fresh (300 ms default), even while holding a zero velocity command. Retain transport leases, replay protection, hardware watchdogs and explicit re-arm outside this core. Never re-arm automatically on the next packet.

Encoder direction, motor ordering, physical forward axis and pose coordinates must match `toWheels()` before integration. That mapping assumes the inherited 38 mm wheel, 2800 counts/revolution and 33.738 mm chassis wheel-center radius. These are not newly measured dimensions. Signed applied duty must reflect clamps, supply shutdown and driver changes, not simply the previous request.

Full mode needs an independent `(x, y, heading, capture timestamp)` source. The current LCD board does not provide this. A separately calibrated external camera/AprilTag channel is one possible implementation; it has not been added by this change. Encoder-only mode is explicit: set `Config::requireBody=false` when constructing the controller. It reports `COAST` and clears output at zero command, has no body drift correction or wall hold, and has not undergone the body-feedback holdout qualification. Full mode never silently falls back to it after losing pose.

## Reproduce without connecting the robot

```sh
c++ -std=c++17 -O1 -g -fsanitize=address,undefined -Wall -Wextra -Werror \
  -I c6_drive/include c6_drive/tests/test_robust.cpp -o /tmp/wbb-test-robust
/tmp/wbb-test-robust

.platformio/packages/toolchain-riscv32-esp/bin/riscv32-esp-elf-g++ \
  -std=gnu++17 -Os -fno-exceptions -fno-rtti -Wall -Wextra -Werror \
  -I c6_drive/include -c c6_drive/tests/robust_embedded_compile.cpp \
  -o /tmp/wbb-robust-c6.o

.venv/bin/python c6_drive/sim/physics/robust_campaign.py --suite nominal
.venv/bin/python c6_drive/sim/physics/robust_campaign.py --suite development --variants eso robust
.venv/bin/python c6_drive/sim/physics/robust_campaign.py --suite holdout --count 60 --variants eso robust
.venv/bin/python c6_drive/sim/physics/robust_campaign.py --suite stress --count 30 --variants eso robust
.venv/bin/python c6_drive/sim/physics/evaluate_robust.py
.venv/bin/python c6_drive/tests/test_robust_campaign.py
.venv/bin/python c6_drive/sim/physics/render_robust.py
.venv/bin/python scripts/c6_drive_sim.py
# http://127.0.0.1:8088/portfolio/robust.html
```

## Research basis

The local knowledge-vault search found no initialized vault; primary documentation was read before implementing the new algorithm. No external search API keys were used. This is independently written code, not imported third-party controller code.

- [MathWorks anti-windup and actuator tracking](https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html) supports tracking constrained actuator output when feedforward and cascaded limits are present. Here the tracked contribution includes duty saturation/rate limits and a dead-zone conversion.
- [WPILib holonomic controller](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/holonomic.html) separates field-frame position feedback from heading control. Its gains and robot geometry were not copied.
- [WPILib delayed pose estimation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html) motivates using capture timestamps and handling measurement latency. Our finite-difference predictor is simpler than WPILib's estimator and has weaker noise/model handling.
- [Herbst, Practical ADRC](https://arxiv.org/abs/1908.04610) discusses magnitude/rate-limited actuation and practical observer/controller transitions. It does not prove this hybrid controller stable under unknown friction or contact loss.
- Existing [MuJoCo/contact and open-source research](sim/physics/RESEARCH.md) documents the plant and its limitations.
