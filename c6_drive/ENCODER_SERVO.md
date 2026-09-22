# Continuous encoder servo — development candidate

This implements a new encoder-only controller in `include/encoder_servo.h`, exercised through the actual `drive::Control` command/lease/fault wrapper. **It is not deployed or wall-qualified.** The user explicitly requested software and simulation only while the physical robot remains on the wall. There were no device connections, commands, resets or uploads during this development turn.

**Later heavy-load sweep:** the earlier floor group below covered only 0.25–0.45 kg with no rubbing pad. The unchanged controller passes 29/54 added 0.5–2 kg resistance/torque sensitivity cases, including 0/12 in the main high-drag group. See [heavy-load findings](HEAVY_LOAD.md); the light-load pass does not qualify the heavier robot.

`Control::selectServo()` is available to native development harnesses. There is deliberately no USB/web selection route, validation shortcut or persisted selection for this candidate. Existing legacy, robust-wheel and encoder-hold modes remain available as before. The ESP32-C6 firmware compiles with the new implementation.

## What changed

- A single body-command profile limits translation acceleration to 0.25 m/s² and angular acceleration to 2 rad/s². Inputs remain limited to 0.08 m/s and 0.6 rad/s. Braking follows the profile before holding its final planned endpoint; release does not promise an instantaneous stop at the measured release position.
- Relative SE(2) odometry uses raw signed encoder increments, as in the existing controller. The desired pose is transformed into three wheel-displacement errors. Each wheel receives position and velocity feedback plus the identified moving-range feedforward. This couples travel/heading correction to wheel displacement rather than checking only whether wheel speeds eventually match.
- A common reference scale decreases when wheel tracking debt exceeds 8 mm and reaches zero at 20 mm. It never resets the goal to conceal error. Scores remain relative to the original unscaled command, so slowing down is not counted as accurate task completion.
- A bounded integral term retains steady load compensation through zero velocity. The quiet condition no longer resets all PWM. Near a held wheel error below 55 counts and speed below 100 counts/s, integration pauses while the existing bias and PD feedback remain active. That trades exact encoder convergence for less static-friction hunting.
- PWM is capped at ±255 during travel and ±180 during settled hold; the integral is capped at ±180. Normal slew is 1200 PWM/s. The hard holding ceiling and a same-sign decrease when ending a breakaway pulse may reduce output faster. Neither exception permits a faster polarity reversal or an increased power ceiling.
- Low-speed recovery uses the existing measured 160/140/140 breakaway magnitudes, up to 250 ms, with a 500 ms cooldown. It preserves learned load bias and ends promptly on motion. A sub-limit stationary hold attempt can return to PI compensation instead of being falsely classified as invalid input. A held error over 100 wheel counts at the holding ceiling with no motion for 350 ms faults as `SERVO_HOLD_LIMIT`. Active demanded travel with sustained output and no feedback still faults, even if the common reference governor has reduced its speed to zero.
- The existing explicit stop, 300 ms lease, raw-command rejection, invalid encoder/odometry guards and fault snapshots remain in force. STOP clears compensation and a later arm cannot inherit torque. Power-off remains the fault response, so this controller is not a fall-arrest system.

There is no current or motor-temperature feedback. A PWM ceiling is not a measured continuous torque/current rating. Sustained compensation needs supported hardware and thermal validation before release.

## Research basis

Position/velocity feedback with retained integral action can maintain a nonzero control input at zero position error. It requires bounded integration and appropriate damping; it is not a general proof for this nonlinear, slipping robot. The implementation was informed by [Modern Robotics position PI and feedforward](https://modernrobotics.northwestern.edu/nu-gm-book-resource/11-3-motion-control-with-velocity-inputs-part-2-of-3/).

The shared acceleration profile separates the requested command from a feasible changing reference, following the rationale in [WPILib motion profiling under gravity](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-elevator.html). The controller does not copy an elevator's gravity constant onto the robot; no robot gravity torque has been measured.

Conditional integration and tracking of output saturation/slew follow the [MathWorks anti-windup examples](https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html). The linear wheel-plus-position PD gains are derived from the previously identified K/tau with nominal natural frequency 9 rad/s and damping 1.05. Static friction, contact and slip violate that linear approximation and are evaluated separately.

## Evidence and limits

`sim/physics/encoder_bridge.cpp` passes only commands, signed count increments, filtered encoder speed and prior PWM register readback to the C++ controller. No truth pose, friction, force, supply level or injected delay reaches it. The scorer independently uses truth. The wheel model includes quantization, unequal gain/time constants, per-wheel delay and external load; MuJoCo uses the existing free-body, driven hubs and passive rollers with gravity/contact. Only moving-range K/tau are measured. All masses, torque scale, grip, magnet strength and added delays/loads remain assumptions.

`encoder-servo-holdout-01.json` preserves the first independent evaluation and its failures. Its results informed the final stationary-load/no-motion distinction. A second manifest, cases and acceptance were then written **before** evaluating the frozen final code on different seeds. Final results are in `reports/encoder-servo-holdout-02.json`; the source hashes bind it to this implementation.

| Final independent group | Previous hold | Candidate | Interpretation |
|---|---:|---:|---|
| 24 wheel-model load/delay cases | 5/24 within all bounds | 16/24 | Eight candidate trials exceed the 5 mm lateral bound; unknown-load transients remain a problem |
| 24 MuJoCo floor cases | 2/24 within all bounds | 24/24 | Limited simulation envelope passed; not physical validation |
| 12 MuJoCo wall cases | 0/12 within all bounds | 0/12 | Four candidate trials leave the test envelope; the remaining trials still fail position accuracy |

The aggregate motion bounds are final task error ≤10 mm, peak sideways deviation ≤5 mm, final heading error ≤5°, final-second position range ≤3 mm, no control fault and no departure from the test envelope. A run can have acceptable translation and still fail heading/settling; the baseline pass counts are not evidence that every baseline straight path is unusable.

Across the **24 floor trials**, mean peak sideways deviation changes from **1.41 to 0.51 mm**, and mean final task-position error from **3.52 to 1.11 mm**. These statistics are scoped to that group, not pooled with failed wall trials or presented as robot measurements. The deterministic strong-stiction fixture is slightly worse in final error (3.79 to 4.11 mm), and retains that result.

In the nominal simulated wall hold, the candidate retains support duty and avoids the previous early shutdown, but after ten seconds its encoder estimate is about **1.3 mm** from zero while the body has actually slid about **14.5 mm**. Friction loss or inadequate attraction still causes departure from the test envelope. This reproduces the observability limitation of [wheel odometry](https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-4-odometry/): encoder-only feedback cannot distinguish all body slip from correct wheel travel.

The wider-load lateral transients and wall failures mean this is still a development candidate, not the requested universally stable controller. Further software work should address load-step heading/transient behavior. Real wall stability additionally needs measured traction/torque and a way to observe chassis slip or mechanically prevent it; tuning wheel gains alone cannot supply that missing measurement.

## Reproduce without hardware

```sh
.venv/bin/python c6_drive/sim/physics/encoder_campaign.py --plant wheels --report /private/tmp/wbb-new-wheels.json
.venv/bin/python c6_drive/sim/physics/encoder_campaign.py --plant mujoco --report /private/tmp/wbb-new-physics.json
.venv/bin/python c6_drive/sim/physics/encoder_campaign.py --holdout --seed-offset 200 --report /private/tmp/wbb-new-holdout.json
c++ -std=c++17 -fsanitize=address,undefined -I c6_drive/include c6_drive/tests/test_encoder_servo.cpp -o /private/tmp/wbb-servo-test
/private/tmp/wbb-servo-test
.venv/bin/pio run -d c6_drive -e c6_drive
```

Reports/manifests refuse overwrites. Development failures in `encoder-servo-*-dev*.json` are retained. Firmware build and five native control/fault suites pass, as does the existing UI test. Native tests include nonzero bias at rest, reset/lease behavior, a jammed commanded wheel, invalid feedback, finite output bounds and held overload; these are not physical stability proofs.

English portfolio media: [floor comparison](portfolio/encoder-servo-floor.png), [wall observability limit](portfolio/encoder-servo-wall.png), [recorded simulation replay](portfolio/encoder-servo-comparison.mp4). `scripts/c6_encoder_servo_report.py` checks source hashes before regenerating them. The video is an animated plot of recorded simulation, not physical footage.
