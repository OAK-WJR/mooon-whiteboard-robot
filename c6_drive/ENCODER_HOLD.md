# Encoder-relative position hold

**Field status:** the user subsequently reported poor straight-line travel and slipping on the wall, with a generic robust fault. Loaded behavior is not qualified; the suspended results are retained as limited evidence, not a successful wall controller. The [incident record](reports/wall-control-incident.md) separates source-confirmed limitations from the unknown physical cause. The diagnostic changes described there are local and not yet deployed.

The current hardware has three motor encoders and no independent chassis pose sensor. `encoder_hold` adds relative odometry, heading correction during driving, and a fixed odometry target after key release. It reuses the measured wheel controller and its bounded load observer. It does **not** implement the independent-pose/slip controller from `ROBUST_CONTROL.md`.

**Released hold limits: 0.08 m/s translation and 0.6 rad/s rotation.** The 0.18 m/s prototype failed physical hold settling; its report is retained. Revision 5 enforces the narrower range in firmware and shows the effective speed in the webpage. Ordinary wheel-speed mode retains its separate adjustable speed ceiling. Hold correction can take several seconds and has not been qualified on a surface or under gravity load.

Revision 6 also permits bounded stiction recovery during an intentional low-speed driving command. A wheel that starts and then stops is not treated as permanently past the startup phase. Recovery uses the same duty/time limits; a missing response still faults and never automatically re-arms a stopped controller.

**Final deployed evidence:** [six directions](reports/encoder-hold-r6-six-directions.json) plus [three clockwise repetitions with ten-second hold observation](reports/encoder-hold-r6-cw-repeat.json) passed. Final-window encoder position error was at most 2.86 mm in those nine fixtures. Translation took 1.07–3.65 seconds after release to reach sustained quiet output in the six-direction set. [Three isolated wheel regression checks](reports/encoder-hold-r6-wheel-regression.json) also passed. The [saved-mode readback](reports/encoder-hold-ready.json) and [reboot check](reports/encoder-hold-reboot.json) confirm the firmware reloads `encoder_hold`, validated but **disarmed**, with zero duties and LOW output pads. This is limited suspended validation, not loaded wall qualification.

## Behavior

- WASD commands body-frame translation; Q/E commands rotation. Wheel order and axis signs match the existing drive UI.
- The controller integrates raw signed count increments with the SE(2) exponential, using the existing 38 mm diameter, 2800 counts/revolution and 33.738 mm wheel-center radius assumptions. These dimensions have not been newly calibrated on a surface.
- While moving, it corrects encoder-derived heading and bounds accumulated translation tracking debt to 25 mm. Release captures the current estimated translation position; releasing rotation captures the current estimated heading.
- Hold uses bounded position/velocity feedback and the same wheel model/PI/observer. Low-speed static friction requires separate, measured breakaway pulses: at most 160 PWM on M1, 140 PWM on M2/M3, for at most 250 ms, with a 500 ms cooldown. Motion detection terminates a hold pulse; a pulse that expires without motion faults instead of repeating indefinitely.
- A quiet band prevents repeated corrections around an unattainable exact count: below 3 mm / 0.043 rad and low estimated velocity, outputs go to zero. Corrections resume beyond 4 mm / 0.052 rad. The heading entry band exceeds the correction pulse's minimum resolvable yaw request, avoiding slow integral buildup in an unreachable gap. **This is not continuous static torque holding.** Gravity that rolls the wheels can trigger another correction; chassis sliding without corresponding wheel rotation cannot.
- Space, Esc, the stop button, loss of page focus, and command expiry disarm and clear every output. Hold requires fresh leased commands, including zero-velocity commands. Stale commands do not turn into autonomous indefinite hold.
- Missing/implausible odometry, excessive correction error (80 mm / 0.65 rad), persistent opposite motion, missing encoder response and output-pin faults revoke the lease. Shutdown does not physically support a robot on a wall.

The position panel explicitly says **encoder estimate**. `body_feedback` and `wall_hold` remain false. Rotating wheels on a suspended robot produces odometry even though the chassis is stationary: suspended tests establish controller/count behavior, not real-world position accuracy.

## Interface

Select a controller while stopped. Wi-Fi can select only a mode that has been separately validated over USB.

```json
{"op":"controller","mode":"encoder_hold"}
{"op":"arm"}
{"op":"drive","lease":123,"seq":1,"values":[0.08,0,0]}
{"op":"drive","lease":123,"seq":2,"values":[0,0,0]}
{"op":"stop"}
```

Use the lease returned by `arm`; advance sequence numbers and send commands at least every 300 ms. `values` is `[vx_mps, vy_mps, yaw_radps]`, translation norm at most 0.08 m/s and yaw at most 0.6 rad/s. Larger requests are rejected and revoke an active lease. These are suspended-test limits, not a qualified loaded operating range. `wheels`/`pwm` commands are rejected in hold mode. `zero_odometry` is accepted only when disarmed and wheels are stationary. Resetting position never arms the robot.

After physical review, USB-only `validate_robust` and `validate_hold` unlock their respective modes. `save_controller` saves the validated mode and flags with the exact firmware ELF digest. The same firmware can restore that selection after reboot, always disarmed. A different build invalidates the saved selection. Legacy gain storage is separate and unchanged.

## Evidence and retained development failures

- `tests/test_encoder_hold.cpp`: exact curved-path odometry and axis inversion, invalid inputs/stale odometry, lease faults, excessive error, commanded release, synthetic displacement correction, static-friction models at 0.85/1.00/1.15 effectiveness in six directions, and the explicit unobservable-slide counterexample. These are development model tests, not a prospective robustness proof.
- `tests/test_ui.cjs`: actual page logic, mode-specific transport, zero-command release, focus stop, arm/stop race, mode locks and compiled-page/source identity.
- `tests/test_hold_ui.mjs`: desktop/mobile rendering of the actual page with mocked transport; never contacts the robot.
- `reports/encoder-hold-smoke-01.json`: first physical implementation stopped about 8.5 mm short of its encoder target; moving-range identification did not capture breakaway behavior.
- `reports/encoder-hold-smoke-02.json`: ending a pulse at the first weak motion sample still left about 7.7 mm.
- `reports/encoder-hold-smoke-03.json`: revised forward release test passed (about 2.4 mm remaining in the encoder estimate).
- `reports/encoder-hold-six-directions-01.json`: forward/back/left passed, right failed the 3.5-second settling window. The failure and transient motion are retained. A longer recovery must be reported as slower recovery, not as passing the original window.
- `reports/encoder-hold-six-directions-02.json`: extending observation to six seconds revealed late lateral/heading correction activity; that test also failed. Revision 4 aligns the heading quiet band with static-friction correction resolution, and native tests now check sustained settling rather than just one endpoint.
- `reports/encoder-hold-r4-six-directions.json`: all six low-speed direction/release tests passed, with continuous quiet state and zero output in the final observation window.
- `reports/encoder-hold-r4-default-speed.json`: 0.18 m/s forward release failed; about 20 mm encoder error and continuing correction remained in the final window. That speed is deliberately unavailable in released hold mode.
- `reports/encoder-hold-r5-six-directions.json`: five directions passed, then M1 stopped during the clockwise driving segment and triggered `NO_ENCODER`. Its raw log shows initial encoder motion followed by rest as drive duty decreased. Revision 6 adds bounded in-motion stiction recovery; this evidence does not rule out mechanical or electrical intermittency.

Run a bounded hardware test only with the powered robot secured off the wall, wheels free:

```sh
.venv/bin/python scripts/c6_encoder_hold_bench.py --plan
.venv/bin/python scripts/c6_encoder_hold_bench.py --suspended --report c6_drive/reports/encoder-hold-new.json
```

The tool refuses report overwrites, blocks concurrent Wi-Fi control, retains raw telemetry, and verifies stopped outputs after each fixture. Its default six-second hold duration is recorded explicitly. Acceptance checks the final 0.7 seconds against <4 mm / <3 degrees of **encoder-derived** error and <150 counts/s on every wheel, continuously in the quiet state with zero PWM. It restores wheel mode, stopped; it never validates or saves a controller automatically.

## Physical limitation and references

Encoder odometry integrates wheel travel; skid, roller slip, wheel geometry error and chassis sliding accumulate unobservable position error. This follows the [Modern Robotics odometry derivation](https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-4-odometry/). A motor disturbance observer estimates wheel-model mismatch, not independent chassis movement. More PWM or an integral term cannot create missing adhesion or traction. Real wall-position stabilization needs independent position feedback and sufficient physical grip/support.

The motor observer and saturation tracking retain the rationale in [ROBUST_CONTROL.md](ROBUST_CONTROL.md), including the [MathWorks actuator-tracking anti-windup example](https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html). Local ESP-IDF `esp_app_desc.h` and Arduino `Preferences.h` were checked for firmware-digest-bound persistence; no new external library was added.
