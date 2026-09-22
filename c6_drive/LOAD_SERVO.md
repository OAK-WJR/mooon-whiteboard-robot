# Encoder load servo — implemented, not qualified for replacement

`include/encoder_load_servo.h` contains the portable C++ successor, `drive::EncoderLoadServo`. It was developed in response to the user's report that the wheels are hard to start and turn slowly under load. Robot mass is unknown. It has no dependency on a supplied mass, friction coefficient, injected load or chassis truth.

The original `encoder_servo.h` and deployed firmware remain unchanged. The candidate passes native safety tests and an **ESP32-C6 compile-only check**, but independent heavy-load results do not justify replacing the previous controller. No robot was connected, reset, flashed or commanded during this development. The last user constraint is software/simulation only while the robot remains on the wall.

## Implemented changes

1. **Per-wheel load estimation.** A discrete two-state observer uses previous applied PWM and measured encoder speed with the identified moving motor gain/time constant. With `a = exp(-dt/tau)` and `b = K(1-a)`, speed prediction is `a*v_hat + b*(effective_pwm - load_hat)`. Innovation corrects both speed and equivalent lost input. This estimates a disturbance in PWM units, not physical torque or current.
2. **Bounded retained compensation.** Estimated loss is limited to ±140 PWM and its compensation changes by at most 400 PWM/s. Observer increments enter the same bounded retained state as the residual position integral. The total state stays within ±180; there is no unbounded integral plus an additional independent bias. Existing output tracking/anti-windup remains active. Residual integral gain is reduced from `0.8*kp` to `0.35*kp` because the observer supplies the faster correction.
3. **Bounded rising startup.** During requested travel, the existing recovery pulse rises from 160/140/140 toward the existing maximum within its original 250 ms deadline. Motion ends recovery; stationary wheels are not kicked solely because other wheels are commanded. The 500 ms cooldown, no-motion latches, PWM slew and holding ceiling remain.
4. **Slower estimation during settling.** Observer bandwidth is 8 rad/s during travel and 3 rad/s during hold. Learning still requires actual wheel motion above 80 counts/s, applied duty below 250 and no active recovery pulse. A stopped wheel is not treated as evidence that the load should keep increasing. The established velocity-loop damping is preserved: extra derivative gain aggravated some cases with unequal delay.
5. **Existing wrapper semantics.** Tests use the actual `drive::Control` command leases, raw-command restrictions, signed-count odometry and fault snapshots. STOP/reset clears load estimates as well as outputs and integrals. Holding retains bounded bias; missing encoder motion under demanded travel still faults. Encoder-only feedback cannot observe all chassis sliding.

The observer follows the disturbance-estimation principle illustrated in [MathWorks' DC motor observer example](https://www.mathworks.com/help/slcontrol/ug/apply-extended-state-observer-for-disturbance-rejection.html). Its input/output model is approximate: [the compensator documentation](https://www.mathworks.com/help/slcontrol/ug/disturbancecompensator.html) explicitly ties rejection quality to model accuracy. These references do not certify the selected robot-specific gains, static-friction behavior or thermal duty.

## Development and independent validation

Development versions and failures are retained in `load-servo-dev01` through `dev06` reports. Earlier source snapshots are preserved alongside the development reports where versions changed. Early startup, frozen hold estimation and stronger derivative damping were explored and their regressions retained. The final implementation does not include those rejected changes.

The final parameters were frozen after the development/regression rounds. `load-servo-holdout02.manifest.json` declares a new seed set, controller hashes, cases and bounds **before** evaluation. Its 36 heavy-load cases vary assumed mass 0.5–2.3 kg, direction, speed ±0.04/±0.08 m/s, grip, per-wheel delays up to 70 ms and assumed motor authority 2.5–5 N/wheel. Other groups use new wheel-load, light-floor and wall seeds. Results are scored against body truth, which the controller never receives.

| Test set | Previous servo | Load servo |
|---|---:|---:|
| Development load cases | 29 / 54 | 36 / 54 |
| **Independent heavy-load floor** | **18 / 36** | **17 / 36** |
| Independent wheel load/delay | 18 / 24 | 18 / 24 |
| Independent light floor, 0.25–0.45 kg | 24 / 24 | 24 / 24 |
| Independent wall hold | 0 / 12 | 0 / 12 |

The motion bounds are unchanged: original-request final error ≤10 mm, peak sideways deviation ≤5 mm, final heading error ≤5°, final-second displacement range ≤3 mm, no fault, envelope departure or solver warning. Fault-injection fixtures are evaluated for their intended stop behavior separately from motion passes.

The development set improves with no previously passing development motion case lost in the final version. That improvement does **not** generalize enough: two previously passing independent heavy cases, `load_unseen_18108` and `load_unseen_18113`, regress, while one becomes a pass. Heavy-load mean final error falls from about 38.3 to 32.1 mm, but aggregate pass count still falls. This is why the candidate is not promoted. No failed case was excluded to obtain a better score.

The independent light-floor group remains within bounds; wall hold remains unqualified. A fixed startup timeout disappearing is not proof that the robot has adequate sustained torque or that it completes the requested path. Force/torque/contact parameters remain assumptions; motor current, terminal voltage, actual pad force and wheel-loaded breakaway have not been measured.

The [case-by-case failure analysis](reports/load-servo-failure-analysis.json) preserves all 36 scores: among the 19 failures, 9 have over 10 mm odometry/body disagreement, 10 have over 10 mm governed-reference shortfall and 3 exceed the 3 mm settling bound (categories overlap). The [English figure](portfolio/load-servo-failures.png), embedded in the existing simulator/gallery, shows examples of all three. This evidence supports distinct development work on limited-load tracking and settling; it does not make arbitrary encoder-only sliding observable. No acceptance threshold or recorded result was changed.

## What is verified

- The existing control, robust-wheel, encoder-hold, fault and servo native suites pass with the successor substituted into the actual wrapper, under address/undefined-behavior sanitizers.
- `test_encoder_load_servo.cpp` independently checks recovery of an unknown 35-PWM equivalent load from known input/output data, total compensation bounds, reset, stationary retention and invalid-feedback latching.
- `load_embedded_compile.cpp` compiles with the installed ESP32-C6 RISC-V compiler. This is an object-level portability/memory check, **not** a complete new flash image or physical validation.
- The final development and independent reports bind source hashes and keep all raw traces. The existing simulator/gallery can switch between previous and successor results for the same 54 development cases; independent results are shown prominently above them.

## Next boundary: physical loaded data

Further model tuning cannot determine whether the actual slow start comes from insufficient breakaway compensation, motor-terminal voltage loss, limited driver/battery current or excess mechanical drag. The next useful step is a bounded loaded response capture using the existing firmware, with the robot taken off the wall and permitted to move briefly on a clear level surface. The successor is not ready for a blind flash-and-drive test. The user's software-only restriction must change before any physical operation.

Total mass would improve force-budget estimates, but the capture can still distinguish wheel response from command/output without it. A loaded capture should preserve per-wheel command, applied PWM, encoder response, onset timing and faults; voltage/current causes require an actual electrical measurement. No software-only observer can infer all those quantities uniquely from the current encoders.

`../scripts/c6_loaded_capture.py` now prepares this capture using the existing `robust_wheels` firmware mode. Its default/`--plan` path never imports serial. Physical execution requires a current user-confirmed off-wall, powered setup on a clear level floor, plus explicit execution flags. It requests one direction at 0.04 m/s for 0.6 s, preserves 100 Hz board-clock samples, checks fresh feedback and encoder travel/heading limits, and verifies stopped outputs before analyzing the data. Encoder limits do not measure actual chassis travel during slip. It neither flashes nor changes gains or stored validation flags.

The capture now also accepts the [authenticated local Wi-Fi transport](WIFI_NETWORK.md), so USB need not remain tethered during an authorized floor capture. Firmware/network deployment and physical validation remain outstanding.

Seven offline tests cover the plan, guards, sustained-onset analysis, a complete mock capture, fault/stale feedback without rearming, failed stop verification and invalid pre-arm odometry. These are transport simulations, not physical test results. No capture has been executed against the robot; permission to leave software-only work is still pending.

```sh
.venv/bin/python scripts/c6_loaded_capture.py --plan
.venv/bin/python c6_drive/tests/test_loaded_capture.py
```

## Reproduction

```sh
.venv/bin/python c6_drive/sim/physics/load_servo_campaign.py --report /private/tmp/wbb-load-dev.json
.venv/bin/python c6_drive/sim/physics/load_servo_campaign.py --holdout --seed-offset 1000 --report /private/tmp/wbb-load-validation.json
c++ -std=c++17 -fsanitize=address,undefined -I c6_drive/include c6_drive/tests/test_encoder_load_servo.cpp -o /private/tmp/wbb-load-test
/private/tmp/wbb-load-test
MPLCONFIGDIR=/private/tmp/wbb-mpl-servo .venv/bin/python scripts/c6_load_servo_report.py
```

The seed-offset 1000 set is now seen; rerunning it is regression, not new independent evidence. `--regression` labels reused randomized sets accordingly. Native builds contain no serial/network control transport. The production/default header is not modified by the native wrapper alias used for these experiments.
