# Wheel-only hardware test integration

The current source now includes the simulated C++ `Wheel` law through `include/robust_wheels.h`. It adds the common target filter/ramp and bounded output-demand reduction, but **does not add independent body sensing, pose control, slip detection or wall hold**. Its behavior is separately tested; the body-feedback MuJoCo qualification does not qualify this hardware adapter.

The integrated firmware **has been flashed and physically tested**, after the user confirmed the chassis was secured off the board with wheels free and motor power connected. Final revision `robust-wheels-4` passed 17 suspended speed fixtures plus immediate reversal, release-to-zero and command-timeout checks. Mean steady speed error was 1.77%, worst 3.19%; peak transient overshoot was 13.07%. [Complete results and retained failures](reports/robust-hardware-summary.md), [English measured-response plot](portfolio/hardware-response.png).

Same-speed T90 spread decreased from 80 to 20 ms. Repeated lateral spread was 160–500 ms with legacy control and 50–130 ms with the final candidate; reverse lateral spread was 30–50 ms. Timings include command shaping and encoder filtering. This is improvement, not perfect synchronization or proof of reduced chassis drift under load.

An intermediate revision had one M1 `NO_ENCODER` stop at a 140-PWM start. A bounded 160-PWM pulse then produced correct forward/reverse encoder counts. The final adapter uses up to 160 PWM for M1 low-speed breakaway, at most 250 ms, ending once filtered speed reaches 150 counts/s. M2/M3 use the previously measured 140 PWM. The first stronger M1 pulse handed over too late and overshot by ~31%; the earlier exit reduced final worst overshoot to ~13%. No additional stalls occurred in the final 17 fixtures. This does not rule out intermittent mechanical or electrical faults.

After reviewing the tests, the candidate was selected and validated for the **current session**, with the robot left stopped and disarmed. The [final USB status](reports/robust-ready-r4.json) confirms zero PWM, zero duty register readings and LOW output pads. The existing WASD page can request this wheel-only controller after the user's explicit enable action. Its wireless end-to-end path and loaded floor/wall motion were not part of these tests. Reboot returns to legacy mode; no gain settings or controller choice were saved.

## Interface and rollback

Boot defaults to the existing legacy controller and stored gains. All existing leases, 300 ms command timeout, feedback-quality checks, encoder-direction/stall checks, output pad readback and physical stop button remain. Candidate selection is USB-only and disarmed-only:

```json
{"op":"stop"}
{"op":"controller","mode":"robust_wheels"}
{"op":"trace","enabled":true}
```

Ordinary `arm` / `wheels` leases still apply. `controller=legacy` restores the old wheel controller without reflashing; candidate selection is not saved in NVS. While candidate mode is unvalidated, Wi-Fi arming remains locked. The USB `validate_robust` operation exists for an explicitly reviewed, successful bench campaign; the comparison tool never calls it or saves gains. Reboot clears its validation flag and returns to legacy mode.

Status identifies `controller`, `controller_revision=robust-wheels-4`, `robust_validated`, `body_feedback=false`, `wall_hold=false`, actual ramp references and disturbance compensation. Candidate mode appears as `RW` on the LCD. LEDs/PWM use the existing six-channel driver; the model reads the actual previous signed LEDC duty, not only its previous request. LEDC duty is register readback, not current or voltage sensing.

Optional trace rows contain `[control_ms, accepted_command_ms, speed1, speed2, speed3, reference1, reference2, reference3, applied_pwm1, applied_pwm2, applied_pwm3]`. Eight buffered samples are included in the existing ~20 Hz USB status. Deduplication recovers the 100 Hz control samples and rejects gaps over 40 ms. Response timing is relative to the first accepted command, not host serial arrival. It includes the controller ramp and encoder filtering and is not an estimate of pure motor dead time.

## Prepared tests

```sh
# Does not connect or move:
.venv/bin/python scripts/c6_robust_bench.py --plan
.venv/bin/python c6_drive/tests/test_robust_bench.py

# Only with chassis secured off the board, wheels free, motor power connected:
.venv/bin/python scripts/c6_robust_bench.py --suspended --smoke \
  --controller robust_wheels --report c6_drive/reports/robust-smoke-01.json
.venv/bin/python scripts/c6_robust_bench.py --suspended \
  --report c6_drive/reports/robust-hardware-bench-01.json
```

The smoke plan requests one wheel at a time, 1800 counts/s for 1.2 seconds. The comparison uses each wheel at ±3000 counts/s and three coordinated vectors for 2.2 seconds each. Both controllers run on the same firmware and trace path. It measures 10/50/90% response times, 10% settling, steady speed error, variation, quadrature efficiency and cross-wheel response spread. Those are suspended-wheel measurements, not chassis motion.

Every segment verifies stopped outputs before proceeding and stops in a `finally` path. Firmware faults, uncommanded wheel motion, stale telemetry or trace gaps abort the campaign. The tool restores legacy mode, disables tracing and confirms zero outputs at the end. An existing report is never overwritten. No automatic parameter fitting or promotion occurs.

Native address/undefined-behavior-sanitized tests cover the adapter, vector proportions, applied-output tracking, lease expiry, invalid input, single-wheel isolation, six analytic direction models and immediate release-to-zero. Existing legacy controller tests, 3900-step simulation parity and the WASD interface checks also remain applicable.
