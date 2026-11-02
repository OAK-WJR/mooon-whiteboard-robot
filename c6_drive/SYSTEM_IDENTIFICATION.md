# System identification and unknown-load compensation

Follow-up implementation: [bounded robust control C++ core and verification](ROBUST_CONTROL.md) is done and connects directly to MuJoCo tests. 60 independent samples within the preset range passed, and mean position error dropped by 52.8%; 30 wide-range stress tests are also kept, some of which stall and slide down, so it cannot be called reliable under all conditions. Full control needs independent body pose feedback; the current hardware has no such signal, and it has not been flashed yet. [English replay and materials](portfolio/robust.html).

2026-09-21. The subject is the Waveshare ESP32-C6-LCD-1.47 + adapter board + base board in use now; the sensors, power supply and motor specs of the future top_board_v2 must not be taken as the current hardware.

Recommended: **measured motor feedforward + wheel-speed feedback + bounded disturbance compensation + independent body motion feedback**. System identification builds the baseline model; online compensation handles changes. They cannot replace the grip between the tires and the board surface. The control architecture below is a research recommendation and has not been flashed to the robot.

The simulation implementation is done: see the [interactive console](sim/README.md), [MuJoCo contact dynamics](sim/physics/README.md) and [open-source references and per-condition responses](sim/physics/RESEARCH.md). There are 28 in-browser engine scenarios and 12 MuJoCo conditions × 2 controllers, and the results where compensation failed, the robot slid down, or feedback faulted are kept. English screenshots and videos are in `portfolio/`. ESO is not better in every condition; these results are not yet real-hardware loaded verification, and have not been flashed.

## What the current program and data show

- The current control period is 10ms, USB status is about one frame per 50ms, and wheel speed already goes through a first-order filter with coefficient 0.4. The historical single-point tests of about 0.9s at 140/170/200 PWM only give a short-term steady-state basis and cannot prove that an acceleration/deceleration model holds. The raw log audit is in `reports/sysid-historical-audit.json`.
- The earlier three-wheel suspended closed-loop tracking passed, but this does not verify whole-robot direction, load capacity, friction on the board or slope hold. On motor 2 at +9500 counts/s, 95 of about 200 PWM came from integral compensation, which shows the original feedforward is clearly too low at this operating point (`reports/speed-range-final.json`). The integral was already compensating part of the error; the existing PI must not be described as having no compensation at all.
- `Wheel::update()` clears the integral and output when the target is zero; when stopped, FI/BI are both LOW. It currently coasts to a stop, with no position hold. Sustained large output with no encoder motion triggers `NO_ENCODER`, and reversed feedback may also trigger direction protection. This logic cannot be used directly as slope-hold control on a vertical wall.
- The current program has no current, voltage, IMU or independent displacement data; `voltage` is explicitly null. The vendor resource table for the regular LCD-1.47 lists no IMU; do not confuse it with the **Touch** model, which has a QMI8658A. [Waveshare board documentation](https://docs.waveshare.com/ESP32-C6-LCD-1.47)

## Runnable identification tool

`plan`, `audit` and `analyze` in `scripts/c6_drive_sysid.py` only read files and do not open the serial port. `collect` spins the motors: it requires the robot to be off the whiteboard, the frame fixed, all three wheels suspended, and motor power connected. Do not use the driving page at the same time.

```sh
.venv/bin/python scripts/c6_drive_sysid.py plan
.venv/bin/python c6_drive/tests/test_sysid.py

# Run only after the physical conditions above are confirmed; the report path must not exist yet.
.venv/bin/python scripts/c6_drive_sysid.py collect --suspended \
  --report c6_drive/reports/sysid-new.json

.venv/bin/python scripts/c6_drive_sysid.py analyze \
  c6_drive/reports/sysid-new.json --report c6_drive/reports/sysid-model.json --plot
```

For each motor, forward and reverse each collect two different multi-step inputs: one set to estimate the model, one set for independent validation. PWM only varies between 120–200/255, starting from 140; one sequence is about 10 seconds, and standstill is verified between wheels and before reversing. Stale USB, encoder anomalies, unexpected motion of other wheels and control-ownership conflicts all abort; the firmware's existing protections keep working. The tool has no command to write or save PID parameters.

What is fitted is an effective model within the motion range: `tau * dv/dt + v = K * PWM(t-delay) + bias`. Parameters are estimated only from training data, then simulated forward from the first measured speed of the validation segment, without correcting the prediction with later measured speeds. The output includes PWM range, sample interval, encoder quality, independent prediction RMSE and R²; data that fails is rejected outright.

The tool's provisional engineering screening line is independent-validation R² ≥ 0.9, RMSE ≤ 10% of the validation speed span, and a time constant covering at least two frames. **This is not a proof of stability.** The fit includes sensor filtering and input time-alignment errors; data at about 50ms is not enough to reliably separate fast mechanical, electrical and communication dynamics. The motion friction intercept is not the static breakaway threshold, and it cannot be extrapolated to zero-speed slope hold. If the model does not pass, first add synchronized sampling or split it by operating range; do not plug it straight into formulas to compute PID. [WPILib diagnostics and low-inertia limits](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/viewing-diagnostics.html), [independent estimation and validation data](https://www.mathworks.com/help/ident/select-data-for-estimation.html)

## How to compensate each unknown condition

This round of measurements is done (`reports/sysid-suspended-03.json`): one training set and one independent validation set for each of the three wheels in forward and reverse, 12 segments in total; zero output and encoder standstill were both confirmed at stop, and the six sets of original PID parameters were unchanged. Motor 1 had normal feedback this round; the first real excitation before this was aborted because motor power was not connected, and the user then confirmed the power was connected, so that no-feedback record must not be taken as a motor fault.

| Motor | Forward/reverse effective time constant | Forward/reverse independent validation R² |
|---|---|---|
| 1 | 0.278 / 0.278 s | 0.940 / 0.945 |
| 2 | 0.328 / 0.328 s | 0.931 / 0.943 |
| 3 | 0.302 / 0.278 s | 0.928 / 0.930 |

The raw sample interval is about 51ms, and independent-validation RMSE is 234–335 counts/s. The six models pass the **preliminary motion-range screening** above, but the curves still show systematic bias; this does not mean the models are accurate or that controller design is done. A zero-delay optimum means this sampling could not separate extra delay; it must not be read as the hardware having no delay. The input this time was 120–200 PWM; low-speed start, gravity, loaded or board-surface slip identification has not been done. [Model parameters](reports/sysid-model-03.json), [independent prediction vs. measurement](reports/sysid-model-03.png)

| Change | Usable method | Current gap or limit |
|---|---|---|
| Eraser pressure or load changes lower wheel speed | Forward/reverse feedforward + PI; once the model passes, try a DOB/ESO disturbance observer | First identify and run controlled load experiments; it can only compensate if PWM has headroom |
| Battery voltage sag, wiring voltage drop | Measure VMOT, normalize feedforward by voltage; keep current limiting | No voltage/current measurement now; PWM cannot stand in for voltage or torque |
| Board surface changes cause tire slip | Compare encoders with independent body speed, lower acceleration/traction demand, freeze bad learning | Three encoders alone cannot uniquely identify overall slip |
| Vertical gravity; force direction changes after the body turns | Attitude/board coordinates + gravity feedforward + position hold; estimate the remaining load online | No body position/attitude closed loop now; releasing the key removes output |
| Weak magnetic attraction, wheels not pressed firmly, board not suited to magnets | Check contact, adhesion and load; pressing force can only be tuned with actuators and sensors | Fixed magnets cannot get stronger attraction by changing PWM |

**Suggested first step: keep the PI whose behavior is known, fix the feedforward, then try a limited ESO/DOB, instead of letting all PID parameters drift automatically all the time.** A disturbance observer estimates the combined effect outside the model from the input and the measured response, and does not require weighing each kind of load in advance. MathWorks has a DC motor example of PID plus ESO; the compensation is still affected by the model approximation, and it cannot promise to remove arbitrary loads. [DC motor ESO example](https://www.mathworks.com/help/slcontrol/ug/apply-extended-state-observer-for-disturbance-rejection.html), [disturbance compensator definition and limits](https://www.mathworks.com/help/slcontrol/ug/disturbancecompensator.html)

Specific implementation suggestions (derived for this project's design, not deployed):

1. Build a discrete speed model for each wheel from the identification results, with forward and reverse kept separate. The observer input must be the actual PWM after final limiting and slew-rate limiting, not a requested value that has not been applied yet.
2. Add a slowly changing disturbance state that estimates the extra resistance as an equivalent PWM; output `feedforward + feedback - estimated disturbance`. Design the observer together with the PI integral, so that two fast integrators do not compensate the same thing twice.
3. Choose the observer bandwidth from the measured delay, noise and the 100Hz control period; do not copy the example's gains directly. Limit the compensation size and rate of change. During sustained saturation, feedback anomalies or detected slip, stop learning gains, and clearly show "insufficient compensation/invalid feedback".
4. First compare the original PI with candidate controllers on a fixed-load fixture: steady-state error, recovery after a sudden load change, overshoot, duty-cycle headroom, current and temperature rise. Without these checks, suspended performance is not taken as on-board capability.

RLS (recursive least squares) suits slowly updating gains or friction parameters later, but constant-speed running may not give enough excitation, and slip data may also be learned as wrong motor parameters. So it only updates in windows where the data passes, is not saturated and shows no evidence of slip, and parameters get upper and lower bounds; large exploration is never run automatically on the wall. A forgetting factor cannot fix unobservability. [MathWorks recursiveLS](https://www.mathworks.com/help/ident/ref/recursivels-system-object.html), [persistent excitation assumption in RLS convergence](https://arxiv.org/abs/2003.07334)

## Why body feedback is needed, and what to use first

When "the wheels turn as commanded but the whole robot slides down", wheel-speed error can be close to zero. The wheel-speed controller cannot see this kind of body displacement error. Wheel speed cannot fully observe tire slip; existing mobile robot research discusses this limit explicitly. [CMU research, §5.2.1](https://www.ri.cmu.edu/pub_files/pub4/kelly_alonzo_2006_1/kelly_alonzo_2006_1.pdf)

We suggest first using a **fixed external camera + an AprilTag on the body** to get x, y and angle in the whiteboard plane as debugging ground truth, and then deciding whether to add onboard sensors. This needs camera calibration, tag size, board coordinates and timestamps, and checks of occlusion, delay and localization quality. This is this project's test plan; AprilTag provides tag detection and pose estimation, and does not close our robot's loop by itself. [Official AprilTag implementation](https://github.com/AprilRobotics/apriltag)

For a final version that does not rely on an external camera, an optical displacement sensor facing the board and an IMU can be evaluated. Compatibility with glossy whiteboards, dust, distance and lighting must be measured; a regular optical flow module is not guaranteed to be reliable on every whiteboard. An IMU can help with gravity direction and attitude, but cannot by itself measure constant-speed sliding accurately over long periods. Research on optical odometry with inertial navigation can serve as a basis, but its ground experiments are not certification for whiteboards. [CMU optical displacement research](https://publications.ri.cmu.edu/outdoor-downward-facing-optical-flow-odometry-with-commodity-sensors)

With independent pose, the outer loop can compare "expected upward motion" with "actual sliding down" and then generate the three wheel targets; the low-level observer handles each wheel's load error. First verify the physical wheel numbers, rotation directions and whole-robot coordinates, then use the current three-wheel kinematics matrix.

## Limits of slope hold and adhesion

On a vertical board, all of these must hold at once: the available upward traction covers the weight, each wheel's contact stays stable, and the motors/drivers/power supply have headroom for sustained output. `sum(mu_i * N_i) >= mg` is only a rough necessary condition that ignores how force is split by direction, and cannot be treated as a sufficient condition for three omni wheels; roller direction, normal force distribution, center of gravity and actual torque must also be considered.

If the robot should stay on the wall after the key is released, it should enter a separate HOLD state, save the position target and keep providing bounded load-bearing output, instead of reusing the current zero-speed branch that clears the integral. But high output at zero speed, disconnects, encoder faults, overheating and similar states must be designed together with mechanical fall protection; existing protections must not be removed for slope hold, and "stops when power is cut" must not be described as "will not slide down". Active short-circuit braking also cannot be treated as a definite static load-bearing capacity. HOLD is not enabled now.

The research order this time followed the project requirements: first a local `hyperresearch search` query, which returned an uninitialized knowledge base; then the built-in web access was used to read the vendor documents and original research above, without calling an external paid search API.
