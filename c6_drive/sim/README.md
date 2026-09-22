# WhiteBoardBot control simulation

This local simulator never opens a serial port or sends robot commands. The full body-feedback controller remains simulation-only. A separate [wheel-only hardware adapter](../ROBUST_HARDWARE.md) has since been flashed and tested with suspended wheels; its results do not qualify the simulated position/slip/hold functions.

New: the [portable C++ robust controller](../ROBUST_CONTROL.md) runs directly in the independent MuJoCo plant. Its [English results and recorded replay](../portfolio/robust.html) include 60 paired holdouts, 30 wider stress pairs and 16 named conditions. The browser lab below still compares the earlier JavaScript controllers; it does not execute the C++ core.

```sh
.venv/bin/python scripts/c6_drive_sim.py
# Interactive lab: http://127.0.0.1:8088/sim/
# English media:   http://127.0.0.1:8088/portfolio/
node c6_drive/sim/scenarios.mjs
node c6_drive/tests/test_sim.mjs
```

Choose the manual scene, press Start, then hold WASD to move in board coordinates; Q/E rotates. Esc removes simulated motor output. Changing settings resets the trial. Losing focus clears keys and pauses. Parameters include mass, friction, normal force, available wheel force, supply scale and camera feedback. The lateral scene defaults to **no body-position feedback**, so wheel-only drift is visible. Toggle the shared acceleration ramp to inspect one specific source of command distortion.

## Two plants, separate evidence

1. `core.mjs` provides a deterministic first-order wheel model and a simplified planar contact surrogate. It runs interactively in the browser and in batch tests. The first three portfolio clips use this plant.
2. [`physics/`](physics/README.md) uses MuJoCo to integrate a free chassis, driven wheel hubs, passive rollers and actual contact constraints. It reuses `ModelWheel` through a local Node subprocess, rather than reimplementing a more favorable controller. Its 12-condition matrix and recorded 3D playback are separate from the browser surrogate.

Neither model is a calibrated digital twin. The source motor logs cover 120–200 PWM while suspended, at approximately 51 ms sample intervals; effective dynamics include filtering. Zero-speed friction, physical inertia, force/PWM conversion, actual mass, adhesion and body sensing remain unmeasured. The inherited 38 mm wheel and 2800 counts/rev assumptions are not newly verified.

## Controller comparison

- **Original wheel PI:** a port of the real `Wheel::update()` in `include/control.h`, including startup boost, ramps, integral limits and zero-target reset. A native C++ comparison covers 3,900 steps. The full firmware lease/fault protocol is not replicated by that wheel-law parity check.
- **Model feedforward + PI:** each motor/direction uses its identified gain and effective time constant. Both candidate variants use the same PI gains: `Kp = tau / (K * 0.30)`, `Ki = 1 / (K * 0.30)`. This is a simulation tuning choice, not a stability guarantee for the real plant.
- **Model PI + ESO:** an augmented discrete observer estimates a slowly varying PWM-equivalent load from measured speed and the last applied integer PWM. It does not receive injected load or true contact forces. The load estimate is limited to ±90 PWM and compensation changes by at most 180 PWM/s. Candidate output is bounded to ±255, with a 1200 PWM/s slew limit and integral anti-windup.

A shared vector ramp applies one interpolation factor to all three target changes. Independent per-wheel ramps can change their ratios during acceleration and introduce an unintended rotation command. The shared ramp fixes that command-level issue; it cannot guarantee equal physical wheel response or eliminate body drift.

For wall trials, candidate position hold uses an explicitly **simulated** 20 Hz pose sensor with 80 ms delay. This extra sensor is unavailable on the current robot. Comparing those variants against the original wheel-only controller therefore measures an architectural change as well as a wheel controller change.

## Results and limits

The six browser wheel-load cases average 295.4 counts/s mean absolute error for model PI and 111.5 counts/s with ESO over 3–5 s, about 62% lower **in that simulated load test**. The independent MuJoCo test does not establish general superiority: ESO is slightly worse in the unknown external-force case (18.9 → 20.8 mm mean task-position error) while improving some supply-loss and wall cases.

Both plants retain unsuccessful trials. Inadequate friction/attraction causes sliding; observer compensation cannot supply missing traction. Sensor-loss output shutdown also does not arrest a robot on a wall. A test-envelope end freezes playback after a fall; it does not model a physical catch. See [physics results](../reports/mujoco-results.json) and [research and condition responses](physics/RESEARCH.md).

## Reproduce and capture

Browser checks use an isolated empty Chrome profile on debugging port 9238; they never attach to a personal profile. Launch Chrome with `--headless=new --remote-debugging-port=9238 --user-data-dir=/private/tmp/wbb-control-sim-chrome about:blank`, then:

```sh
node c6_drive/tests/test_sim_browser.mjs
node c6_drive/sim/capture_portfolio.mjs
.venv/bin/python -m pip install -r c6_drive/sim/physics/requirements.txt
.venv/bin/python c6_drive/sim/physics/run_suite.py
.venv/bin/python c6_drive/tests/test_physics.py
.venv/bin/python c6_drive/sim/physics/render.py
.venv/bin/python c6_drive/portfolio/package.py
node c6_drive/tests/test_portfolio.mjs
```

The macOS MuJoCo renderer needs access to a graphics session. All media are English, 1920×1080, 20 fps, H.264 and silent. Captures are deterministic simulation playback, not videos of the physical robot.
