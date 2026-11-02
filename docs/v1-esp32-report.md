# WhiteBoardBot project overview report

A summary of the architecture, hardware, software, tuning and operations notes. **No ROS** — the ESP32 controls itself.

---

## 1. Project goal

**WhiteBoardBot** is a whiteboard/desktop robot based on the **ESP32**, with three omni wheels (120° layout).
All control runs on the ESP32; the host only sends targets and position corrections:

| Layer | Runtime environment | Role |
|------|----------|------|
| Host | Mac/PC (Python, WiFi UDP) | Send target area / position corrections, draw the trajectory, PID tuning |
| Robot | ESP32 (Arduino + FreeRTOS) | Path planning, lookahead path following, inverse kinematics, FF+PID, encoder odometry |

---

## 2. System architecture

```
┌─────────────────────────┐   WiFi UDP (JSON)   ┌──────────────────────────────┐
│ Host whiteboard_app.py  │  ──── area/pos ───► │ ESP32 (env:esp32)            │
│  target area /          │  ◄──── status ───── │  PathPlanner → Follower      │
│  position correction    │   :8888 / :8889     │  → Kinematics → FF/PID       │
│  live trajectory        │                     │  → RZ7899 ×3 → 3×N20         │
└─────────────────────────┘                     │  encoders → Odometry         │
                                                └──────────────────────────────┘
```

**UDP commands (JSON, port `UDP_LISTEN_PORT=8888`)**

| `cmd` | Fields | Description |
|-------|------|------|
| `area` | `x y w h spacing` | Plan a zigzag coverage path |
| `start` / `stop` | — | Start / stop execution |
| `pos` | `x y theta` | Correct odometry with a vision position fix |
| `set_speed` | `v` | Set the max linear speed |

**Status report (JSON, port `UDP_REPORT_PORT=8889`, 10Hz)**: `state, x, y, theta, progress, wp, wp_total`.

The control loop maps one-to-one to the host commands, and the serial console reuses the same `RobotController::handleCommand` logic.

---

## 3. Firmware (PlatformIO)

### 3.1 Build environments (`platformio.ini`)

| Environment | Use |
|------|------|
| `esp32` (default) | Main firmware: autonomous control + WiFi UDP + live tuning over serial |
| `motor_test` | Single/three-motor test bench; open loop / PID / kinematics, interactive over serial, works with `pid_tuner.py` |
| `native` | Desktop `pio test -e native`, kinematics / PID / odometry simulation |

### 3.2 Main firmware (`src/main.cpp`)

- Two FreeRTOS tasks: `controlTask` (Core1, 50Hz control loop) + `commTask` (Core0, WiFi send/receive).
- `loop()` runs the serial tuning console: change PID / speed / lookahead / tolerance at runtime without reflashing (see the comment at the top of the file or the README for the command table).
- Control loop: odometry update → path following gives body velocity → inverse kinematics → wheel-speed **FF+PID**; feedback uses the direct wheel speed from `Odometry::getWheelVelocity()`.

### 3.3 Controller and tracking

- `RobotController`: state machine (idle/planning/ready/running/done) + command dispatch + control loop + serial tuning interface (`tunePid` / `setLookAhead` / `setGoalTolerance` / `setLinearGain` / `printParams` / `printTelemetry`).
- `TrajectoryFollower`: lookahead path tracking; the three parameters `look_ahead_` / `goal_tol_` / `kp_linear_` can be tuned at runtime (defaults come from `config.h`).
- `PathPlanner`: zigzag coverage of a rectangular area.

### 3.4 Odometry (`src/odometry.cpp`)

- Encoder differences → wheel speed, with an EMA low-pass to reduce PID jitter; wheel speed is zeroed on `reset()`.

### 3.5 PID (`src/pid_controller.*`)

- FF + PID, derivative-on-measurement, anti-windup (integral clamping), PWM slew rate limit.
- A full set of runtime getters/setters (`setKp/Ki/Kd/Kf` etc.) for serial tuning and `pid_tuner.py`.

### 3.6 Motor driver (`src/motor_driver.cpp`)

- RZ7899-MS with two pins: forward FI=PWM/BI=LOW, reverse the opposite, brake both LOW.

### 3.7 Motor test firmware (`src/motor_test_main.cpp`)

- Serial commands: `0/1/2` select motor, `+/-`, `f/r`, `s/a`, `p`, `z`, `c` (open loop/PID), `k`+`w/x/q/e/j/l` (kinematics).
- Uppercase multi-character commands `P/I/D/F/L/G/R/S/V/A` are for PID tuning; in PID closed loop it outputs `$tgt,act,pwm,FF,P,I,D` frames for `pid_tuner.py` to plot.

---

## 4. Hardware config summary (`include/config.h`)

### 4.1 Mechanics and encoders

- Omni wheel diameter 38 mm; wheel center about 33.738 mm from the chassis center.
- Encoder: `ENCODER_CPR = 700` (7 PPR × 1:100 reduction).

### 4.2 Pin assignment (must match the hardware)

**RZ7899-MS drivers (one H-bridge each, 2 pins FI+BI)**

- M0: FI **32**, BI **33**
- M1: FI **25**, BI **26**
- M2: FI **27**, BI **14**

**Encoders (A/B phases)**

- M0: **34 / 35**
- M1: **36 / 39**
- M2: **19 / 18**

Encoder direction correction `ENCODER_DIR_M*`: M2 is inverted (`-1`).

**Note (troubleshooting log)**

- **GPIO 12** is the Flash voltage select pin; if a jumper wire near **GPIO 14** accidentally touches it and pulls GPIO 12 high, the Flash runs in 1.8 V mode and `esptool` reports "Failed to communicate with the flash chip". Route wires away from it or add a pull-down on GPIO 12.
- Some encoder pins (GPIO34/35/36/39) are input-only with no internal pull-up; per ESP32 errata 3.11, GPIO36/39 noise can be suppressed with `adc_power_acquire()`.

### 4.3 RZ7899 / power notes

- VM (motor power) is separate from the logic supply; **common ground** with the ESP32 and the battery negative.
- A damaged driver may have a VCC–GND short, and sharing ground with the ESP32 will burn the regulator — check the board with a multimeter in continuity mode before power-up.

### 4.4 WiFi / UDP

- `WIFI_SSID` and `WIFI_PASS` are in `config.h` (ignored by `.gitignore`).
- `UDP_LISTEN_PORT=8888` (receives commands), `UDP_REPORT_PORT=8889` (sends status). The ESP32 automatically replies to the source address that sent the command.

---

## 5. Toolchain and environment (this machine)

- **Python venv**: create it with Homebrew `python3.10`, then `pip install -r requirements.txt` (includes platformio + host GUI dependencies). An empty `pio` script / broken pip once made commands print nothing, and the venv had to be rebuilt.
- **ESP32 upload**: on an Intel Mac, a binutils path fix was once needed.
- **Serial port**: `pio device monitor` may fail in some environments; use `pyserial` or a system serial tool instead.

---

## 6. Recorded issues and conclusions (troubleshooting summary)

| Symptom | Likely cause | What to do |
|------|----------|----------|
| Upload reports Flash communication failure, Manufacturer `ff` | GPIO12 pulled high, wiring interference | Check GPIO12/14; test the driver board before sharing ground |
| ESP32 gets hot as soon as GND is connected | Driver VCC–GND short | Power off, test VCC/GND in continuity mode, replace the module |
| PWM has output, encoder always 0 | VM/GND not connected, motor not turning, or encoder not powered | Check driver power and encoder wires |
| Jittery motion | PID overshoot, wheel-speed noise | Lower KP/KI, wheel-speed EMA, PWM slew; use serial `t` to view telemetry and tune live |

---

## 7. Common commands

```bash
# Main firmware (autonomous control + serial tuning)
pio run -e esp32 -t upload && pio device monitor

# Motor test bench (no wireless; tune PID over serial)
pio run -e motor_test -t upload

# Native tests
pio test -e native

# Host
python scripts/whiteboard_app.py   # control panel
python scripts/pid_tuner.py        # single-motor PID tuning
```

---

## 8. Directory structure (core)

```
WhiteBoardBot/
├── include/config.example.h  # hardware + PID + WiFi (copy to config.h)
├── platformio.ini            # esp32 / motor_test / native
├── src/
│   ├── main.cpp              # main firmware + serial tuning console
│   ├── robot_controller.*    # state machine + control loop + tuning interface
│   ├── path_planner.*        # zigzag coverage
│   ├── trajectory_follower.* # lookahead path following (tunable parameters)
│   ├── kinematics.* odometry.* pid_controller.* motor_driver.* wifi_comm.*
│   └── motor_test_main.cpp   # motor test bench
├── scripts/                  # whiteboard_app.py + pid_tuner.py
└── test/test_all.cpp         # native tests
```

---

## 9. Safety and next steps

- Do not commit the WiFi password in `config.h` to a public Git repository.
- Before powering up a new driver, test VCC–GND in continuity mode, to avoid burning the ESP32 regulator.
- Long-term suggestion: use an ESP32-DevKitC with onboard USB, to reduce the risks of the external CP2102 and GPIO12.

---

*This report follows the current source files in the repository; please update it when the code changes.*
