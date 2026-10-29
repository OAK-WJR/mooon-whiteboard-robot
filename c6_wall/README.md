# c6_wall — wall firmware

Current firmware for WhiteBoardBot. It runs on the Waveshare ESP32-C6-LCD-1.47,
the adapter v2 PCB and the RZ7889 base board.

## Layout

| Path | Contents |
|---|---|
| `src/main.cpp` | Hardware, Wi-Fi, the HTTP/WebSocket API, OTA, NVS persistence and the LCD |
| `include/wbb_core.h` | Hardware-independent control core: kinematics, per-wheel position servo, path runner, boustrophedon planner, slip-compensated odometry, feed-forward calibrator |
| `include/network_config.h` | Validation of saved Wi-Fi credentials |
| `web/index.html` | Control page. `tools/prebuild.py` gzips it into the firmware at build time |
| `tests/sim_test.cpp` | Desktop simulation tests for `wbb_core.h` |
| `tools/` | Host scripts (Python 3), listed below |

## Build, flash and test

```bash
# from the repository root: python3 -m venv .venv && .venv/bin/pip install platformio
../.venv/bin/pio run -t upload          # first install over USB
python3 tools/ota.py                    # every later update, over Wi-Fi (~20 s)
c++ -std=c++17 -Iinclude tests/sim_test.cpp -o /tmp/sim && /tmp/sim
```

The first build creates `.ota_token`, a random token that both the firmware and
`tools/ota.py` use. It is gitignored. OTA images are marked valid only after the
robot comes back online, so a bad image rolls back by itself.

## Connecting

- **Robot hotspot.** The robot always opens an access point named
  `WBB-C6-XXXX`. Its password is random, generated on first boot, and shown on
  the LCD. The control page is at `http://192.168.4.1/`.
- **Your own Wi-Fi.** Join your 2.4 GHz network from the page's Wi-Fi section.
  After that the robot is reachable at `http://wbb-c6-xxxx.local/` or at the IP
  shown on the LCD.
- **Saved address for the tools.** Put that IP in `c6_wall/.robot_host`
  (gitignored) so the host tools find the robot.

API endpoints:

| Endpoint | Method | Use |
|---|---|---|
| `/api/status` | `GET` | Current status |
| `/api/cmd` | `POST` | JSON command, for example `{"op": "cover"}` |
| `/ws` | WebSocket | Live driving and telemetry |
| `/ota` | `POST` | Firmware upload, requires the token |

## Camera tools

These scripts need the system `python3` with OpenCV.

**Setup.** Four red tapes mark the corners of the writable surface. The robot
shows a cyan/magenta beacon on its LCD. Board dimensions and the camera
distance are in `tools/vision.json`.

| Script | Purpose |
|---|---|
| `vision.py` | Camera ground truth: corner tapes give a homography, the beacon gives the robot pose |
| `run_board.py` | Camera-guided start, then a full-board coverage run with live position fixes |
| `sysid_wall.py`, `slip_doe.py`, `explore.py` | Measure wheel slip on the board and fit the slip model |
| `campaign.py`, `fit_trips.py`, `roundtrip.py` | Round-trip accuracy campaigns without camera correction, and the iterative refit |
| `bench.py`, `safe_step.py`, `wheel_diag.py`, `jitter_probe.py`, `chatter.py`, `leg_probe.py` | Scripted moves, guarded steps, per-wheel health checks, chatter scoring |
| `erase_test.py` | Spin-erase test (wheels sweep while the robot turns) |
| `step_assembly.py` | Reads robot geometry from a CAD STEP assembly without OpenCASCADE |

## Safety notes from testing

- Keep at least 11 cm between the robot centre and the board frame.
- If a camera check disagrees with the command (turned more than about 3°, or
  moved less than 60 % of the distance), stop sending moves and look first.
- Spinning on the wall caused two power-on resets, and the robot fell both
  times. Run spin tests on a table first.
