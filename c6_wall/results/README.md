# Key run logs

A few logs from the real robot on the whiteboard, copied from the local
`c6_wall/logs/` folder (about 1 GB, kept offline). Videos are not included.
Text logs are in Chinese, as printed by the tools.

| File | Run | What to look at |
|---|---|---|
| `board-1026-222231.json` | 2026-10-26, full-board coverage run (`run_board.py`) | `report.segments`: planned vs camera-measured end of each lane; `start` and `end` poses (returned within 0.2 mm, heading change -0.18 deg) |
| `sysid-erase-1028-214047-motion.json`, `sysid-motion-run.log` | 2026-10-28, erase-speed motion test (`sysid_erase.py motion`) | Out-and-back moves in 8 directions, open loop vs live camera fixes; end error per move |
| `sysid-erase-1028-214624-erase.json`, `.jpg` | 2026-10-28, eraser-width test (`sysid_erase.py erase`) | How much ink each pass removes along a line; measured slip per direction |
| `clean-1028-223528-session.json`, `clean-demo2.log` | 2026-10-28 22:35, autonomous erase (`clean_dirty.py`) | Ink area per pass (130 -> 2.31 -> 1.69 cm2, pass 3 clears the rest), every waypoint target vs measured, then the fine wipe |
| `draw67-1031-114853.json` | 2026-10-31, drawing "67" with a pen (`draw_digits.py`) | 30 steps, target vs camera-measured point, heading, camera fixes and rejections |

Absolute paths in the logs were shortened to `c6_wall/...`. No network
names or addresses are stored in these files.
