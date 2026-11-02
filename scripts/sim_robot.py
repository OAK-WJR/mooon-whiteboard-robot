#!/usr/bin/env python3
"""WhiteBoardBot local simulated robot - tests whiteboard_app.py point-to-point navigation + 2D visualization without hardware.

Replicates the firmware UDP protocol: listens on 8888 for commands (goto/area/pos/stop), integrates the pose as a holonomic omni-wheel robot,
and sends the state (state,x,y,theta,progress,wp,wp_total) at ~20Hz to 127.0.0.1:8889.
whiteboard_app.py learns this machine's IP automatically when it receives it, so clicking the map drives this simulated robot.

Usage (two terminals):
  python scripts/sim_robot.py
  python scripts/whiteboard_app.py      # left-click on the 2D map = set target and go
"""

import socket, json, time, math

LISTEN_PORT = 8888           # receives commands (= firmware UDP_LISTEN_PORT)
REPORT_ADDR = ("127.0.0.1", 8889)   # sends state to the host PC
MAX_SPEED   = 0.15           # m/s, = config MAX_LINEAR_SPEED
GOAL_TOL    = 0.01           # m
DT          = 0.05           # 20Hz

def main():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("0.0.0.0", LISTEN_PORT))
    rx.settimeout(0.0)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    x, y, theta = 0.0, 0.0, 0.0
    path = []            # waypoint list [(x,y), ...]
    wp = 0               # current waypoint index
    state = "idle"
    print(f"[sim] listening :{LISTEN_PORT}, sending state -> {REPORT_ADDR}. Waiting for commands...")

    while True:
        # --- receive commands (drain as many as possible) ---
        try:
            while True:
                data, _ = rx.recvfrom(2048)
                try:
                    cmd = json.loads(data.decode())
                except Exception:
                    continue
                c = cmd.get("cmd", "")
                if c == "goto":
                    path = [(float(cmd.get("x", 0)), float(cmd.get("y", 0)))]
                    wp, state = 0, "running"
                    print(f"[sim] goto {path[0]}")
                elif c == "area":
                    path = zigzag(cmd); wp, state = 0, "running"
                    print(f"[sim] area -> {len(path)} waypoints")
                elif c == "pos":
                    x, y, theta = float(cmd.get("x", 0)), float(cmd.get("y", 0)), float(cmd.get("theta", 0))
                    print(f"[sim] pos correction -> ({x:.3f},{y:.3f},{theta:.2f})")
                elif c in ("stop",):
                    path, state = [], "idle"
                elif c == "start" and path:
                    wp, state = 0, "running"
        except BlockingIOError:
            pass
        except socket.timeout:
            pass

        # --- motion integration: move omnidirectionally toward the current waypoint ---
        if state == "running" and wp < len(path):
            tx_, ty_ = path[wp]
            dx, dy = tx_ - x, ty_ - y
            dist = math.hypot(dx, dy)
            if dist < GOAL_TOL:
                wp += 1
                if wp >= len(path):
                    state = "done"
            else:
                step = min(MAX_SPEED * DT, dist)
                x += step * dx / dist
                y += step * dy / dist
                theta = math.atan2(dy, dx)        # face the target (for visualization)

        # --- send state ---
        total = len(path)
        status = {
            "state": state, "x": round(x, 4), "y": round(y, 4), "theta": round(theta, 4),
            "progress": round(wp / total, 3) if total else 0.0,
            "wp": wp, "wp_total": total,
        }
        try:
            tx.sendto(json.dumps(status).encode(), REPORT_ADDR)
        except Exception:
            pass
        time.sleep(DT)

def zigzag(cmd):
    ax, ay = float(cmd.get("x", 0)), float(cmd.get("y", 0))
    w, h = float(cmd.get("w", 0.1)), float(cmd.get("h", 0.1))
    sp = float(cmd.get("spacing", 0.02))
    pts, ltr, i = [], True, 0
    if w <= 0 or h <= 0 or sp <= 0:
        return pts
    while i * sp <= h + 1e-9 and len(pts) < 400:
        yy = min(ay + i * sp, ay + h)
        pts += [(ax, yy), (ax + w, yy)] if ltr else [(ax + w, yy), (ax, yy)]
        ltr = not ltr; i += 1
    return pts

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[sim] exit")
