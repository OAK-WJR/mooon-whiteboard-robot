#!/usr/bin/env python3
"""WhiteBoardBot 本地仿真机器人 —— 无硬件测试 whiteboard_app.py 的点到点导航 + 2D 可视化。

复刻固件的 UDP 协议: 监听 8888 收命令 (goto/area/pos/stop), 全向轮holonomic积分位姿,
把状态 (state,x,y,theta,progress,wp,wp_total) 以 ~20Hz 发到 127.0.0.1:8889。
whiteboard_app.py 收到后会自动学到本机 IP, 于是点地图就能驱动这个仿真机器人。

用法 (两个终端):
  python scripts/sim_robot.py
  python scripts/whiteboard_app.py      # 在 2D 图上左键点击 = 设目标并前往
"""

import socket, json, time, math

LISTEN_PORT = 8888           # 收命令 (= 固件 UDP_LISTEN_PORT)
REPORT_ADDR = ("127.0.0.1", 8889)   # 发状态到上位机
MAX_SPEED   = 0.15           # m/s, = config MAX_LINEAR_SPEED
GOAL_TOL    = 0.01           # m
DT          = 0.05           # 20Hz

def main():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("0.0.0.0", LISTEN_PORT))
    rx.settimeout(0.0)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    x, y, theta = 0.0, 0.0, 0.0
    path = []            # 航点列表 [(x,y), ...]
    wp = 0               # 当前航点下标
    state = "idle"
    print(f"[sim] 监听 :{LISTEN_PORT}, 发状态 -> {REPORT_ADDR}. 等待命令...")

    while True:
        # --- 收命令 (尽量取干净) ---
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
                    print(f"[sim] area -> {len(path)} 航点")
                elif c == "pos":
                    x, y, theta = float(cmd.get("x", 0)), float(cmd.get("y", 0)), float(cmd.get("theta", 0))
                    print(f"[sim] pos 校正 -> ({x:.3f},{y:.3f},{theta:.2f})")
                elif c in ("stop",):
                    path, state = [], "idle"
                elif c == "start" and path:
                    wp, state = 0, "running"
        except BlockingIOError:
            pass
        except socket.timeout:
            pass

        # --- 运动积分: 朝当前航点全向移动 ---
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
                theta = math.atan2(dy, dx)        # 朝向目标 (可视化用)

        # --- 发状态 ---
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
        print("\n[sim] 退出")
