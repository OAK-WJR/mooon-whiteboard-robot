#!/usr/bin/env python3
"""USB 串口 <-> UDP 桥 —— 让 whiteboard_app.py 直接走 USB 串口跑导航 (无需 WiFi)。

把通过 USB 连接的 ESP32 (esp32 主固件) 伪装成一台「WiFi 机器人」给上位机:
  · 读固件的位姿 JSON 流 (串口发 `j` 开启) -> 转发到 127.0.0.1:8889 (上位机收, 自动学到本机)
  · 监听 UDP 8888 收上位机命令 (goto/area/stop/...) -> 翻译成串口控制台命令写给固件

用法 (两个终端):
  python scripts/serial_bridge.py [/dev/cu.usbserial-0001]
  python scripts/whiteboard_app.py     # 2D 图上左键点击 = 设目标并前往 (走 USB)
"""

import sys, time, json, socket, threading
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbserial-0001"
BAUD = 115200
APP_STATUS_ADDR = ("127.0.0.1", 8889)   # 上位机监听口
CMD_LISTEN_PORT = 8888                   # 收上位机命令 (= 固件 UDP_LISTEN_PORT)

def open_serial():
    s = serial.Serial(PORT, BAUD, timeout=0.1, dsrdtr=True)
    s.dtr = False; s.rts = True; time.sleep(0.1); s.rts = False
    time.sleep(2.0); s.reset_input_buffer()
    s.write(b"j\n")            # 开启固件位姿 JSON 流
    print(f"[bridge] 串口 {PORT} 已开, 已发 'j' 开启位姿流")
    return s

def cmd_loop(ser):
    """收上位机 UDP 命令 -> 翻译成串口命令"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", CMD_LISTEN_PORT))
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            d = json.loads(data.decode())
        except Exception:
            continue
        c = d.get("cmd", "")
        line = None
        if c == "goto":
            line = f"goto {d.get('x',0)} {d.get('y',0)}"
        elif c == "area":
            line = f"area {d.get('x',0)} {d.get('y',0)} {d.get('w',0.1)} {d.get('h',0.1)} {d.get('spacing',0.02)}"
        elif c == "pos":
            line = f"pos {d.get('x',0)} {d.get('y',0)} {d.get('theta',0)}"
        elif c == "set_speed":
            line = f"v {d.get('v',0.15)}"
        elif c == "start":
            line = "go"
        elif c == "stop":
            line = "x"
        if line is not None:
            ser.write((line + "\n").encode())
            print(f"[bridge] app->{line}")

def main():
    ser = open_serial()
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    threading.Thread(target=cmd_loop, args=(ser,), daemon=True).start()
    print(f"[bridge] 转发位姿 -> {APP_STATUS_ADDR}, 监听命令 :{CMD_LISTEN_PORT}")
    while True:
        line = ser.readline().decode("utf-8", "ignore").strip()
        if line.startswith("{"):
            try:
                json.loads(line)               # 校验是合法 JSON
                out.sendto(line.encode(), APP_STATUS_ADDR)
            except Exception:
                pass

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[bridge] 退出")
