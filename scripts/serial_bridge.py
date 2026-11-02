#!/usr/bin/env python3
"""USB serial <-> UDP bridge - lets whiteboard_app.py run navigation directly over USB serial (no WiFi needed).

Presents the USB-connected ESP32 (esp32 main firmware) to the host PC as a "WiFi robot":
  · Reads the firmware's pose JSON stream (send `j` over serial to start it) -> forwards to 127.0.0.1:8889 (the host PC receives it and learns this machine automatically)
  · Listens on UDP 8888 for host PC commands (goto/area/stop/...) -> translates them into serial console commands for the firmware

Usage (two terminals):
  python scripts/serial_bridge.py [/dev/cu.usbserial-0001]
  python scripts/whiteboard_app.py     # left-click on the 2D map = set target and go (over USB)
"""

import sys, time, json, socket, threading
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbserial-0001"
BAUD = 115200
APP_STATUS_ADDR = ("127.0.0.1", 8889)   # host PC listen port
CMD_LISTEN_PORT = 8888                   # receives host PC commands (= firmware UDP_LISTEN_PORT)

def open_serial():
    s = serial.Serial(PORT, BAUD, timeout=0.1, dsrdtr=True)
    s.dtr = False; s.rts = True; time.sleep(0.1); s.rts = False
    time.sleep(2.0); s.reset_input_buffer()
    s.write(b"j\n")            # start the firmware pose JSON stream
    print(f"[bridge] serial {PORT} open, sent 'j' to start the pose stream")
    return s

def cmd_loop(ser):
    """Receive host PC UDP commands -> translate them into serial commands"""
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
    print(f"[bridge] forwarding pose -> {APP_STATUS_ADDR}, listening for commands :{CMD_LISTEN_PORT}")
    while True:
        line = ser.readline().decode("utf-8", "ignore").strip()
        if line.startswith("{"):
            try:
                json.loads(line)               # check it is valid JSON
                out.sendto(line.encode(), APP_STATUS_ADDR)
            except Exception:
                pass

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[bridge] exit")
