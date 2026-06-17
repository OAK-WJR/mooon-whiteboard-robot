#!/usr/bin/env python3
"""WhiteBoardBot 控制面板 — 发送目标区域 + 接收状态 + 可视化轨迹"""

import sys, json, socket, time, threading
import numpy as np
from collections import deque

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QDoubleSpinBox, QPushButton, QGridLayout, QTextEdit
)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QFont
import pyqtgraph as pg

ESP32_IP = "192.168.1.74"   # ESP32 的 WiFi IP
ESP32_PORT = 8888
LISTEN_PORT = 8889

class WhiteBoardApp(QMainWindow):
    status_received = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("WhiteBoardBot Controller")
        self.resize(1200, 800)

        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.send_sock.bind(("0.0.0.0", LISTEN_PORT))  # 绑定固定端口，ESP32 回传到这个端口
        self.send_sock.settimeout(0.01)

        self.esp32_ip = None
        self.traj_x = deque(maxlen=2000)
        self.traj_y = deque(maxlen=2000)
        self.plan_x = []
        self.plan_y = []

        self._build_ui()
        self.status_received.connect(self._on_status)

        # 接收线程
        self.running = True
        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.recv_thread.start()

        # UI 刷新
        self.timer = QTimer()
        self.timer.timeout.connect(self._update_plots)
        self.timer.start(50)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main = QHBoxLayout(central)

        # === Left: trajectory plot ===
        self.plot = pg.PlotWidget(title="Trajectory (m)")
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.addLegend()
        self.plan_curve = self.plot.plot(pen=pg.mkPen('c', width=1, style=Qt.DashLine), name="Plan")
        self.traj_curve = self.plot.plot(pen=pg.mkPen('y', width=2), name="Actual")
        self.robot_dot = self.plot.plot(pen=None, symbol='o', symbolSize=10, symbolBrush='r', name="Robot")
        # 目标区域矩形
        self.area_rect = pg.PlotDataItem(pen=pg.mkPen('g', width=2, style=Qt.DashLine))
        self.plot.addItem(self.area_rect)
        main.addWidget(self.plot, stretch=3)

        # === Right: controls ===
        ctrl = QVBoxLayout()

        # Area input
        area_grp = QGroupBox("Target Area")
        ag = QGridLayout(area_grp)
        self.spins = {}
        for i, (label, default) in enumerate([("X", 0.0), ("Y", 0.0), ("W", 0.15), ("H", 0.10), ("Spacing", 0.02)]):
            ag.addWidget(QLabel(label), i, 0)
            s = QDoubleSpinBox()
            s.setRange(-1.0, 1.0)
            s.setSingleStep(0.01)
            s.setDecimals(3)
            s.setValue(default)
            ag.addWidget(s, i, 1)
            self.spins[label.lower()] = s
        ctrl.addWidget(area_grp)

        # Buttons
        btn_grp = QGroupBox("Control")
        bg = QVBoxLayout(btn_grp)

        send_btn = QPushButton("Send Area")
        send_btn.setStyleSheet("background-color: #4488ff; font-weight: bold; padding: 8px;")
        send_btn.clicked.connect(self._send_area)
        bg.addWidget(send_btn)

        start_btn = QPushButton("START")
        start_btn.setStyleSheet("background-color: #44cc44; font-weight: bold; padding: 8px;")
        start_btn.clicked.connect(lambda: self._send({"cmd": "start"}))
        bg.addWidget(start_btn)

        stop_btn = QPushButton("STOP")
        stop_btn.setStyleSheet("background-color: #ff4444; font-weight: bold; padding: 8px;")
        stop_btn.clicked.connect(lambda: self._send({"cmd": "stop"}))
        bg.addWidget(stop_btn)

        clear_btn = QPushButton("Clear Trajectory")
        clear_btn.clicked.connect(self._clear_traj)
        bg.addWidget(clear_btn)
        ctrl.addWidget(btn_grp)

        # Status
        status_grp = QGroupBox("Status")
        sg = QVBoxLayout(status_grp)
        self.lbl_state = QLabel("--")
        self.lbl_state.setFont(QFont("Menlo", 16, QFont.Bold))
        self.lbl_state.setAlignment(Qt.AlignCenter)
        sg.addWidget(self.lbl_state)

        self.lbl_pos = QLabel("x=-- y=-- θ=--")
        self.lbl_pos.setFont(QFont("Menlo", 12))
        sg.addWidget(self.lbl_pos)

        self.lbl_progress = QLabel("Progress: --")
        self.lbl_progress.setFont(QFont("Menlo", 12))
        sg.addWidget(self.lbl_progress)
        ctrl.addWidget(status_grp)

        # Log
        self.log = QTextEdit()
        self.log.setMaximumHeight(150)
        self.log.setReadOnly(True)
        ctrl.addWidget(self.log)

        ctrl.addStretch()
        main.addLayout(ctrl, stretch=1)

    def _send(self, data):
        msg = json.dumps(data).encode()
        target = self.esp32_ip or ESP32_IP
        try:
            self.send_sock.sendto(msg, (target, ESP32_PORT))
        except Exception as e:
            self.log.append(f"Send error: {e}")
            return
        self.log.append(f"→ {data}")

    def _send_area(self):
        area = {
            "cmd": "area",
            "x": self.spins["x"].value(),
            "y": self.spins["y"].value(),
            "w": self.spins["w"].value(),
            "h": self.spins["h"].value(),
            "spacing": self.spins["spacing"].value()
        }
        self._send(area)
        # 画目标区域
        x, y, w, h = area["x"], area["y"], area["w"], area["h"]
        self.area_rect.setData([x, x+w, x+w, x, x], [y, y, y+h, y+h, y])
        # 画规划路径预览
        self._preview_zigzag(x, y, w, h, area["spacing"])

    def _preview_zigzag(self, ax, ay, w, h, spacing):
        self.plan_x.clear()
        self.plan_y.clear()
        lines = int(h / spacing) + 1
        ltr = True
        for i in range(lines):
            yy = ay + i * spacing
            if yy > ay + h: yy = ay + h
            if ltr:
                self.plan_x.extend([ax, ax + w])
                self.plan_y.extend([yy, yy])
            else:
                self.plan_x.extend([ax + w, ax])
                self.plan_y.extend([yy, yy])
            ltr = not ltr

    def _clear_traj(self):
        self.traj_x.clear()
        self.traj_y.clear()

    def _recv_loop(self):
        while self.running:
            try:
                data, addr = self.send_sock.recvfrom(1024)
                self.esp32_ip = addr[0]
                msg = json.loads(data.decode())
                self.status_received.emit(msg)
            except socket.timeout:
                pass
            except Exception as e:
                print(f"Recv error: {e}")

    def _on_status(self, msg):
        state = msg.get("state", "?")
        x = msg.get("x", 0)
        y = msg.get("y", 0)
        theta = msg.get("theta", 0)
        progress = msg.get("progress", 0)
        wp = msg.get("wp", 0)
        wp_total = msg.get("wp_total", 0)

        self.lbl_state.setText(state.upper())
        colors = {"idle": "#888", "planning": "#ff8", "ready": "#8f8", "running": "#4f4", "done": "#88f"}
        self.lbl_state.setStyleSheet(f"color: {colors.get(state, '#fff')};")
        self.lbl_pos.setText(f"x={x:.3f}  y={y:.3f}  θ={theta:.2f}")
        self.lbl_progress.setText(f"Progress: {wp}/{wp_total} ({progress*100:.0f}%)")

        if state == "running":
            self.traj_x.append(x)
            self.traj_y.append(y)

    def _update_plots(self):
        if self.plan_x:
            self.plan_curve.setData(self.plan_x, self.plan_y)
        if len(self.traj_x) > 1:
            self.traj_curve.setData(list(self.traj_x), list(self.traj_y))
        if len(self.traj_x) > 0:
            self.robot_dot.setData([self.traj_x[-1]], [self.traj_y[-1]])

    def closeEvent(self, event):
        self.running = False
        self._send({"cmd": "stop"})
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = WhiteBoardApp()
    win.show()
    sys.exit(app.exec_())
