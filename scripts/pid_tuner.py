#!/usr/bin/env python3
"""WhiteBoardBot live PID tuning panel (PyQt5 + pyqtgraph)"""

import sys, time, struct
import serial
import numpy as np
from collections import deque

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QDoubleSpinBox, QPushButton, QComboBox, QGridLayout
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
import pyqtgraph as pg

PORT = "/dev/cu.usbserial-0001"
BAUD = 115200
MAX_POINTS = 500

class PIDTuner(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("WhiteBoardBot PID Tuner")
        self.resize(1200, 700)

        # Serial
        self.ser = serial.Serial(PORT, BAUD, timeout=0.05, dsrdtr=True, rtscts=False)
        self.ser.dtr = False; self.ser.rts = True
        time.sleep(0.1)
        self.ser.rts = False
        time.sleep(2)
        self.ser.reset_input_buffer()

        # Data buffers
        self.t_buf = deque(maxlen=MAX_POINTS)
        self.tgt_buf = deque(maxlen=MAX_POINTS)
        self.act_buf = deque(maxlen=MAX_POINTS)
        self.pwm_buf = deque(maxlen=MAX_POINTS)
        self.sample_idx = 0

        self._build_ui()

        # Query current gains
        self.ser.write(b'G\n')

        # Timer for reading serial data
        self.timer = QTimer()
        self.timer.timeout.connect(self._update)
        self.timer.start(30)  # 33 Hz refresh

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)

        # === Left: Plots ===
        plot_layout = QVBoxLayout()

        # Speed plot
        self.speed_plot = pg.PlotWidget(title="Speed (m/s)")
        self.speed_plot.addLegend()
        self.speed_plot.showGrid(x=True, y=True, alpha=0.3)
        self.speed_plot.setYRange(-0.5, 0.5)
        self.tgt_curve = self.speed_plot.plot(pen=pg.mkPen('c', width=2, style=Qt.DashLine), name="Target")
        self.act_curve = self.speed_plot.plot(pen=pg.mkPen('y', width=2), name="Actual")
        plot_layout.addWidget(self.speed_plot)

        # PWM plot
        self.pwm_plot = pg.PlotWidget(title="PWM")
        self.pwm_plot.showGrid(x=True, y=True, alpha=0.3)
        self.pwm_plot.setYRange(-255, 255)
        self.pwm_curve = self.pwm_plot.plot(pen=pg.mkPen('r', width=2), name="PWM")
        plot_layout.addWidget(self.pwm_plot)

        main_layout.addLayout(plot_layout, stretch=3)

        # === Right: Controls ===
        ctrl_layout = QVBoxLayout()

        # Motor select
        motor_group = QGroupBox("Motor")
        mg = QHBoxLayout(motor_group)
        self.motor_btns = []
        for i in range(3):
            btn = QPushButton(f"M{i}")
            btn.setCheckable(True)
            btn.clicked.connect(lambda _, idx=i: self._select_motor(idx))
            mg.addWidget(btn)
            self.motor_btns.append(btn)
        self.motor_btns[0].setChecked(True)
        ctrl_layout.addWidget(motor_group)

        # Speed control
        speed_group = QGroupBox("Target Speed")
        sg = QVBoxLayout(speed_group)
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(-0.5, 0.5)
        self.speed_spin.setSingleStep(0.01)
        self.speed_spin.setDecimals(3)
        self.speed_spin.setSuffix(" m/s")
        self.speed_spin.setFont(QFont("Monospace", 16))
        sg.addWidget(self.speed_spin)

        speed_btns = QHBoxLayout()
        for val in [-0.2, -0.1, -0.05, 0, 0.05, 0.1, 0.2]:
            label = "STOP" if val == 0 else f"{val:+.2f}"
            btn = QPushButton(label)
            if val == 0:
                btn.setStyleSheet("background-color: #ff6666;")
                btn.clicked.connect(self._stop)
            else:
                btn.clicked.connect(lambda _, v=val: self._set_speed(v))
            speed_btns.addWidget(btn)
        sg.addLayout(speed_btns)

        send_speed_btn = QPushButton("Send Speed")
        send_speed_btn.setStyleSheet("background-color: #66cc66; font-weight: bold;")
        send_speed_btn.clicked.connect(self._send_speed)
        sg.addWidget(send_speed_btn)
        ctrl_layout.addWidget(speed_group)

        # PID gains
        pid_group = QGroupBox("PID Gains (Enter to send)")
        pg_layout = QGridLayout(pid_group)

        self.gain_spins = {}
        gains = [
            ("Kp", "P", 0, 500, 1.0, 80.0),
            ("Ki", "I", 0, 500, 1.0, 120.0),
            ("Kd", "D", 0, 100, 0.5, 3.0),
            ("Kf", "F", 0, 2000, 5.0, 635.0),
            ("I-Lim", "L", 0, 500, 1.0, 40.0),
        ]
        for row, (label, cmd, vmin, vmax, step, default) in enumerate(gains):
            lbl = QLabel(label)
            lbl.setFont(QFont("Monospace", 12))
            pg_layout.addWidget(lbl, row, 0)

            spin = QDoubleSpinBox()
            spin.setRange(vmin, vmax)
            spin.setSingleStep(step)
            spin.setDecimals(1)
            spin.setValue(default)
            spin.setFont(QFont("Monospace", 14))
            spin.setProperty("cmd", cmd)
            spin.editingFinished.connect(lambda s=spin: self._send_gain(s))
            pg_layout.addWidget(spin, row, 1)

            send_btn = QPushButton("Send")
            send_btn.clicked.connect(lambda _, s=spin: self._send_gain(s))
            pg_layout.addWidget(send_btn, row, 2)

            self.gain_spins[cmd] = spin

        ctrl_layout.addWidget(pid_group)

        # Actual values display
        val_group = QGroupBox("Live Values")
        vg = QGridLayout(val_group)
        self.lbl_target = QLabel("0.000")
        self.lbl_actual = QLabel("0.000")
        self.lbl_pwm = QLabel("0")
        for lbl in [self.lbl_target, self.lbl_actual, self.lbl_pwm]:
            lbl.setFont(QFont("Monospace", 18, QFont.Bold))
            lbl.setAlignment(Qt.AlignCenter)
        vg.addWidget(QLabel("Target:"), 0, 0)
        vg.addWidget(self.lbl_target, 0, 1)
        vg.addWidget(QLabel("Actual:"), 1, 0)
        vg.addWidget(self.lbl_actual, 1, 1)
        vg.addWidget(QLabel("PWM:"), 2, 0)
        vg.addWidget(self.lbl_pwm, 2, 1)
        ctrl_layout.addWidget(val_group)

        # Reset button
        reset_btn = QPushButton("Reset PID Integrators")
        reset_btn.clicked.connect(lambda: self.ser.write(b'R\n'))
        ctrl_layout.addWidget(reset_btn)

        ctrl_layout.addStretch()
        main_layout.addLayout(ctrl_layout, stretch=1)

    def _select_motor(self, idx):
        for i, btn in enumerate(self.motor_btns):
            btn.setChecked(i == idx)
        self.ser.write(f"{idx}\n".encode())

    def _set_speed(self, val):
        self.speed_spin.setValue(val)
        self._send_speed()

    def _send_speed(self):
        val = self.speed_spin.value()
        self.ser.write(f"S{val:.3f}\n".encode())

    def _stop(self):
        self.speed_spin.setValue(0)
        self.ser.write(b'A\n')

    def _send_gain(self, spin):
        cmd = spin.property("cmd")
        val = spin.value()
        self.ser.write(f"{cmd}{val:.1f}\n".encode())

    def _update(self):
        lines_read = 0
        while self.ser.in_waiting and lines_read < 100:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                lines_read += 1
            except Exception:
                continue

            # Parse GAINS response
            if line.startswith('GAINS:'):
                try:
                    pairs = line[6:].split(',')
                    for p in pairs:
                        k, v = p.split('=')
                        if k in self.gain_spins:
                            self.gain_spins[k].setValue(float(v))
                except Exception:
                    pass
                continue

            # Skip non-data lines
            if not line.startswith('$'):
                continue

            # Parse data frame: $tgt,act,pwm,FF,P,I,D
            parts = line[1:].split(',')
            if len(parts) < 3:
                continue
            try:
                tgt = float(parts[0])
                act = float(parts[1])
                pwm = float(parts[2])
            except ValueError:
                continue

            self.t_buf.append(self.sample_idx)
            self.tgt_buf.append(tgt)
            self.act_buf.append(act)
            self.pwm_buf.append(pwm)
            self.sample_idx += 1

            # Update live values
            self.lbl_target.setText(f"{tgt:.4f}")
            self.lbl_actual.setText(f"{act:.4f}")
            self.lbl_pwm.setText(f"{int(pwm)}")

        # Update plots
        if len(self.t_buf) > 1:
            t = np.array(self.t_buf)
            self.tgt_curve.setData(t, np.array(self.tgt_buf))
            self.act_curve.setData(t, np.array(self.act_buf))
            self.pwm_curve.setData(t, np.array(self.pwm_buf))

    def closeEvent(self, event):
        self.ser.write(b'A\n')  # Stop motors on exit
        self.ser.close()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = PIDTuner()
    win.show()
    sys.exit(app.exec_())
