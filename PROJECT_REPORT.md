# WhiteBoardBot 项目总览报告

汇总架构、硬件、软件、调参与运维要点。**无 ROS** —— ESP32 自主控制。

---

## 1. 项目目标

**WhiteBoardBot** 是一台基于 **ESP32** 的三轮全向轮（120° 布置）白板/桌面机器人。
全部控制都在 ESP32 上完成，上位机只下发目标与位置校正：

| 层级 | 运行环境 | 职责 |
|------|----------|------|
| 上位机 | Mac/PC（Python，WiFi UDP） | 发目标区域 / 位置校正、画轨迹、PID 调参 |
| 机器人 | ESP32（Arduino + FreeRTOS） | 路径规划、前瞻循线、逆运动学、FF+PID、编码器里程计 |

---

## 2. 系统架构

```
┌─────────────────────────┐   WiFi UDP (JSON)   ┌──────────────────────────────┐
│ 上位机 whiteboard_app.py │  ──── area/pos ───► │ ESP32 (env:esp32)            │
│  目标区域 / 位置校正     │  ◄──── status ───── │  PathPlanner → Follower      │
│  实时轨迹               │   :8888 / :8889     │  → Kinematics → FF/PID       │
└─────────────────────────┘                     │  → RZ7899 ×3 → 3×N20         │
                                                 │  编码器 → Odometry           │
                                                 └──────────────────────────────┘
```

**UDP 命令（JSON，端口 `UDP_LISTEN_PORT=8888`）**

| `cmd` | 字段 | 说明 |
|-------|------|------|
| `area` | `x y w h spacing` | 规划 zigzag 覆盖路径 |
| `start` / `stop` | — | 开始 / 停止执行 |
| `pos` | `x y theta` | 视觉位置校正里程计 |
| `set_speed` | `v` | 设最大线速度 |

**状态回传（JSON，端口 `UDP_REPORT_PORT=8889`，10Hz）**：`state, x, y, theta, progress, wp, wp_total`。

控制环与上位机的命令一一对应，串口控制台复用同一套 `RobotController::handleCommand` 逻辑。

---

## 3. 固件（PlatformIO）

### 3.1 构建环境（`platformio.ini`）

| 环境 | 用途 |
|------|------|
| `esp32`（默认） | 主固件：自主控制 + WiFi UDP + 串口实时调参 |
| `motor_test` | 单/三电机测试台；开环 / PID / 运动学，串口交互，配合 `pid_tuner.py` |
| `native` | 桌面 `pio test -e native`，运动学 / PID / 里程计模拟 |

### 3.2 主固件（`src/main.cpp`）

- 双 FreeRTOS 任务：`controlTask`（Core1，50Hz 控制环）+ `commTask`（Core0，WiFi 收发）。
- `loop()` 跑串口调参控制台：运行时改 PID / 速度 / 前瞻 / 容差，无需重新烧录（命令表见文件顶部注释或 README）。
- 控制环：里程计更新 → 循线得机体速度 → 逆运动学 → 轮速 **FF+PID**；反馈用 `Odometry::getWheelVelocity()` 的直接轮速。

### 3.3 控制器与跟踪

- `RobotController`：状态机（idle/planning/ready/running/done）+ 命令分发 + 控制环 + 串口调参接口（`tunePid` / `setLookAhead` / `setGoalTolerance` / `setLinearGain` / `printParams` / `printTelemetry`）。
- `TrajectoryFollower`：前瞻路径跟踪，`look_ahead_` / `goal_tol_` / `kp_linear_` 三个参数运行时可调（默认取 `config.h`）。
- `PathPlanner`：矩形区域 zigzag 覆盖。

### 3.4 里程计（`src/odometry.cpp`）

- 编码器差分 → 轮速，做 EMA 低通减小 PID 抖动；`reset()` 时轮速清零。

### 3.5 PID（`src/pid_controller.*`）

- FF + PID，derivative-on-measurement，anti-windup（积分限幅），PWM slew rate 限制。
- 全套运行时 getter/setter（`setKp/Ki/Kd/Kf` 等），供串口调参与 `pid_tuner.py`。

### 3.6 电机驱动（`src/motor_driver.cpp`）

- RZ7899-MS 双引脚：正转 FI=PWM/BI=LOW，反转相反，刹车双 LOW。

### 3.7 电机测试固件（`src/motor_test_main.cpp`）

- 串口命令：`0/1/2` 选电机、`+/-`、`f/r`、`s/a`、`p`、`z`、`c`（开环/PID）、`k`+`w/x/q/e/j/l`（运动学）。
- 大写多字符命令 `P/I/D/F/L/G/R/S/V/A` 用于 PID 调参，PID 闭环时输出 `$tgt,act,pwm,FF,P,I,D` 帧供 `pid_tuner.py` 绘图。

---

## 4. 硬件配置摘要（`include/config.h`）

### 4.1 机械与编码器

- 全向轮直径 38 mm，轮心距底盘中心约 33.738 mm。
- 编码器：`ENCODER_CPR = 700`（7 PPR × 1:100 减速）。

### 4.2 引脚分配（务必与实物一致）

**RZ7899-MS 驱动（每颗单 H 桥，2 引脚 FI+BI）**

- M0：FI **32**，BI **33**
- M1：FI **25**，BI **26**
- M2：FI **27**，BI **14**

**编码器（AB 相）**

- M0：**34 / 35**
- M1：**36 / 39**
- M2：**19 / 18**

编码器方向修正 `ENCODER_DIR_M*`：M2 取反（`-1`）。

**注意（排障记录）**

- **GPIO 12** 为 Flash 电压选择脚；邻近 **GPIO 14** 的杜邦线若误碰拉高 GPIO 12，会导致 Flash 工作在 1.8 V 模式、`esptool` 报 “Failed to communicate with the flash chip”。布线远离或给 GPIO 12 下拉。
- 部分编码器脚（GPIO34/35/36/39）仅输入、无内部上拉；ESP32 errata 3.11 下 GPIO36/39 噪声可用 `adc_power_acquire()` 抑制。

### 4.3 RZ7899 / 供电要点

- VM（电机电源）与逻辑供电独立；与 ESP32、电池负极 **共地**。
- 损坏的驱动可能 VCC–GND 短路，与 ESP32 共地会烧稳压器——上电前用万用表蜂鸣档先测板子。

### 4.4 WiFi / UDP

- `WIFI_SSID`、`WIFI_PASS` 在 `config.h`（已被 `.gitignore` 忽略）。
- `UDP_LISTEN_PORT=8888`（收命令）、`UDP_REPORT_PORT=8889`（回状态）。ESP32 自动回复发命令的来源地址。

---

## 5. 工具链与环境（本机）

- **Python venv**：用 Homebrew `python3.10` 创建，再 `pip install -r requirements.txt`（含 platformio + 上位机 GUI 依赖）。曾出现空 `pio` 脚本 / pip 损坏导致命令无输出，需重建 venv。
- **ESP32 上传**：Intel Mac 上曾需 binutils 路径修正。
- **串口**：`pio device monitor` 在部分环境下可能失败，可用 `pyserial` 或系统串口工具。

---

## 6. 已记录问题与结论（排障摘要）

| 现象 | 可能原因 | 处理方向 |
|------|----------|----------|
| 上传报 Flash 通信失败、Manufacturer `ff` | GPIO12 被拉高、接线干扰 | 检查 GPIO12/14，共地前测驱动板 |
| 一连 GND ESP32 发烫 | 驱动 VCC–GND 短路 | 断电，蜂鸣档测 VCC/GND，更换模块 |
| PWM 有输出、编码器始终 0 | VM/GND 未接、电机未转或编码器未供电 | 查驱动电源与编码器线 |
| 运动抖动 | PID 过冲、轮速噪声 | 降 KP/KI、轮速 EMA、PWM slew；串口 `t` 看遥测实时调 |

---

## 7. 常用命令速查

```bash
# 主固件（自主控制 + 串口调参）
pio run -e esp32 -t upload && pio device monitor

# 电机测试台（无线无关，串口调 PID）
pio run -e motor_test -t upload

# Native 测试
pio test -e native

# 上位机
python scripts/whiteboard_app.py   # 控制面板
python scripts/pid_tuner.py        # 单电机 PID 调参
```

---

## 8. 目录结构（核心）

```
WhiteBoardBot/
├── include/config.example.h  # 硬件 + PID + WiFi（复制为 config.h）
├── platformio.ini            # esp32 / motor_test / native
├── src/
│   ├── main.cpp              # 主固件 + 串口调参控制台
│   ├── robot_controller.*    # 状态机 + 控制环 + 调参接口
│   ├── path_planner.*        # zigzag 覆盖
│   ├── trajectory_follower.* # 前瞻循线（参数可调）
│   ├── kinematics.* odometry.* pid_controller.* motor_driver.* wifi_comm.*
│   └── motor_test_main.cpp   # 电机测试台
├── scripts/                  # whiteboard_app.py + pid_tuner.py
└── test/test_all.cpp         # native 测试
```

---

## 9. 安全与后续建议

- 不要把 `config.h` 中的 WiFi 密码提交到公开 Git。
- 新驱动上电前蜂鸣档测 VCC–GND，避免烧 ESP32 稳压器。
- 长期建议：用带板载 USB 的 ESP32-DevKitC，减少外接 CP2102 与 GPIO12 风险。

---

*报告以仓库当前源文件为准；代码变更后请同步更新。*
