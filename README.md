# WhiteBoardBot

基于 **ESP32** 的三轮全向轮（120° 布置）白板/桌面机器人。

**ESP32 自己完成全部控制** —— 路径规划、循线跟踪、前馈 + PID，**无 ROS**。
上位机（Mac/PC）只通过 WiFi UDP 下发「目标区域 / 位置校正」并接收状态回传。
PID、速度、前瞻等参数可在运行时经串口实时调，无需重新烧录。

```
┌─────────────────────────┐   WiFi UDP (JSON)   ┌──────────────────────────────┐
│ 上位机 whiteboard_app.py │  ───── area ──────► │ ESP32 (env:esp32)            │
│  发目标区域 / 位置校正   │  ◄──── status ───── │  规划→循线→逆运动学→FF/PID   │
│  画实时轨迹              │                     │  →RZ7899 ×3 → N20 电机       │
└─────────────────────────┘                     │  编码器→里程计               │
                                                 └──────────────────────────────┘
```

---

## 快速开始

```bash
# 1. Python 环境（上位机 + PlatformIO）
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# 2. 硬件配置：复制模板并填入 WiFi（config.h 已被 .gitignore 忽略）
cp include/config.example.h include/config.h
#   编辑 include/config.h，填 WIFI_SSID / WIFI_PASS；引脚和 PID 默认值按需调

# 3. 编译并烧录主固件
pio run -e esp32 -t upload && pio device monitor

# 4. 启动上位机控制面板
python scripts/whiteboard_app.py
```

---

## 构建环境（`platformio.ini`）

| 环境 | 用途 |
|------|------|
| `esp32`（默认） | **主固件**：自主控制 + WiFi UDP + 串口实时调参 |
| `motor_test` | 单/三电机测试台：开环 / PID 闭环 / 运动学，配合 `pid_tuner.py` 标定 |
| `native` | 桌面模拟测试 `pio test -e native`（运动学 / PID / 里程计），无需硬件 |

```bash
pio run -e esp32 -t upload      # 主固件
pio run -e motor_test -t upload # 电机测试台
pio test -e native              # 桌面单元/集成测试
```

---

## 串口实时调参（主固件）

烧录 `esp32` 固件后，打开串口（115200），发送以下命令（回车结束）。改完满意的值再写回 `config.h` 固化。

| 命令 | 说明 |
|------|------|
| `s` | 打印状态 JSON |
| `p` | 打印所有可调参数 |
| `t` | 切换遥测流（每轮 目标/实际/PWM，10Hz） |
| `m <0\|1\|2\|a>` | 选择调参电机（`a`=全部，默认全部） |
| `kp/ki/kd/kf/il <v>` | 设 P / I / D / 前馈 / 积分限幅（作用于选中电机） |
| `v <v>` | 最大线速度 (m/s) |
| `la <v>` | 前瞻距离 (m) |
| `gt <v>` | 到达容差 (m) |
| `kl <v>` | 循线比例增益 |
| `area <x y w h sp>` | 本地规划一条 zigzag 路径（台架测试，无需上位机） |
| `pos <x y th>` | 校正里程计位姿 |
| `go` / `x` | 开始执行当前路径 / 停止 |
| `h` 或 `?` | 帮助 |

> 单电机 PID 标定走 `motor_test` 固件 + `python scripts/pid_tuner.py`（PyQt5 实时曲线 + 增益滑块）。

---

## 上位机脚本（`scripts/`）

| 脚本 | 说明 |
|------|------|
| `whiteboard_app.py` | 控制面板：发目标区域、接收状态、画实时轨迹（WiFi UDP） |
| `pid_tuner.py` | 单电机 PID 实时调参面板（串口，配合 `motor_test` 固件） |

---

## 硬件

- 3× N20 编码电机（1:100 减速）+ 3× RZ7899-MS 单 H 桥驱动 + ESP32
- 38mm 全向轮，120° 布置，`ENCODER_CPR = 700`
- 引脚分配、接线要点、排障记录见 [`PROJECT_REPORT.md`](PROJECT_REPORT.md) 与 `include/config.example.h`

**烧录前注意**（详见报告）：

- GPIO12 是 Flash 电压选择脚，邻近杜邦线误碰会导致烧录失败（`Failed to communicate with the flash chip`）。
- 新 RZ7899/TB6612 上电前用万用表蜂鸣档测 VCC–GND，短路会烧 ESP32 稳压器。

---

## 目录结构

```
WhiteBoardBot/
├── platformio.ini          # 三个构建环境：esp32 / motor_test / native
├── include/config.example.h# 硬件 + PID + WiFi 配置模板（复制为 config.h）
├── src/
│   ├── main.cpp            # 主固件：自主控制 + 串口调参控制台
│   ├── robot_controller.*  # 状态机 + 命令分发 + 控制环
│   ├── path_planner.*      # zigzag 覆盖路径
│   ├── trajectory_follower.*# 前瞻循线（参数可运行时调）
│   ├── kinematics.*        # 三轮全向轮正/逆运动学
│   ├── odometry.*          # 编码器里程计
│   ├── pid_controller.*    # FF + PID（带 anti-windup / slew）
│   ├── motor_driver.*      # RZ7899 双引脚驱动
│   ├── wifi_comm.*         # WiFi UDP + JSON 收发
│   └── motor_test_main.cpp # 电机测试台固件（MOTOR_TEST_MODE）
├── scripts/                # 上位机：控制面板 + PID 调参
├── test/test_all.cpp       # native 单元/集成测试
└── PROJECT_REPORT.md       # 架构 / 硬件 / 排障 参考
```
