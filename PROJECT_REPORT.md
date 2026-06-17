# WhiteBoardBot 项目总览报告

生成日期：2026-04-02（工作区快照）。本文档汇总架构、硬件、软件、ROS2、已知问题与运维要点。

---

## 1. 项目目标

**WhiteBoardBot** 是一台基于 **ESP32** 的三轮全向轮（120° 布置）白板/桌面机器人，采用分层控制：

| 层级 | 运行环境 | 职责 |
|------|----------|------|
| 规划与轨迹跟踪 | PC / Docker 内 ROS2 Humble | 路径点、前瞻跟踪、速度平滑、`/cmd_vel` |
| 底层驱动与里程计 | ESP32 + micro-ROS | 编码器里程计、`/odom`、电机 FF+PID |

---

## 2. 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│  Docker: osrf/ros:humble-desktop-full (+ micro-ROS Agent)   │
│  ┌──────────────┐  /waypoints (String JSON)                  │
│  │ test_square  │──────────────────────────┐                 │
│  │ test_zigzag  │                          ▼                 │
│  └──────────────┘              ┌─────────────────────┐     │
│                                  │ trajectory_controller│    │
│                                  │  Look-ahead + 平滑   │     │
│                                  └──────────┬──────────┘     │
│                                             │ /cmd_vel       │
│  micro_ros_agent udp4 :8888 ◄───────────────┼──────────────► │
└─────────────────────────────────────────────┼────────────────┘
                                              │ WiFi UDP
                                              ▼
┌─────────────────────────────────────────────────────────────┐
│ ESP32 (esp32wifi)                                            │
│  订阅 /cmd_vel → 逆运动学 → FF+PID → TB6612 ×2 → 3×N20 电机   │
│  编码器 → 里程计 → 发布 /odom                                  │
└─────────────────────────────────────────────────────────────┘
```

**主要 ROS2 话题（典型配置）**

- `/odom`：`nav_msgs/Odometry`（ESP32 发布）
- `/cmd_vel`：`geometry_msgs/Twist`（PC 发布）
- `/waypoints`：`std_msgs/String`（JSON 路径点列表）
- `/trajectory_status`：`std_msgs/String`（状态/进度）
- `/tf`：可选，由测试脚本内联节点从 `/odom` 生成（Foxglove 可视化）

---

## 3. 固件（PlatformIO）

### 3.1 构建环境（`platformio.ini`）

| 环境 | 用途 |
|------|------|
| `esp32dev` | 默认；micro-ROS **串口** 传输 |
| `esp32wifi` | **推荐（macOS + Docker）**；`MICRO_ROS_TRANSPORT_WIFI`，UDP 连 Agent |
| `motor_test` | 无 micro-ROS；`motor_test_main.cpp` 串口交互测电机/编码器/PID/运动学 |
| `native` | 桌面 `pio test -e native`，单元/集成模拟 |

### 3.2 主固件逻辑（`src/main.cpp`）

- micro-ROS 节点：`/odom` 发布、`/cmd_vel` 订阅。
- **CMD_VEL 超时**：`CMD_VEL_TIMEOUT_MS = 1000`，防止 WiFi 抖动误停车。
- 控制环：里程计更新 → 目标机体速度 → 逆运动学 → **轮速 FF+PID**；反馈使用 **直接轮速**（经 `Odometry::getWheelVelocity()`），避免 body 速度来回变换引入噪声。
- 依赖：`kinematics`、`odometry`、`motor_driver`、`pid_controller`。

### 3.3 里程计（`src/odometry.cpp`）

- 编码器差分 → 轮速；对轮速做 **EMA 低通**（减小 PID 抖动）。
- `reset()` 时轮速清零。

### 3.4 PID（`src/pid_controller.*`）

- FF + PID；支持 **PWM slew rate** 限制，平滑输出。

### 3.5 电机驱动（`src/motor_driver.cpp`）

- TB6612 语义：`setSpeed` 正转 IN1=H/IN2=L，反转相反；`stop()` 短刹（双高 + PWM=0）。

### 3.6 电机测试固件（`src/motor_test_main.cpp`）

- 串口命令：`0/1/2` 选电机、`+/-`、`f/r`、`s/a`、`p`、`z`、`c`（PID）、`k` + `w/x/q/e/j/l`（运动学）。
- 用于硬件排障（与 ROS 无关）。

---

## 4. 硬件配置摘要（`include/config.h`）

### 4.1 机械与编码器

- 全向轮直径 38 mm，轮心距底盘中心约 33.738 mm。
- 编码器：`ENCODER_CPR = 700`（7 PPR × 减速比等效，按项目注释校准）。

### 4.2 引脚分配（务必与实物一致）

**TB6612 #1**

- M0（B 通道）：PWM **22**，DIR **5 / 17**
- M1（A 通道）：PWM **23**，DIR **19 / 18**

**TB6612 #2**

- M2（A 通道）：PWM **21**，DIR **16 / 4**（DIR 交换以统一转向）

**编码器**

- M0：**32 / 33**
- M1：**27 / 14**
- M2：**25 / 26**

**注意（排障记录）**

- **GPIO 12** 为 Flash 电压选择脚；邻近 **GPIO 14** 的杜邦线若误碰拉高 GPIO 12，会导致 Flash 工作在 1.8 V 模式、`esptool` 报 “Failed to communicate with the flash chip”。布线时远离或给 GPIO 12 下拉。
- **GPIO 5** 等为 strapping 相关脚，上电电平影响启动；外设勿在启动瞬间强行拉错电平。

### 4.3 TB6612 接线要点

- **VM**：电机电源（与逻辑电平独立）。
- **VCC**：逻辑供电（3.3 V / 5 V，与数据手册一致）。
- **GND**：与 ESP32、电池负极 **共地**。
- **STBY**：须拉高，否则 H 桥待机。
- 损坏的 TB6612 可能出现 **VCC–GND 短路**，与 ESP32 共地会导致稳压器过热——须用万用表蜂鸣/电阻档先测板子再上电。

### 4.4 WiFi / Agent（仅 `esp32wifi`）

- `WIFI_SSID`、`WIFI_PASS`、`AGENT_IP`、`AGENT_PORT` 在 `config.h`。**勿将含密码的仓库公开**；生产环境建议改用 `sdkconfig` 分区或环境注入。

---

## 5. ROS2 工作区（`ros2_ws`）

### 5.1 包：`whiteboard_bot`

**入口（`setup.py`）**

- `trajectory_controller`
- `vision_node`
- `test_square`
- `sim_odom`
- `test_zigzag`

### 5.2 轨迹节点 `trajectory_controller.py`（核心）

- **前瞻路径跟踪**：沿折线路径找最近点 + `look_ahead_distance` 前瞻点，减少拐点停车。
- **路径进度**：`min_seg_idx` 等逻辑，避免起点与终点重合时误判 “已完成”；终点需 **前瞻到路径末端 + 距离终点在容差内** 才结束。
- **速度平滑**：线/角加速度与减速度限制，比例缩放保持方向。
- **终端减速**：`end_decel_distance`、`min_approach_speed`。
- **输出 EMA**：`output_filter_alpha` 平滑 `cmd_vel`。
- 主要参数（可在 launch/yaml 覆盖）：`look_ahead_distance`、`goal_tolerance`、`max_linear_speed`、`max_linear_accel` / `max_linear_decel`、`max_angular_accel` / `max_angular_decel`、`kp_linear`、`kp_angular`、`output_filter_alpha`、`control_rate` 等。

### 5.3 测试节点

- **`test_square`**：正方形路径；末点 `theta: None` 避免终点角速度卡死。
- **`test_zigzag`**：矩形区域 S 型覆盖，行末 U 型过渡中间点。

### 5.4 脚本

- `test_real_hardware.sh`：单容器内 **Agent (UDP 8888) + trajectory_controller + TF + Foxglove Bridge**，避免 DDS 跨容器发现失败。
- `test_1m_foxglove.sh`：Foxglove 相关测试（见脚本内说明）。

### 5.5 Launch / 配置

- `share/whiteboard_bot/launch/bringup.launch.py`
- `share/whiteboard_bot/config/params.yaml`

---

## 6. Docker（`docker/`）

- `docker-compose.yml`：构建 `whiteboard_bot` 镜像，挂载 `ros2_ws/src`，暴露 **8888/udp**。
- `Dockerfile`、`entrypoint.sh`：镜像内 ROS2 与构建入口。

**与本项目相关的常见镜像（本机可能还有 CVAT 等无关镜像，按需清理）。**

---

## 7. 工具链与环境（本机）

- **Python venv**：项目 `.venv` 建议用 **Homebrew `python3.10`** 创建（`/usr/local/bin/python3.10 -m venv .venv`），再 `pip install platformio`。曾出现 **空的 `pio` 脚本** 或 pip 损坏导致 “命令无输出”，需重建 venv。
- **ESP32 上传**：Intel Mac 上 micro-ROS 构建曾需 **binutils** 路径（`/opt/homebrew/opt/binutils` → 链到 `/usr/local/opt/binutils`）。
- **串口**：`pio device monitor` 在部分环境下可能失败，可用 `pyserial` 脚本（如 `scripts/motor_serial_test.py`）或系统串口工具。

---

## 8. 已记录问题与结论（排障摘要）

| 现象 | 可能原因 | 处理方向 |
|------|----------|----------|
| 上传报 Flash 通信失败、Manufacturer `ff` | GPIO12 被拉高、接线干扰 | 检查 GPIO12/14，共地前测 TB6612 |
| 一连 GND ESP32 发烫 | TB6612 **VCC–GND 短路** 等 | 断电，蜂鸣档测 VCC/GND，更换模块 |
| PWM 有输出、编码器始终 0 | VM/STBY/GND 未接、电机未转或编码器未供电 | 查 TB6612 电源与编码器线 |
| `cmd_vel` 断断续续停车 | 超时过短 / WiFi 延迟 | 已加大 `CMD_VEL_TIMEOUT_MS` |
| 运动抖动 | PID 过冲、轮速噪声 | 已降 KP/KI、轮速 EMA、PWM slew、ROS 侧平滑 |

---

## 9. 常用命令速查

```bash
# 固件（WiFi micro-ROS）
cd WhiteBoardBot
.venv/bin/pio run -e esp32wifi -t upload

# 电机测试（无 ROS）
.venv/bin/pio run -e motor_test -t upload

# Native 测试
.venv/bin/pio test -e native

# ROS2 包编译（容器或本机已 source humble）
cd ros2_ws && colcon build --symlink-install --packages-select whiteboard_bot
```

---

## 10. 目录结构（核心）

```
WhiteBoardBot/
├── include/config.h          # 硬件与 PID、WiFi
├── platformio.ini            # 多环境构建
├── src/
│   ├── main.cpp              # micro-ROS 主程序
│   ├── motor_test_main.cpp   # 电机测试
│   ├── motor_driver.*
│   ├── odometry.*
│   ├── pid_controller.*
│   └── kinematics.*
├── ros2_ws/
│   ├── src/whiteboard_bot/   # Python 节点与 launch/config
│   ├── test_real_hardware.sh
│   └── test_1m_foxglove.sh
├── docker/
├── scripts/
├── test/                     # native 测试
└── .scripts/                 # compile_commands / clangd
```

---

## 11. 安全与后续建议

- **不要将 `config.h` 中的 WiFi 密码提交到公开 Git**；可用 `.gitignore` 或本地覆盖文件。
- 新 TB6612 上电前：**蜂鸣档测 VCC–GND**，避免再次烧毁 ESP32 稳压器。
- 长期建议：使用带 **板载 USB** 的 ESP32-DevKitC，减少外接 CP2102 与 GPIO12 风险。

---

*本报告由仓库当前文件与对话上下文整理；若代码有变更，请以实际源文件为准。*
