#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
// WhiteBoardBot 硬件配置（模板）
// 3x N20 编码电机 + 3x RZ7899-MS 驱动 + ESP32
// 三轮全向轮 120° 配置
// PCB v2: 2026-04-02 网表
//
// 使用方法：
//   cp include/config.example.h include/config.h
// 然后在 config.h 中填入真实的 WiFi 凭证与 Agent IP。
// config.h 已被 .gitignore 忽略，不会提交到仓库。
// ============================================================

// --- 全向轮参数 (38mm omni wheel) ---
#define WHEEL_DIAMETER      0.038f   // 轮直径 (m), 38mm
#define WHEEL_RADIUS        0.019f   // 轮半径 (m), 19mm
#define WHEEL_WIDTH         0.02342f // 轮宽 (m), 23.42mm
#define ROBOT_RADIUS        0.033738f// 轮心到底盘中心距离 (m), 33.738mm
#define ENCODER_CPR         700      // N20 编码器 7PPR * 1:100 减速 = 700 CPR

// --- RZ7899-MS 驱动 (每颗单 H 桥, 2 引脚控制: FI + BI) ---
// 正转: FI=PWM, BI=LOW    反转: FI=LOW, BI=PWM
// 刹车: FI=LOW, BI=LOW

// 电机 0 (controller1, Motor1)
#define MOTOR0_FI_PIN       32       // GPIO32 - controller1 FI (正向输入)
#define MOTOR0_BI_PIN       33       // GPIO33 - controller1 BI (反向输入)

// 电机 1 (controller2, Motor2)
#define MOTOR1_FI_PIN       25       // GPIO25 - controller2 FI (正向输入)
#define MOTOR1_BI_PIN       26       // GPIO26 - controller2 BI (反向输入)

// 电机 2 (controller3, Motor3)
#define MOTOR2_FI_PIN       27       // GPIO27 - controller3 FI (正向输入)
#define MOTOR2_BI_PIN       14       // GPIO14 - controller3 BI (反向输入)

// --- 编码器引脚 (AB 相) ---
// motor0 编码器 (Motor1 connector) — 实测确认
#define ENC0_A_PIN          34       // GPIO34 - Motor1 Enc A (仅输入, 无内部上拉)
#define ENC0_B_PIN          35       // GPIO35 - Motor1 Enc B (仅输入, 无内部上拉)
// motor1 编码器 (Motor2 connector) — 实测确认
#define ENC1_A_PIN          36       // GPIO36 - Motor2 Enc A (仅输入, 无内部上拉)
#define ENC1_B_PIN          39       // GPIO39 - Motor2 Enc B (仅输入, 无内部上拉)
// motor2 编码器 (Motor3 connector) — 实测确认
#define ENC2_A_PIN          19       // GPIO19 - Motor3 Enc A
#define ENC2_B_PIN          18       // GPIO18 - Motor3 Enc B

// --- 编码器方向修正 ---
// 约定: +1 保持原方向, -1 取反
// 用于修正“编码器相位方向”问题，不改变电机驱动方向。
// 若仅电机接线反，通常会看到: sign(PWM) 与 sign(测速) 同号，但物理运动方向仍与期望相反。
#define ENCODER_DIR_M0      (+1)
#define ENCODER_DIR_M1      (+1)
#define ENCODER_DIR_M2      (-1)     // 按当前排查需求: 对 M2 编码器计数取反

// --- PWM 参数 ---
#define PWM_FREQUENCY       20000    // PWM 频率 (Hz)
#define PWM_RESOLUTION      8        // PWM 分辨率 (bits), 0-255
#define PWM_MAX             255

// --- 电机特性 (N20 + 1:100 减速 + 38mm 全向轮 + RZ7899 驱动) ---
// RZ7899 直通 H 桥, RDS(on)=80mΩ, 效率比 TB6612 略高
// 需实测校正: 以下为初始估算值
#define MOTOR_MAX_SPEED     0.40f    // 最大轮线速度 (m/s), 待实测校正
#define MOTOR_MIN_SPEED     0.005f   // 死区阈值 (m/s), 低于此值直接输出 0

// --- FF + PID 参数 (每个电机) ---
// 注意: 更换驱动芯片后 Kf 可能需要重新标定
#define PID_KF              600.0f   // 前馈增益 (实测校正)
#define PID_KP              500.0f   // 比例增益
#define PID_KI              10.0f    // 积分增益
#define PID_KD              25.0f    // 微分增益
#define PID_INTEGRAL_LIMIT  300.0f   // I项输出限幅

// 每个电机独立 PID（默认先与全局一致，后续可单独调）
#define PID_KP_M0           500.0f
#define PID_KI_M0           10.0f
#define PID_KD_M0           25.0f
#define PID_KF_M0           600.0f
#define PID_ILIM_M0         300.0f

#define PID_KP_M1           500.0f
#define PID_KI_M1           10.0f
#define PID_KD_M1           25.0f
#define PID_KF_M1           600.0f
#define PID_ILIM_M1         300.0f

#define PID_KP_M2           500.0f
#define PID_KI_M2           10.0f
#define PID_KD_M2           25.0f
#define PID_KF_M2           600.0f
#define PID_ILIM_M2         300.0f

// --- 控制频率 ---
#define CONTROL_RATE_HZ     50       // 控制/里程计更新频率
#define CONTROL_PERIOD_MS   (1000 / CONTROL_RATE_HZ)

// --- micro-ROS (legacy) ---
#define ODOM_FRAME_ID       "odom"
#define BASE_FRAME_ID       "base_link"

// --- WiFi（请填入真实值）---
#define WIFI_SSID           "YOUR_WIFI_SSID"
#define WIFI_PASS           "YOUR_WIFI_PASSWORD"
#define AGENT_IP            IPAddress(192, 168, 1, 100)  // 上位机/micro-ROS Agent IP
#define AGENT_PORT          8888

// --- 自主模式 WiFi UDP ---
#define UDP_LISTEN_PORT     8888    // ESP32 监听端口
#define UDP_REPORT_PORT     8889    // ESP32 回传端口
#define STATUS_RATE_HZ      10      // 状态回传频率

// --- 轨迹跟踪 ---
#define LOOK_AHEAD_DIST     0.06f   // 前瞻距离 (m)
#define GOAL_TOLERANCE       0.01f   // 到达判定距离 (m)
#define MAX_LINEAR_SPEED     0.15f   // 最大线速度 (m/s)
#define KP_LINEAR            2.0f    // 线速度比例增益
#define MAX_WAYPOINTS        200     // 最大路径点数

#endif // CONFIG_H
