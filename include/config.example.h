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
// 然后在 config.h 中填入真实的 WiFi 凭证 (WIFI_SSID / WIFI_PASS)。
// config.h 已被 .gitignore 忽略，不会提交到仓库。
//
// 调参提示：下面的 PID / 速度 / 前瞻是上电默认值；运行时可用串口控制台
// 实时改 (见 src/main.cpp 顶部命令表)，调好后再把满意的值写回这里固化。
// ============================================================

// --- 全向轮参数 (38mm omni wheel) ---
#define WHEEL_DIAMETER      0.038f   // 轮直径 (m), 38mm
#define WHEEL_RADIUS        0.019f   // 轮半径 (m), 19mm
#define WHEEL_WIDTH         0.02342f // 轮宽 (m), 23.42mm
#define ROBOT_RADIUS        0.033738f// 轮心到底盘中心距离 (m), 33.738mm
#define ENCODER_CPR         700      // N20 编码器 7PPR * 1:100 减速 = 700 CPR

// --- RZ7899-MS 驱动 (每颗单 H 桥, 2 引脚控制: FI + BI) ---
// 正转: FI=PWM, BI=LOW    反转: FI=LOW, BI=PWM
// 滑行停止: FI=LOW, BI=LOW；RZ7889-MS 主动刹车为两脚 HIGH

#ifdef CONFIG_IDF_TARGET_ESP32C6
// ===================== ESP32-C6 (转接板 + Waveshare C6-LCD-1.47) =====================
// 索引 = 物理电机号 (motor[0]=Motor1, [1]=Motor2, [2]=Motor3), 与运动学一致。
// 注意: 转接板排针与底板座子的实测配对是错位的 —— M0头→底板H2(电机1)、
//       M1头→底板H1(电机3)、M2头→底板H3(电机2)。所以这里按物理电机顺序绑定引脚,
//       [1] 取 M2 头的脚、[2] 取 M1 头的脚, 运动学矩阵无需改动。
// C6 禁用脚: GPIO12/13(USB-Serial/JTAG 烧录), GPIO9(BOOT), 6/7/8/14/15/21/22(屏幕板占用)

// 电机 0 = 物理 Motor1 (走转接板 M0 头 -> 底板 H2 -> controller1)
#define MOTOR0_FI_PIN       0        // GPIO0
#define MOTOR0_BI_PIN       1        // GPIO1

// 电机 1 = 物理 Motor2 (走转接板 M2 头 -> 底板 H3 -> controller2)
#define MOTOR1_FI_PIN       4        // GPIO4
#define MOTOR1_BI_PIN       5        // GPIO5

// 电机 2 = 物理 Motor3 (走转接板 M1 头 -> 底板 H1 -> controller3)
#define MOTOR2_FI_PIN       2        // GPIO2
#define MOTOR2_BI_PIN       3        // GPIO3

// --- 编码器引脚 (AB 相), 同样按物理电机顺序 ---
#ifdef WBB_TOP_BOARD_V2
#define ENC0_A_PIN          7        // New integrated board: GPIO16 remains UART TX.
#else
#define ENC0_A_PIN          16       // GPIO16 (UART0 TXD, 经转接板 100Ω 串阻)
#endif
#define ENC0_B_PIN          17       // GPIO17 (UART0 RXD)
#define ENC1_A_PIN          20       // GPIO20  <- M2 头
#define ENC1_B_PIN          23       // GPIO23  <- M2 头
#define ENC2_A_PIN          18       // GPIO18  <- M1 头
#define ENC2_B_PIN          19       // GPIO19  <- M1 头

#else
// ===================== ESP32 (旧的一体板, 保留以便回退) =====================
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
// motor1 编码器 (Motor2 connector) — v2 板走线改到 GPIO22/23 (普通脚, 有内部上拉)
#define ENC1_A_PIN          22       // GPIO22 - Motor2 Enc A (内部上拉, 免外部电阻)
#define ENC1_B_PIN          23       // GPIO23 - Motor2 Enc B (内部上拉, 免外部电阻)
// motor2 编码器 (Motor3 connector) — 实测确认
#define ENC2_A_PIN          19       // GPIO19 - Motor3 Enc A
#define ENC2_B_PIN          18       // GPIO18 - Motor3 Enc B
#endif

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
// 静摩擦破除 (实测破静摩擦 PWM, 2026-06-17, 开环阶梯法): M0=100 M1=140 M2=140。
// 仅在轮子卡住(|实际速度|<MOTOR_BREAKAWAY_VEL)时叠加, 掀动后撤掉,
// 避免持续偏置在空转时过冲/引入旋转。
#define MOTOR_STICTION_M0   100.0f
#define MOTOR_STICTION_M1   140.0f
#define MOTOR_STICTION_M2   140.0f
#define MOTOR_BREAKAWAY_VEL 0.02f

// --- 堵转保护 ---
// 判据: PWM 已经很大(在用力) 但实际速度几乎为 0(没动) 且持续若干拍 -> 判定堵转。
// 撞墙/笔卡住/轮子压线时, 避免 PID 把 PWM 长期顶满(那是所有发热与过流的最坏工况)。
#define STALL_PWM_THRESH    200      // |PWM| 超过此值才算"在用力"
#define STALL_VEL_THRESH    0.01f    // |实际轮速| 低于此值算"没动" (m/s)
#define STALL_TRIP_TICKS    25       // 连续多少拍判定 (50Hz -> 500ms)
#define STALL_COOLDOWN_MS   1000     // 触发后停多久再自动重试
#define STALL_MAX_TRIPS     3        // 连续触发这么多次后锁定该轮, 等上位机干预

// 轮速合理性闸: 实测最快轮速 ~0.38 m/s, 超过此值判为编码器噪声/串扰直接丢弃。
// 主要挡 M1 电机 PWM 串扰进 ENC1(GPIO36/39) 造成的幽灵恒速 (~0.58 m/s)。
#define MAX_WHEEL_VEL       0.45f

// --- FF + PID 参数 (每个电机) ---
// 注意: 更换驱动芯片后 Kf 可能需要重新标定
// 全局缺省 = 自动调参 M1 实测结果 (kf_true=636)
#define PID_KF              636.0f   // 前馈增益 = 电机直流增益 PWM/(m/s)
#define PID_KP              63.4f    // 比例增益
#define PID_KI              3.9f     // 积分增益 (消稳态误差)
#define PID_KD              0.0f     // 微分增益 (50Hz 下 Kd>0 易引发极限环, 自动调参取 0)
#define PID_INTEGRAL_LIMIT  250.0f   // I项输出限幅

// 每个电机独立 PID（默认先与全局一致，后续可单独调）
// 每个电机独立 PID —— 自动调参 (tools/autotune.cpp), 三轮各不相同。
// 真机实测 kf_true (开环 PWM=200, 2026-06-16): M0=504 M1=636 M2=515。
// Kf 直接 = 各自实测直流增益; Kd=0 避免 50Hz 极限环; Ki 消稳态。
// M0: 手动调参 (PID Tuner 实测调出)
#define PID_KP_M0           500.0f
#define PID_KI_M0           10.0f
#define PID_KD_M0           50.0f
#define PID_KF_M0           470.0f
#define PID_ILIM_M0         250.0f

// M1: 闭环温和增益。注意 M1 编码器(ENC1 GPIO36/39)有 PWM 串扰, 闭环会抖;
// 但开环又没法控速(跑太快)。两难的根在编码器, 必须硬件修(上拉+分线)才能既顺又准。
// 这里先用闭环(速度受控), 抖是编码器的锅。
#define PID_KP_M1           63.4f
#define PID_KI_M1           3.9f
#define PID_KD_M1           0.0f
#define PID_KF_M1           636.0f
#define PID_ILIM_M1         250.0f

#define PID_KP_M2           108.0f
#define PID_KI_M2           7.7f
#define PID_KD_M2           0.0f
#define PID_KF_M2           515.0f
#define PID_ILIM_M2         250.0f

// --- 控制频率 ---
#define CONTROL_RATE_HZ     50       // 控制/里程计更新频率
#define CONTROL_PERIOD_MS   (1000 / CONTROL_RATE_HZ)

// --- 工作模式 ---
// 0 = USB 串口模式 (默认): 不启 WiFi/UDP, 命令走串口, 位姿走 'j' 流, 调参走 pid_tuner。
//     启动不阻塞, 无 WiFiUdp 报错, 且消除 ENC1(GPIO36/39) 射频噪声。
// 1 = WiFi 模式: 连 WIFI_SSID/PASS, 走 UDP 与上位机通信。
#define USE_WIFI            0

// --- WiFi（USE_WIFI=1 时请填入真实值）---
#define WIFI_SSID           "YOUR_WIFI_SSID"
#define WIFI_PASS           "YOUR_WIFI_PASSWORD"

// --- 自主模式 WiFi UDP（上位机 whiteboard_app.py 通信）---
#define UDP_LISTEN_PORT     8888    // ESP32 监听端口（接收命令）
#define UDP_REPORT_PORT     8889    // ESP32 回传端口（状态）
#define STATUS_RATE_HZ      10      // 状态回传频率 (Hz)

// --- 轨迹跟踪 ---
#define LOOK_AHEAD_DIST     0.06f   // 前瞻距离 (m)
#define GOAL_TOLERANCE       0.02f   // 到达判定距离 (m); 翻台无负载下 8mm 停不进, 2cm 是甜点
#define MAX_LINEAR_SPEED     0.15f   // 最大线速度 (m/s)
#define KP_LINEAR            2.0f    // 线速度比例增益
#define MAX_WAYPOINTS        200     // 最大路径点数
// 终端减速: 距目标 END_DECEL_DIST 内开始按距离线性减速到 MIN_APPROACH_SPEED,
// 避免冲过头绕圈, 让机器人爬进容差停住。
#define END_DECEL_DIST      0.08f   // 终端减速起始距离 (m)
#define MIN_APPROACH_SPEED  0.04f   // 终端最低速度 (m/s)

// 航向保持: 主动用角速度把 theta 锁回 0, 抵消旋转漂移让点到点走直线。
// 默认关 (KP_ANGULAR=0 -> omega=0): 翻台/无负载下 odom theta 不可信, 开了会自旋。
// 落地 + M1 编码器干净(theta 可信)后改成 ~3.0 启用, 才能拉直。
#define KP_ANGULAR          0.0f    // 航向保持比例增益 (0=关; 落地可信后设 3.0)
#define MAX_ANGULAR_SPEED   1.5f    // 最大角速度 (rad/s)

// R3 environments force these public defaults before private config.h is read.
// Wireless credentials are provisioned to NVS by the integrated runtime.
// The profile is intentionally unqualified for both three-6V and three-12V sets.
#ifdef WBB_TOP_BOARD_R3
#include "board_profile_r3.h"
#endif

#endif // CONFIG_H
