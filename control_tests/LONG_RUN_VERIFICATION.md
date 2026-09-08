# 长时间运行边界修正 — 2026-09-07

本轮修改了主运行固件的编码器计数、里程计输入检查和堵转冷却计时。PCB 仍为嘉立创中的 `motor_controller / top_board_v2_kicad_r2`，电源与实板验证尚未完成；没有烧录或启动电机。

## 缺陷与修正

| 触发条件 | 原行为 | 修正后行为 |
|---|---|---|
| 编码器长期正转/反转到整数边界 | ISR 对有符号 `long` 加减，越界属于未定义行为；反向取反和差值也可能越界 | 内部使用 `uint32_t` 模运算；方向修正、对外有符号窗口和相邻采样差值均显式处理回绕 |
| `Odometry::update` 收到 NaN 或无穷时间值 | NaN 污染轮速和位姿，并持续留在状态中 | 拒绝该次更新，保留位姿、轮速和上次有效计数基准 |
| 堵转冷却的到期时间跨过 `millis()` 回绕，或恰为零 | 绝对时间比较及零哨兵失效；目标板可能提前重试 | 以固定 32 位开始时间计算经过时间，并用独立布尔值记录冷却状态 |
| ISR 与读数/清零同时访问计数 | 只关闭当前核心中断，双核 ESP32 上不能构成完整互斥 | ISR、快照和清零共用 FreeRTOS 临界区；方向转换在临界区外进行 |

编码器遥测 `encoder_counts` 仍是方向校正后的有符号 32 位窗口，范围为 −2147483648 至 2147483647，累计越界后回绕，不是无限增长的总里程。轮速使用模差值；每两次有效采样间的净脉冲变化量绝对值必须小于 2³¹。主运行循环另有 100ms 延迟停机机制。新的计数处理不校准编码器 CPR、方向或电机参数，也不消除电气噪声。

采用模运算的依据是 C++ 对无符号整数和有符号溢出的不同规定；没有依赖超范围的无符号转有符号强转结果。[C++ 工作草案](https://eel.is/c++draft/basic.fundamental)

临界区做法依据已安装 ESP-IDF 5.5.4 的 `task.h` 与 Xtensa/RISC-V `portmacro.h`。官方文档明确说明关闭中断只作用于当前核心，SMP 共享数据应使用带 spinlock 的临界区；C6 单核对应实现仍使用其端口的临界区 API。[Espressif FreeRTOS 文档](https://docs.espressif.com/projects/esp-idf/en/v5.5.4/esp32/api-reference/system/freertos_idf.html#critical-sections)

## 复现与验证范围

修改前的结果保存在 [odometry_before.json](../hardware/top_board_v2/validation/reverification/odometry_before.json) 和 [stall_timing_before.json](../hardware/top_board_v2/validation/reverification/stall_timing_before.json)，均记录被测源码散列。计数溢出由 UBSAN 实际报告，非法时间值测试实际得到 NaN。桌面 `long` 为 64 位，ESP32 为 32 位：旧冷却逻辑在该桌面测试中表现为跨界后仍不重试；目标板到期加法先回绕，也可能提前重试。没有把桌面表现说成目标板实测。

新增测试执行实际 `odometry.cpp`、GPIO 中断回调及 `robot_controller.cpp` / `motor_driver.cpp`：

- 三个 ISR 计数器的正反向全范围回绕。
- 18 个带方向修正的遥测边界，24 个正反转跨界轮速积分案例。
- 通过注册的中断处理器输入 120,000 次模拟 GPIO 边沿，并检查轮速和复位。
- 零、负数、NaN、无穷时间值，以及之后恢复有效更新。
- 三路电机分别测试普通冷却、跨计时回绕、到期时间恰为零，共 9 个冷却案例；检查到期前保持停止、到期后原有重试策略生效，显式 stop 后不再恢复。

执行 `.venv/bin/python control_tests/verify.py` 会运行完整套件及六个固件构建环境；每次生成 [software_verification.json](../hardware/top_board_v2/validation/reverification/software_verification.json)，以其当前散列和结果为准。ASAN/UBSAN 发现问题时直接令检查失败，不只在日志中告警。

这些测试在主机上模拟 GPIO、32 位时钟和互斥 API。它们不是 50 天实机耐久试验，也没有模拟 C6 中断延迟、双核调度、RF 共存、电机温升或真实电源切断。堵转的阈值和有限自动重试策略沿用原配置，尚未做热安全验收；板级启动限制仍保持 `BOARD_POWER_UNREVIEWED`。
