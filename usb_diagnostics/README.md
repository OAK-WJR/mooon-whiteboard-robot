# USB 诊断

默认只采集数据，电机不运行。运行时每 100ms 输出 JSON，主机把原始帧存为 `telemetry.jsonl`，结论存为 `report.json`。这提供可复查的故障证据，不能使 MCU 自己识别所有 PCB 故障。

## 快速使用：现有 C6 转接板

在 WhiteBoardBot 根目录执行，先让轮子离开接触面并断开电机电源，再连接 USB 数据线：

```sh
./.venv/bin/pio run -d usb_diagnostics -e adapter -t upload
./.venv/bin/python scripts/pcb_diagnostics.py --list
./.venv/bin/python scripts/pcb_diagnostics.py --seconds 10
```

仅有一个 Espressif USB 串口时自动选择；有多个时加 `--port` 指定 `--list` 中的设备。上位机会核对固件协议和板型，不会向不匹配的程序发送电机测试指令。报告默认写入当前目录的 `diagnostic-reports/时间/`。

新板的编译命令如下，**目前只开放无运动的诊断**；其电源尚未定型，不能用于打样验收或直接接 12V：

```sh
./.venv/bin/pio run -d usb_diagnostics -e top_board_v2
```

`adapter` 的 ENC0_A 是 GPIO16；新板草稿 `top_board_v2` 的 ENC0_A 是 GPIO7。两个配置不能混用。旧板 GPIO16 在 ROM 阶段可能输出 UART；固件进入 `setup()` 后改成输入，不能倒过来消除 ROM 阶段的硬件冲突。

诊断固件现支持 USB `reboot`，会先停止输出、回复接受状态，再重启。AI 可调用 `scripts/pcb_control.py reboot --port PORT --profile top_board_v2`（旧转接板用 `adapter`）；上位机验证新会话、复位原因和各路空闲，不能仅凭 ACK 判断成功。`reset` / `bootloader` 调用 esptool 原生 USB ROM 恢复，不擦写 Flash；USB 外设或供电本身失效时不保证可恢复。

集成板另有 USB/Wi-Fi/BLE 主固件，使用不同的 `scripts/robot_control.py` 协议，见 [统一控制说明](../control_tests/README.md)。当前 USB 诊断固件保持协议 1，主固件是协议 2，二者不能混用。

## 电机台架测试

当前没有板载电压/电流检测。带电测试必须使用**与已验证的电机/板额定值匹配的外部限流电源**，手测电机端电压，并让轮子悬空。先断电做静态检查，测试参数从低占空比开始。`--bench-current-limit` 只是记录你实际设置的电源限流值，脚本不会替你设置电源，也不会测到这个电流。

命令格式如下，尖括号内容需要换成实际确认的值：

```text
./.venv/bin/python scripts/pcb_diagnostics.py --run-motors \
  --motor-voltage <6或12> --supply-voltage <实测伏特> \
  --bench-current-limit <已核算并设置的安培数> --duty 64 --pulse-ms 500
```

脚本依次测试三轮的正/反向响应、未驱动通道的变化、编码器 A/B 相和计数方向，再测试停止心跳后的软件停机。每次脉冲前等待编码器静止；任何一项需要检查就取消后续动作。只有单轮测试通过后，才可按已核算的总启动电流增加 `--all-at-once` 测三轮同时启动。

每次运行最多 1000ms；心跳失联 350ms 自动撤销命令，正常到时也解除使能，下次运行必须重新 arm。USB 断开、接收溢出或非法命令使电机控制脚恢复低电平。低/低对应驱动器高阻滑行，**不是机械急停，也不表示电机电源已经物理断开**。现有 v2 草稿的 MT3608 EN 不能替代电源隔离。

## 报告能说明什么

| 证据 | 可以判断 | 仍不能独自判断 |
|---|---|---|
| 协议、帧号、会话号、复位原因 | 通讯完整性、重启、应用是否活着 | 完全无枚举时到底是线缆、3V3、焊接还是 MCU 损坏 |
| 每路 PWM 初始化状态 | 六个 LEDC 输出是否成功分配 | 驱动输入焊盘及电机输出端是否真的出现正确波形 |
| A/B 电平、两相边沿、带方向计数、非法跳变 | 电机动作响应、单相缺失迹象、方向反转、串扰迹象 | 无响应究竟由欠压、占空比、机械负载、驱动器还是编码器引起 |
| 软件停止原因和命令归零 | 失联/超时后软件停止逻辑 | 功率管短路、物理断电或实际轮子是否已停止 |
| `motor_voltage_mv/current_ma/rail_3v3_mv = null` | 当前无对应检测能力 | 不得解读为 0V、0A 或供电正常 |

`USB_CAPTURE_PASSED` 只代表这次 USB 采集通过；`BENCH_CHECKS_PASSED` 只代表执行的台架检查通过。它们均不是整板制造放行结果。`REVIEW` 保留歧义，不会把“编码器没变”直接写成“ESP32 坏了”。

## 新 PCB 完整验收矩阵

| 阶段 | 方法与记录 | 通过条件 |
|---|---|---|
| 断电 | 万用表/夹具测输入极性、GND、3V3、VMOT、每个排针及电机线束；记录阻值 | 和定型网表一致，无不应存在的短路/开路；不靠 MCU 推挽输出测短路 |
| USB 独立供电 | 电池/电机电源移除，USB 两个插入方向、数据线、冷启动/复位/BOOT 下载各验证 | MCU 稳定枚举、重复上传成功；3V3 实测在规格内；USB 不给电机供电、不反灌外部输入 |
| 两种电压 | 按最终规格分别上 6V、12V，对照万用表校准电压采样 | 实际 VMOT 匹配电机额定电压；不匹配模式禁止运行；芯片引脚均不超限 |
| 电流检测/保护 | 用可控负载和校准仪表检查电流采样，再验证硬件限流/切断、故障锁存 | 在设计阈值及响应时间内动作；逻辑/USB 继续工作并报告故障；不得用 GPIO 或任意裸短接做故障注入 |
| 手转编码器 | 电机不通电，依次手转三轮，USB 记录两相边沿和方向 | 每路两相有效；独立通道无异常跳变；实测方向和物理电机映射记录在配置中 |
| 单轮运行 | 每轮正反转、分级占空比；USB + 示波器检查 FI/BI/FO/BO 与编码器 | 三轮都响应；非目标通道不动；反向计数变号；波形及相位符合手册 |
| 同时启动 | 三轮同时启动、负载变化；示波器记录 VMOT、3V3、复位脚，USB 记录电流和编码器 | 无超规格跌落/尖峰、无复位；总电流及所有接点温升在批准的设计额定值内 |
| 停机/回生 | 断心跳、拔 USB、复位 MCU、关电机电源；检查真实驱动输出和 VMOT 瞬态 | 默认不驱动；物理断电级按设计动作；回生/热插拔电压不超过设计上限 |
| 热与堵转 | 参数确定后，在受控夹具和限流条件下按批准的电流/时间测试 | 驱动、稳压器、接头、铜线温升和保护动作合格；当前未知堵转电流，不能预填“通过” |
| 故障注入 | 断开单相/电机连接、模拟电压/电流告警、USB 报文故障；记录预期与实测 | 可观测故障被报告且停止条件满足；检测盲区写入报告；不用反复“重试启动”掩盖失败 |

最终 PCB 必须补充电压/电流反馈、默认关闭的功率开关、独立硬件保护和足够的测试点。共用电流采样可辅助逐轮定位；若要同时运行时直接分辨每个驱动的电流/输出故障，需三路采样或改底板的测试接口。高速尖峰和 USB 信号质量仍需示波器，USB 日志不能替代它。

## 软件验证

```sh
c++ -std=c++17 -I usb_diagnostics/include usb_diagnostics/tests/test_safety.cpp -o /tmp/wbb-diag-safety
/tmp/wbb-diag-safety
./.venv/bin/python -m unittest discover -s usb_diagnostics/tests -p 'test_*.py' -v
c++ -std=c++17 -fsanitize=address,undefined \
  -I usb_diagnostics/tests/mock -I usb_diagnostics/include \
  usb_diagnostics/tests/test_firmware.cpp -o /tmp/wbb-diag-firmware
/tmp/wbb-diag-firmware
```

最后一项执行实际固件的命令解析器和循环，模拟 USB/GPIO/PWM，覆盖丢心跳、USB 断开、换行前的长报文溢出、PWM 故障和运行中重启。完整两板型与主机验证可执行 `.venv/bin/python control_tests/verify.py`。输出是**模拟数据**，本轮没有连接到实际 ESP32，没有实测 USB 下载、转动电机、电流或温升。
