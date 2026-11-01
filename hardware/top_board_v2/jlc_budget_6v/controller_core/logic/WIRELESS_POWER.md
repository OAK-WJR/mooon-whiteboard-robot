# 顶部板的无线供电策略

C6 审查固件已把无线启动与电池接口电压联系起来，保留 USB 诊断、下载与自动恢复。2026-10-29 起适用于当前 EasyEDA 板 `top_board_v2_logic`：原生 USB（无 USB-UART 桥）、板上 IP2326 充电。电压是 IO4 分压的诊断读数，整板供电及实物无线性能仍未合格。

## 运行方式

| 条件 | 固件行为 |
| --- | --- |
| 上电／复位后 | 原生 USB CDC 和电压采样先工作；不初始化蓝牙或 Wi-Fi。USB `status`、`rails` 和重启可用 |
| 有效电池读数连续满足条件 | 读数处于 6.5–8.8V，连续采样跨度至少 1 秒，相邻采样开始时刻相差不超过 150ms；看门狗可用且没有待处理重启时，才启动无线 |
| 启动无线 | 先撤销已有控制租约；每次启动周期仅初始化一次 BLE，有有效密钥和 SSID 时也启动 Wi-Fi；客户端重新取得控制租约。初始化返回后立即复查采样，不会因样本过期误判断电。已知问题：正常路径没有在初始化后刷新控制周期计时，约 300ms 的阻塞被记为一次 `CONTROL_OVERRUN`，`control_timing_fault` 保持到重启（见下） |
| 无线已启动 | 有效读数在 6.0–8.8V 范围内可继续运行；启动与保持阈值分开，避免边界反复切换 |
| 运行中电池读数越界、ADC 失败／过期 | 在处理下一批通信数据之前撤销控制、请求安全输出，并执行软件复位；下一次启动重新判断供电条件 |
| GPIO 安全输出失效（`gpio_fault` 已锁存） | 不因故障本身复位，保持只读：采样继续，无线按同样的电池条件启动，供电失效时照常复位。Wi-Fi/BLE 只回答 `hello`、`status`、`rail_status`、`stop`，`reboot` 仅限 USB。故障循环在 `transport.begin()` 返回后立即重新采样；在 `RADIO_STARTING` 时发现的 GPIO 故障也不触发供电复位 |
| 无线初始化失败 | 报告 `init_failed`，不在同一启动周期反复初始化；USB 诊断及用户／AI 重启入口保留。部分 SDK 初始化可能已完成，仍持续监控供电 |
| 应用重启正在等待发送回复 | 继续电压采样；供电正常时保留原回复等待流程，供电失效时优先自动复位，回复可能未发完 |

软件复位会重新执行启动程序，并复位 Wi-Fi、BT 等指定外设；它不是切断 PCB 电源。USB 是 C6 自带的 USB Serial/JTAG，没有独立桥接器，因此 `esp_restart()` 和 SW1 断电时主机串口如何断开、重新枚举，取决于 MCU 本身，需实板测量。[乐鑫 `esp_restart()` 说明](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32c6/api-reference/system/misc_system_api.html#_CPPv411esp_restartv)。

本项目安装的 BLE 库在初始化中等待主机同步，并有额外 200ms 延时。这里使用一次初始化及软件复位切换模式，不反复调用 `BLEDevice::deinit` 重建异步对象。任务看门狗在无线初始化之前已经注册；初始化返回后立即复查采样。SDK 阻塞期间没有连续电压采样，因此不能据此保证拔电池后的最大关断延迟或 USB 瞬态电流。

**已知问题（2026-10-29，固件待修）：** 正常路径启动无线后没有刷新 `lastControlAt`，同一轮的控制周期检查看到约 300ms 的间隔：`control_overruns` 加 1，`control_timing_fault=true` 保持到重启，`reason` 改写为 `CONTROL_OVERRUN`（初始化失败时也覆盖 `TRANSPORT_INIT_FAILED`）。每次电池合格、无线启动后的 `status` 都是这样，[status.json 样本](firmware_verification/status.json)即为此结果（`control_gap_ms` 310）。运动本就禁用，输出不受影响；初始化结果以 `rails` 的 `wireless_power_state` 为准。修正是在 `transport.begin()` 返回后刷新 `lastControlAt`。故障循环不运行控制周期，不受此影响。[乐鑫 BLE 实现](https://github.com/espressif/arduino-esp32/blob/master/libraries/BLE/src/BLEDevice.cpp)。本地实际编译版本和哈希见下方验证记录。

## USB 可读诊断

```sh
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind c6usb rails
```

原电压报告增加 `wireless_power_state`、`wireless_start_attempted`、`previous_radio_power_reset`。前两项区分等待电压、确认中、启动中、运行和初始化失败；“已尝试启动”不代表无线连接成功。最后一项使用 RTC no-init 的标记及反码，只在软件复位原因匹配时接受，并在本次启动消费清除，供 AI 识别前一次自动供电复位。它不写 Flash、不保证完整掉电后保留，也不是详细故障波形记录。

## 验证与未完成项

2026-09-09 的生产主循环测试覆盖 USB 单独供电、缺少 ADC 校准、看门狗不可用、电压波动后重新计时、运行中断电／ADC 故障／过期／GPIO 故障、初始化期间断电、四种 SDK 初始化失败，以及复位标记的接受和拒绝；其中 GPIO 故障当时按复位处理，现已改为上表的只读行为。纯策略测试还覆盖时间回绕和重复使用旧样本。原 USB/Wi-Fi/BLE 重启、认证、分片及共享 R3 回归保留；当时的来源与范围见[记录](verification/wireless_power.json)。所有故障输入和复位都是测试替身，尚未证明实物 RTC 保持或无线链路性能。当前验证方式见[固件说明](FIRMWARE.md#验证)。

6.5V／6.0V／8.8V 是针对当前 2S 电池接口的**软件读数阈值**，外部分压未逐板校准，不是电芯保护阈值、USB 电流额度或 6V 稳压保证。单个 ADC 卡在合理数值仍可能漏检；输入保护、ADC 掉电行为、USB 枚举与挂起、插拔和浪涌、射频／电源瞬态、整板布线及完整成本仍需完成。运动保持禁用，不能称为整板稳定性验证通过。
