# 顶部板 ADC 读出：电池电压与电机电流

2026-10-29，当前 EasyEDA 板 `top_board_v2_logic`。审查固件通过 `rail_status` 报告 ADC1 的两路读数：

| 通道 | 网络 | 电路 | 衰减／量程 | 报告字段 |
| --- | --- | --- | --- | --- |
| GPIO4／U3.9 | VBAT_SENSE | R33 120k／R34 10k，名义倍率 13 | 0dB，0–1000mV | `battery_voltage_mv` |
| GPIO5／U3.10 | MOT_ISENSE | INA180A1（U7，增益 20）测 R49 0.1Ω，经 R36 1k／C21，2V/A | 12dB，0–3300mV | `motor_current_ma` = `adc_pin_mv[1] // 2` |

IO5 自 2026-10-28 起是电机电流，不再是电机电压分压；`motor_voltage_mv` 恒为 null。实板精度和掉电行为尚未验证，运动仍保持禁用。

上位机入口如下，`PORT` 替换为实际串口；本次未连接设备或执行此实物命令：

```sh
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind c6usb rails
```

Wi-Fi/BLE 使用相同只读命令，需显式选择 `--profile top_board_v2_kiss_c6_review` 并通过既有认证；不需要运动控制租约。普通 `status` 中电机电压、电流仍为 null，详细读数单独通过 `rails` 返回，以保持现有状态报文长度。

## 转换与采样

转换使用 ADC1、12 位和每通道的芯片校准接口；VBAT 用 0dB，电流用 12dB。校准初始化失败时报告错误，不用名义参考电压代替。外部分压未逐板校准，因此报告保留 `divider_calibrated=false`、`hardware_qualified=false`。[乐鑫 ADC 采样文档](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32c6/api-reference/peripherals/adc_oneshot.html)、[校准文档](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32c6/api-reference/peripherals/adc_calibration.html)。

初始化后等待至少 10ms，每次采样间隔至少 100ms，每通道取 8 次读数，两路完成后一起发布。以下任一情况都会使整对读数不可用：读取或转换失败、超量程、VBAT 饱和、采样跨度超过 25ms。有效期为采样开始后 250ms。不可用时电压、电流、原始值和样本时间字段均为 null，`motor_current_saturated` 不出现，同时保留故障阶段和 SDK 错误码。25ms 检查发生在 SDK 调用返回后，不能中断底层调用，也不是硬实时保证。

电流通道到轨时不作废整对读数。任一样本校准后 ≥3150mV，或原始值为 4095，就置 `motor_current_saturated=true`，表示实际电流不小于报告值；原始值 4095 或高于 3300mV 的样本按 3300mV 计。INA180 输出最高约为 3V3 减 0.03V，增益误差只规定到约 1.4A（VS−0.5V），约 1.6A 到轨。

## 供电来源

`usb_power` 读取 IO8（USB/充电器 5V 存在时为高；无 USB 时 R10 经 Q1 把它拉到约 0.72V 以下，见[固件说明](FIRMWARE.md#io8-usb-供电检测)）。`charge` 在 `usb_power` 为真时为 `"external_power"`，否则为 `"battery"`。IP2326 的状态脚没有接到 MCU；VBAT 未校准，误差约 ±150mV，而 CV 电压约为 8.3V。因此固件不推断充电中或已充满，只报告电池电压。

## 故障与复位

采样前后均检查控制租约。GPIO 故障锁存后采样继续，只读状态经 USB 和无线都保持完整。应用重启等待发送回复期间也继续监控电压。无线尚未启动时，ADC 错误保留 USB 诊断和用户重启；无线已启动后，ADC 失效或过期会触发自动复位，下一次启动重新判断供电，见[无线供电策略](WIRELESS_POWER.md)。

还需完成外部分压校准、ADC 掉电处理、USB 供电与挂起、整板布线和实物验证。软件阈值和自动复位不能证明 USB 供电已经合格。

## 历史：2026-09 KiCad 版

本目录 KiCad 六页图的 IO5 是电机电源 120k/10k 分压，两路都用 0dB，读数为两路电压。当时原理图只更新注释和 ADC 实现状态，70 个元件的参数、封装及连接关系未变，旧输入与注释版本的对应关系见[证据衔接记录](verification/adc_revision_evidence.json)。当时的软件报告（75 项记录）和测试电压样本（模拟输入 7.8V／5.98V）已被当前验证覆盖，旧版见 git 历史。现在的 [rails 样本](firmware_verification/rails.json) 同样来自模拟输入，不是 PCB 实测。
