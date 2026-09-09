# 顶部板电压诊断

审查固件已实现 `rail_status`，读取当前顶部原理图的两路电压：GPIO4／U3.9 对应电池接口，GPIO5／U3.10 对应电机电源。使用现有分压器，本次未新增板上元件。实板精度和掉电行为尚未验证，运动仍保持禁用。

上位机入口如下，`PORT` 替换为实际串口；本次未连接设备或执行此实物命令：

```sh
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind ch343g rails
```

Wi-Fi/BLE 使用相同只读命令，需显式选择 `--profile top_board_v2_kiss_c6_review` 并通过既有认证；不需要运动控制租约。普通 `status` 中电机电压仍为 null，详细电压单独通过 `rails` 返回，以保持现有状态报文长度。

转换使用 ADC1、12 位、0dB 衰减和每通道的芯片校准接口。校准初始化失败时报告错误，不使用名义参考电压代替。外部分压名义倍率为 13，电阻误差尚未逐板校准，因此报告明确保留 `divider_calibrated=false`、`hardware_qualified=false`。[乐鑫 ADC 采样文档](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32c6/api-reference/peripherals/adc_oneshot.html)、[校准文档](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32c6/api-reference/peripherals/adc_calibration.html)。

初始化后等待至少 10ms，每次采样间隔至少 100ms，每通道取 8 次读数。两路完成后一起发布；任一路失败、饱和、超量程或采样跨度超过 25ms，整对读数即不可用。有效期为采样开始后 250ms。不可用时电压、原始值和样本时间字段均为 null，同时保留故障阶段和 SDK 错误码。25ms 检查发生在 SDK 调用返回后，不能中断底层调用，也不是硬实时保证。

采样前后均检查控制租约；GPIO 故障会停止采样。应用重启等待发送回复期间继续监控电压。无线尚未启动时，ADC 错误保留 USB 诊断和用户重启；无线已启动后，ADC 失效将触发自动复位，下一次启动重新判断供电。

最新[软件报告](firmware_verification/report.json)共有 75 项记录：73 项离线检查与 C6/R3 两次交叉编译均通过。其内覆盖 9 组 ADC 边界测试、39 个生产运行场景和 131 项上位机测试；这些计数是不同层级，不能相加当成独立测试总数。包含三种传输替身的电压读取、校准缺失、超时、错误通道、量程、过期及租约到期。常规状态认证报文仍为 1399 字节。

[测试电压样本](firmware_verification/rails.json)中的 7.8V／5.98V 来自模拟输入，不是 PCB 实测。还需完成外部分压校准、ADC 掉电处理、USB 供电与挂起、整板布线和实物验证。电压读出已接入[无线供电策略](WIRELESS_POWER.md)，软件阈值和自动复位不能证明 USB 供电已经合格。

本次原理图只更新注释和 ADC 实现状态，70 个元件的参数、封装及连接关系未变，PCB 未改动。既有电路计算、启动仿真和布局报告保留原始哈希及原有适用范围，没有重跑后冒充新结果；旧输入与当前注释版本的对应关系见[证据衔接记录](verification/adc_revision_evidence.json)。新增 ADC 软件测试使用独立的最新固件报告。
