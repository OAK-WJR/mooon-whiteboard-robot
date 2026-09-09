# 验签期间控制租约到期：停止遗漏修正

2026-09-08（America/Los_Angeles）。当前预算版尚未定稿；本次修正的是各集成板版本共用的运行固件，不代表新增 PCB 已通过审核。

`command()` 入口检查 750ms 控制期限，随后解析 JSON 并验签。如果期限在这段处理期间到达，`Session::accept()` 会清除控制权并返回 `LEASE_EXPIRED`。旧处理器直接回包，没有调用 `halt()`；下一轮也无法通过已经清空的 owner 找到过期会话。控制器因此可能保留原先的运行状态。

修正后，处理器收到 `LEASE_EXPIRED` 时先调用 `halt("HEARTBEAT_TIMEOUT")`，再发送原有错误响应。租约不会续期，接口错误码保持一致；停止路径也会执行所选板型的电机使能请求 LOW 操作。

回归使用实际命令处理器和 `Session`，在模拟时钟的 749ms 开始验签，向 HMAC 测试适配器注入 1ms 耗时。覆盖 Wi-Fi、BLE 和两种时钟起点，其中一种跨越 32 位毫秒计时回绕。测试直接检查控制器停止调用及运行状态，不只检查错误码或 owner 已清空。未修正代码下，新断言失败（进程退出 -6）：

```text
Assertion failed: (!robot.moving&&robot.stops>previousStops)
```

复现本次修正后的离线验证：

```sh
python3 control_tests/verify_lease_deadline.py --build
```

该命令复用现有公共源码暂存流程，跳过私人配置，并放入禁止被包含的替代文件。验证在临时目录编译，不连接串口或无线设备，不上传固件。

本次结果保存在 [report.json](lease_deadline_results/report.json)：

- 原生 USB 配置及 R3 UART 配置各通过 21 组实际命令/循环场景，包含新的期限边界回归和一万条固定种子的畸形输入。
- 租约/组帧、实际循环时间计算，以及 6 种传输队列/重启回归通过。
- 原生 C++ 测试启用 AddressSanitizer、UndefinedBehaviorSanitizer；C6 `top_board_v2_r3` 交叉编译通过。

原生测试的时钟、SDK、控制器和无线接口使用替身；传输重启回归另执行实际传输源码。本次证明停止处理的控制流，不能证明物理 PWM 电平、H 桥断电、无线时延或电机停转距离。现有电路审核门禁仍然禁止运动；本次没有开放它，也没有完成预算、供电、充电、线束或实板验收。
