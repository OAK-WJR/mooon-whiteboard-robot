# 外置充电 C6 版固件与 USB 控制

2026-09-09 UTC。独立环境 **`top_board_v2_kiss_c6`** 和上位机 **`--usb-kind ch343g`** 已迁移到当前 C6-MINI 六页原理图。最新 **73 项离线检查及 C6/R3 两项交叉编译均通过（报告共 75 行）**，绑定当前原理图和网表。供电与 PCB 未完成，运动保持禁用；没有刷写或实板测试。

## 本次实现

固件身份为 `top_board_v2_kiss_c6_review`。UART0 使用 TX16/RX17，电机电源请求 GPIO14 在启动和运行期间保持 LOW。六路 FI/BI 先写 LOW，再启用输出；审查版不连接 PWM 外设，底层驱动也拒绝非零转速，避免未完成的电源电路承受启动过程的不确定输出。

MINI 不引出 GPIO10/11：ARM 改用 GPIO14，M0_BI 改用 GPIO23；其余逻辑 GPIO 保持。三组驱动和编码器按 Motor1→M0、Motor2→M2、Motor3→M1 对应。固件可报告原始编码器计数；最终减速比、编码器方向和轮径尚未标定，速度与位置返回 null，`odometry_qualified=false`。两路电压已实现芯片校准转换，通过独立 `rail_status` 命令报告；普通状态中的电压仍为 null，电流尚未测量。外部分压校准和掉电行为未完成，见[电压诊断及测试边界](ADC_READOUT.md)。

新环境使用公开配置并覆盖引脚，拒绝其他板型、S3、原生 USB CDC 和电机测试入口。验证在临时公开源码副本中执行，其中 `config.h` 被特意设为编译错误哨兵；真实私有配置未被读取或复制。原生编译依赖记录也未包含该文件。

GPIO14 初始化或后续写低失败时，程序锁存故障并尝试释放输出。无线尚未启动时保留 USB 故障诊断与重启；无线已启动时自动复位并重新执行供电判断。初始错误和释放错误分别保留；写低与状态日志不能证明实物电机电源已经断开。

无线已改为按电压条件启动，供电失效后自动复位切换模式，见[无线供电策略及限制](WIRELESS_POWER.md)。无线初始化前已注册看门狗；初始化失败通过 USB 报告，不在同一启动周期重复分配。

## 上位机入口

以下为实现后的接口形式，`PORT` 需替换为目标设备实际本地串口。本次验证没有执行这些实物命令。

```sh
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind ch343g status
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind ch343g rails
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind ch343g reboot
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind ch343g reset
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind ch343g bootloader
```

`reboot` 请求应用重启，并校验同一设备、不同启动标识、软件复位原因及空闲/零输出。`reset` 通过 C6 ROM 读取 MAC 后复位，校验桥接设备身份和新板固件返回；未观察到复位前应用时，不声称已经比对新旧启动标识。`bootloader` 留在 ROM，不写入或擦除 Flash，也不声称已经确认定制 PCB 的身份。

CH343G 与旧 CH343P 共用 CH343 家族 USB ID，因此必须显式区分 `ch343g`（本版 C6）与 `ch343p`（旧 Pico/S3）。错误或含混的设备身份、旧板固件和错误能力字段会被拒绝。Wi-Fi/BLE 使用同一认证协议，需显式指定 `--profile top_board_v2_kiss_c6_review`；密钥配置方式沿用[控制说明](../../../../../control_tests/README.md)。

ROM 子进程指定 `esp32c6`、115200 波特、UART 复位策略，并通过临时配置固定 `reset_delay=0.05`，不继承用户自定义复位脚本。测试执行项目已安装 esptool 5.3.0 的 `ClassicReset` / `UnixTightReset` 及 `HardReset`，通过模拟串口记录确认 100ms EN 复位、随后 50ms BOOT 保持与正常退出序列。它保留 esptool 对操作系统线控制的处理；真实 USB 驱动时延仍需测量。[Espressif 配置说明](https://docs.espressif.com/projects/esptool/en/latest/esp32/esptool/configuration-file.html)、[5.3.0 复位实现](https://github.com/espressif/esptool/blob/v5.3.0/esptool/reset.py)。

## 验证结果与边界

- **39 个新板运行场景通过**：执行生产主循环、驱动、编码器 ISR 和通信实现；SDK、GPIO、时钟、UART、无线及 NVS 边界使用替身。覆盖四种 ARM 故障、12 种电机 GPIO 初始化故障、USB/Wi-Fi/BLE 重启、电压报告、ADC 采样跨越租约期限、启动和状态报文长度。
- 另有 **9 组 ADC 边界测试**，覆盖两路对应关系、校准缺失、读数超时、饱和与量程异常、过期、时钟回绕和资源释放错误。
- **6 组无线供电策略边界测试**及原共享传输测试通过，覆盖等待有效电压、时间回绕、初始化错误、复位标记、分片、旧连接数据丢弃及通知失败。
- 编译后实际 GPIO 操作和编码器计数变化已与本目录 **KiCad 导出网表**比较；六个输入均只改变对应实体电机的计数。四种不兼容构建被编译器拒绝。
- 原生 USB、旧 R3 UART、新 C6 UART 的共享指令回归通过，包含验签期间租约到期、重放、错误签名和永久运动门禁。R3 的 LOW 初始化和驱动启动回归保持通过。
- **131 项上位机测试通过**，包含 8 项 CH343G 测试和 9 项电压报告测试；实物串口和 ROM 子进程均被替换。控制协议测试包含本机 UDP 回环，不是实际 Wi-Fi/BLE 链路测试。新固件实际序列化的状态及电压报告也被上位机接受。
- 电压诊断实现后，新 C6/CH343G 和保留 R3/C6 均交叉编译通过。计数值扩展场景的状态载荷为 **1185 字节**、带认证封装为 **1399 字节**，通过现有缓冲区检查；不代表所有未来新增字段自动适用。

最新命令、日志、源文件哈希及范围见 [验证报告](firmware_verification/report.json)。仅离线复查可省略 `--build`；以下命令同时复现编译和离线检查：

```sh
.venv/bin/python control_tests/verify_kiss_c6.py --build
```

尚未完成供电/默认停机硬件、ADC 掉电处理和外部分压校准、无线供电切换实物验证、实际 USB 枚举和恢复、底板接口负载验证、无线共存/距离、PCB 布线和完整成本核算。软件审查版不是制造或运动放行。
