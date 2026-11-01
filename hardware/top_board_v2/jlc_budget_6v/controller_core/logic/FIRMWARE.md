# KISS C6 固件与 USB 控制（top_board_v2_logic）

2026-10-29。环境 **`top_board_v2_kiss_c6`** 面向当前 EasyEDA 板 `top_board_v2_logic`：ESP32-C6-MINI-1、原生 USB、板上 IP2326 2S 充电。引脚以 [`include/board_profile_kiss_c6.h`](../../../../../include/board_profile_kiss_c6.h) 和 EasyEDA 网表 [`hardware/top_board_v2/easyeda/logic_netlist.json`](../../../easyeda/logic_netlist.json) 为准。PCB 未制造，运动保持禁用；没有刷写或实板测试。本目录的 KiCad 六页图和 2026-09-09 CH343G 版说明已被取代，见文末[历史记录](#历史2026-09-09-kicadch343g-版已取代)。

## 板级接口

固件身份为 `top_board_v2_kiss_c6_review`。控制台是 C6 自带的 USB Serial/JTAG（GPIO12/13，经 R50/R51 22Ω）：Arduino HWCDC，`ARDUINO_USB_CDC_ON_BOOT=1`，`Serial.begin(115200)` 不指定引脚。USB 身份为 VID 303A、PID 1001，序列号是芯片 MAC。板上没有 USB-UART 桥（CH343G 及其自动下载晶体管已于 2026-10-29 移除）。构建拒绝其他板型、S3、UART 控制台（`CDC_ON_BOOT≠1`）和电机测试入口。

电机电源请求 ARM（MOT_EN → U5 EN，R24 下拉）为 **GPIO0**，启动和运行期间保持 LOW。六路 FI/BI 先写 LOW，再启用输出；审查版不连接 PWM 外设，底层驱动也拒绝非零转速。

| 实体电机 | 排针 | FI / BI GPIO | 编码器 A / B GPIO |
| --- | --- | --- | --- |
| Motor1 | M0 | 22 / 23 | 20 / 21 |
| Motor2 | M2 | 14 / 7 | 19 / 18 |
| Motor3 | M1 | 1 / 6 | 2 / 3 |

| GPIO | 用途 |
| --- | --- |
| 4 | VBAT 采样，120k/10k 分压 |
| 5 | INA180A1 电机总电流，2V/A |
| 8 | USB 供电检测输入：USB/充电器 5V 存在时为高（无 USB 时的低电平见下文，当前取值不保证） |
| 9 | BOOT：R9 10k 上拉，另有 BOOT/GND 恢复焊盘 |
| 12 / 13 | USB D− / D+ |
| 15 / 16 / 17 | 传感头 J1：I2C SDA / SCL / IR_EN_N（R25 上拉） |

GPIO15/16/17 是应用引脚，电机与 GPIO 安全服务不驱动它们。

### IO8 USB 供电检测

网表连接：VBUS 经 R52 1k／R10 2.2k 分压得到 `USB_DIV`；Q1（2N7002，1 脚 G 接 3V3、2 脚 S 接 IO8、3 脚 D 接 `USB_DIV`）把分压点接到 IO8；IO8 另有 R53 100k 上拉到 3V3。有 USB 时分压点约 3.4V（VBUS 跌到 4.4V 时约 3.0V），栅极比 Q1 两端最多高约 0.3V，Q1 截止，IO8 由 R53 拉到约 3.3V，读为高（VIH 为 0.75 × VDD ≈ 2.48V）。SW1 关断 3V3 而仍在充电时，Q1 栅极为 0V，沟道截止、体二极管反偏，分压不会反灌未上电的 C6。

**无 USB 时的低电平（2026-10-29 改值）。** 此时 VBUS 无人驱动（D1 反偏，IP2326 是 VBUS 的负载而不是电源）。流入 `USB_DIV` 的只有两路，都经 R10 2.2k 到地：一路是 D1（SS14）的反向漏电，经 R52 进入；另一路是 R53 经 Q1 流下的约 26µA。漏电为 0.3mA 时，分压点约 (0.3 + 0.026)mA × 2.2k ≈ 0.72V。这时 Q1 的 Vgs 约 2.6V，导通，IO8 约 0.72V，低于 ESP32-C6-MINI-1 数据手册 v1.5 的 VIL 最大值 0.25 × VDD ≈ 0.825V。常温下约 7V 反压的漏电远小于 0.3mA，IO8 接近 0V。但 SS14 的反向漏电上限在 40V、25°C 时就是 0.2–0.5mA 量级，并随温度上升（[JLC C2480 SS14](https://jlcpcb.com/partdetail/Mdd-SS14/C2480)），所以 D1 很热时余量会变小。

原来的 22k／33k 中，单是 R53 的电流就把 IO8 抬到约 0.82V（VIL 边界），约 25µA 的漏电就会误报 `external_power`。改成 1k／2.2k 后，有 USB 时分压电流约 1.6mA（约 8mW），相对充电电流可以忽略。新值都是 JLC 基础库：R52 C21190、R10 C4190、R53 C25803、Q1 C8545。

待实测：只接电池时 IO8 的电压（包括 D1 发热以后），以及有 USB 时 IO8 的高电平。网表只有连接、没有元件值，`verify_kiss_c6.py` 只能检查拓扑，查不出取值问题。

固件可报告原始编码器计数。减速比、编码器方向和轮径尚未标定，速度与位置返回 null，`odometry_qualified=false`。

## 刷写

应用正常运行时，USB Serial/JTAG 可以自动进入下载模式：

```sh
.venv/bin/pio run -e top_board_v2_kiss_c6 -t upload --upload-port PORT
```

首次刷写，或应用无法响应 USB 时，手动下载：

1. SW1 置 OFF。SW1 控制 3V3 降压 U4 的 EN。
2. 插入 USB。3V3 上电后 IO8 为高，满足 GPIO8=1、GPIO9=0 的下载条件。
3. 短接 BOOT 焊盘（GPIO9 接地）。
4. SW1 置 ON，执行上面的刷写命令。
5. 移除短接，再把 SW1 关一次、开一次。

USB Serial/JTAG 把 RTS 当作内核复位，它不重新采样 GPIO9。经 BOOT 焊盘进入下载后，`reset` 和 esptool 的 hard-reset 都回不到应用，必须按第 5 步断电重启。来源：[乐鑫 USB Serial/JTAG 控制台](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c6/api-guides/usb-serial-jtag-console.html)、[esptool 复位选项](https://docs.espressif.com/projects/esptool/en/latest/esp32c6/esptool/advanced-options.html)、[启动模式](https://docs.espressif.com/projects/esptool/en/latest/esp32c6/advanced-topics/boot-mode-selection.html)。

## 上位机入口

`PORT` 替换为实际串口，例如 macOS 的 `/dev/cu.usbmodemXXXX`。本次没有执行这些实物命令。

```sh
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind c6usb status
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind c6usb rails
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind c6usb reboot
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind c6usb reset
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind c6usb bootloader
```

`c6usb` 只接受显式路径上的 303A:1001 设备，并绑定 `top_board_v2_kiss_c6_review`；`ch343g` 已删除。`reboot` 请求应用重启，校验同一设备、不同启动标识、软件复位原因及空闲/零输出。`reset` 经 esptool `--before usb-reset` 进入 C6 ROM 读取 MAC，再复位并校验同一 USB 设备和新板固件返回；未观察到复位前应用时，不声称已经比对新旧启动标识。应用没有返回时，结果带 `manual_power_cycle_required=true`，即需按上面第 5 步断电重启。`bootloader` 留在 ROM，不写入或擦除 Flash。

Wi-Fi/BLE 使用同一认证协议。密钥只能经 USB `provision` 写入（GPIO 故障状态下拒绝）；`wifi.local` 的格式和密钥文件说明见[控制说明](../../../../../control_tests/README.md)。KISS 板经 USB 时必须带 `--usb-kind c6usb`，经无线时必须带 `--profile top_board_v2_kiss_c6_review`，否则 `hello` 会以板型不符拒绝：

```sh
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind c6usb provision --wifi-config wifi.local --key-file control-key.local
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind c6usb status
.venv/bin/python scripts/robot_control.py --wifi HOST --key-file control-key.local --profile top_board_v2_kiss_c6_review status
.venv/bin/python scripts/robot_control.py --wifi HOST --key-file control-key.local --profile top_board_v2_kiss_c6_review rails
.venv/bin/python scripts/robot_control.py --ble BLE_ID --key-file control-key.local --profile top_board_v2_kiss_c6_review status
```

`HOST` 是 USB `status` 中的 `wifi_ip`；无线只在电池合格后启动（见下文）。

## 电压、电流与充电

普通 `status` 中 `motor_voltage_mv`、`motor_current_ma`、`rail_3v3_mv` 保持 null；ADC 读数通过独立的 `rails`（固件命令 `rail_status`）返回：

| 字段 | 含义 |
| --- | --- |
| `battery_voltage_mv` | IO4，名义倍率 13，未逐板校准（`divider_calibrated=false`） |
| `motor_current_ma` | IO5 INA180 输出，等于 `adc_pin_mv[1] // 2`；线性约到 1.4A，约 1.6A 到轨 |
| `motor_current_saturated` | 仅在读数有效时出现；任一样本校准后 ≥3150mV 或原始值 4095 时为 true，实际电流不小于报告值 |
| `motor_voltage_mv` | 恒为 null，没有 VMOT 分压 |
| `usb_power` | IO8 |
| `charge` | `usb_power` 为真时为 `"external_power"`，否则为 `"battery"`；不会是 null、`"charging"` 或 `"full"` |
| `adc_raw`、`adc_pin_mv`、`fresh`、`state`、`divider_gain` | 原始值、校准后引脚电压、有效性、ADC 状态、倍率 13 |
| `wireless_power_state`、`wireless_start_attempted`、`previous_radio_power_reset` | 无线供电策略状态 |

IP2326 的充电状态没有接到 MCU，所以固件只报告供电来源，不判断充电中或已充满。转换、采样和失效规则见 [ADC 读出](ADC_READOUT.md)。

IP2326 自主充电，固件无法开关它（`charge_enabled=false`）：约 0.6A 进入电池（R40 150k），5V 侧约 1.2A（最坏约 1.3A）。2026-10-30 起 IP2326 的 EN 由 USB-C CC 门控：U8（LM393）把两路 CC（10k/1µF 滤波）与 U9（CJ431）基准分出的 0.65V 比较，只有电源声明 ≥1.5A（Rp 为 1.5A 或 3A）时 Q2 才放开 EN（R54 100k 接 VBUS、C35 470nF，约 15ms 后变高）。默认 USB 电源和 USB-A 线只给板子供电，不充电。SW1 关断 3V3 时充电照常进行。

## GPIO 故障

GPIO0 初始化或后续写低失败时，程序锁存 `gpio_fault`，原因为 `GPIO0_SAFE_LOW_FAILED`。此时禁止运动、清除控制权、尝试释放输出，初始错误和释放错误分别保留。写低与状态日志不能证明实物电机电源已经断开。启动时锁存的故障不会主动出现在 USB 上：HWCDC 在主机第一次收发 USB 包之前不算已连接，`setup()` 里的故障回复被丢弃（计入 `tx_dropped`），主机连上后用 `hello`／`status` 读取。

KISS 在故障状态下保持只读，不因故障本身复位：

- 电压采样继续进行。
- 无线仍按同样的电池条件启动：VBAT 6.5–8.8V 持续 1 秒，并要求任务看门狗可用。供电失效时照常自动复位。
- 无线启动后，Wi-Fi/BLE 只回答 `hello`、`status`、`rail_status` 和 `stop`；其余命令返回故障错误，`reboot` 只能经 USB。
- 故障状态报文包含 `wireless_status`、`wireless_power_state`、`wifi_connected`、`wifi_ip`、`wifi_rssi_dbm`、`ble_ready`、`ble_connected` 和 `authenticated_wireless`。

故障循环在阻塞的 `transport.begin()` 返回后立即重新采样，数百毫秒的 SDK 初始化不会因样本过期触发 `RADIO_POWER_LOST` 复位。在 `RADIO_STARTING` 时发现的 GPIO 故障也不再触发供电复位，下一轮进入只读故障循环。R3/Pico 的故障行为不变，仍只走 USB。

无线按电压条件启动，见[无线供电策略及限制](WIRELESS_POWER.md)。无线初始化前已注册看门狗；初始化失败通过 USB 报告，不在同一启动周期重复分配。

**已知问题（2026-10-29，固件待修）：** 正常路径（非故障）启动无线时，`transport.begin()` 在 SDK 中阻塞约 300ms，之后没有刷新控制周期计时 `lastControlAt`。同一轮的控制周期检查把这段阻塞当成超时：`control_overruns` 加 1，`control_timing_fault=true` 一直保持到重启，`reason` 被改写为 `CONTROL_OVERRUN`；初始化失败时也会覆盖 `TRANSPORT_INIT_FAILED`。因此电池合格、无线启动后的每次启动，`status` 都显示这个时序故障，[firmware_verification/status.json](firmware_verification/status.json) 样本就是如此（`control_gap_ms` 310）。运动本就禁用，输出不受影响，但此后的 `reason` 和 `control_timing_fault` 不能当作诊断依据；初始化是否失败看 `rails` 的 `wireless_power_state`（`init_failed`）。修正方法是在两条启动路径的 `transport.begin()` 返回后刷新 `lastControlAt`。

## 验证

`control_tests/verify_kiss_c6.py --build` 编译 KISS 与保留的 R3 环境，运行原生场景和上位机回归，并把编译后的 GPIO 操作与 EasyEDA 网表 `hardware/top_board_v2/easyeda/logic_netlist.json` 比较。IO8 只接受上面的隔离拓扑（Q1 栅极接 3V3、IO8 上拉、IO8 和分压点上没有其他器件）；验证程序还确认这项检查会拒绝五种网表改动，包括把分压直接接到 IO8。验证在临时公开源码副本中执行，其中 `config.h` 被设为编译错误哨兵，真实私有配置不会被读取。结果写入本目录 [firmware_verification/](firmware_verification/report.json)：报告记录每项检查、网表和源文件哈希，`all_passed` 与 `cross_compiled` 字段说明是否全部通过、是否含交叉编译；同目录的 status、fault_status、rails 样本来自模拟输入，不是实板读数。

首次运行前先准备环境。`.pio/libdeps` 不入库，ArduinoJson 头文件要先由 PlatformIO 取回：

```sh
python3 -m venv .venv && .venv/bin/pip install platformio -r scripts/requirements-control.txt
.venv/bin/pio pkg install -e top_board_v2_kiss_c6
.venv/bin/python control_tests/verify_kiss_c6.py --build
```

缺少这两项时，验证程序在开始前退出并给出上面的命令，不改动报告目录。

2026-10-29 带 `--build` 的运行全部通过（`all_passed=true`、`cross_compiled=true`）。这些都是离线检查，SDK、GPIO、无线和 USB 均为替身。

尚未完成：只接电池时 IO8 的实测电压（取值已改，见上）、供电与默认停机硬件、ADC 掉电处理和外部分压校准、无线供电切换的实物验证、实际 USB 枚举和恢复、充电电流与 USB 电源的配合、底板接口负载、无线共存/距离和完整成本核算。软件审查版不是制造或运动放行。

## 历史：2026-09-09 KiCad/CH343G 版（已取代）

以下结果针对本目录的 KiCad 六页图：外置充电，CH343G USB-UART 桥（`--usb-kind ch343g`，DTR/RTS 自动下载，ROM 子进程 115200 波特、`reset_delay=0.05`），UART0 TX16/RX17，ARM GPIO14，IO5 为电机电压分压。它们不代表当前板；当时 `firmware_verification/` 中的报告和样本已被上面的验证覆盖，旧版可在 git 历史中查看。

- 当时 73 项离线检查及 C6/R3 两项交叉编译通过（报告共 75 行），绑定 KiCad 网表。
- 39 个新板运行场景通过：执行生产主循环、驱动、编码器 ISR 和通信实现；SDK、GPIO、时钟、UART、无线及 NVS 使用替身。
- 9 组 ADC 边界测试、6 组无线供电策略边界测试通过。
- 编译后的 GPIO 操作与当时的 KiCad 导出网表比较；四种不兼容构建被编译器拒绝。
- 131 项上位机测试通过，其中 8 项 CH343G 测试、9 项电压报告测试；实物串口和 ROM 子进程均为替身。
- 状态载荷 1185 字节，带认证封装 1399 字节。
