# 通过 USB、Wi-Fi、BLE 控制集成板

**2026-09-08 共用运行固件修正：** 补上验签期间租约到期时遗漏的停止调用；实际命令处理器的边界回归、相关重启回归及 C6 编译通过。见[期限边界修正与验证](LEASE_DEADLINE_FIX.md)。这是新增的专项验证，不能将下方历史完整套件报告视为本次重新执行的结果。

**R3 独立 USB 桥版本：** 使用 `top_board_v2_r3`，UART TX16/RX17、ENC0_B=GPIO6；电脑端需显式指定 `--usb PORT --usb-kind cp2102n`。见 [R3 固件](../hardware/top_board_v2/firmware_r3/README.md) 与 [CP2102N 主机工具说明](../hardware/top_board_v2/firmware_r3/host_cp2102n/README.md)。这两个版本仍未完成电源及实板验收。以下保留的是原生 USB 的 R2/旧集成板流程，不能不加区分地用于 R3。

`src/top_v2_main.cpp` 是新集成板主固件，PlatformIO 环境名 `top_board_v2`。它与 `usb_diagnostics` 的专用诊断固件是两个不同程序，一次运行其中一个。主工程 `c6` 仍是旧转接板配置，不能混用：新板 ENC0_A=GPIO7，旧板=GPIO16。

当前电源没有合格，主固件和新板诊断固件都保持 `BOARD_POWER_UNREVIEWED`，没有软件开关绕过。可检查 USB、无线、编码器和停止/复位流程；不能据此验收带载运行。尚未接到实板。

从项目根目录安装本地主机工具并编译：

```sh
.venv/bin/pip install -r scripts/requirements-control.txt
.venv/bin/pio run -e top_board_v2
.venv/bin/python scripts/pcb_control.py list
```

如果缺少 `include/config.h`，从 `include/config.example.h` 复制；保留已有配置。新板无线凭证通过 USB 配置，未使用旧配置文件的 Wi-Fi 凭证。构建为 4MB Flash、3MB 应用分区；本版没有 OTA 分区，通过 USB 更新。以下 `PORT`、`HOST`、`BLE_ID` 必须替换为实际设备。

PlatformIO 平台固定为 pioarduino `55.03.39` 发布包，使用 Arduino-ESP32 `3.3.9` / ESP-IDF `5.5.4`；ArduinoJson 固定为 `7.4.3`。此前的裸 `espressif32` 名称依赖本机已有安装，不能保证另一台电脑选择相同平台。发布包已下载并与发布者 SHA256 核对；本轮使用现有工具缓存编译，未做全新电脑安装测试。[平台发布](https://github.com/pioarduino/platform-espressif32/releases/tag/55.03.39)、[PlatformIO 版本配置](https://docs.platformio.org/en/latest/projectconf/sections/env/options/platform/platform.html)

电源路径确认能稳定给 MCU 供电之后，才能烧录；现有 BAT 1S 不能接 12V：

```sh
.venv/bin/pio run -e top_board_v2 -t upload --upload-port PORT
.venv/bin/python scripts/robot_control.py --usb PORT status --report diagnostic-reports/control-status.json
.venv/bin/python scripts/robot_control.py --usb PORT stop
.venv/bin/python scripts/robot_control.py --usb PORT reboot --report diagnostic-reports/reboot.json
```

`reboot` 先停止。主机只发送一次重启命令，并记录 `reboot_ack_received`；确认包丢失时仍尝试重新连接。只有核对新启动标识、同一板身份、软件复位原因、idle 和零 PWM 后才输出 `APP_RETURN_VERIFIED`。设备明确拒绝命令则直接报告失败，不当作确认包丢失。无线重连仍必须通过 HMAC 校验。

固件等待发给该 BLE 连接的应用发送队列排空后再留约 100ms，持续拥塞时最多等待 2500ms。队列排空不证明手机/电脑已经收到通知，最终依据仍是主机的回包与新启动验证。`reset` 调用 esptool 的原生 USB ROM 同步并硬复位；`bootloader` 同步 ROM、读取 MAC 后停留 ROM，不写入或擦除 Flash：

```sh
.venv/bin/python scripts/robot_control.py --usb PORT reset
.venv/bin/python scripts/robot_control.py --usb PORT bootloader
```

如果实际运行的是 USB **诊断固件**，使用另一个上位机协议：

```sh
.venv/bin/python scripts/pcb_control.py reboot --port PORT --profile top_board_v2
.venv/bin/python scripts/pcb_control.py reset --port PORT --profile top_board_v2
```

原生 USB 失效、MCU 无电、EN 被拉低等情况不能由这些命令保证恢复。完整无人值守恢复还需要独立 EN/BOOT 控制和供电电路，见 [硬件报告](../hardware/top_board_v2/VERIFICATION.md)。

**首次无线配置。** 本地创建 `wifi.local`，内容只有 `ssid` 和 `password`，例如下面占位值；替换后通过 USB 配置：

```json
{"ssid":"YOUR_2_4_GHZ_SSID","password":"YOUR_WIFI_PASSWORD"}
```

```sh
.venv/bin/python scripts/robot_control.py --usb PORT provision --wifi-config wifi.local --key-file control-key.local
.venv/bin/python scripts/robot_control.py --usb PORT status
```

脚本生成 32 字节随机设备鉴权密钥，文件权限 0600；已有密钥不会覆盖。凭证和密钥写入同一个 NVS 配置记录，重启应用。`.local` 文件已被仓库忽略，不要分享密钥或 Wi-Fi 配置。这是本地设备密钥，不需要外部搜索/云 API key。USB status 中的 `wifi_ip` 是下面的 HOST。

```sh
.venv/bin/python scripts/robot_control.py --wifi HOST --key-file control-key.local status
.venv/bin/python scripts/robot_control.py --wifi HOST --key-file control-key.local watch --seconds 30 --report diagnostic-reports/wifi.json
.venv/bin/python scripts/robot_control.py --scan-ble --seconds 5
.venv/bin/python scripts/robot_control.py --ble BLE_ID --key-file control-key.local status
.venv/bin/python scripts/robot_control.py --ble BLE_ID --key-file control-key.local reboot
```

Wi-Fi 使用 STA、UDP 4243，需要在可达的局域网；不开放 SoftAP。BLE 使用 GATT：服务 `6e400001-b5a3-f393-e0a9-e50e24dcca9e`，RX/TX 为 `...0002...` / `...0003...`。脚本按 20 字节分片以兼容默认 MTU；BLE_ID 使用扫描结果的 address，macOS 为 UUID。扫描结果尚未鉴权，以后续有签名的握手为准。[Bleak 扫描 API](https://bleak.readthedocs.io/en/latest/api/scanner.html)

BLE SDK 通知发送失败时，固件记录错误、清除应用发送队列并断开该连接，让客户端丢弃残缺报文；控制租约随断线撤销。新连接不会接收上一连接剩下的发送或接收分片。当前 Arduino-ESP32 3.3.9 的 `SUCCESS_NOTIFY` 同时用于 SDK 接受发送等事件，不能当作应用已收妥；错误从 `onStatus` 回调记录。[Espressif 实现](https://github.com/espressif/arduino-esp32/blob/3.3.9/libraries/BLE/src/BLECharacteristic.cpp)

**协议与可观察性。** `hello` 返回启动标识和能力；无线请求与响应均为 `{payload,mac}` 信封，MAC 校验精确的 payload 字符串。数据有完整性鉴权，没有应用层加密。配置只接受 USB；无线使用 Wi-Fi 自身安全设置与设备密钥。

运行类命令需要独占 claim、租约和递增序号。750ms 无有效控制心跳停止，错误顺序及断线撤销控制权；重连必须重新获取，旧任务不恢复。USB/BLE 组帧溢出立即触发停止；Wi-Fi 报文按数据包边界分隔。本版所有 start/goto/wheel/arm/pulse 均拒绝，未来通过硬件审核后也须保留监督与停止机制。

报文必须是完整对象，末尾只允许空格、制表符、CR、LF；签名信封和内部 payload 都检查完整消费。关键字符串拒绝嵌入 NUL，避免 `reboot\u0000ignored` 被截断成 reboot。解析最大嵌套为 8 层；这是命令格式限制，不是通用 JSON 合规性认证。实现使用 ArduinoJson 的公开 [自定义 Reader 接口](https://arduinojson.org/v7/api/json/deserializejson/)。

控制循环使用实际经过时间计算轮速；间隔超过 100ms 就停止、撤销租约，并记录直到重启仍可见的时序故障。PWM 写入失败则记录故障、尝试解除 PWM 外设并拉低两脚，同时停止另外两路。Arduino-ESP32 3.x 提供写入/解除操作的布尔结果；旧 2.x 写入没有相同反馈。[LEDC API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html) 普通 stop 或运行命令不会清除 PWM 故障；显式重新初始化或重启仍需重新检查硬件。

| 字段 | 含义 |
|---|---|
| boot / reset_reason / uptime_ms | 启动会话与复位证据 |
| pwm_ready / motor_pwm_ready | PWM 初始化及运行故障后的可用状态，不是焊盘电压测量 |
| motor_pwm_fault | 每路故障位：1=初始化失败、2=写入失败、4=解除外设失败，可相加；非零必须处理 |
| encoder_counts / wheel_speed_mps | 三路方向校正的有符号 32 位回绕计数及按配置算出的轮速，需校准编码器参数 |
| pwm_command / physical_motors / header_order | 主运行固件数组按物理 Motor1、Motor2、Motor3，即排针 M0、M2、M1；诊断固件按排针排序，二者有意不同 |
| wifi_connected / wifi_rssi_dbm / ble_connected | 当前连接观察值，不是无线可靠性验收 |
| watchdog_ready / tx_dropped / reason | 看门狗初始化、发送丢弃、最近停止/限制原因 |
| ble_tx_failures | SDK 通知错误回调累计数；不是实际丢包率 |
| control_gap_ms / max_control_gap_ms | 最近一次及本次启动以来最长控制更新间隔 |
| control_overruns / control_timing_fault | 超过 100ms 的次数及持续保留的故障标记 |
| motor_voltage_mv / motor_current_ma / rail_3v3_mv | `status` 中为 null。KISS C6 的电机电流（INA180）和电池电压只经 `rails` 报告，电机电压没有传感器；其他板型没有对应板载传感器 |
| power_disconnect | 当前 false；软件停止不代表物理断电 |

所有 CLI 命令返回 JSON，失败返回非零退出码；AI 应保存报告，遇到错误不得自动重试启动。无线 watch 只采集状态，不持有运动租约。BLE 扫描需要系统允许蓝牙访问；本轮未实际扫描/连接无线板子。

**本地验证。** 在 macOS、已安装依赖并构建主固件后，执行 `.venv/bin/python control_tests/verify.py`。本地套件包括：

- 196 项控制数学断言、12 项控制客户端测试、13 项诊断/恢复客户端测试。
- 实际 MotorDriver 源码的 29 个故障场景：六引脚、双向切换、停止失败、部分初始化及解除 PWM 失败。
- 实际 RobotController/路径规划/里程计/驱动一起运行，验证六条故障传递路径及两种路径替换停止行为；中断和 GPIO 为模拟。
- 实际编码器 ISR / 里程计的正反转计数回绕、异常时间值和 120,000 次 GPIO 边沿；另测三路堵转冷却跨计时回绕，详见 [长时间运行边界修正](LONG_RUN_VERIFICATION.md)。
- 实际主命令处理器/循环的 20 个场景，包括 1 万条固定种子的畸形输入；另有实际时间间隔回归。此组的 RobotController 和传输层使用替身。
- 实际 ControlTransport 的连接/收发/错误回调，覆盖队列容量、分片、旧连接隔离、UDP 边界、Wi-Fi 重连计时和通知失败。队列、BLE/Wi-Fi SDK 是测试替身。
- 实际传输层与主循环一起执行六种情形：正常积压、ACK 入队失败、持续拥塞、发送失败、时钟回绕、发送失败后的控制权撤销。确认包丢失后的上位机重连另有测试。
- 原生租约/组帧、两种实际诊断源码解析器/循环，以及六个固件交叉编译环境：主工程 esp32/c6/top_board_v2/motor_test，诊断 adapter/top_board_v2。

原生 C++ 使用 ASAN/UBSAN。命令处理器的 HMAC 适配到 macOS CommonCrypto，不是在模拟 C6 的 mbedTLS。签名 UDP 回环发生在本机；本项目回调源码虽已被执行，但 SDK 回调调度、FreeRTOS 并发、底层无线缓冲和 RF 共存仍没有实测。驱动测试故意保留“解除失败时模拟 PWM 仍为非零”的情形，确认软件报告失败，不能把 GPIO-low 调用当成真实断电。

证据写到 `hardware/top_board_v2/validation/reverification/`；报告记录固件、测试、主机及已查阅 SDK/JSON 关键依赖的散列，并检查测试期间没有变化。实际 USB ROM 复位、功率、电气波形和射频验收仍须实板完成。

# KISS C6 当前板（top_board_v2_logic，板上 IP2326 充电）

当前板使用独立环境 `top_board_v2_kiss_c6`。USB 是 C6 原生 USB Serial/JTAG（303A:1001，无 USB-UART 桥，CH343G 已于 2026-10-29 移除），指定 `--usb PORT --usb-kind c6usb`；Wi-Fi/BLE 指定 `--profile top_board_v2_kiss_c6_review`。上面“首次无线配置”的命令用于 KISS 板时要加这些参数，否则 `hello` 以板型不符拒绝：

```sh
.venv/bin/python scripts/robot_control.py --usb PORT --usb-kind c6usb provision --wifi-config wifi.local --key-file control-key.local
.venv/bin/python scripts/robot_control.py --wifi HOST --key-file control-key.local --profile top_board_v2_kiss_c6_review status
.venv/bin/python scripts/robot_control.py --ble BLE_ID --key-file control-key.local --profile top_board_v2_kiss_c6_review status
```

引脚以 `include/board_profile_kiss_c6.h` 和 EasyEDA 网表 `hardware/top_board_v2/easyeda/logic_netlist.json` 为准；ARM 请求 GPIO0 保持 LOW，运动仍禁用。电池电压、电机电流（INA180，IO5）、`usb_power` 和 `charge` 只在 `rails` 中报告。首次刷写需短接 BOOT 焊盘；经焊盘手动下载后，要用 SW1 断电重启才能回到应用。原始计数可用，速度/位置待标定；旧 R3/Pico 引脚不能直接套用。刷写步骤、命令和验证见[固件说明](../hardware/top_board_v2/jlc_budget_6v/controller_core/logic/FIRMWARE.md)。本节以上为各历史板型的说明。
