# C6 / CH343G 上位机接入待办

**此为接入前的检查记录，已由 [C6 固件与上位机实现](FIRMWARE.md)取代。以下代码位置和缺项描述保留为历史依据；新 `ch343g` 分支已实现并完成离线测试，实物检查仍未完成。**

2026-09-08，只读代码与厂家资料复核。当前原理图的 USB 桥已连到 C6，但新板固件身份和主机命令尚未适配；没有打开串口、运行 ROM 恢复或烧录。

## 现有实现与新板不一致之处

- [pcb_control.py](../../../../../scripts/pcb_control.py) 的 `usb_chip("ch343p")` 返回 `esp32s3`。该选项属于旧 Pico 方案，不能只因 VID/PID 相同就用于新 C6。
- [robot_control.py](../../../../../scripts/robot_control.py) 的 `USB_PROFILES` 将 `ch343p` 绑定 `top_board_v2_pico_review`。当前只接受原生 USB、旧 R3/CP2102N 和旧 Pico/CH343P；尚无这版 C6/CH343G 身份。
- 旧 R3 固件的 GPIO 分配不等于[本版引脚表](c6_pin_plan.json)。需要单独的固件配置、状态身份和主机校验，不能通过把新板伪装成旧 R3 或 Pico 来绕过检查。

## 已核实的外部接口事实

WCH 官方驱动 V2.2 将 `1A86:55D3` 列为 CH343 家族，并进一步读取芯片类型来区分 G/P、J、K 等。**这个 VID/PID 不能证明后端是 C6，也不能证明是本项目的 PCB。** 后续仍须显式选择本地串口、保留设备身份锚点、指定 C6 ROM，并核对新应用身份。[WCH 官方驱动](https://github.com/WCHSoftGroup/ch343ser_linux/blob/main/driver/ch343.c)，`ch343_configure` 与 `ch343_ids`。

项目 [requirements-control.txt](../../../../../scripts/requirements-control.txt) 锁定 esptool 5.3.0。该版本 `ClassicReset` 先保持 EN 复位 100ms，然后释放 EN、继续保持 BOOT 至 `reset_delay`；当前电路的有限模拟使用随后 50ms。主机布尔 DTR/RTS 是有效低的控制语义，不能把 `True` 直接当作原理图中的高电平。[Espressif 5.3.0 reset.py](https://github.com/espressif/esptool/blob/v5.3.0/esptool/reset.py)、[C6 自动下载说明](https://docs.espressif.com/projects/esptool/en/latest/esp32c6/advanced-topics/boot-mode-selection.html)。

esptool 支持 `ESPTOOL_CFGFILE` 与自定义复位序列。现有 `robot_control.py` 对 UART 恢复建立临时空配置，避免用户全局配置替换策略。新分支应保留这种隔离，并明确固定与本版电路匹配的复位/退出 ROM 时序；不能只改 `--chip` 就宣布兼容。[Espressif 配置文件说明](https://docs.espressif.com/projects/esptool/en/latest/esp32/esptool/configuration-file.html)。

## 完成条件

后续实现需一并检查新固件引脚和身份、CH343G 选项、C6 ROM 命令、复位时序、重启后同一设备的新启动标识、错误板型拒绝和复位后零输出。离线测试可以核对命令构造、模拟串口交互和状态校验；操作系统驱动的真实 DTR/RTS 时延、枚举、插拔和恢复仍需实板验证。保持当前运动禁用状态，直至新供电与整机验收完成。
