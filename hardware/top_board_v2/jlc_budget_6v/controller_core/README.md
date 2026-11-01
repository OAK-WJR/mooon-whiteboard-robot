# 外置充电版（2026-09，已取代）：当时的主控工程与 USB 子电路

> **已被取代（2026-10-28／10-29）。** 当前板是 EasyEDA `top_board_v2_logic`（见 [hardware/README.md](../../../README.md)）：板上 IP2326 充电。2026-10-29 起 CH343G 及其 DTR/RTS 自动复位晶体管已移除，改用 C6 原生 USB Serial/JTAG；EN 由 R8 上拉、C4 延时，BOOT（GPIO9）由 R9 10k 上拉并留 BOOT/GND 恢复焊盘。位号 Q1 现在是 IO8 隔离用的 2N7002（栅极接 3V3），IO8 经它接 VBUS 分压，并由 R53 100k 上拉到 3V3，作 USB 供电检测（高 = 有 USB；无 USB 时的低电平余量见[固件说明](logic/FIRMWARE.md#io8-usb-供电检测)）。当前引脚以 [`include/board_profile_kiss_c6.h`](../../../../include/board_profile_kiss_c6.h) 和 EasyEDA 网表为准，刷写见[固件说明](logic/FIRMWARE.md)。下文的外置充电、六页 KiCad 工程和 CH343G 子电路是 2026-09 历史。

**当时入口：[六页原理图与两层布局](logic/README.md)**：70 个实体元件、246 个物理引脚，C6-MINI 审查候选；ERC 0 错误/0 警告，73 项离线检查及当时的 C6/R3 交叉编译通过。PCB 整板布线未完成，MINI 国际站 Standard Only 的贴装限制已记入[成本复核](../COST_REVIEW.md)，没有低成本或制造定案。

以下是 2026-09-08 建立的独立 USB 子电路及其验证依据，含 USB-C、数据保护、CH343G 和 EN/BOOT 自动恢复，共 **20 个实体元件、74 个物理引脚**。其连接已完整保留到上述六页工程。

## 独立 USB 子电路的验证依据

- [打开 KiCad 工程](usb_recovery.kicad_pro) / [原理图](usb_recovery.kicad_sch)
- [查看整张图](preview/usb_recovery.png)
- [复位时序图](preview/reset_sequence.png) / [验证记录](verification/report.json)

CH343G 的 VDD5/VBUS 接 USB，VIO 与 ESP32 共用主控 3.3V。采用厂家双电源接法，省去旧稿的独立串口和复位信号隔离器。V3 只接去耦，不给 ESP32 供电；DTR 不接下拉，避免进入 TNOW 模式。依据为 [WCH V2.0 手册](https://files.waveshare.com/wiki/ESP32-S3-LCD-1.3/CH343DS1_EN.pdf) §4–5，[本地副本](sources/ch343.pdf)。

自动恢复使用两颗交叉连接的 NPN：DTR 高、RTS 低时拉低 EN；反过来拉低 GPIO9；二者相同时释放两根控制线。GPIO8 保留上拉。[Espressif 自动下载说明](https://docs.espressif.com/projects/esptool/en/latest/esp32c6/advanced-topics/boot-mode-selection.html)给出这种连接关系和 EN 延时要求。

EN 支路增加 680Ω 限制电容放电电流，EN 上拉取 22kΩ、延时电容 1µF；与原厂常用 10kΩ 参数不同，因此单独检查了低电平余量和启动保持时间。在明确的模型条件下，RTS 峰值电流估算上界约 6.49mA；R +1%、C +20% 的三个时序案例，在 EN 越过 0.75VDD 后仍有约 14ms 的 BOOT 保持时间，超过手册的 3ms。上位机必须落实本次模拟的 100ms 复位、随后 50ms BOOT 保持序列，不能把模型当作现有串口驱动的实测。

## 验证范围

实际网表已检查 USB D+/D−、CC1/CC2、VIO/USB 供电隔离关系和两颗晶体管的物理脚号。检查程序能拒绝人为加入的数据线接反、复位发射极接反及 CC2 短路。原生 ERC 为 **0 错误、3 警告**；三个警告分别是尚未连接到 MCU 的 TX、RX 和 GPIO8 接口，保持原样显示，没有关闭规则。

ngspice 跑通 **72 个直流状态和 3 个启动时序案例**。另断开复位基极电阻，检查能正确发现 EN 无法拉低。SPICE 使用通用 NPN 模型，覆盖连接逻辑及有限参数变化；并非芯片厂家模型或全温度保证。饱和压降计算参考 [onsemi MMBT3904L 数据表](https://www.onsemi.com/pdf/datasheet/mmbt3904lt1-d.pdf)的指定测试点，并明确将其列为计算假设，尚未选定最终采购晶体管。

图中的电源标记表示 **USB VBUS、主控 3.3V 和公共系统地是外部接口条件**。它们不表示稳压、USB/电池切换、放电保护或完整供电已经设计通过。

MCU 已在六页工程接入，启动脚分配已复查并修正。尚须完成供电与默认停机、实际 USB 枚举/取电额度、断电插拔与 TNOW 启动模式、驱动及 esptool 实测、ESD/射频/线束和底板接口测试。CH343G 不能直接套用 P 版本可配置 EEPROM 的能力；枚举成功也不等于整板自动获准从 USB 取 500mA。USB 独立诊断模式的射频和编码器耗电须纳入系统预算。

补充 [DTR 启动偏置检查](logic/usb_startup_review/README.md)：24 个有限条件模型场景支持保留当前 4.7k 上拉，错误下拉对照检出高电平余量不足；没有新增器件，也没有获得 CH343 实际采样／USB 供电通过结论。[供电状态检查](logic/USB_POWER_MODES.md)确认停机门禁没有同时禁止无线初始化，需继续完成 USB／电池供电策略。

## 元件与成本

[嘉立创 C2844153](https://jlcpcb.com/partdetail/WCH_Jiangsu_Qin_Heng-CH343G/C2844153)当次公开页为 SOP16、扩展料、支持经济型贴装，1+ 为 $1.2332，五颗 $6.166。它比历史 CH340C 五颗 $2.95 的参考价高 $3.216；减少隔离外围有电路简化收益，但**没有据此确认整批成本下降或低于预算**。元件公开价不是完整 PCBA 报价。其余元件未完成逐项嘉立创定料和核价。

主控已作为成本待确认候选接入逻辑图，充电器与电池保护没有因此定料，PCB 层数和布局也没有冻结。当时接线以 [logic/c6_pin_plan.json](logic/c6_pin_plan.json) 为准；它是 2026-09 KiCad 引脚表（UART 16/17、ARM GPIO14），现已被 `include/board_profile_kiss_c6.h` 取代。上级 [c6_pin_plan.json](c6_pin_plan.json) 仅留作生成器的历史输入，其中 GPIO22 电源请求已被当时 MINI 图的 GPIO14 替代，M0_BI 为 GPIO23，不能直接用历史表接线。当前 MINI 按原厂 v1.5 手册独立核对 53 个物理脚号。

重建及验证：

```sh
cd hardware/top_board_v2/jlc_budget_6v/controller_core
python3 build_usb.py
python3 verify_usb.py
python3 plot_usb.py
```

`build_usb.py` 只写此目录；`verify_usb.py` 通过 KiCad 导出实际网表后运行检查和 ngspice，不访问串口或真实固件配置。预览由 KiCad SVG 导出并光栅化，旧 PCB 与固件均未改动。整板未完成、未制造放行。
