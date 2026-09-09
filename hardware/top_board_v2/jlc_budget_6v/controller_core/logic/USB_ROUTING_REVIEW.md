# 顶部 USB 接口的手工布线检查

USB 阶段新增 **67 段线、14 个过孔和一块背面地铜**，调整 U2、C1、C2、C3 的位置；当时整板为 **170 段线、38 个过孔、123 项未连接**，没有增加元件。该版已保存在[复位布线前检查点](../archive/reset_before_control/README.md)。此后继续完成了[复位与串口局部布线](RESET_ROUTING_REVIEW.md)，当前状态以[主板说明](README.md)为准；五片成品成本仍未达标。

[当前 PCB](top_board_v2_logic.kicad_pcb) · [USB 局部放大](preview/usb_interface_detail.png) · [整板预览](preview/placement.png) · [USB 阶段原生检查报告](../archive/reset_before_control/top_board_v2_logic.verification.json)

## 接线与布局

U2 转到 90°、移近 Type-C 接口，两条数据线分别穿过它的 1/6 和 3/4 焊盘，再到 CH343G。两组正反插数据触点都接入：A6/B6 为 D+，A7/B7 为 D−。镜像触点交叉使用一个背层跨接；CC1、CC2 各保留独立的 5.1kΩ 下拉。ST 的布板说明要求防护器件靠近干扰入口，并缩短信号、VBUS 和地回路；这些是布局依据，不是整板 ESD 合格证明。[ST USBLC6-2 §2.3](https://www.st.com/resource/en/datasheet/usblc6-2.pdf)。

C1、C2、C3 分别靠近 CH343G 的 VDD5、V3、VIO 引脚，仍使用原有三颗 100nF。定位孔与 Type-C 原始封装全部保留，VBUS 绕开两个定位孔。USB 地铜与已有 U5 地铜连通；U4 的地铜和全板供电分配仍未接完整。WCH 手册要求上述三路去耦靠近对应引脚，并将 CH343 定义为 USB 全速设备。[WCH v2.0 §7.1](https://files.waveshare.com/wiki/ESP32-S3-LCD-1.3/CH343DS1_EN.pdf)。

所有新铜线按明确坐标手动安排，没有自动寻路，方向为 30° 的整数倍。0.2mm 数据线宽只是当前几何参数；没有取得可据以确认 90Ω 差分阻抗的制造叠层，也没有完成信号完整性或眼图验证。

## 实际完成的检查

- KiCad 原生连通图核对 **21 组局部连接**，包括原来的两个电源区、USB 数据、CC、VBUS 和去耦。两种插入方向的触点均与对应 CH343G 数据脚连通。
- 全部 **246 个物理引脚的网络**与现有 XML 网表一致。原来的 **127 个走线/过孔记录**逐字保留；19 个对接触点、Type-C 完整封装和定位孔、模组及板框未变。
- 原生 DRC 剩余 **123 项未连接和一处原有 M2/PWR1 装配边界重叠**，没有新增短路、铜间距、定位孔边距或装配边界问题。
- 临时副本分别断开 CH343G 前的 D+、D− 最后一段线，原生连通图均检出两组 Type-C 触点到该输入的断开，DRC 未连接项均变为 124。这是 CAD 断线对照，不是拔线实测。
- `.kicad_dru` 保持相同。原 85 字节工程文件只含元数据，新工程显式保存了默认参数；通过原生 SaveBoard 展开临时旧版本后，确认两版 PCB 设计参数、规则级别及网络类别相同，没有新增 DRC 排除。
- [上下板配准](bottom_alignment.json)重新计算，19 个对应点最大名义偏差仍约 0.00103mm；[天线二维投影](antenna_projection.json)未变，原有制造公差和射频限制仍在。

## 尚未完成

USB 阶段之后，UART 和自动复位到主控的局部走线已继续完成；后续[主控供电阶段](MCU_SUPPLY_REVIEW.md)已接通 MAIN_3V3 与主控地，USB 电源到 D1、供电输入和 VMOT 分配仍未完成。当前检查不证明 USB 枚举、下载、复位、故障恢复或 Wi-Fi/BLE 已在实板工作。USB 浪涌、挂起电流、回灌、ADC 掉电、电源动态、射频净空、成品报价和 JLCEDA 往返导入仍未完成。

上一版保存在 [USB 布线前检查点](../archive/usb_before_interface/README.md)。[版本记录](verification/usb_layout_revision.json)验证 123 个固件及原理图源文件未变，没有重跑或重新归属旧固件测试。ADC 条件模型继续绑定保存的旧 PCB，并未因本轮布局而重新验证。

从 `controller_core` 重现保存的 USB 阶段铜连接检查；当前完整局部铜检查命令见[复位布线说明](RESET_ROUTING_REVIEW.md)：

```sh
python3 verify_usb_interface.py --board archive/reset_before_control/top_board_v2_logic.kicad_pcb --baseline archive/usb_before_interface/top_board_v2_logic.kicad_pcb
```

`route_usb_interface.py` 是本轮明确坐标方案，只接受上一版 103 段线的基底；不得直接重新添加到当前板。上述检查不访问实物串口。
