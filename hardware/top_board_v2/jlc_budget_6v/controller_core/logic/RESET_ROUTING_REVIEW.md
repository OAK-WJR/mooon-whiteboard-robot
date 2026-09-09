# 自动复位与串口的手工布线检查

此文记录复位阶段：新增 **85 段线、18 个过孔和一块背面地铜**，调整 11 个元件的位置或方向。当时整板为 **255 段线、56 个过孔、98 项未连接**，仍有原来的一处 M2/PWR1 装配边界重叠。没有增加元件，原理图、BOM 和固件未变；五片顶部成品预算仍未满足。

[当前 PCB](top_board_v2_logic.kicad_pcb) · [整板预览](preview/placement.png) · [复位控制区](preview/reset_control_detail.png) · [主控 EN/BOOT 区](preview/mcu_reset_detail.png) · [该阶段原生检查报告](../archive/mcu_before_supply/top_board_v2_logic.verification.json)

## 已接通的局部网络

| 功能 | 已核对的铜连接 |
| --- | --- |
| DTR | U1.10 → R3.1、R6.2、Q2 发射极 |
| RTS | U1.11 → R4.1、R7.2、Q1 发射极 |
| EN 驱动 | R3.2 → Q1 基极；Q1 集电极 → R5.1 |
| 主控 EN | R5.2、22kΩ 上拉 R8.2、1µF C4.1 → U3.8 |
| BOOT 驱动 | R4.2 → Q2 基极；Q2 集电极、上拉 R9.2 → U3.23 / GPIO9 |
| GPIO8 启动配置 | R10.2 → U3.22 / GPIO8 |
| 主控接收串口 | U1.2 / TXD → U3.30 / GPIO17 |
| 主控发送串口 | U3.31 / GPIO16 → R11.2；R11.1 → U1.3 / RXD |

R5、R8 和 C4 移到模组 EN 引脚一侧；Q2 转向以便分开基极和集电极走线。R20 小幅移动，为 EN 支路留出实际铜间距；R25 避开原有电源反馈线。串口沿控制区右侧手动安排，R11 保持在主控发送端附近。方向为 30° 的整数倍，没有使用自动寻路。

R6/R7 上拉接到既有 CH343 VIO 去耦节点；R8/R9/R10 的电源端在另一组局部铜线上相连。当时两组供电与主控电源/地尚未接通；这些连接已在后续[主控供电阶段](MCU_SUPPLY_REVIEW.md)完成，当前状态以[主板说明](README.md)为准。 表中的网络连通不代表板子已经能够启动、诊断或自动下载。

## 实际验证

KiCad 原生连通图核对 **33 组局部连接**，包括已有电源区、USB、复位和串口；全部 **246 个物理引脚网络**与当前 XML 一致。旧版 **208 个走线/过孔记录**逐字保留，63 个未调整封装的完整记录也相同，包含主控、Type-C 定位孔及原有 19 个对接触点。

四个临时副本分别切断 EN、GPIO9、GPIO17 接收、GPIO16 发送的主控引出线。连通图均检出对应断线，DRC 未连接项均由 **98 增至 99**。GPIO17 的前层引出线切断后，其过孔只剩背层连接，产生预期的悬空过孔警告，已保留在对照记录；没有把它加入主板的检查豁免。

主板没有新增短路、铜间距、板边或装配边界错误。自定义规则、展开后的 KiCad 设计参数及网络类别与上一版相同。上下板 19 点配准重新计算，最大名义残差仍约 0.00103mm；天线二维投影未变，原有公差与射频限制仍未解决。

## 三极管料号与限制

核对了嘉立创基础料 **JSCJ MMBT3904(RANGE:100-300) / C20526**，为 SOT-23、支持经济型和标准型贴装。厂家 Rev.2.1 给出的 1 基极、2 发射极、3 集电极，与本图符号和原生焊盘编号一致。[JLC 料号页](https://jlcpcb.com/partdetail/21237-MMBT3904/C20526)、[厂家手册](https://datasheet.lcsc.com/datasheet/pdf/9335632bcfa83df8351353eab379720e.pdf?productCode=C20526)。

该料号仍是经脚位核对的候选，尚未写入主 BOM，也未取得本批次价格或库存确认。原通用符号继承的 PZT3904 链接不能用于 SOT-23 封装核对。厂家 hFE=100–300 的测试条件是 VCE=1V、IC=10mA，不是本电路小电流或全温度的保证；旧通用 NPN 仿真没有因此变为厂家模型。依据与限制保存在[料号检查记录](../sources/reset_transistor_review.json)。

## 剩余工作

后续阶段已接通 MAIN_3V3 分配与主控地；后续控制阶段也已接通 GPIO15 与 M2 控制；供电输入、VMOT 分配和其余底板信号仍未完整；输入保护、ADC 掉电、USB 用电及回灌、电源动态、射频、完整成品成本与 JLCEDA 往返仍待完成。实际复位时序、启动配置、USB 下载和故障恢复尚无实板证据。

[版本记录](verification/reset_layout_revision.json)核对 123 个固件及原理图源文件未变；没有重跑或重新归属旧固件测试。上一版保存在[复位布线前检查点](../archive/reset_before_control/README.md)。

从 `controller_core` 验证存档的复位阶段铜连接：

```sh
python3 verify_reset_control.py --board archive/mcu_before_supply/top_board_v2_logic.kicad_pcb --baseline archive/reset_before_control/top_board_v2_logic.kicad_pcb
```

`route_reset_control.py` 只接受上一版 170 段线的基底，不能重复添加到当前板。检查不访问实物串口。
