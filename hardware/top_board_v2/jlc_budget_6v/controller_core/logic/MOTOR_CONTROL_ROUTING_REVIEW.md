# 使能、启动配置与底板控制接口布线

此文记录控制布线阶段：新增 **59 段线、22 个过孔**，移走 R25 原供电支路的三段线和一个过孔，调整 R20、R21、R25。当时为 **375 段线、109 个过孔、两块地铜、35 项未连接**。没有增加元件，六页原理图、70 件 BOM 和固件未改。后续编码器改动及当前总数见[主板说明](README.md)。

[当前 PCB](top_board_v2_logic.kicad_pcb) · [整板预览](preview/placement.png) · [控制区域放大图](preview/motor_controls_detail.png) · [该阶段原生验证](../archive/encoder_before_links/top_board_v2_logic.verification.json)

## 已接通

| 网络 | 本轮核对的实际焊盘连接 |
| --- | --- |
| MOTOR_ARM_REQUEST | U3.19 / GPIO14 → U5.2 / EN、R24.1 / 10kΩ 下拉 |
| JTAG_SEL_HIGH | U3.20 / GPIO15 → R25.2；R25.1 接 MAIN_3V3 |
| M2_FI | U3.26 / GPIO20 → M2.3、R20.1 |
| M2_BI | U3.27 / GPIO21 → M2.2、R21.1 |
| M0_FI / M0_BI | M0.3 → R12.1；M0.2 → R13.1，尚未接到主控 |
| M1_FI / M1_BI | M1.3 → R16.1；M1.2 → R17.1，尚未接到主控 |
| 控制下拉的地 | R12.2、R13.2、R16.2、R17.2、R20.2、R21.2 → 公共地 |

四个对接连接器仍是原有位置、孔径和方向。M0/M1/M2 均以 **2 脚 BI、3 脚 FI** 接线；M2 的 4/5 脚分别是编码器 B/A，在本阶段尚未接完。实体 Motor1/2/3 对应 M0/M2/M1，不能按连接器编号猜测实体电机编号。

ARM 从主控下方引出，在正面穿过背面的电源线，再沿左侧去 U5 的 EN/下拉节点。GPIO15 上拉移到模组附近，删除原来的长供电支路。M2 的两根控制线从模组内侧引出，经换层避开使能、启动配置和 3.3V。全部坐标明确给定，使用 30° 的整数倍方向，没有自动寻路。

六个控制下拉补上就近地过孔。M2 的 R20/R21 移到能避开电源电容、排针孔和既有走线的位置。清除旧铺铜缓存后，删除四块已被整板地铜覆盖的历史局部铺铜定义，避免低优先级区域留下孤立铜；保留整板背面地、模组正面地和全部禁布区。模组下新增一个地过孔连接新信号路径之间的回流区域。

## 已验证的范围

KiCad 原生连接图检查八组新增网络、六个下拉地端；所有原先连通的实体焊盘组继续连通。全部 22 个 MAIN_3V3 焊盘和 30 个主控实体接地焊盘保持连接，其中九个同名 49 脚分别检查。全部 246 个引脚网络与当前 XML 相同。

403 个既有线段/过孔记录逐字保留，唯一删除的是明确记录的 R25 原供电支路。71 个未移动封装的完整记录不变，包含原 19 个对接触点、模组、Type-C 定位孔和四个安装孔。保留的铺铜和禁布区定义也逐项比较；DRC 规则、网络类别与板框未变。

五个临时副本分别切断 ARM、GPIO15、M2_FI、M2_BI 的主控引出，以及 R20 的地回路。原生连通图均检出预期断开，未连接数由 **35 增到 36**。这些对照只证明检查能识别相应铜断线，不是实物自动关断测试。

本阶段 DRC 没有新增短路、铜间距、板边、过孔或孤立铜问题，仍报告原有的一处 **M2/PWR1 装配边界重叠**。上下板 19 点配准最大名义残差仍约 0.00103mm；天线投影及其约 0.046mm 名义公差问题未解决。[阶段版本记录](../archive/encoder_before_links/verification/motor_controls_layout_revision.json)核对 123 个固件与原理图源文件未变，没有重复运行无关固件测试。

## 未完成

本阶段未完成的编码器连接已在后续[编码器布线阶段](ENCODER_ROUTING_REVIEW.md)部分接通；[当前未连接明细](remaining_connections.json)记录实际剩余项。M0/M1 到主控的控制线、采样、电源输入与 VMOT 分配仍未完成。板子仍不能作为完整成品制造或上电验收。

默认关闭还需要验证实际上电/欠压/掉电、USB 复位与下载、底板负载和 EN 行为；GPIO15 的作用仍依赖当前文档所述 eFuse 条件。原有离线测试和局部模拟没有因此变成实板证据。ADC 掉电、USB 取电/回灌、完整输入保护、功率动态与温升、射频、装配、公差、BOM 和五片成品预算仍未定案。

在 `controller_core` 验证本阶段存档：

```sh
python3 verify_motor_controls.py --board archive/encoder_before_links/top_board_v2_logic.kicad_pcb --baseline archive/controls_before_routing/top_board_v2_logic.kicad_pcb
```

[319 段线检查点](../archive/controls_before_routing/CHECKPOINT.md)保留本轮前的主板。`route_motor_controls.py` 只接受该旧版哈希，不能重复添加到当前主板。
