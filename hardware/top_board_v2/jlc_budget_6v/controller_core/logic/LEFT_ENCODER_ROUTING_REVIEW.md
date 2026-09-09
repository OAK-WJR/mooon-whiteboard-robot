# M1 编码器 A/B 到主控的布线

此文记录 M1 阶段，完成 **M1.5 / ENC1_A → U3.5 / GPIO2** 和 **M1.4 / ENC1_B → U3.6 / GPIO3**，保留 R18/R19 上拉。M1 对接原底板 H1，对应实体 Motor3；不能按连接器编号猜测电机编号。M2 的编码器路径保持接通，本阶段结束时 M0 的两根主控引线仍未完成，后续进展见[主板说明](README.md)。

当时主板为 **456 段线、131 个过孔、两块地铜、25 项未连接**。新增 40 段线、16 个过孔，替换 6 段线和 3 个过孔。全部 74 个封装记录保持，未改变元件位置、BOM、原理图或固件。

[当前 PCB](top_board_v2_logic.kicad_pcb) · [整板预览](preview/placement.png) · [左侧编码器放大图](preview/left_encoder_pair_detail.png) · [该阶段原生铜连接报告](../archive/right_encoder_before_pair/top_board_v2_logic.verification.json)

## 路径与回流

M1 A 从主控左侧引出，经正面短桥跨过供电，沿左板边前往已连通的 R18。M1 B 从模组内侧换层，经正面跨过 3.3V 和 6V 的既有铜线；在 U5 使能线与反馈采样线处换层，最后以 30° 支路接入原来的编码器 B 网络。全部新信号线为 0.25mm，坐标明确给定，没有自动寻路。

主控旁一段 3.3V 背面铜线向内调整，保持 0.65mm 线宽和原供电过孔；C4 地过孔移入 0.3mm，保留其短接地线。ARM 跨越供电的局部路径让出编码器引线位置，U3.19、U5.2 和 R24.1 的连接不变。

重新填铜后曾发现主控下边地铜失去公共回路，已用两条明确地支路和两个过孔重新接通。检查分别覆盖 U3 的 30 个实体地焊盘，包括九个同名 49 脚；没有用模组内部可能存在的接地连接替代顶板铜连接。

## 验证与限制

KiCad 原生连接图确认 M1 A/B 全路径，以及所有原有实体焊盘连接组。全部 22 个 MAIN_3V3 焊盘、30 个主控地焊盘和六个控制下拉地端保持连通；246 个物理引脚网络与原理图 XML 一致。531 个旧铜记录逐字保留，仅替换[坐标方案](left_encoder_pair_review.routing.json)明确列出的九项；19 个对接触点、板框、封装、地铜/禁布区定义、网络类别和规则均保持。

九个临时副本分别断开 M1 A/B 主控引出和接口支路、改道的 3.3V 支路、C4 地线、ARM 路径及两个新接地过孔。每个副本均检出对应断开，未连接项从 **25 增到 26**。地铜与短接地线并联，因此两个主控接地对照实际移除了过孔，未把删除冗余铜线当作断路。

DRC 仍有原来的 **M2/PWR1 装配边界重叠**，没有新增铜间距、短路、悬空或孤立铜错误。图纸采用现有两层规则；这些结果不证明制造公差、温升、负载动态、USB 或无线稳定性。[阶段版本记录](../archive/right_encoder_before_pair/verification/left_encoder_pair_layout_revision.json)核对 123 个固件及原理图源文件未变，既有固件测试未重复运行。

后续已完成[M0 编码器主控布线](RIGHT_ENCODER_ROUTING_REVIEW.md)；[当前剩余明细](remaining_connections.json)仍包含 M0/M1 控制主控引线、采样、电源输入和 VMOT 分配。ADC 掉电、完整输入连接器与保护、实际默认关闭、USB 自动恢复、射频和装配验证仍未完成。五片顶部成品预算仍未达标，没有制造放行。

在 `controller_core` 验证本阶段存档：

```sh
python3 verify_left_encoder_pair.py --board archive/right_encoder_before_pair/top_board_v2_logic.kicad_pcb --baseline archive/left_encoder_before_pair/top_board_v2_logic.kicad_pcb
```

[422 段线检查点](../archive/left_encoder_before_pair/CHECKPOINT.md)保留本轮前的主板。`route_left_encoder_pair.py` 仅接受该基底哈希，不可重复添加到当前板。
