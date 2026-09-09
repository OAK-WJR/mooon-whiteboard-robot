# M0 控制信号与主控地回流检查（554 段线历史阶段）

本页记录电源分配布线前的阶段；当前板见[后续说明](VMOT_DISTRIBUTION_REVIEW.md)。

本轮接通 **U3.28 / GPIO22 → M0.3 / FI → R12.1** 和 **U3.29 / GPIO23 → M0.2 / BI → R13.1**。M0 对接原底板 H2，对应实体 Motor1；M2 的两根控制线和全部六根编码器信号继续连通。M1 的 FI/BI 主控路径仍未完成，不能据此宣称三路电机已经工作。

该阶段主板为 **554 段线、156 个过孔、两块地铜、21 项未连接**。新增 38 段线、12 个过孔，替换六段局部 3.3V 线和两个过孔；没有增加元件或移动封装。全部 74 个封装、19 个对接触点、板框及规则保持。

[该阶段 PCB](../archive/vmot_before_distribution/top_board_v2_logic.kicad_pcb) · [该阶段预览](../archive/vmot_before_distribution/preview/placement.png) · [该阶段放大图](../archive/vmot_before_distribution/preview/right_motor_pair_detail.png) · [该阶段连接报告](../archive/vmot_before_distribution/top_board_v2_logic.verification.json)

## 路径与回流

两根控制线从模组右侧引出，换层跨过 UART，再沿右侧板边按 30°/60° 方向绕过 3.3V 电源区接到原 M0 下拉和排针。新控制线宽 0.25mm，过孔外径 0.6mm、孔径 0.3mm；接口端的转角加入斜段。采用明确坐标，没有自动寻路。

布线中检出了主控原先沿板边的地回路被分开。最终在 C6 附近增加一条明确的局部接地路径，经新地过孔接入原 C5/C4 与主控左下地回路。电容焊盘间的短地线为 0.2mm，后段为 0.3mm；C5 的一个并联供电过孔移位，C5 到 C6 的供电改从背面连接，保持两个输入过孔及 0.65mm 供电线宽。没有把去耦电容接到独立反馈采样支路。

这条短地回路通过了铜连接和间距检查；电源阻抗、射频回流和温升仍需要后续验证。

## 验证结果

KiCad 原生连接图逐个检查全部 30 个主控实体地焊盘，包含九个同名 49 脚；同时确认 22 个 MAIN_3V3 焊盘、六根编码器、M0/M2 四根完整控制线、全部六个下拉地端，以及所有原有实体焊盘连接组保持连通。246 个物理引脚网络与当前 XML 网表逐项一致。

660 个旧铜记录逐字保留，仅替换[坐标记录](right_motor_pair_review.routing.json)列明的八项。74 个封装完整记录、19 接点、轮廓、铺铜与禁布区定义、DRC 规则及有效项目设置不变。

八个独立故障副本分别移除两根 M0 主控引出线、两根外侧路径、两根接口连接、C5/C6 供电桥和新主控地过孔。每项均检出指定端点分离，未连接数由 **21 增到 22**；故障副本没有替换主板。没有放宽短路、间距、板边或孤立铜检查。

当前 DRC 只剩原有的 **M2/PWR1 装配边界重叠**，没有新增非连接类错误。[版本记录](verification/right_motor_pair_layout_revision.json)核对固件和原理图源文件未变；本轮没有重复运行此前固件或子电路模拟。

## 仍未完成

[当时剩余 21 项连接](../archive/vmot_before_distribution/remaining_connections.json)包括 M1 的两根控制线、ADC 信号及四个采样地端、电源输入和 VMOT 分配。完整输入连接器与保护、ADC 掉电、实际 USB 自动恢复、默认关断、负载动态、温升、射频、装配及 JLCEDA 导入往返仍待验证。五片顶部成品的预算未通过，没有制造放行。

在 `controller_core` 验证该阶段板：

```sh
python3 verify_right_motor_pair.py --board archive/vmot_before_distribution/top_board_v2_logic.kicad_pcb --baseline archive/right_motor_before_pair/top_board_v2_logic.kicad_pcb
```

[522 段线检查点](../archive/right_motor_before_pair/CHECKPOINT.md)保存布线前的主板、预览与原报告。`route_right_motor_pair.py` 仅接受该基底，不能对当前板重复添加。
