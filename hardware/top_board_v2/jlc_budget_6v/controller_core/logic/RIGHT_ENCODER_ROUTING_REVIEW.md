# M0 编码器布线与六路连接检查（522 段线历史阶段）

本页记录 M0 控制线布线前的阶段；当前板见[后续说明](RIGHT_MOTOR_ROUTING_REVIEW.md)。

本轮完成 **M0.5 / ENC0_A → U3.12 / GPIO0** 和 **M0.4 / ENC0_B → U3.13 / GPIO1**，保留 R14/R15 上拉。M0 对接原底板 H2，对应实体 Motor1。至此，顶部三组编码器的六根 A/B 信号均已在 CAD 中接通；这不是实际线束、编码器方向或带载测试结果。

该阶段主板为 **522 段线、146 个过孔、两块地铜、23 项未连接**。新增 78 段线、22 个过孔，替换 12 段线和 7 个过孔。全部 74 个封装及原 19 个对接触点保持不变，没有改动元件位置、BOM、原理图或固件。

[该阶段 PCB](../archive/right_motor_before_pair/top_board_v2_logic.kicad_pcb) · [该阶段预览](../archive/right_motor_before_pair/preview/placement.png) · [该阶段放大图](../archive/right_motor_before_pair/preview/right_encoder_pair_detail.png) · [该阶段连接报告](../archive/right_motor_before_pair/top_board_v2_logic.verification.json)

## 路径与局部调整

M0 A 从主控下边直接引出，M0 B 从模组内侧换层。两条信号在启动配置与供电之间穿过，分别换层跨越既有 UART 和 3.3V 分配，再从 U4 右侧进入 M0 接口。启动配置附近的短背面通道用 0.2mm 线宽，其余新增编码器线为 0.25mm；新增过孔保持 0.6mm 外径、0.3mm 孔径。全部路径使用明确坐标及 30° 的整数倍方向，没有自动寻路。

R25 供电过孔下移，GPIO15 信号路径保持；U4 输入背面支路侧移，保持 0.6mm 线宽，C8 地过孔与其短连接作相应调整。U4 的反馈采样与两条明确接地回路保持，编码器没有接入电源采样支路。M2 编码器 A 在模组内部的小桥改道，为 14 脚接地引出让位；M2 接口端与主控脚号不变。

新走线曾分开主控下边的地铜。现已增加明确的地支路，将 11/52 脚及 14 脚接回公共地；相关地过孔移到不阻挡信号的位置。所有 30 个主控实体地焊盘均逐个检查，包括九个同名 49 脚，未依赖模组内部接地来代替顶板铜连接。

## 验证

KiCad 原生连接图确认全部六组编码器网络，每组均覆盖主控脚、对应排针和上拉端。所有此前连通的实体焊盘组继续连通；全部 22 个 MAIN_3V3 焊盘、30 个主控地焊盘、六个控制下拉地端及 246 个物理引脚网络保持正确。568 个旧铜记录逐字保留，仅替换[坐标方案](right_encoder_pair_review.routing.json)列明的 19 项；封装、板框、规则、网络类别、地铜及禁布区定义不变。

八个临时副本分别切断 M0 A/B 主控引出和接口支路、R25 供电、U4 输入支路、改道的 M2 编码器 A 小桥及新的左下地过孔。每项均检出对应断开，未连接数由 **23 增到 24**。另一个副本移除 14 脚的新地过孔后，所有焊盘连接组仍保持、未连接数仍为 23，确认该处已有并联地回路；该故障副本产生的孤立铜警告保留在报告中，仅证明焊盘连接保持。没有把删除冗余过孔误记成断路测试通过，也没有放宽主板的孤立铜检查。

当前 DRC 未报告新增短路、铜间距、板边、过孔、悬空或孤立铜错误，仍报告原来的 **M2/PWR1 装配边界重叠**。[版本记录](verification/right_encoder_pair_layout_revision.json)核对 123 个固件及原理图源文件未变，既有固件检查未重复运行。

## 仍未完成

[当时剩余 23 项连接](../archive/right_motor_before_pair/remaining_connections.json)包括 M0/M1 到主控的四根控制线、采样及四个采样地端、电源输入和 VMOT 分配。完整输入连接器与保护、ADC 掉电、实际默认关闭、USB 供电与自动恢复、负载动态、温升、射频和装配验证仍未完成。两层铜连接通过不能证明运行稳定性，五片顶部成品预算仍未达标，没有制造放行。

在 `controller_core` 验证该阶段板：

```sh
python3 verify_right_encoder_pair.py --board archive/right_motor_before_pair/top_board_v2_logic.kicad_pcb --baseline archive/right_encoder_before_pair/top_board_v2_logic.kicad_pcb
```

[456 段线检查点](../archive/right_encoder_before_pair/CHECKPOINT.md)保存本轮前的主板。`route_right_encoder_pair.py` 仅接受该基底哈希，不可重复添加到当前板。
