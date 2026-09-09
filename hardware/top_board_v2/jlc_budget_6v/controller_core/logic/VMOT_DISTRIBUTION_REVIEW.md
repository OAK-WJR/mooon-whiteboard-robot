# VMOT 分配与 ADC 接地检查

本文件记录 566 段线阶段；当前主板见[USB/OR 输入布线检查](LOGIC_INPUT_ROUTING_REVIEW.md)。

本轮用 **1.2mm 正面铜线**把 C19 输出端接到 PWR1 的 2/3/4 三个 VMOT 触点，R35 的输入端从该宽铜路径取样；同时接通 C20/R34 与 C21/R36 的四个接地端。全部 **74 个实体 GND 焊盘**已在 KiCad 铜连接图中属于同一组。

本阶段主板为 **566 段线、158 个过孔、两块地铜、13 项未连接**。新增 12 段线和两个地过孔，710 个旧铜记录全部保留；全部 74 个封装、19 个对接触点、板框、铺铜定义及 DRC 规则不变，没有增加元件或修改原理图、BOM、固件。

[本阶段 PCB](../archive/logic_before_input/top_board_v2_logic.kicad_pcb) · [整板预览](../archive/logic_before_input/preview/placement.png) · [电源分配放大图](../archive/logic_before_input/preview/vmot_distribution_detail.png) · [原生连接报告](../archive/logic_before_input/top_board_v2_logic.verification.json)

## 电源与回流路径

新增负载供电从 C19 的输出焊盘出发，沿 30°/60° 方向进入中间供电触点，再分配到两侧触点。原 L2 到 C19 的 1.0mm 段保持，新增分配段为 1.2mm；没有把负载接到 R28 的独立反馈细线或 R30 的泄放细线上。线宽和连通检查不代替闭环稳定性、压降、启动冲击或热验证。

两组 ADC 下臂与滤波电容分别通过 0.3mm 铜线及 0.6/0.3mm 地过孔接入公共地。C21/R36 的过孔向右侧移，避开背面的 ENC1_B；两处孔位均在贴片焊盘外。采样输入、MCU 信号及掉电保护仍未完成，本轮只接通已有地端。

## 验证

原生连接图确认 VMOT 九个焊盘连通：L2.2、C18.1、C19.1、R28.1、R30.1、R35.1 及 PWR1.2/3/4。全部 74 个实体地焊盘、22 个 MAIN_3V3 焊盘、M0/M2 控制线、六根编码器，以及所有以前连接的实体焊盘组保持正确；246 个物理引脚网络与 XML 网表逐项一致。

七个故障副本分别移除两段新增供电路径、两段触点分配线、原 L2 到 C19 线段及两个新地过孔，均检出指定端点断开，未连接数由 **13 增到 14**。位于同一焊盘铜皮内的短段没有被当作有效断线对照。

另一个副本移除全部六段旧的 0.3mm 背面 VMOT 反馈/泄放线。R28 与 R30 的输入按预期断开，未连接数增到 **15**；L2、C18、C19、R35 和三个供电触点仍相互连通。这项检查确认负载铜路径不依赖上述细线，没有把反馈支路断开误称为整板正常运行。

主板 DRC 只剩原有的 **M2/PWR1 装配边界重叠**，没有新增短路、铜间距、板边或孤立铜错误。[版本记录](../archive/logic_before_input/verification/vmot_distribution_layout_revision.json)核对 123 个固件与原理图源文件未变；此前固件与子电路模拟没有因本轮铜连接检查变成实板证据。

## 未完成

[剩余 13 项连接](../archive/logic_before_input/remaining_connections.json)包括 M1 的两根控制线、两路 ADC 信号、电池与 USB 输入及供电 OR。M1 在主控附近仍需局部调整，当前板不能驱动三路电机完成验收。完整输入连接器与保护、ADC 掉电、USB 自动恢复、默认关断、功率动态、温升、射频、装配、完整五片成品报价和 JLCEDA 导入往返仍未通过。

在 `controller_core` 验证本阶段板：

```sh
python3 verify_vmot_distribution.py --board archive/logic_before_input/top_board_v2_logic.kicad_pcb --baseline archive/vmot_before_distribution/top_board_v2_logic.kicad_pcb
```

[554 段线检查点](../archive/vmot_before_distribution/CHECKPOINT.md)保存布线前的主板与预览。`route_vmot_distribution.py` 仅接受该基底，不能对当前板重复添加。没有制造放行。
