# USB 与逻辑电源输入布线检查

USB VBUS 已接到 D1 的阳极；D1/D2 的阴极汇合后接入 U4 输入和 C7/C8 去耦。U4 附近增加一段正面地跨线，接回公共地。D2 的电池侧输入仍待布线，本阶段不能证明双输入供电已经可用。

主板为 **587 段线、167 个过孔、两块地铜、10 项未连接**。新增 21 段线和九个过孔，724 个旧铜记录全部保留；74 个封装、19 个对接触点、板框、铺铜定义、原理图、BOM 和固件均未改变。

[当前 PCB](top_board_v2_logic.kicad_pcb) · [整板预览](preview/placement.png) · [输入路径放大图](preview/logic_input_detail.png) · [原生连接报告](top_board_v2_logic.verification.json)

新增供电线宽 0.8mm，沿 30°/60° 方向和水平、垂直方向布线，以局部换层跨过既有信号和 3.3V 铜线。USB 取电从接口的既有 VBUS 过孔分支，避开 MH4 安装区域；它不依赖 CH343 的 0.3/0.4mm 背面供电支路。原接口焊盘到过孔及 C7/C8 区域的供电铜保持原线宽。新增地跨线宽 0.4mm，位于 U4 输出电感附近，连接输入侧地与 C11 所在公共地。

新过孔为 0.6mm 焊盘、0.3mm 钻孔。连接图确认全部 74 个实体地焊盘属于同一组，但这不证明回流阻抗、开关噪声或瞬态行为合格。

## 验证范围

- 原生网表的二极管符号定义为 1 脚 K、2 脚 A；PCB 中 D1 的 A 接 VBUS_RAW，D2 的 A 接 VBAT_PROTECTED，两个 K 接 LOGIC_VIN_OR。该检查核对符号和铜网，不代替实物极性、漏电或其他芯片引脚的回灌验证。
- USB 输入网七个焊盘和逻辑 OR 网六个焊盘均连通；全部原有连接组、22 个 MAIN_3V3 焊盘、九个 VMOT 焊盘、六根编码器、M0/M2 控制信号保持连通。246 个引脚网络逐项符合当前 XML 网表。
- 八个故障副本分别移除七处供电路径和新增 U4 地过孔，均在原生连接图中检出对应端点中断，DRC 未连接项由 10 增到 11。
- 另一个副本移除七段旧 CH343 背面供电支路后，两个 USB VBUS 触点仍与 D1 和 U2 相连；CH343 的两个电源输入按预期断开，未连接项为 12。这验证新增取电路径的独立性。
- 主板 DRC 没有新增短路、铜间距、板边或孤立铜错误，仍有原来的 M2/PWR1 装配边界重叠。该问题未被屏蔽或豁免。

[版本记录](verification/logic_input_layout_revision.json)核对未修改的固件与原理图来源。原有离线测试没有因此成为新 PCB 的实板证据。

## 仍需完成

[10 项未连接](remaining_connections.json)包括 M1 两根控制线、VBAT_PROTECTED 两处连接及两路 ADC 各三处连接。完整输入连接器与保护、ADC 掉电、USB 电流额度/浪涌/挂起/切换回灌、自动恢复、供电动态和温升、实际叠板与射频、五片成品价格及 JLCEDA 导入往返仍待验证。没有制造放行。

在 `controller_core` 目录复核当前板：

```sh
python3 verify_logic_input.py --board logic/top_board_v2_logic.kicad_pcb --baseline archive/logic_before_input/top_board_v2_logic.kicad_pcb
```

[566 段线检查点](../archive/logic_before_input/CHECKPOINT.md)保存修改前的主工程和预览。`route_logic_input.py` 仅接受该基底，不可重复添加到当前板。
