# 主控与编码器 3.3V 分配、地回流布线

此文记录主控供电阶段：新增 **66 段线、32 个过孔和两块地铜**，以新路径替换两段 BOOT 线，调整 C5、C6、R21、R23 四个元件。当时为 **319 段线、88 个过孔、6 块地铜、51 项未连接**。仍保留原有的一处 M2/PWR1 装配边界重叠，没有放宽规则或增加检查豁免。

[当前 PCB](top_board_v2_logic.kicad_pcb) · [整板预览](preview/placement.png) · [主控供电放大图](preview/mcu_supply_detail.png) · [该阶段原生连通报告](../archive/controls_before_routing/top_board_v2_logic.verification.json)

## 连接与布局

3.3V 主干从 L1 的输出焊盘直接引出，使用 0.8mm 铜线和输出端的两个 0.4mm 孔径过孔。它接到主控旁 C5/C6、U3.3、CH343 VIO、启动配置上拉、六个编码器上拉和 M0.1 编码器电源。原有细线仅保留为 U4 输出反馈采样，没有用它给新增负载供电。

主干在背层沿板的右侧前往主控；穿过背层串口线时用两条正面短桥及成对过孔换层。BOOT 仅局部改道以让出该区域。R21、R23 移开后，VIO 与中间编码器上拉通过原 M2 排针之间的实际间隙布线。原 19 个对接焊盘及所有孔、方向、轮廓保持不变。

C5 改向，C6 下移，形成局部 10µF / 100nF 去耦回路；局部供电 0.65mm，模组小焊盘处为 0.4mm 引出。C5/C6 地端各有就近过孔。模组下增加正面地铜和 12 个地过孔，过孔孔径 0.3mm，安排在接地焊盘的间隙和周围。背面增加按板框填充的地铜，保留原天线禁布区及安装孔禁布区。地铜与主控的 **30 个实体接地焊盘**、U4/U5 和 USB 地连接。

原厂强调模块基板应有足够地铜及接地过孔，并需在最终结构中验证无线通信。本次采用的具体线宽和过孔安排是设计选择，不构成原厂对这块板的认证；同一指南内的裸芯片电源脚编号没有套用到 MINI 模组。[Espressif PCB 指南：模块在基板上的布局](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32c6/pcb-layout-design.html#general-principles-of-pcb-layout-for-modules-positioning-a-module-on-a-base-board)。

## 验证范围

- 全部 22 个 MAIN_3V3 实网表焊盘属于同一铜连接组。
- 逐一检查主控全部 30 个接地焊盘对象，包括同名 49 脚的九个独立焊盘，均接到稳压器和 USB 的公共地组。没有用其中一个 49 脚代表其余八个。
- 原板所有已连通的实体焊盘组保持连通；全部 246 个引脚网络与当前原理图 XML 一致。
- 原有 309 个线段/过孔记录逐字保留，仅替换明确列出的两段 BOOT 线；70 个未移动封装的完整记录不变，包括 19 个对接触点和 Type-C 定位孔。
- 四个临时副本分别切断 3.3V 主干、MCU 电源引出、VIO 支路和改道后的 BOOT 线。原生连通图均检出预期断开，DRC 未连接项从 **51 增至 52**。对照中的悬空警告保留在报告，不作为主板豁免。
- 两层、板框、设计规则与网络类别不变；原 M2/PWR1 装配问题继续报告。19 点上下板配准的最大名义残差仍为约 0.00103mm。

[版本记录](verification/mcu_supply_layout_revision.json)核对既有 123 个固件及原理图源文件未变。原理图、BOM 和固件没有因此增加器件或改变功能，也没有重复运行无关固件测试。

## 仍未完成

后续[控制布线阶段](MOTOR_CONTROL_ROUTING_REVIEW.md)已接通 ARM、GPIO15 和 M2 控制；供电输入、VMOT 分配、其余电机/编码器及采样线路仍未完成。当前总数和地铜定义以[主板说明](README.md)为准。主控电源网布完不代表电源输入已可用，更不证明实物启动、自动复位、USB 下载或无线功能已正常。

这轮没有进行闭环稳压器模型、负载瞬态、温升、实际压降或射频测试。C5 的精确商城料号和有效容量、输入连接器与保护、ADC 掉电问题、USB 电流额度/挂起/浪涌/回灌、装配净空与实际 RF 公差仍未定案。原天线到下板的约 0.046mm 名义间隙仍不足以证明制造后的净空。五片顶部成品成本未达标，没有制造放行。

在 `controller_core` 验证本阶段存档：

```sh
python3 verify_mcu_supply.py --board archive/controls_before_routing/top_board_v2_logic.kicad_pcb --baseline archive/mcu_before_supply/top_board_v2_logic.kicad_pcb
```

[255 段线检查点](../archive/mcu_before_supply/CHECKPOINT.md)保留本轮前的主板。`route_mcu_supply.py` 仅接受该旧版哈希，不可再次添加到当前主板；它使用明确坐标，未调用自动布线器。
