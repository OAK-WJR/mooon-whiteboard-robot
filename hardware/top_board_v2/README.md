# top_board_v2 — 五片顶部成品板

**只负责五片顶部 PCB、板上元件和焊接；合计目标 ¥30、上限 ¥50。** 底板、电机、电池及充电器不计入，不继续选购。顶部保留 ESP32、USB 自动恢复、供电和原底板接口，板上不加入充电或均衡。

当前为 **C6-MINI 两层布局审查稿，整板布线未完成**：[PCB 预览](jlc_budget_6v/controller_core/logic/preview/placement.png)、[KiCad 工程](jlc_budget_6v/controller_core/logic/top_board_v2_logic.kicad_pro)、[六页原理图 PDF](jlc_budget_6v/controller_core/logic/top_board_v2_review.pdf)。70 个元件、246 个物理引脚，ERC 0 错误/0 警告；固件 73 项离线检查及 C6/R3 交叉编译通过。19 个对接焊盘位置保留，名义配准最大偏差约 0.00103mm，顶板轮廓位于底板内。

**预算尚未通过。** MINI 的国际站料号为 Standard Only，不能沿用经济型贴装核价；该布局是空间/接线研究，不是低成本定案。[成本复核](jlc_budget_6v/COST_REVIEW.md)与[当前 63/70 位号已填料号的清单](jlc_budget_6v/TOP_COMPONENTS.md)均没有完整成品总价。

PCB 仍有 153 项未连接及一处 M2/PWR1 装配边界重叠。输入保护、USB 用电/浪涌、ADC 掉电与外部分压校准、供电动态、天线及实物验证未完成；运动保持禁用，不能下单制造。当前文件与完整范围见[审查说明](jlc_budget_6v/controller_core/logic/README.md)及[固件说明](jlc_budget_6v/controller_core/logic/FIRMWARE.md)。

## 历史版本

上一阶段方向为 [整车 6V、成品模块、复用底板](kiss_6v/README.md)。[两层电气工作稿](kiss_6v/electrical_layout/README.md)已开始局部布局布线，现因成本暂停；整板未完成、不可下单。下述 147 元件 R3 保留为历史审查参考，不再继续扩展双电压全定制架构。

历史模块版可看[55元件阶段载板预览](kiss_6v/electrical_layout/carrier_preview.png)或打开[暂停处的KiCad电气PCB](kiss_6v/electrical_layout/top_board_v2_6v.kicad_pcb)。预览只裁取板框附近，且尚未反映恢复区7个元件；最新原生文件有62个板内元件、103个板外待布局封装。顶部不限高，板框和原19个对接触点保留；充电、保护、完整接线及装配净空仍待完成。旧[C6顶部安装稿](kiss_6v/kicad_installation/README.md)及[Pico联合安装研究](kiss_6v/pico_joint_installation/README.md)均为历史资料。[模块复核](kiss_6v/MODULE_REVIEW.md)保留具体依据及限制。

**历史 R3 状态和文件入口见 [R3_STATUS.md](R3_STATUS.md)。** 它是 2S 电池、独立 USB 恢复方向的原生 KiCad 原理图审查稿，已整合 10 页、147 个元件，含固定看门狗和故障锁存；R3 电源硬件及 PCB 布局布线尚未完成。已有 [原理图 PDF](kicad/r3_system/top_board_v2_r3.pdf) 和 [随附本地符号/封装的 KiCad 审查工程](review_r3_schematic/top_board_v2_r3.kicad_pro)。旧架构核价参考见 [成本更新](procurement/R3_COST_UPDATE.md)，不适用于当前 6V 模块版，也不是完整 PCBA 报价。

**已布线并往返导入嘉立创的版本仍为 R2：** 已先在 KiCad 修正 D3/D5 极性，再导入嘉立创 `motor_controller → top_board_v2_kicad_r2`。请以 [R2 验证与文件入口](R2_VERIFICATION.md) 为准。嘉立创交付使用已通过往返核验的 `.epro2`；旧 `.epro` 会改变间距规则，已撤出交付入口，见 [往返记录](ROUNDTRIP_VERIFICATION.md)。下面保留的是上一轮转角修正版记录；整机仍未制造放行。

以下为历史转角修正版记录：修正版曾同步到嘉立创 **motor_controller → top_board_v2_kicad**，尚未制造放行。当时的整机 6V/12V 双电压要求已被上述 6V 单电压决定取代。

当前文件：[KiCad PCB](kicad/top_board_v2_refined_4layer.kicad_pcb)、[KiCad 工程](kicad/top_board_v2_refined_4layer.kicad_pro)、[独立嘉立创工程](top_board_v2_kicad.epro)、[完整工程备份](motor_controller_with_kicad_4layer.epro)。原 top_board_v2 和另外三块 PCB 保留；请打开带 _kicad 的新板查看本轮修改。

![嘉立创修正版回读](validation/reverification/eda_render_after.png)

59 处普通线路直角已改成分段 30° 转向；剩余 17 处检测点在焊盘/过孔连接处。使用明确指定的转折坐标，没有调用自动寻路器。固定接口保持原位并锁定。

四层决定：L1 元件和主要信号，L2 GND 参考平面，L3 3V3 分配与少量局部连接，L4 其余信号。实际厂家叠层、铜厚及 USB 阻抗仍待确认。

KiCad：49 元件、215 焊盘、539 段铜线、93 过孔，0 未连接，1 项 M2/PWR 装配边界重叠。EDA 拆分为 574 段线后，铜箔并集和网络比对通过，PCB DRC 返回 0 项。19 个上下板引脚逐一匹配，最大中心差约 0.00103 mm；顶板轮廓在底板内，面积约为其 81%。

当前未放行事项包括：约 6.6V 的固定电源设定、升压关闭后的被动通路、缺少采样和独立功率切断、USB 电源路径、天线下方底板投影、USB 阻抗、装配边界和原理图 1 项告警。不能把 12V 接到现有 BAT 1S 插口。

已新增 USB / Wi-Fi / BLE 主固件及 AI 可调用的 JSON 上位机，支持状态、停止、配置、重启和 USB ROM 恢复。当前集成板固件以 BOARD_POWER_UNREVIEWED 拒绝电机启动。没有实板测试结果。

- [完整验证报告](VERIFICATION.md)
- [USB、Wi-Fi、BLE 控制说明](../../control_tests/README.md)
- [USB 诊断与三轮测试方法](../../usb_diagnostics/README.md)
- [ngspice 图](validation/reverification/power_simulation.png)、[仿真脚本](simulate_power.py)
- [引脚与外形图](validation/reverification/mating_verification.png)

当前改用三台同规格的 6V、300rpm 编码器 N20；此前提供的 MECCANIXITY 12V 300RPM 型号保留为历史参考，不能作为 6V 版的性能依据。复用底板三颗 RZ7889-MS，详情和插座标注冲突见 [底板与电机复查](bottom_board_review/README.md)。用户已提供额定/堵转电流表及编码器线色功能，见[当前电机输入](jlc_budget_6v/motor_selection/README.md)；仍需确认实物与所选表格匹配、实际插头针序、编码器校准，以及板间距、电池和金属结构位置。
