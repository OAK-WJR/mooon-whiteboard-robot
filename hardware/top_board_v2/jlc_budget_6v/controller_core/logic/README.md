# 顶部板：MINI 原理图与两层布局审查稿

当前预算只覆盖 **五片顶部 PCB、板上元件和焊接成品**，合计目标 ¥30、上限 ¥50。当前候选没有满足预算，也未完成布线或制造验证。

- [KiCad 工程](top_board_v2_logic.kicad_pro) / [PCB](top_board_v2_logic.kicad_pcb) / [布局预览](preview/placement.png)
- [六页原理图 PDF](top_board_v2_review.pdf) / [根原理图](top_board_v2_logic.kicad_sch)
- 分页：[目录](preview/index.png)、[主控](preview/controller.png)、[USB 恢复](preview/usb_recovery.png)、[底板接口](preview/bottom_interfaces.png)、[采样](preview/voltage_sense.png)、[电源](preview/power_stage.png)
- [实际网表](logic.net.xml)、[原理图验证](verification/report.json)、[布局记录](placement_report.json)、[DRC](placement_drc.json)、[上下板配准](bottom_alignment.json)、[固件](FIRMWARE.md)

原理图为 **6 页、70 个实体元件、246 个物理引脚，ERC 0 错误、0 警告**。主控、USB 自动恢复、原底板接口、两路分压与两路降压已合并；ARM 下拉 R24 只装一颗。底板三颗 H 桥继续复用，顶部没有板上充电或均衡。

## MCU 与引脚

空间审查候选改为 **ESP32-C6-MINI-1-N4 / C5736265**。在保留 19 个通孔接点与天线禁布区的保守粗网格筛选中，原 WROOM 没有找到位置；这不是数学上不可布局的证明。MINI 试布局可放入现有板框，但其国际站料号为 **Standard Only、需 X 光检查**，不能沿用经济型贴装价格。因此这份图是空间与接线审查候选，**没有获得低成本定案**。[JLC 料号限制](https://jlcpcb.com/partdetail/ESPRESSIFINC-ESP32_C6_MINI_1N4/C5736265)。

MINI 按[Espressif v1.5 手册](https://www.espressif.com/sites/default/files/documentation/esp32-c6-mini-1_mini-1u_datasheet_en.pdf)表 3-1 和推荐焊盘独立核对，使用 53 个物理脚号、61 个原生焊盘对象，含 9 个同属 49 脚的接地焊盘。不能沿用 WROOM 的 29 脚映射。MINI 不引出 GPIO10/11，故 ARM 改为 **GPIO14 / 19 脚**，M0_BI 改为 **GPIO23 / 29 脚**；相应固件已同步。

| 实体电机 | 顶板 → 底板 | FI / BI GPIO | 编码器 A / B GPIO |
| --- | --- | --- | --- |
| Motor1 | M0 → H2 | 22 / 23 | 0 / 1 |
| Motor2 | M2 → H3 | 20 / 21 | 6 / 7 |
| Motor3 | M1 → H1 | 18 / 19 | 2 / 3 |

UART TX16/RX17 对应模组 31/30 脚；USB 原生 GPIO12/13 暂不使用。GPIO4/5 的采样物理脚为 9/10。GPIO15 上拉及 GPIO6/7 的编码器使用依赖默认 JTAG 选择，不能覆盖任意 eFuse 配置。GPIO14 在芯片复位阶段浮空，外加 10k 下拉；这不等于已经验证实际电机供电默认关闭。[C6 芯片手册](https://www.espressif.com/sites/default/files/documentation/esp32-c6_datasheet_en.pdf)。

## 布局与接口

PCB 是两层、70 个电气封装、4 个 Ø2.4mm 安装孔，**587 段走线、167 个过孔、两块地铜、10 项未连接**。除四个原底板对接排针外，电气元件均在正面。布局坐标由[手工位置表](../placement.json)给定，没有使用自动布线器；走线沿 30° 的整数倍方向安排。当前孔周预留 Ø6mm，实际固定件和顶部托架尚未验证。

四个连接器的 19 个焊盘中心、尺寸、孔径、角度和通孔属性与原顶板逐项相同。PWR 改名为 PWR1：1 脚 GND，2/3/4 脚 VMOT。M0.1 为主控 3.3V，M1.1/M2.1 为 GND。底板 CAD 经约 30° 旋转及平移后，19 个对应点最大残差 **0.00103mm**，没有镜像；顶板轮廓在底板内。这是名义二维尺寸核对，不是实物公差或三维插合验证。

最新 DRC 保留 **一处 M2/PWR1 装配边界重叠**；没有屏蔽该项。当前放置元件未报告铜短路/铜间距错误，但只有上述局部铜线，不能代表整板已经连通。U4 固定封装相邻焊盘间距 0.15mm，使用仅作用于 U4 内部焊盘的 0.15mm 规则；普通铜间距保持 0.2mm，铣边间距 0.3mm，模组边缘接地焊盘局部为 0.2mm。这些采用 1oz 两层工艺依据，不能直接用于 2oz。[JLC 工艺能力](https://jlcpcb.com/capabilities/pcb-capabilities/)。

按用户允许“与底板一样的形状”，顶板改用底板配准后向内缩 0.05mm 的名义轮廓，约 69.23 × 60.15mm；原 19 焊盘、孔径和角度保持。这是 CAD 内缩量，不是成品尺寸公差保证。USB 向新底边移 2.5mm；MINI 上移 3.84mm，去耦电容和 TX 串阻随之调整。空间与选型依据见[筛选记录](../outline_fit_review/README.md)。

MINI 天线端伸出顶板约 **5.5mm**。原生全铜层禁布区与上下板的名义二维交叠均为 0，原位置对底板约有 61.37mm² 交叠。[投影核对](antenna_projection.json)测得禁布区到下板外框仅约 **0.046mm**、最近模组焊盘到顶板边约 **0.204mm**。前者小于普通铣边公差，**零名义交叠仍不代表实际射频净空合格**；制造公差、叠板高度、固定件和通信测试仍需验证。[原厂模组布板指导](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32c6/pcb-layout-design.html#general-principles-of-pcb-layout-for-modules-positioning-a-module-on-a-base-board)。

3.3V 电源区已重新安排 C7/C8、反馈阻容和使能电阻，保留两个 SW 线段，重画 FB 并加入输入、输出、使能及地回流铜线。[局部放大图](preview/u4_supply_detail.png)中，C8 靠近 U4 输入脚，C7 移到电感上方以避开原 M0 贯穿焊盘；反馈节点留在 U4 左侧，独立输出采样从 C11 取样，反馈地单独返回 U4 地脚过孔。右侧背面地铜给输入和输出电容提供回路。依据为 [TI TPS563252 布局指导 §8.4](https://www.ti.com/lit/ds/symlink/tps563252.pdf)；这仍需实板动态和热验证。

在先前 3.3V 阶段，KiCad 原生铜连接图核对了 VIN、GND、VOUT、SW、FB、EN 六组连接；临时副本断开输入线和输出线均检出预期断线，未连接数从当时的 174 增到 175。[该阶段的原始检查记录](../archive/u5_before_supply/top_board_v2_logic.verification.json)。本次 6V 阶段完整保留这些铜线和过孔，并重新核对六组连通关系。输入二极管到该区域已在后续输入布线阶段接通，其余电路仍未布完；3.3V 负载分配已在后续主控供电阶段接通，**局部连通不等于稳压器或整板已能工作**。后续负载不能接到细的独立输出采样支路上。

6V 区域将 L2 旋转 90°，重新放置 C14–C19、反馈阻容、EN 下拉 R24 和泄放 R30；没有增加元件。[6V 局部图](preview/u5_supply_detail.png)显示输入去耦、BST/SW、软启动、反馈、输出电容和泄放连接。R24 已移到 U5 的 EN 附近，MCU 到 EN 的连接已在后续控制布线阶段接通。独立输出采样和泄放支路在 C19 输出端过孔分别引出；R30 的支路不接在反馈分压节点上。布局按 [TI TPS62933 §12.1](https://www.ti.com/lit/ds/symlink/tps62933.pdf)缩短关键引脚路径；当前 1oz 铜、底板大电容及实际负载的热和动态行为尚未验证。

USB 接口完成本轮局部手动布线：U2 转向并靠近 Type-C 接口，数据线穿过防护焊盘，两组正反插触点、CC 下拉、USB 侧 VBUS 和去耦连接已核对。新增 67 段线、14 个过孔，没有增加元件。[USB 局部图](preview/usb_interface_detail.png)与[完整检查说明](USB_ROUTING_REVIEW.md)记录了定位孔避让、电容位置和未完成条件。

随后完成自动复位与串口的局部布线：EN 的阻容移到主控同侧，DTR/RTS、GPIO9、GPIO8 和 UART 收发按现有网表连接。新增 85 段线、18 个过孔，没有增加元件。[复位控制区](preview/reset_control_detail.png)、[主控 EN/BOOT 区](preview/mcu_reset_detail.png)和[完整说明](RESET_ROUTING_REVIEW.md)记录了具体连接与剩余供电条件。

在主控供电阶段接通了全部 22 个 MAIN_3V3 网络焊盘，包括主控、USB 串口芯片的 VIO、启动与编码器上拉、底板编码器供电。C5/C6 组成局部去耦回路，主控全部 30 个实体地焊盘接入公共地；两段 BOOT 线为供电换层短桥让位。[主控供电放大图](preview/mcu_supply_detail.png)与[本轮完整说明](MCU_SUPPLY_REVIEW.md)记录了 66 段新线、32 个新过孔及四个元件调整。

随后接通 GPIO14 到 U5 EN、GPIO15 上拉、M2 的 FI/BI 全路径，以及三组接口的六个下拉回路。R25 移到主控附近，R20/R21 避开排针孔和电源电容，原理图与 BOM 未变。旧的四块局部地铜定义由整板背面地接替，并保留模组正面地；重新填充后没有孤立铜错误。[控制区放大图](preview/motor_controls_detail.png)与[阶段说明](MOTOR_CONTROL_ROUTING_REVIEW.md)记录了这些改动。

先前接通六个编码器接口到对应上拉电阻的连接，并完成 M2 编码器 A/B 到主控的两条路径。当时 M0/M1 的四根编码器主控引线仍未连接。所有元件位置不变；GPIO15、M2_FI、BOOT 局部改道，一个地过孔移位，为编码器引出让位。新增 57 段线、11 个过孔，替换旧的 10 段线和 2 个过孔；外侧折线使用 30°/60° 转向。[编码器放大图](preview/encoder_links_detail.png)与[阶段说明](ENCODER_ROUTING_REVIEW.md)记录这些改动。

随后完成 M1 编码器 A/B 到主控 GPIO2/3 的路径，当时 M0 的两根编码器主控引线仍待完成。所有封装位置不变；主控局部 3.3V 支路保持原线宽改道，C4 地过孔与 ARM 局部路径让位，并补上主控下边两处明确地回流。新增 40 段线、16 个过孔，替换 6 段线和 3 个过孔。[左侧编码器放大图](preview/left_encoder_pair_detail.png)与[阶段说明](LEFT_ENCODER_ROUTING_REVIEW.md)记录这些改动。

随后完成 M0 编码器 A/B 到 GPIO0/1 的路径，顶部三组编码器的六根信号均已接通。所有封装位置不变；R25 供电支路、U4 输入铜线与 C8 地过孔局部调整，模组内部的 M2 编码器 A 小桥改道，并补齐下边地回流。新增 78 段线、22 个过孔，替换 12 段线和 7 个过孔。[右侧编码器放大图](preview/right_encoder_pair_detail.png)与[完整说明](RIGHT_ENCODER_ROUTING_REVIEW.md)记录该阶段路径。

随后完成 M0 的 FI/BI 到 GPIO22/23 的路径，按 30°/60° 方向沿右侧避开电源区。C5/C6 供电桥改到背面，保留两个供电输入过孔与 0.65mm 线宽；C6 附近增加明确的主控地回流。新增 38 段线、12 个过孔，替换六段线和两个过孔；全部封装、原理图、BOM 和固件未变。[右侧控制线放大图](preview/right_motor_pair_detail.png)与[完整说明](RIGHT_MOTOR_ROUTING_REVIEW.md)记录该阶段路径。

随后接通 C19 到 PWR1 三个供电触点的 VMOT 分配，以及 C20/R34、C21/R36 的四个接地端。新增 12 段线与两个地过孔，所有旧铜、封装、原理图、BOM 和固件保持。新增供电铜为 1.2mm，原 L2 到 C19 的 1.0mm 段不变；负载不接入反馈和泄放细支路。[电源分配放大图](preview/vmot_distribution_detail.png)与[完整说明](VMOT_DISTRIBUTION_REVIEW.md)记录该阶段路径。

本轮接通 USB VBUS 到 D1 及 D1/D2 阴极到 U4 输入的路径，并补充 U4 附近的地跨线。新增 21 段线和九个过孔，没有移除旧铜或修改封装、BOM、固件。新供电线为 0.8mm，直接从 USB 触点的既有过孔取电；D2 的电池侧输入仍未连接。[输入路径放大图](preview/logic_input_detail.png)与[完整说明](LOGIC_INPUT_ROUTING_REVIEW.md)记录当前路径。

[当前原生铜连接报告](top_board_v2_logic.verification.json)确认 USB 和供电 OR 网连通，以及全部 74 个实体地焊盘、22 个 MAIN_3V3 焊盘、九个 VMOT 焊盘、M0/M2 控制线及六根编码器保持连通。246 个引脚网络、19 个对接触点、全部 74 个封装、724 个旧铜记录和铺铜/禁布区定义均已检查。八个断路副本检出对应中断，未连接数由 10 增到 11；另一个副本删除七段 CH343 背面供电细线后，USB 触点仍连接 D1，CH343 两个供电输入按预期断开，未连接数为 12。二极管 K/A 脚与实际 XML 网表一致；这不证明实物反向漏电和插拔切换合格。主板仍有原来的 M2/PWR1 装配边界问题。

[剩余连接明细](remaining_connections.json)包括 M1 到主控的两根控制线、两路采样各三处连接，以及电池输入网两处连接。铜连接检查不能证明实际 USB、默认关闭、启动时序、功率动态或无线稳定性。

## 验证与未完成事项

原生网表检查覆盖全部 246 个物理引脚、19 接点、USB/EN/BOOT、电源子图及原厂模组焊盘；七种故意改错的对照均被检出。原 USB 子电路的 72 个直流和 3 个时序模拟仅覆盖该子电路，电源的局部 RC 模拟也不等于闭环稳压器稳定性仿真。

新增 [24 个 DTR 启动偏置模拟](usb_startup_review/README.md)：在电源达到 3.3V 后，现有外部上拉保持足够的模型高电平；增加错误下拉的对照能检出余量不足，没有因此新增器件。采样时刻、实际缓升／欠压及整板 USB 行为仍未验证。[无线供电策略](WIRELESS_POWER.md)已实现按有效电压启动无线，并在供电失效后自动复位；USB 取电额度与挂起电流仍需整板验证。

最新固件 **73 项离线检查及 C6/R3 两次交叉编译通过**，见[实际日志及源文件绑定](firmware_verification/report.json)。运动仍由 `BOARD_POWER_UNREVIEWED` 禁止；无刷写、USB 枚举、无线或带载实测。

此前 ADC 更新只改变原理图注释及实现状态，当时没有改变 PCB，见[原注释版本记录](verification/adc_revision_evidence.json)。后续 U4、U5 布线修改了 PCB；之前 [7 段铜线](../archive/u4_before_supply/README.md)和 [55 段铜线](../archive/u5_before_supply/README.md)的报告与预览分别保存在历史检查点。[103 段铜线检查点](../archive/usb_before_interface/README.md)保存 USB 布线前的主板，[170 段铜线检查点](../archive/reset_before_control/README.md)保存复位与串口布线前的主板。[255 段铜线检查点](../archive/mcu_before_supply/CHECKPOINT.md)保存主控供电布线前的主板。[319 段线检查点](../archive/controls_before_routing/CHECKPOINT.md)保存控制布线前的主板。[375 段线检查点](../archive/encoder_before_links/CHECKPOINT.md)保存编码器布线前的主板。[422 段线检查点](../archive/left_encoder_before_pair/CHECKPOINT.md)保存 M1 编码器主控布线前的主板。[456 段线检查点](../archive/right_encoder_before_pair/CHECKPOINT.md)保存 M0 编码器主控布线前的主板。[522 段线检查点](../archive/right_motor_before_pair/CHECKPOINT.md)保存 M0 控制线布线前的主板。[554 段线检查点](../archive/vmot_before_distribution/CHECKPOINT.md)保存 VMOT 分配前的主板。[566 段线检查点](../archive/logic_before_input/CHECKPOINT.md)保存 USB/OR 布线前的主板。当前[输入布线版本记录](verification/logic_input_layout_revision.json)核对 123 个固件和原理图源文件未变，不把既有子电路模拟当作新 PCB 的动态、热、USB、复位或射频验证。

两路采样仍为 120k/10k、100nF，名义倍率 13；MINI GPIO4/5 的实网表和[掉电敏感性计算](ADC_POWER_OFF_REVIEW.md)已重核。[ADC 只读转换](ADC_READOUT.md)已实现，使用芯片校准数据；**掉电保护和外部分压校准尚未完成**，读数已用于无线启动和供电失效复位，策略的实物验证尚未完成。USB 用电额度/浪涌/回灌、完整输入连接器和保护、供电动态及默认关断、整板布线、装配净空、射频、完整 BOM/报价与 JLCEDA 导入往返仍未完成。

新增[采样隔离候选检查](adc_isolation_review/README.md)，包括 32 个解析组合、6 个 ngspice 条件模型和[波形](adc_isolation_review/transients.png)。两个 20kΩ 下臂在导通时保持标称倍率 13；模型显示快速掉电仍有缺口，也检出了把滤波电容留在 MCU 一侧时的存储电荷问题。该候选尚未加入原理图或 PCB，没有新增实际元件或改动现有固件。

[元件清单](../../TOP_COMPONENTS.md)已有 63 个位号填写料号，7 个待选；料号存在不代表已获装配资格。[费用审查](../../COST_REVIEW.md)保留币种和装配类型区分，没有完整成品总价。历史 WROOM 图与固件保存在 [迁移前备份](../archive/wroom_before_layout.zip)。

重建原理图：在上级 `controller_core` 运行 `python3 build_logic.py`、`python3 verify_logic.py`。布局生成器 `build_placement.py` 会重建 PCB，**不可用于覆盖现有手工走线**。`route_local_power.py` 和 `verify_local_power_routes.py` 只适用于历史 7 段铜线阶段；本次明确坐标方案为 `route_logic_supply.py`，仅接受原 7 段铜线的基底，不能直接重跑覆盖当前板。

U5 手工坐标方案 `route_motor_supply.py` 只接受旧版 55 段铜线的基底；`route_usb_interface.py` 只接受 103 段线的基底；`route_reset_control.py` 只接受 170 段线的基底；`route_mcu_supply.py` 只接受 255 段线的基底；`route_motor_controls.py` 只接受 319 段线的基底；`route_encoder_links.py` 只接受 375 段线的基底；`route_left_encoder_pair.py` 只接受 422 段线的基底；`route_right_encoder_pair.py` 只接受 456 段线的基底；`route_right_motor_pair.py` 只接受 522 段线的基底；`route_vmot_distribution.py` 只接受 554 段线的基底；`route_logic_input.py` 只接受 566 段线的基底，均不可重复添加到当前板。在 `controller_core` 目录验证当前局部铜线及保留的连接：

```sh
python3 verify_logic_input.py --board logic/top_board_v2_logic.kicad_pcb --baseline archive/logic_before_input/top_board_v2_logic.kicad_pcb
```

`verify_placement.py` 使用 KiCad 自带 Python 核对二维接口与板框；项目虚拟环境运行 `verify_antenna_projection.py` 核对天线禁布区投影。`plot_placement.py` 从原生 KiCad 图层生成预览，扩大显示范围以保留伸出板框的天线轮廓；预览未显示飞线，不能凭图片判断已连通。检查不访问实物串口或私有配置。
