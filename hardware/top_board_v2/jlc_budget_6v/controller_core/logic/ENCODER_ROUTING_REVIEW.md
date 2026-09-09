# 编码器接口与 M2 A/B 布线检查

此文记录六路接口上拉与 M2 编码器阶段：新增 **57 段线、11 个过孔**，替换既有 10 段线和 2 个过孔。当时为 **422 段线、118 个过孔、两块地铜、27 项未连接**。后续 M1 主控连接及当前数量见[主板说明](README.md)。所有元件位置、六页原理图、70 件 BOM 和固件均未改变。

[当前 PCB](top_board_v2_logic.kicad_pcb) · [整板预览](preview/placement.png) · [编码器区域放大图](preview/encoder_links_detail.png) · [该阶段原生连接检查](../archive/left_encoder_before_pair/top_board_v2_logic.verification.json)

## 接通范围

| 网络 | 本轮核对的焊盘连接 | 到主控的状态 |
| --- | --- | --- |
| ENC0_A | M0.5 → R14.2 | U3.12 尚未连接 |
| ENC0_B | M0.4 → R15.2 | U3.13 尚未连接 |
| ENC1_A | M1.5 → R18.2 | U3.5 尚未连接 |
| ENC1_B | M1.4 → R19.2 | U3.6 尚未连接 |
| ENC2_A | M2.5 → R22.2 → U3.15 / GPIO6 | 全路径已连通 |
| ENC2_B | M2.4 → R23.2 → U3.16 / GPIO7 | 全路径已连通 |

六颗上拉电阻的另一端在前一供电阶段已接 MAIN_3V3。每组对接排针仍是 **4 脚编码器 B、5 脚编码器 A**；实体 Motor1/2/3 对应 M0/M2/M1。这里检查的是顶部铜连接，不能替代实际线束针序和编码器方向核对。

M2 A/B 由主控下边引出，换层跨过既有供电、启动配置和控制线。GPIO15、M2_FI、BOOT 的局部路径及一个地过孔为编码器引出让位；替换前后的实体焊盘连接均重新检查。ENC2_B 在模组焊盘和启动配置走线之间用 0.2mm 引出，其余新增信号为 0.25mm，未放宽已有规则。新增过孔均为 0.6mm 外径、0.3mm 孔径。外侧折线使用 30°/60° 转向；所有坐标明确给定，没有自动寻路。

## 检查结果

KiCad 原生铜连接图确认六组表列连接，全部原有实体焊盘连接组仍然连通。22 个 MAIN_3V3 焊盘、30 个主控实体地焊盘和六个控制下拉地端均保留原连接；同名 49 脚的九个地焊盘逐一检查。全部 246 个物理引脚网络与当前 XML 一致。

74 个封装的完整记录逐字保持，包括原 19 个对接触点及四个安装孔。472 个既有线段/过孔记录逐字保留，12 个替换记录列在[坐标方案](encoder_links_review.routing.json)。两块地铜和全部禁布区定义、板框、设计规则与网络类别均不变，铺铜从清空缓存的状态重新填充。

11 个临时副本分别切断六个接口到上拉的连接、两根 M2 编码器主控引出，以及改道的 M2_FI、GPIO15 和 BOOT。每个副本的原生连接图都检出预期断开，未连接项从 **27 增到 28**；没有把仅删去焊盘内短线当作断路对照。这些检查证明相应铜断线可被 CAD 检出，不是实板 USB 故障诊断测试。

该阶段 DRC 未报告新增短路、铜间距、过孔、板边或孤立铜错误，仍有原来的 **M2/PWR1 装配边界重叠**。19 点上下板配准最大名义残差约 0.00103mm；天线到下板的约 0.046mm 名义净空仍未获得制造公差或射频验证。[阶段版本记录](../archive/left_encoder_before_pair/verification/encoder_links_layout_revision.json)核对 123 个固件和原理图源文件未变，没有重复运行无关固件测试。

## 尚未完成

本阶段剩余的 M1 编码器主控连接已在[后续布线](LEFT_ENCODER_ROUTING_REVIEW.md)完成；M0 编码器、M0/M1 控制主控引线、采样及电源仍有未完成项，见[当前剩余明细](remaining_connections.json)。ADC 掉电问题、完整输入连接器与保护、USB 用电及自动恢复、实际默认关闭、功率动态、温升、装配和无线测试仍未完成。

本轮未增加器件费用，也没有改变当前方案预算不合格的结论。五片顶部成品的全 BOM 与装配报价仍缺失；CAD 局部通过不能作为制造或整机运行放行。

在 `controller_core` 验证本阶段存档：

```sh
python3 verify_encoder_links.py --board archive/left_encoder_before_pair/top_board_v2_logic.kicad_pcb --baseline archive/encoder_before_links/top_board_v2_logic.kicad_pcb
```

[375 段线检查点](../archive/encoder_before_links/CHECKPOINT.md)保留本轮前的主板。`route_encoder_links.py` 仅接受该旧版哈希，不可重复添加到当前板。
