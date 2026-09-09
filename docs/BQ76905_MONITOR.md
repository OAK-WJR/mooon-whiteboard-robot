# BQ76905 独立监测驱动

此交付实现 **2S 监测、易失配置校验和故障后数据失效**。`Monitoring` 只表示配置与读回流程完成，**不表示允许充电或放电**。当前实现始终要求外部禁止信号保持有效，没有 CHG_ON、DSG_ON、FET_ENABLE 翻转或自动解除禁止的路径。

原始独立交付未接入主固件。2026-09-08 已完成[主循环、GPIO39唤醒和主机 cells 诊断接入](PICO_CELL_RUNTIME.md)，仍未访问实物、串口或 USB。下文独立驱动交付及示例保留为历史接口说明；机械证据与软件测试结果无关。

## 文件与复跑

| 文件 | 内容 |
|---|---|
| `include/bq76905_monitor.h` | 无 Arduino 依赖的公开 C++11 接口 |
| `src/bq76905_monitor.cpp` | 固定内存、非阻塞有限状态机 |
| `control_tests/bq76905/test_monitor.cpp` | 独立 TI 协议模拟器、源表示例与故障注入 |
| `control_tests/bq76905/source_contract.json` | 保留资料标识、32 项实际源文检查与已知冲突 |
| `control_tests/bq76905/run_tests.py` | 只编译上述驱动和测试，不运行项目固件构建 |
| `control_tests/bq76905/test_report.json` | 本次编译、检测结果及对应代码摘要 |
| `control_tests/bq76905/.gitignore` | 隔离本目录临时构建输出 |
| `docs/BQ76905_MONITOR.md` | 本说明与待集成约束 |

在仓库根目录运行：

```sh
python3 control_tests/bq76905/run_tests.py
```

默认使用 `clang++ -std=c++11 -Wall -Wextra -Werror -pedantic`，同时启用 AddressSanitizer 与 UndefinedBehaviorSanitizer。可显式指定 `--compiler g++`；不支持 sanitizers 的主机可使用 `--no-sanitizers`，报告会记录这一限制。所有编译产物留在本测试目录的 `.build/` 中。

本次验证通过 **18 组测试、全部 296 次初始化传输的逐次不完整完成注入、32 项保留源文检查**。296 次指完整显式电压/内部过温配置到首次快照所需的模拟总线事务；不是实测器件时序或性能承诺。每个中断位置均单独从初始状态运行，并验证不能发布半帧或放行。

模拟器依据实际寄存器、延迟模式切换、CFG 写入条件、W1C 锁存、长度提交以及 0x61 自动移窗等协议规则实现，没有导入驱动的内部命令常量。TI 波形中的 CRC 输入/输出字节是固定外部参照。测试覆盖 NAK、短读、短写、总线错误、一直忙、一直 Pending、错误回显/长度/校验和/配置读回、CFG 未进入/未退出、运行中及配置提交前 POR、CRC 退回 OTP 默认、数据陈旧、计时器回绕、下节低于 2 V、上节异常、TS 越界、保护状态和运行中配置漂移。

保护方向测试另按 TRM 表 11-2/4、12-12/13/14 独立解码实际写入的路由字节，在有/无 profile 下检查 34 个故障组合。它隔离验证“该保护对哪一路增加关断条件”，同时逐项确认实际 CHG/DSG 仍被 held OFF、所有充放电请求仍被拒绝。仅检查最终双 OFF 会掩盖路由错误，因此不作为方向正确的唯一证据。

这属于软件与协议仿真，**不是电池、保护 MOSFET、电源链或实物充电仿真/认证**。

## 所用资料与纠错

先执行了 `hyperresearch search`，本地未找到 vault；随后读取现有 `cell_monitor_integration/sources/bq76905.pdf`、`bq76905_trm.pdf` 及其保留文本，并核对了 TI 官方资料。未使用外部搜索 key。原 PDF 的 SHA 与实际文本表检查保存在测试契约中；没有改写原资料。

| 结论 | 依据 |
|---|---|
| Cell1 电压 `0x14`，物理 Cell5 电压 `0x1C`，均为 I2/mV | [SLUUCI8](https://www.ti.com/lit/ug/sluuci8/sluuci8.pdf)，表 5-1、11-1 |
| 2S 上节接 VC5–VC4A，下节 VC1–VC0；`VcellMode=2` | SLUUCI8 表 5-8、12-6 |
| `VcellMode` 在 `0x901B`，宽度 **H1** | SLUUCI8 表 12-6、数据存储器汇总 |
| Stack `0x26` 为 U2/mV；TS `0x2A` 为有符号 ADC 码 | SLUUCI8 表 11-1 |
| 设备识别返回 `0x7605` | [SLUAAM4 §2.1](https://www.ti.com/document-viewer/lit/html/SLUAAM4/GUID-4CF2CBF4-59FC-4D52-BF6B-6DBF6FD985E0) |
| CRC 默认关闭；更改位在退出 CFG 后才生效 | SLUUCI8 §9.1、表 12-4 |
| DSG/CHG 各 bit7 分别是 CUV/COV，各 bit0 是 OTINT；Both bit0 是 REGOUT | SLUUCI8 表 12-12、12-13、12-14 |
| CRC 首字节和后续字节的固定波形参照 | [SLUAAM4 §4](https://www.ti.com/document-viewer/lit/html/SLUAAM4/GUID-8BA27471-7E9C-46CC-9D03-ECBD6DC8419B) |
| 电气输入条件和标准器件信息 | [SLUSE97](https://www.ti.com/lit/ds/symlink/bq76905.pdf) |

已明确处理三处资料冲突：

1. TRM §5.1.2 的旧地址正文不能覆盖真实命令表；2S 上节不能读取 Cell2/`0x16`。
2. 软件指南 §3.4 的摘要表把 Vcell Mode 写成 H2；这里按 TRM 使用 H1，避免覆盖相邻 `0x901C` Alarm Mask。
3. REGOUT 默认摘要 `0x08` 与 3.3 V 默认文字不一致。驱动读取实际 `REGOUT CONTROL` 和 DM `REGOUT Config`，要求低四位完全一致；不推测电压，也不改其配置。退出 CFG 会重新载入该值，因此不一致时在退出前拒绝继续。

## 运行序列与禁止控制

1. 外部电路先保持禁止，完成 TS/唤醒时序后调用 `beginAfterWake()`。驱动不产生 GPIO 唤醒脉冲。
2. 读取真正的 Battery Status，仅在这一最初只读阶段对 NAK 做有截止时间的重试（实际后端不能区分地址与寄存器字节 NAK）；SEC 尚未初始化也只有限等待。用 `DEVICE_NUMBER` 间接读取、回显、长度和校验确认 `0x7605`。
3. 写 `FET CONTROL=0x0C` 请求双 OFF，并检查 Battery Status 的实际 CHG/DSG 位。默认 HOST_FETOFF_EN 未打开，**写成功本身不是双 OFF 的证据**。已开启自主 FET、实际输出为高、SEALED 或已被另一主机置于 CFG 的器件均拒绝接管；不盲发会翻转状态的 FET_ENABLE，也不尝试猜测解封密钥。
4. 如处于 DEEPSLEEP，发 EXIT_DEEPSLEEP 并等待状态确认。写 `CB_ACTIVE_CELLS=0`、读回为零；已经禁止写入且仍在均衡的异常器件会被拒绝。
5. 发 SET_CFGUPDATE `0x0090`，**看到 CFGUPDATE=1 后**才开始 DM 配置。每个 DM 提交前再次检查 CFG/SEC/POR/实际 FET 状态；每项写后读回比较，再检查 CFG 状态。
6. 校验 CRC 位与调用参数吻合；配置下表各项。清除 TS_ON，但保留现有 LDO 使能与电压。检查 DM 将在退出时载入的 LDO 值和 I2C 地址也与当前值一致。
7. 在测量暂停时设置并清除需要的锁存事件；发 EXIT_CFGUPDATE `0x0092`，等待 CFGUPDATE=0 且 POR=0。重新请求双 OFF、禁用 SLEEP，并确认实际输出、自主控制、FET CONTROL 和 Alarm Enable。
8. 使用已使能、已清除的 **Alarm Status 锁存 INITCOMP/FULLSCAN** 等待真实新测量。Raw Status 的同名位是短脉冲，不能当锁存事件轮询。等待期间交替读取 Battery Status，检测 POR。
9. 每轮先检查状态并审计全部已配置 DM、CRC、LDO、I2C 地址、FET CONTROL 和均衡状态；清除 FULLSCAN 后等待下一次锁存。读取 Cell1、Cell5、Stack、TS、Safety、Alarm；再检查 Battery Status，最后才发布快照。

间接读会取回 32 字节缓冲区、checksum、length（读 `0x61` 放在末尾）；按器件返回的数据长度计算校验，检查范围和命令预期长度。读 length 会自动移窗，所以下一次操作总是重新选地址。写采用地址、缓冲数据、checksum/length 的顺序；length 最后提交。长度/校验失败均不采用部分数据。

| 配置 | 当前写入值/来源 |
|---|---|
| FET Options `0x901E` | `0x40`：只允许 HOST_FETOFF；禁 HOST_FETON、FET_EN、SFET 体二极管覆盖、PWM 等 |
| Vcell Mode `0x901B` | `2`，单字节 |
| DA Config `0x9019` | `0x01C0`：TSMODE=电压输入，CCMODE=停用电流转换，标准 ADC 速度 |
| Balancing Configuration `0x9020` | `0x03`：禁止均衡写命令，保留标准 1 ms 等待；实际均衡状态还必须为零 |
| Default Alarm Mask `0x901C` | `0xC285`：标准来源加 FULLSCAN、INITCOMP、POR |
| Enabled Protections A/B | 无显式 profile：`0x01/0x00`，这些使能位中仅 REGOUT；有完整电压/内部过温 profile：`0xC1/0x02` |
| DSG FET Protections A `0x9026` | 有 profile：`0x81` = CUV(bit7) + OTINT(bit0)；无 profile：`0x00` |
| CHG FET Protections A `0x9027` | 有 profile：`0x81` = COV(bit7) + OTINT(bit0)；无 profile：`0x00` |
| Both FET Protections B `0x9028` | 两种模式均 `0x07`：保留默认 VREF(bit2)/VSS(bit1) 共用关断映射，并加 REGOUT(bit0)；此字节没有 OTINT 位 |
| 电流、TS 温度保护 | 禁用：当前硬件无法提供这些量的真实保护判断 |
| I2C Config、REGOUT Config、I2C Address | **只读核对，不修改** |

无 profile 的模式不依赖芯片默认 COV/CUV 阈值；它明确关闭未配置的保护项目，并一直要求外部禁止。即便配置了电压/内部过温项目，当前输出仍为 OFF。

本轮纠正了把 Enabled Protections A/B 掩码直接复用到 FET 路由的实现错误。两组寄存器位定义不同；例如 OTINT 的使能在 `0x9025.bit1`，关断路由却分别在 `0x9026.bit0`、`0x9027.bit0`，而 `0x9028.bit1` 是 VSS。VREF/VSS 在这里保留的是默认关断路由，不据此宣称已完整验证诊断功能。

显式 profile 的保护方向如下，仅表示该故障额外施加的保护关断条件：

| 故障 | DSG 保护关断 | CHG 保护关断 |
|---|---|---|
| CUV | 是 | 否 |
| COV | 否 | 是 |
| 内部 OTINT | 是 | 是 |
| REGOUT | 是 | 是 |
| VREF / VSS | 是 | 是 |

CUV 的映射只向 DSG 增加保护关断，COV 的映射只向 CHG 增加保护关断。**方向未被该保护阻断的输出仍受全局 held OFF 和 permission 禁止约束；本交付没有批准恢复充电或任何放行。**

## 给主线程的接口

按本轮主线程提供的 CM3 约定：GPIO39/contact19 为 `WAKE_REQUEST_N`，GPIO41/contact22 为 `CHARGE_REQUEST`，GPIO40 保持 NC。这里仅记录交接约定；独立驱动不绑定这些引脚、不输出请求，也未修改主图、board profile 或主固件。

`I2c` 与 `Clock` 必须比 `Monitor` 活得更久；Monitor 禁止复制。I2C 适配器需要满足：

- `start()`、`poll()`、`cancel()` 都及时返回。`start=false` 表示尚未执行任何总线操作；Pending 不能暗含阻塞等待。
- 读取用 register pointer 后的 **repeated START**，最后一个字节 NACK 后 STOP。不能换成带 STOP 的另一种读法而仍使用这里的 CRC。
- 支持最多 69 字节 TX、68 字节 RX。不能套用固定 32 字节 Wire 缓冲区后静默截断。
- 完成计数对应传入缓冲区字节数，包含 CRC，**不包含地址/RW 和 ACK 位**。读写短计数都会报错。
- 取消返回前，DMA/IRQ 等不得再访问旧缓冲区。Monitor 析构或显式重启时会取消自己仍持有的事务。
- 此驱动独占该 BQ 的间接命令窗口；其他 I2C 器件可由适配器仲裁，但不能让另一个 BQ 客户端穿插写 `0x3E…0x61`。

每次 `tick()` 最多调用一次适配器 start 或 poll；超时时可改为一次 cancel，没有等待硬件的循环。默认参数在主机测试中按 1 ms tick 驱动；建议平台集成先保持这一调度间隔并测量真实耗时。调用更慢可能达到帧超时，不能直接宣称已验证实时性能。

```cpp
// adapter 与 clock 由平台实现；此代码未接入当前 top_v2_main.cpp。
bq76905::Monitor monitor(adapter, clock);
bq76905::Options options;
options.startup_crc = bq76905::CrcMode::Disabled; // 必须对应实际器件/现有运行状态
// 外部禁止已有效，硬件唤醒时序结束后：
bool accepted = monitor.beginAfterWake(options);

// 调度器每次迭代：
monitor.tick();
auto diagnostic = monitor.diagnostics();
auto frame = monitor.snapshot();
// 仅 frame.fresh / 各 Reading.available 成立时使用数值。
// 不可用的 current_ma / pack_temperature_mc 在 JSON 中必须输出 null。
// diagnostic.configuration_verified 不能作为充电许可。

// 已知芯片复位、掉电或进入 SHUTDOWN 时立即调用：
monitor.notifyResetOrPowerLoss();
```

默认时间参数：单事务 100 ms、首次响应窗口 2000 ms、模式确认 1000 ms、完整初始化 15000 ms、启动事件/单轮采集 1500 ms、轮询间隔 5 ms、两轮间最小间隔 250 ms、数据最长有效年龄 3000 ms。它们是显式软件截止时间，未作为真实电源链的安全响应时间验证。时钟使用单调 `uint32_t` 毫秒并处理回绕；不会用墙钟。

错误会锁存 Fault，使配置和 profile 校验标志失效、快照不可用、generation 改变；不会自动重试配置或恢复许可。修复原因并重新完成外部唤醒/禁止条件后，显式 `beginAfterWake()` 开始新代次。POR 被观察到后立即失效；CRC 随 POR 退回不同 OTP 默认时可能先表现为 CRC/通信错误，同样失效。主机无法在未采到状态的瞬间知道物理 POR，外部默认禁止不能依靠这个轮询窗口。

如果故障发生在 CFG 内，芯片可能仍停在 CONFIG_UPDATE。为避免把部分配置应用到运行态，驱动不会在故障时盲发 EXIT_CFGUPDATE；再次 begin 也拒绝接管已经处于 CFG 的芯片。此时必须由集成方在外部禁止有效时执行已验证的 BQ 复位/恢复流程，再重新初始化。仅重启 MCU 或再发唤醒脉冲不保证清除 BQ 的 CFG 状态。当前 API 不发 RESET、SHUTDOWN 或解封命令。

`fets_off_observed` 是最后观察值，不是物理输出反馈或硬接线保证。通信故障时驱动不能保证再发一条 OFF 命令会被芯片执行；集成方必须保持独立的禁止电路。故障/过期后的 `status_available=false`，不可将旧 status 数字当作当前状态。

## 数据有效性与 CRC

- 下节取物理 Cell1，上节取物理 Cell5。上节只有在下节测量有效且至少 2000 mV 时才可能有效；下节低于 2 V 时上节返回不可用原因 `LowerCellBelow2V`。
- 电芯非正值或超过 5500 mV、Stack 非正值或超过两节的 11000 mV 均不可用。Cell1+Cell5 与 Stack 的差值还需满足显式 `stack_agreement_mv`（默认 250 mV）才能标记成对有效。这个容差是顺序采样的诊断参数，不能代替保护阈值或校准。
- 读取寄存器不是原子 ADC 快照；`read_span_ms` 记录软件读取跨度，可能跨越转换更新。通信与状态通过不证明采样同时发生，也不证明线束无开路。
- TS 返回原始有符号 ADC 码和名义微伏数，按 1.1962 V × 5/3 ÷ 32768 换算；只接受名义 0–1.8 V 区间。没有热敏电阻换算，不是电池温度。
- SRP/SRN 接 RAW B−，电流永远 `available=false`、原因为 `NoCurrentShunt`；不读电流命令，不把原始零当 0 A。
- `snapshot()` 在调用时再次计算过期状态，即便调度器暂停，也不会通过 getter 把超龄值标成 fresh。集成方缓存下来的旧副本仍必须检查年龄/generation。
- CRC 明确由 `Options.startup_crc` 选择，并以 DM 位核对。标准未定制器件启动默认关闭；已有运行状态或定制 OTP 必须按实际选择。不会自动探测切换、修改 CRC 位或把 CRC 错误降级为无 CRC。
- CRC8 多项式 0x07，初值 0；每个数据字节分别校验。第一写字节含写地址/寄存器，第一读字节含 repeated START 前后地址和寄存器。实现完整事务结束后验证；任一 CRC 失败丢弃整帧。适配器可在硬件支持时提前中止错误 CRC，当前接口不要求逐字节提前 NACK 能力。

## 显式保护参数与拒绝放行

`ProtectionProfile.requested=false` 是默认。若设为 true，下列参数必须全部显式提供并合法，缺一个即在总线访问前拒绝：

| 参数 | 单位/约束 | DM 地址 |
|---|---|---|
| `cuv_mv` / `cov_mv` | mV；驱动限制 CUV ≥2000，COV>CUV，均≤5500 | `0x902E` / `0x9032`，各 I2 |
| `cuv_delay_scans` / `cov_delay_scans` | 1–255 次 ADSCAN，不能当毫秒 | `0x9030` / `0x9034` |
| `cuv_recovery_code` / `cov_recovery_code` | 0=无自动恢复；1/2/3=50/100/200 mV，恢复窗口不得交叠 | `0x9031` / `0x9035` |
| `die_ot_c` / `die_ot_recovery_c` | 25–150 °C 的内部芯片温度，恢复必须更低 | `0x904C` / `0x904E` |
| `die_ot_delay_scans` | 1–255 次内部温度测量 | `0x904D` |

它只是一组经过读回的电压/内部芯片过温配置。测试中的 2800/4200 mV 等数字是测试输入，**未批准用于 GNB 电池充电**。

`requestPermission({charge, discharge})` 没有总线或 GPIO 副作用，并按当前证据返回：未初始化、数据过期、电芯无效、保护故障、缺 profile，或 `SystemProtectionUnqualified`。两项都 false 返回 `HeldOff`。即便 profile 全部写入读回并且电压正常，也返回 `SystemProtectionUnqualified`，`inhibitRequired()` 始终 true。

仍未具备整机完整保护放行所需的电池温度与电流保护信息、具体电池认可的全部阈值、保护/充电模块和实际 CHG/DSG 许可电路的联合验证。没有一个用户可勾选的“已确认”布尔值能绕过这些缺项。要实现真正的电源放行，主线程需单独完成并审查系统保护逻辑；本驱动当前 API 不提供开启出口。

待集成工作明确限于：平台非阻塞 I2C/时钟适配器、外部唤醒/默认禁止控制、reset 通知、快照与 null 语义序列化。后续实物测试仍需核实实际 OTP/CRC/LDO、68 字节读取和取消行为、采样误差、总线超时、掉电/复位及保护电路动作。本交付没有宣称这些已通过。
