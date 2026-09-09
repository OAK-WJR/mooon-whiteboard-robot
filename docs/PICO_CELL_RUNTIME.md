# Pico 两节电芯监测与电脑端诊断

2026-09-08：BQ76905 已接入实际主循环，与 INA3221 共用项目内 I²C 工作任务。
GPIO39 执行一次开漏唤醒，随后检查芯片身份、配置读回并轮询两节电芯。
这是离线验证的审查固件；未刷写、连接设备或取得真实读数。
**运动、充电和电源循环仍被禁止，电池专属保护门限尚未获准。**

## 启动与总线调度

GPIO/UART/无线/任务看门狗初始化成功且总线就绪后，GPIO39 拉低约100ms，
释放后等待100ms，再启动固定地址0x08、默认非CRC型号的 BQ 检查。
脉冲释放被拖延至150ms时报告超时；GPIO API 失败会停止诊断服务。
这些时间是工程选值，必须实测隔离器、PhotoMOS、稳压器及 TS 的实际波形。
`wake_requested` 仅表示软件请求，不是 TS 电压反馈。

依据保留的 [TI 技术手册](https://www.ti.com/lit/ug/sluuci8/sluuci8.pdf)
及 [BQ76905 数据表](https://www.ti.com/lit/ds/symlink/bq76905.pdf)理解唤醒与配置流程；
工程等待值不是厂商保证的整条隔离电路启动上限。没有错误后自动唤醒、
切换CRC模式或重复不确定的配置写入。初始 Battery Status 只读事务的 NAK
允许在2秒窗口内重试；后端不能区分地址与寄存器字节在哪一步收到 NAK。

每轮主循环最多推进一次 I²C 提交或轮询。已经开始的 INA 采集先完成；
仅当 BQ 物理事务完成、取消后的缓冲区也已归还时，才开始下一组 INA 采样。
BQ 配置及长报文在这些采集组之间推进，初始化期间也保留 INA 读数机会。
没有扩大 INA 的4ms传输/5ms操作预算。7次延后轮询的假设备测试检查了交错，
但不能证明真实无线负载下的调度余量或硬性响应时间。

## USB、Wi-Fi、BLE 诊断

使用更新后的 `scripts/robot_control.py`。接入并核对实板后，USB 查询示例为：

```sh
python3 scripts/robot_control.py --usb <PORT> --usb-kind ch343p cells
```

`cells` 发送只读 `cell_status`，不申请运动控制权；无线仍需要正确的 profile 和 HMAC。
正常 `status` 只增加 `cell_monitor`/`cell_error` 摘要，完整报告单独请求，保持原有
1792字节 JSON 负载和2304字节线路缓冲上限。

| 报告字段 | 含义与限制 |
| --- | --- |
| `cell_mv` | 下节/上节，对应 BQ 物理输入1和5；不是输入1和2 |
| `stack_mv`, `cell_pair_valid`, `stack_consistent` | 总电压及有效性/250mV一致性筛选；不是测量精度保证 |
| `seq`, `generation` | 采样序号与配置/失效代次，不能跨重启代替设备会话身份 |
| `age_ms`, `read_span_ms` | 生成报文时自发布点的年龄、小于1500ms的读取跨度；不是同步ADC时间戳 |
| `ts_counts`, `ts_nominal_uv` | TS 原始值及标称换算；**不是电池温度** |
| `battery_current_ma`, `pack_temperature_mc` | 当前硬件没有相应测量，始终为 `null` |
| `configuration_verified` | 指定的只读监测配置完成写入与读回 |
| `voltage_profile_verified`, `system_protection_qualified` | 当前均为 `false`；不能据配置完成批准充放电 |
| `fets_off_observed` | 新鲜状态里观察到 BQ 关断控制；不是整车无电的物理证明 |

报告生成时样本年龄必须小于3000ms；传输及主机排队时间不在其中。
整组过期、配置变化、POR、断线或重启后撤销读数与相关元数据，输出 `null`。
新鲜状态也可能含无效的单节测量；下节不足2V时上节读数不被接受。
主机严格检查类型、范围、状态、配置代次、设备身份及 HMAC，但通过解析不代表
报文到达时仍新鲜。当前诊断不用于批准运动、充电或恢复供电。

## 验证与仍需完成的工作

[本轮验证记录](PICO_CELL_RUNTIME_VERIFICATION.json)保存了实际命令、公开源码摘要及范围。
14个新增主循环场景、74项 Pico 主机测试、27个 GPIO 场景、18组 BQ 驱动测试、
共享传输层取消竞态检查以及 Pico/R3 公开配置构建通过。构建检查实际应用保留
BQ、INA 与本项目后端符号，没有用强制链接代替接入证明。

GNB8502S80A 的专属充电温区、正常低电压停机及保护配合仍待可靠规格与实测确认。
当前使用未启用保护门限的监测配置，CHG/DSG 和外部请求保持关闭。
如果中途配置失败，BQ 可能留在 CONFIG_UPDATE；单独 MCU reboot 或再唤醒不能保证
恢复。已验证的 BQ 复位/恢复事务、充放电许可、电源动态、实物线序和最终 PCB
仍需完成。已有 MOS 瞬态不收敛记录仍然失败，未被软件测试替代。

此前 `PICO_I2C_BACKEND_VERIFICATION.json`、`PICO_RAIL_RUNTIME_VERIFICATION.json`
和硬件 GPIO 检查点作为历史记录保留，其源码摘要不冒充当前文件摘要。
