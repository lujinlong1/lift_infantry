# LIFT_DIAGNOSTICS_P0_REPORT

验证日期：2026-10-19。目标工程：`MDK-ARM/lift_infantry.uvprojx`，STM32F407，Arm Compiler 6.21。

## 1. 交付结果与范围

已新增独立、只观测的 `lift_diagnostics` 模块，复用现有控制状态、反馈快照、命令缓存快照、故障/堵转快照和 VOFA JustFloat 协议。完整链路为：

`命令及来源 → 流程阶段 → 轴目标/轨迹参考 → PID需求及前馈 → 控制限幅 → Guard → 顶层故障停机 → 输出门禁/收回轴清零 → 命令缓存 → 实际CAN打包 → HAL接受结果 → 带时间戳的电机反馈`

未修改 PID、前馈公式、轨迹规划、限幅算法及阈值、保护阈值、状态机转移和电机输出策略。`pid.c`、`lift_control.c`、`lift_homing.c`、`lift_seq.c`、`lift_sync.c` 无本次改动。`lift_top` 和 `lift_guard` 仅添加状态采集字段及赋值。台架模块仅为既有所有权丢失计数补充时间戳和短临界区。

工作区开始时已有修改：`lift_control_task.[ch]`、`test_lift_axes.c`、上阶梯调试文档、`.uvoptx`，以及未跟踪的 `check_stack_budget.ps1`。本次保留这些内容，未将已有功能改动算作 P0 实现；未提交 Git commit。

## 2. 修改文件清单

| 文件 | 本次用途 |
| --- | --- |
| `Application/Lift/Inc/lift_diagnostics.h`、`Src/lift_diagnostics.c`（新增） | 统一帧、枚举、计数器、快照接口、缓存/TX记录、循环黑匣子 |
| `Application/Lift/Inc/lift_control_task.h`、`Src/lift_control_task.c` | 周期ID、参数版本、命令来源、统一采集、故障分类、旧快照证据周期 |
| `Application/Lift/Inc/lift_top.h`、`Src/lift_top.c` | 保存 Guard 前限幅电流、Guard输出、分支有效标志、故障轴号 |
| `Application/Lift/Inc/lift_guard.h`、`Src/lift_guard.c` | 在原有故障判断处记录准确轴号；组级超时为 -1 |
| `Application/Remote/Src/remote_lift.c` | 使用带来源的目标命令入口，原目标值和调用顺序不变 |
| `Devices/RobotMotors/Inc/robot_motor_commands.h`、`Src/robot_motor_commands.c` | 缓存写入序号/周期随原子快照传递；在真实打包和发送处记录结果 |
| `Diagnostics/MotorTest/Inc/lift_jog_task.h`、`Src/lift_jog_task.c` | 复用 own_lost_count，增加精确事件时间；与主控制诊断关联 |
| `Diagnostics/Telemetry/Inc/telemetry_task.h`、`Src/telemetry_task.c` | source=2 使用一致性快照；新增 source=3 单轴全链路28通道 |
| `MDK-ARM/lift_infantry.uvprojx` | 加入诊断源文件，继续使用既有 6.21 配置 |
| `Tests/Lift/test_lift_diagnostics.c`、`test_lift_transport.c`（新增） | 黑匣子边界、计数器、实际缓存/打包/发送任务软件测试 |
| `Tests/Lift/test_lift_axes.c` | 增加真实控制任务集成诊断断言，保留原有用例 |
| `Tests/Lift/run_tests.ps1`、`compile_check.ps1` | 注册新测试和诊断模块严格编译检查 |
| `Tests/Lift/fwstubs/robot_motor_commands.h`、`jogstubs/FreeRTOS.h` | 补充新诊断入口和主机临界区宏 |
| `Tests/Lift/Results/P0/*.txt`（新增） | 保存本次主机、Keil、静态栈预算结果 |
| `LIFT_DIAGNOSTICS_P0_REPORT.md`（本文件） | 设计、Watch、VOFA、定位流程、验证和剩余风险 |

## 3. 数据与一致性约定

- `g_lift_diag.frame` 是最后一个**完整结束**的控制周期。周期 ID 从 1 开始，0 表示未知/其他任务；每周期递增，回绕跳过 0。
- `tick_ms` 是控制周期入口时间。`feedback_observed_ms`、`transport_observed_ms` 分别是取得反馈/发送快照后的观察时间。反馈年龄使用前者，发送年龄使用后者，避免抢占后出现“时间戳在未来”的无符号假超时。
- `parameter_version` 在成功重建配置、重载控制增益、测试行程配置生效及接受测试/阶梯流程预设时递增。拒绝的配置不递增；调试字段尚未 apply 不算已生效配置。
- `source`：0 Watch、1通用API、2流程、3找零、4 JOG、5 CAN测试、6遥控、7其他、8主控制缓存写入者。`source` 表示当周期有效路径；`transport.cache_source` 表示缓存写入者，通过 `cache_cycle_id` 回溯其命令来源。直接 Watch 写命令时应同时将 `g_lift_task_command.diagnostic_source=0`，无法自动识别调试器写内存。
- 四轴位置和速度默认是逻辑伸出为正，位置单位 ecd，速度单位转子 ecd/s；电流是 C620 电流命令单位。`direction` 为 ±1；`position_raw`、`zero_raw`、`speed_raw_rpm`、`feedback_raw`、`submitted_raw`、缓存/打包 `_raw` 数组均为原始电机方向。逻辑电流 = 原始电流 × direction。
- `pid_current` 为现有控制器给出的 P+I+D 未作最终输出钳制的需求（保留现有抗饱和积分行为）；`feedforward_current` 单列；`demand_current` 为两者之和。`limit_current` 在 Guard 跟踪回写前保存，避免旧 `axis_final_current` 被覆盖后丢失限幅证据。
- `guard_current` 是 Guard 的输出；`protected_current` 是顶层故障停机策略执行后的逻辑输出；`submitted_raw` 是门禁后的任务提交意图。只有 `cache_written=1` 才表示主控制任务实际调用缓存写入。被台架占用时该意图可能为0，但真实缓存仍属于台架，不能误判缓存为0。
- `controller_ran`、`guard_ran` 表示相应函数在本周期执行；禁用命令时控制器仍可能被调用并返回零。找零、顶层故障等跳过控制器的周期，相关参考/PID/限幅字段清零并标为未执行，不继承旧周期数据。
- 流程的既有监督输入包含上一周期输出/跟踪信息，用 `decision_feedback_cycle_id` 明示。`stage_before`、`sequence_target_mm_before` 保存转移前阶段/目标，`stage` 是转移后阶段。它们不伪装成本周期 PID 的输入。
- 旧 `g_lift_fault_snapshot` / `g_lift_stall_snapshot` 保留接口和内容；新增 `feedback_cycle_id` 与 `control_evidence_cycle_id`，明确前者为触发反馈周期，后者为此前输出所属周期。旧 fault 快照的反馈电流是逻辑方向，旧 stall 快照反馈电流及输出电流为原始方向。新代码建议统一使用 `g_lift_diag`。

发布采用单控制任务写入 + 32位序列号 + DMB。序列号奇数表示写入中。`lift_diag_get_snapshot()`、`lift_diag_get_transport()` 和 `lift_diag_get_record()` 最多尝试3次，失败立即返回 false，调用方必须丢弃失败结果。VOFA 读取完整快照，不再直接拼接多个周期的旧状态字段。黑匣子可按时间顺序读取历史、before、trigger、post；无效索引/记录返回 false。

缓存与发送由不同任务更新，但对小型 transport 记录的写入均在短临界区内串行化。主控制帧读取 transport 时仍保留其独立周期/序号，**一致快照不等于同步发送**。大帧和历史区复制不关中断；高频任务不打印、不分配动态内存。

## 4. CAN证据分层

| 证据 | 含义 | 不能推出 |
| --- | --- | --- |
| controller_ran、limit/guard/protected_current | 控制链本周期计算/保护结果 | 已写缓存或已发送 |
| cache_written + transport.cache_raw/cache_generation/cache_cycle_id | 主任务写入情况及设备层实际限幅后缓存 | 当前帧已经由CAN发送任务选中 |
| packed_raw + tx_cache_generation/tx_cycle_id | 实际8字节打包内容反解值、被选中的缓存版本 | HAL已经接受或电机已收到 |
| send_result=0、accepted_count、last_success_ms | HAL_CAN_AddTxMessage 接受发送请求 | 总线上发送完成、对端ACK或电机执行 |
| axis.feedback_seen/count/tick/age/current | 已解码到该轴反馈和反馈新鲜度 | 某一条命令的逐帧确认 |

`send_result`：0 OK、1 BUSY、2 INVALID_ARGUMENT、3 HAL_ERROR、255从未尝试。`failed_count` 逐次失败累计，`consecutive_failures` 成功即清零，`last_success_ms` 仅成功更新，`ever_accepted` 区分“时间0成功”和“从未成功”。最近失败另存时间、原因、周期、缓存序号和四轴电流，因此控制周期之间先失败后成功也不会丢失失败证据。

命令过期只清发送任务局部副本，诊断分别保留原缓存、实际打包零值和 `timed_out`。CAN测试整帧替换保留 `test_override`，`tx_cycle_id=0`，不冒充主控制输出；原缓存版本仍可查看。零保持阶段也记录真实零帧。

`can_fault_bits`：bit0发送失败，bit1所选缓存已过期，bit2有运动需求且发送未开始/最近发送超过20ms。20ms仅为诊断阈值，未增加任何控制保护动作。

## 5. 黑匣子与故障码

默认20ms采样、101槽，正常运行暖机后覆盖最近至少2秒（包含两端采样点）。故障不需要等到采样时刻：每个5ms控制周期均检查诊断原因。首次故障冻结历史，不搬移整块历史；额外保存：

1. `before`：上一个完整控制周期，通常距触发5ms，保留20ms采样点之间的最近输出。
2. `trigger`：本周期状态、各级电流、故障原因和真实异步发送证据；同周期中已执行的保护结果包含在此帧。
3. `post`：下一个完整控制周期，观察保护的后续输出；用 `post_valid` 标识是否已取得。

冻结后实时诊断和计数器继续更新，首个故障证据保持不变。向 `g_lift_diag_rearm` 写1可重新布防，清历史计数，不清控制故障、不重置控制器、不清累计诊断计数。持续原因未消除时会立即再次锁存。上电/重新布防不足2秒时只能保存已存在的历史。

`reasons` 是位图，可同时记录多类故障；`reason_axis[]=-1` 表示组级/无单轴信息，否则0..3。多轴同时异常时 reason_axis 记第一个轴，完整异常分布保留在四轴字段中。

| bit/计数索引 | 枚举 | 具体证据 |
| --- | --- | --- |
| 0 | HOMING | 每轴homing_state/homing_fault、top_fault、原因轴 |
| 1 | STALL | 找零CONTACT_WAIT、流程test_stall_axis、Guard下降堵转 |
| 2 | SEQUENCE | sequence_fault、sequence_detail、sequence_fault_axis、stage_before/stage |
| 3 | TOP | top_fault及对应轴；找零确认缺失等组级原因可为-1 |
| 4 | GUARD | guard_state/fault/fault_axis，保持原保护判定位置的轴号 |
| 5 | GATE | gate枚举；有命令/找零/流程/故障需求时捕捉，空闲上电离线门禁只显示不冻结 |
| 6 | OWNER | 主控制认领失败或台架既有own_lost_count变化 |
| 7 | CAN | can_fault_bits、返回结果、最近失败证据、发送年龄 |
| 8 | FEEDBACK | 有控制需求时掉线/未收到反馈，逐轴年龄与计数 |
| 9 | LIMIT | Guard limit_blocked，普通电流/斜率限幅仅记录各级值，不当故障冻结 |
| 10 | COMMAND_TIMEOUT | Guard DESCENDING/PARKED，即原有命令超时缓降路径 |

`g_lift_diag.event_count[i]` 按对应原因0→1边沿计数，持续故障不刷屏，清除后再出现会加1，达到UINT32_MAX饱和。新诊断的TX/失败/接受/锁存计数同样饱和；缓存generation和发布sequence为自然回绕序列号，既有台架own_lost_count仍保持原计数规则。

所有权事件：`owner_requester` 使用 lift_owner_t；主控制失败有 `owner_event_cycle_id` 和事件时间。台架属于另一个任务，原始事件周期为0，保留 `jog_owner_event_ms/jog_owner_lost_count`，统一帧的 cycle_id 是观察到该事件的控制周期。不会将其误标为主控制失败发生的周期。

## 6. Keil Watch变量表

建议先添加整个 `g_lift_diag`、`g_lift_blackbox`；Watch读取持续更新的大结构并不原子，严格分析时暂停内核，或使用上述快照API。在线观察至少确认 sequence 为同一偶数，不能将分次刷新字段视为一个快照。

| Watch表达式 | 用途 |
| --- | --- |
| `g_lift_diag.sequence` | 发布一致性；偶数稳定，奇数更新中 |
| `g_lift_diag.frame.cycle_id`、`.tick_ms`、`.dt_s` | 控制周期及节拍 |
| `g_lift_diag.frame.parameter_version`、`.source` | 生效参数版本、当前命令路径 |
| `g_lift_diag.frame.stage_before`、`.stage`、`.state` | 流程/顶层状态转移 |
| `g_lift_diag.frame.reasons`、`.reason_axis`、`.sequence_detail` | 多类原因、轴和文本细节 |
| `g_lift_diag.frame.axis[0]`（至[3]） | 完整目标、参考、位置、速度、分级电流及方向 |
| `g_lift_diag.frame.axis[0].feedback_age_ms` | UINT32_MAX表示从未收到反馈 |
| `g_lift_diag.frame.controller_ran`、`.guard_ran`、`.cache_written` | 区分支路未执行、已计算和已写缓存 |
| `g_lift_diag.frame.gate`、`.owner_conflict`、`.retracted_mask` | 输出门禁、认领冲突、既有收回轴锁零 |
| `g_lift_diag.frame.transport` | 与该控制帧关联的完整缓存/发送快照 |
| `g_lift_diag_transport`、`g_lift_diag_transport_sequence` | 发送/缓存最新状态，比控制帧刷新更快，独立序列保护 |
| `g_lift_diag.event_count` | 按故障类边沿累计 |
| `g_lift_blackbox.frozen`、`.count`、`.head` | 冻结状态、已有历史数、下一写入槽 |
| `g_lift_blackbox.before_valid`、`.before` | 最近一个完整控制周期 |
| `g_lift_blackbox.trigger_cycle_id`、`.trigger` | 触发周期及同周期保护结果 |
| `g_lift_blackbox.post_valid`、`.post` | 后续完整周期 |
| `g_lift_blackbox.history` | 固定101槽，最旧槽=(head+101-count)%101 |
| `g_lift_diag_rearm` | 写1重新布防；只操作诊断数据 |
| `g_lift_fault_snapshot`、`g_lift_stall_snapshot` | 保留的旧接口，注意新增证据周期标识 |
| `g_can_rx_debug.dropped_count`、`g_dji_can_test_status`、`g_lift_jog_status` | 复用既有接收队列/点动辅助证据 |

Gate：0无门禁、1配置未生效、2离线、3 JOG或主控制认领失败、4 CAN测试。实际优先级为CAN测试→JOG→配置→离线；同值3的两种情形由 owner_conflict 区分。

## 7. 故障定位流程

1. 先看黑匣子 frozen/trigger_cycle_id，记录 trigger 的 reasons、reason_axis、参数版本、source、stage_before→stage。保留原数据后再重新布防。
2. 看执行有效标志。找零支路检查每轴homing_state/fault；流程FAULT且controller_ran=0时，不用“PID为0”推断PID故障。
3. 看目标→参考→实测位置和目标/实际速度。目标未到参考表示规划/同步/限幅状态需要结合现有配置检查；参考已经推进但反馈不动，继续看电流链。
4. 比较 pid+ff需求→limit_current→guard_current→protected_current；前者明显大而后者小，结合 current_limit、limit_blocked、Guard fault、stop_action 定位首次变化处。检查 retracted_mask，避免将既有收回轴清零误认为CAN丢命令。
5. 看 cache_written 和 gate/owner_conflict，再比较 submitted_raw 与 cache_raw。主链未写缓存时，先判断当前所有者；不要用主链的零提交意图替代台架缓存。
6. 用 cache_generation、tx_cache_generation、cache_cycle_id、tx_cycle_id 确认发送选中了哪个周期。比较 packed_raw，并检查 timed_out/test_override。
7. 看 send_result、failed_count、last_failure_*、ever_accepted、last_success_ms。HAL接受只说明入邮箱成功；失败后又成功时也要保留最近失败信息。
8. 最后看每轴feedback_seen/count/age/current，与实际运动对应。反馈电流不等于逐条命令ACK；反馈新鲜但电流/运动异常还需结合电调、供电、负载与硬件测量。对比 before、trigger、post及101槽历史，避免跨周期拼接。

## 8. VOFA使用

沿用 USART1 115200、JustFloat、小端float、尾标 `00 00 80 7F`、20ms发送周期；默认source=0不变。source=1台架布局不变。source=2仍为28通道，每轴7条：实际逻辑位置、轴目标、主链逻辑提交电流、实际速度、目标速度、当周期电流上限、逻辑反馈电流；现在整帧来自一次一致性快照。新 source=3 仍为28通道、116字节，不改变串口带宽；设置 `g_telemetry_status.diagnostic_axis=0..3` 选择轴。

| source=3通道 | 数据 | source=3通道 | 数据 |
| --- | --- | --- | --- |
| 0 | 控制cycle_id | 14 | 逻辑提交意图电流 |
| 1 | 参数版本 | 15 | 当前缓存电流，转逻辑方向 |
| 2 | 轴号 | 16 | 最近实际打包电流，转逻辑方向 |
| 3 | 轴最终目标ecd | 17 | 逻辑反馈电流 |
| 4 | 轨迹参考ecd | 18 | 反馈年龄ms |
| 5 | 实际逻辑位置ecd | 19 | 输出gate |
| 6 | 目标速度ecd/s | 20 | reasons位图 |
| 7 | 实际速度ecd/s | 21 | 缓存所属控制周期 |
| 8 | PID需求电流 | 22 | TX所属控制周期，其他任务/测试为0 |
| 9 | 前馈电流 | 23 | CAN发送返回值 |
| 10 | PID+FF需求 | 24 | 累计发送失败次数 |
| 11 | 控制限幅后电流 | 25 | 最近HAL接受请求的年龄ms，从未接受为-1 |
| 12 | Guard后电流 | 26 | 该轴反馈帧计数 |
| 13 | 顶层保护后电流 | 27 | source |

建议将3/4/5和6/7分别叠图，再比较10/11/12/13/14/15/16/17；同时观察19/20/21/22/23/18。source=3电流虽然转成逻辑符号，仍需查看Watch中的周期归属和有效标志，不能默认同一列全部是同一时刻的发送。

快照尚未发布或3次读取均失败时丢弃本遥测帧，增加 `snapshot_drop_count`；不阻塞控制任务。Float通道无法逐整数精确表达超过2^24的计数/周期；精确计数看Watch。VOFA电流图本身不能证明某条命令已到电机。

## 9. 验证与构建结果

### 软件测试

执行：`powershell -ExecutionPolicy Bypass -File Tests/Lift/run_tests.ps1`

结果：**16组可执行测试全部通过；7个固件源文件严格编译检查通过**。使用真实固件源文件，GCC `-std=c11 -Wall -Wextra -Wpedantic -Werror`。

- 新诊断测试：101槽回绕、完整2000ms跨度、故障非采样时刻触发、before/trigger/post稳定、重布防、持续故障/再次故障、全部11类边沿计数、UINT32_MAX饱和、毫秒时间回绕、序列号回绕、奇数序列有界失败、历史索引0/100/101/UINT16_MAX/非法类型/null。
- 新发送测试：执行真实 `robot_motor_commands.c` 任务和真实DJI打包器，模拟HAL返回；验证±电流钳制、四槽逐字节反解、缓存周期/序号、BUSY/HAL_ERROR、失败后恢复、原缓存保留而过期包归零、JOG写入、CAN测试整帧替换及零保持。
- 控制任务集成测试：420个周期填满历史；版本、方向、时间、速度/电流各节点、远控来源；掉线Guard准确轴号、触发周期与下一周期保护输出；主链及台架所有权事件、找零失败轴；流程故障和堵转证据与旧快照周期对齐。
- 原控制、Guard、找零、流程、台架、运动学、性质/仿真等14组回归继续通过。
- 初次接入台架短临界区时主机桩缺少宏，已补齐并完整重跑；交付日志为修复后结果。

完整日志：[host-tests.txt](Tests/Lift/Results/P0/host-tests.txt)。主机RTOS/HAL桩不验证真实中断时序。

### Keil / Arm Compiler 6.21

使用既有安装：`D:\keil5_32\UV4\UV4.exe`，编译器目录 `D:\keil5_32\ARM\ARMCLANG\Bin`。

全量重建命令：`UV4.exe -r <工程绝对路径> -t lift_infantry -o <日志绝对路径>`。

最终结果：**0 Error(s), 0 Warning(s)**，生成 `MDK-ARM/lift_infantry/lift_infantry.axf` 和 `.hex`。

| 指标 | 最终值 |
| --- | --- |
| Code | 135908 B |
| RO-data | 3520 B |
| RW-data | 276 B |
| ZI-data | 170708 B |
| 总RW+ZI | 170984 B |
| 单诊断帧（ARM链接结果） | 560 B |
| 黑匣子（101历史+before/trigger/post及元数据） | 58264 B |
| 192KiB SRAM+CCM剩余 | 25624 B，非运行时余量保证 |
| lift控制任务静态栈 | 已有8192 B分配，Keil调用链4256 B，额外预留1024 B后检查通过 |

完整日志：[keil-build.txt](Tests/Lift/Results/P0/keil-build.txt)、[stack-budget.txt](Tests/Lift/Results/P0/stack-budget.txt)。实际栈高水位和最坏执行时间需要上板测量。

### 静态复核

`git -c core.whitespace=cr-at-eol diff --check`通过，保持既有文件换行风格；对禁止修改的算法文件做diff检查无改动。确认诊断数据没有被反馈到控制决策、没有新增动态分配或高频打印；历史区没有批量临界区复制；缓存和TX来源标签与实际原子缓存快照一起传递；发送失败/过期/覆盖不修改原有输出分支。

静态复核记录：[static-validation.txt](Tests/Lift/Results/P0/static-validation.txt)。

## 10. 剩余风险及仅记录的P1/P2

1. **未进行实机验证**：无电机/CAN总线/机械系统，未验证抬升成功、物理方向、真实电流、故障保护效果、CAN ACK、DMA时序或控制任务WCET。本次软件测试与编译不代表这些结论。
2. **P1：既有所有权竞争窗口**。`lift_owner_claim/release` 的检查和更新没有统一互斥，台架还有既有主动释放CONTROL的逻辑。新增诊断保留观察能力，未改变令牌策略或修复该窗口。
3. **P1：既有遥测DMA缓冲区复用**。现有流程先构建共享帧再调用HAL发送；前一DMA仍工作时存在缓冲区被改写风险。未改变DMA发送策略，未将此当作诊断快照问题掩盖。
4. **P1：HAL接受之后的硬件证据仍有限**。本次记录了真实包和返回值，未新增TX完成中断、逐邮箱结果、错误寄存器或bus-off事件流；DJI协议本身没有命令序号ACK。反馈可能正常而某条命令丢失，不能由软件诊断虚构确认。
5. **P2：RAM、调试暂停与实时性**。黑匣子常驻约57KiB，链接总剩余约25KiB；未来增加静态任务/缓冲需重查。序列快照不关中断，但5ms采集和复制仍有成本，需上板量测。Watch暂停会影响既有看门狗与反馈，不宜带载使用断点验证。
6. **P2：历史与持久化边界**。上电/重布防未满2秒、控制任务阻塞、掉电、HardFault等无法保证2秒完整现场；任务不再执行时post_valid可保持false。记录仅在RAM，不持久化，不回填虚假的采样点。
7. **P2：接口边界**。版本跟随工程的生效入口，不检测绕过入口直接修改内部纯逻辑结构体的调试器写入；需要更强参数溯源时可在后续阶段增加配置归档/指纹。当前固件明确选择四轴控制路径；未启用的旧模态兼容路径不具备同样的逐轴PID细节，不能把未运行分支的零值当作有效测量。
8. **P2：计数与存储解释**。32位毫秒/序号按模运算回绕，累计诊断计数饱和；VOFA float会丢失大整数精度。导出含静态字符串指针的RAM记录时必须同时保存对应固件/符号文件；不能将结构体二进制视为跨版本稳定协议。
