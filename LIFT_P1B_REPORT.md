# LIFT P1-B：抬升机构故障安全与控制边界修复报告

日期：2026-10-19。基于当前已完成 P0、P1-A 的工作区；未覆盖此前未提交改动。本文描述软件审查、代码修改和无硬件验证结果，不构成机构功能安全认证，也不批准无机械支撑的载荷试验。

## 1. 结论与交付边界

**已修复**：测试入口自动确认布局、软件零点冒充机械找零、200 mm 临时行程扩展、特定故障自动复位重试、参数重载清除故障、故障停机越过电流/离线/软边界、反馈恢复自动重启故障停机电流、测试通道绕过主链锁存故障，以及缺少复位前置条件和异常状态防护的问题。

**待实机验证**：实际每轴方向、机械参考点、有效行程/余量、载荷下电流与位移关系、既有下降支路的速度/姿态、断电/掉线时电调行为、5 ms 任务最坏时延、ISR 开销及动态栈水位。没有上板、CAN 总线抓包或载荷测试；上述项目均未标记通过。

**必须依赖机械措施**：非自锁机构的失电、断线、控制器失效、防坠及人员防护。电机持续通电、HOLD、LOWER、TEST_PAUSED、低位窗口或 CAN 发出了零电流都不能证明可靠承载、停稳或安全落地。需要经设计和验证的机械支撑/制动/防坠装置、适当的隔离与独立急停链。

PID、前馈、轨迹规划、模态/独立轴架构、正常控制限幅和保护阈值未修改；保留 5 ms 控制周期、100 ms 命令失效窗口及设备层超时归零。完成前置条件的既有正常控制计算路径保留；以前依赖自动找软件零点、自动扩展、自动复位或未确认行程的入口会被有意拒绝。

## 2. 风险先行审查

### 2.1 实际停机处理链

| 触发情形 | 当前处理链及实际电气输出 | 能否证明机构仍被支撑 |
|---|---|---|
| 从未使能运动，找零完成后空闲 | `motion_command_seen=false`，不启动超时下降，输出零 | 否；取决于物理支撑 |
| 曾运动后禁用/正常停机 | `lift_control_enable(false)` → top → 既有 Guard DESCENDING/PARKED → 原配置下降电流/到窗口清零 | 否；保留既有行为，下降并非经验证的安全缓降 |
| 上层命令未按窗口刷新，但控制任务仍运行 | `command_fresh=false` → Guard 下降；下降堵转/超时进入 Guard FAULT → top FAULT → 零电流 | 否；下降电流、停车窗口和零电流均无承载证明 |
| 控制任务暂停或卡死，TX 仍运行 | 缓存不刷新；`command_is_fresh()` 超过 100 ms 后，2 ms TX 服务将四轴打包为零；保留缓存原值便于诊断 | 否；控制任务死掉时也不能自行记录它自己的故障；实际响应含调度/排队延迟 |
| TX 任务/调度器/CPU 失效 | 无法保证继续发送或清零；软件发送超时机制本身可能不运行 | 否；不能由同一 CPU 软件保证 |
| CAN 掉线/发送邮箱忙/发送错误 | 记录发送失败与最近成功；反馈超龄后进入离线保护；但零帧也可能无法到达电调 | 否；电调失联保持/关断行为须实测 |
| 单轴反馈超龄/未初始化 | top/Guard/sequence 报故；主链全轴离线门禁清零；故障电流分支也遵守全组在线条件并锁存停机资格失效 | 否；剩余轴清零可能失去承载 |
| 找零失败 | 记录逐轴 homing fault/触发周期；top 锁存；没有有效完整基准时拒绝故障 HOLD/LOWER | 否；找零期间没有完整位置基准，须机械支撑 |
| 序列运动堵转 | 原有普通流程报故；原有测试流程进入 TEST_PAUSED 并保留位置控制请求；Guard 下降堵转报故清零 | 否；测试暂停仍会通电、发热，不能认作防坠措施 |
| Guard 故障 | 保留 Guard 原保护；top 故障执行器不允许重新加回 HOLD/LOWER 电流 | 否；电气停止不能证明停稳 |
| sequence abort | 活动序列 → ABORTED 锁存 → legacy 策略请求 → top 边界 → 缓存 → CAN；默认实际为零 | 否；这是功能性中止，不是独立硬件急停 |
| JOG abort/结束 | 原有台架结束快照、清请求、自己持有通道时写零并释放 | 否；不能跨越机械防护条件 |
| 外部紧急停止/切断动力 | 工程中未找到经验证的独立制动、硬接线急停输入与抬升防坠联锁；不能把遥控关闭或 abort 当作已验证急停 | 必须由机械/独立安全链保证 |

`remote_lift` 在遥控失联/退出接管时撤销抬升使能，仍走上述既有下降链。遥控长按还会发出 reset/clear 请求；P1-B 不再允许它在缺少新的机械支撑确认时清除锁存故障。没有改写遥控架构。

本版本仍未启用 IWDG/WWDG 模块；FreeRTOS 栈溢出/分配失败 hook 现有函数体无故障安全动作。记录为后续系统失效处理工作，未擅自配置复位或电流保持方案。

### 2.2 HOLD / LOWER / CUT 与 `stop_capable`

审查发现 `lift_build_config()` 先清零配置，生产默认配置没有填写 `stop.hold_current/current_limit`。因此默认 HOLD/LOWER 请求实际上退化成零输出。不能据枚举名称推断已保持或已缓降。

- `HOLD=0`：遗留执行器请求各轴等幅正逻辑电流；没有位置保持闭环，也没有载荷能力证明。
- `LOWER=1`：遗留执行器在“任一轴未到停放点”时请求等幅负逻辑电流；没有下降速度闭环，等电流不等于等位移或姿态稳定。
- `CUT=2`：零电流命令；不等于机械制动、物理断电或安全停稳。
- `lift_stop_can_hold()` 为兼容保留，只检查正电流幅值和正上限是否配置，**不证明托得住**。
- Watch 的 `stop_capable` 和诊断 `safety.physically_verified` 均保持 false。另用 `safety.output_configured` 表达电气配置，避免“配置了电流”等同“具备防坠能力”。

遗留 `lift_safety_policy_for()` 数值决策保持不变。`seq_support_now()` 仍由流程阶段/人工声明推断支撑，`seq_fault()` 的 homed/calibrated 假设也不是物理证据；top 依据实际 homed、在线和 Guard 状态再次拦截，任务再检查标定门禁。本文不把这些阶段推断当作安全传感器。

## 3. 最小软件修复

### 3.1 停机电流边界及锁存

1. 修复 `current_limit<=0` 时仍输出正 hold_current 的错误。无效输入且输出指针有效时覆盖为零，避免残留上一周期电流。
2. top 的故障执行器继续受原 Guard 电流上限约束；未找零、离线、Guard FAULT、非法动作/状态或无可用幅值时选择 CUT。
3. 故障电流也遵守每轴原有软边界：处于上界不继续正向推，处于下界不继续负向推；未扩展行程，未提高电流阈值。只约束命令，不证明动力学安全。
4. 新增 `fault_stop_inhibited/stop_inhibited`：故障期间停机资格因离线、Guard 故障、无基准/无配置、非法状态/动作被撤销后锁存。恢复反馈或更改请求不会自动重启 HOLD/LOWER；只在合格显式复位时清除。
5. `requested_action`、实际 `stop_action`、`stop_active`、`stop_inhibited` 和四轴输出分别记录。`stop_action/stop_active` 描述故障执行器；LOWER 到停车点或所有轴被边界挡住时仍需结合 active/输出判断，不能只看枚举。正常 Guard 下降由 `guard_state` 和电流链描述。

### 3.2 零点与行程资格

- 软件 `set_zero_now` 仅在 IDLE、未 homed、无故障时存参考点，保持 IDLE/!homed；任务另要求禁用主命令、序列空闲、无所有者、四轴在线静止。它不再清故障、进入 READY 或替代真实找零。
- 记录每轴 datum：NONE=0、SOFTWARE=1、ENDSTOP=2、CONTACT=3、CONFIRMED_MIN=4。ENDSTOP 代表既有找零算法结果，不是新增的物理限位传感器证明；CONTACT 与机械最低点含义不同。
- 删除 one-click 的自动软件置零。保留 `test_auto_zero` 字段以兼容 Watch，默认 false，置 true 也不会赋予 homed。
- 删除 200 mm 临时补偿造成的前轴 173250、后轴 189000 计数上界/换算扩展；不再把 866.25/945 counts/mm 写回配置。`test_extend_travel` 默认 false；true 时拒绝测试启动，结果 11。
- 新增 `travel_calibration_confirmed`，上电默认 false。完整参数应用、找零/清找零请求及软件置零会清除此确认。正常运动需明确确认当前每轴机械基准、换算、有效行程和余量；找零仍可运行以建立基准。
- 测试/序列目标依据**已应用配置**和运动学换算检查每轴区间，最高目标不得超过 `max-limit_margin`，最低目标不得低于 min。缺少标定确认时禁用/门禁归零，并撤销本次待执行的手动 enabled。
- `apply_gains` 和软件置零不允许夹带方向、counts/mm、min/max 变更；几何变化须完整应用、重新找零和重新确认标定。参数重载不能成为故障复位入口。

保留的 `0..150000`、750 counts/mm、1500 counts 余量是历史配置数值，**不是当前实测机械行程**。按其数值计算，200 mm 超过当前含余量的目标区间，应拒绝；198 mm 只是软件测试夹具选用的边界值，**不是对实际机构推荐或批准的工作行程**。前轴接触零点改变后可用剩余机械行程须重新量测，不能沿用最低死点至最高端的历史跨度。

### 3.3 状态转换和恢复

- 测试启动不再自动设置 `sequence_layout_confirmed`。未确认布局返回 10；标定未确认/目标越界/遗留扩展请求返回 11；已有 0..9 返回语义保留。
- 删除“轴已经在软限位外”故障的自动 reset/retry；恢复反馈和重复 test_start 都不能清锁存。
- 活动阶段持续检查 homed、标定、布局和在线条件；丢失前置条件即故障，不继续使用上拍有效目标。
- 非法 top/sequence 状态明确转故障；序列在暂停/释放处理前验证枚举，防止非法状态被暂停逻辑变成带电保持。
- reset 必须有本周期 `fault_recovery_confirmed`（人工声明已安装机械支撑）、原始运动命令禁用、四轴在线静止。top 使用原始 rpm ±2 的既有静止窗口；sequence 使用原有 settled_speed 条件。sequence 同拍 start/abort 会拒绝复位。
- 任务使用**门禁前的原始 enabled**检查复位，防止“命令被门禁禁用”冒充操作手主动撤销命令。
- 未满足条件记录拒绝位，不清故障。接受复位后当拍不再启动/下降；清 `motion_command_seen`，需新的 enabled/start 请求。没有自动复位、自动重试；持续遥控来源的停用/重新使能操作仍需实机验证。
- 若序列故障尚在，top 不允许绕过；可在禁用且机械支撑确认的同次服务周期同时请求 sequence reset 和 top reset。只复位一个层级不保证另一层故障消失。

### 3.4 测试通道不能穿过主链故障

`lift_owner_inhibit_tests()` 在 P1-A 既有短锁内锁存测试禁入，并原子撤销 JOG/CAN_TEST 租约；CONTROL 的停机执行通道保留。故障时 claim、override、may_write 对测试所有者均拒绝。

主任务故障锁存后，不再因为 JOG/CAN_TEST 的 running 标志而跳过故障输出。JOG 清除三种模式的待执行 enabled，结束快照原因新增 `LIFT_TEST_END_FAULT_INHIBIT=9`；CAN1 0x200 测试取消请求/运行状态。TX 最终打包检查阻止无控制周期来源的旧台架缓存或测试覆写继续发送；CONTROL 产生的合格故障停机输出仍由原执行器决定。测试恢复后必须重新给出启动请求。

该边界只针对抬升组，不改变 CAN2 等其他设备的独立诊断功能。JOG/CAN_TEST 本身仍是绕过正常位置规划/行程控制的台架入口，不应作为正常抬升入口；必须在隔离、机械支撑条件下使用。已进入 CAN 邮箱/在途的帧不能由此撤回，软件故障响应也受任务调度影响。

## 4. P0 诊断复用和 Keil Watch

继续使用 `lift_diag_publish/get_snapshot`、transport 快照和黑匣子，不新增第二套采样系统。诊断模块仍不参与控制决策；运动门禁状态属于 safety/控制层，诊断只复制其结果。沿用 P1-A 有界一致性读取和短锁，不在高频路径打印，不新增动态分配。

| Watch 表达式 | 含义/用法 |
|---|---|
| `g_lift_diag.sequence`、`g_lift_diag.frame.cycle_id` | 读 sequence 为偶数且前后相同的一致周期；程序读者使用快照 API，失败本次跳过 |
| `g_lift_diag.frame.safety.reason` | 停机/门禁观察原因：0无、1禁用、2命令超时、3离线、4找零、5Guard、6序列、7中止、8堵转、9边界、10基准拒绝、11非法状态 |
| `g_lift_diag.frame.safety.requested_action`、`.stop_action` | 请求动作与 top 实际选择；HOLD0/LOWER1/CUT2；结合输出和 active 解读 |
| `g_lift_diag.frame.safety.output_configured/physically_verified` | 电气幅值有配置 / 物理能力验证（固定 false） |
| `g_lift_diag.frame.safety.stop_inhibited/tests_inhibited` | 故障停机资格已撤销锁存 / 台架通道禁止穿越主链故障 |
| `g_lift_diag.frame.safety.output_zero` | 门禁后提交意图是否全零；真正缓存/打包仍以 transport 为准 |
| `g_lift_diag.frame.safety.datum_kind[0..3]` | 四轴零点类型；SOFTWARE 不授予运动资格 |
| `g_lift_diag.frame.safety.travel_confirmed` | 本周期机械行程确认声明；声明不是传感器证据 |
| `g_lift_diag.frame.safety.reject/reject_axis/reject_cycle_id` | 最近请求拒绝原因、轴号和周期。reject：0无/1故障/2忙/3离线/4运动/5基准/6布局/7行程/8恢复条件 |
| `g_lift_diag.frame.safety.recovery_blocked` | 本周期复位拒绝位：1缺支撑声明、2命令未禁用/同时启动、4离线、8仍运动、16序列故障仍锁存 |
| `g_lift_diag.frame.reason_axis[]`、`guard_fault_axis`、`sequence_fault_axis` | 各原因触发轴，0..3；-1为整组/未知。结合同周期故障码及 detail |
| `g_lift_diag.frame.axis[ch]` | 原目标→参考→PID需求→限幅→Guard→保护后→submitted_raw→反馈；逻辑正=伸出，raw按电机原始方向 |
| `g_lift_diag.frame.transport` | cache_raw/cache_cycle_id/generation；packed_raw/tx_cycle_id/generation；send_result、failed_count、last_success_ms、timed_out |
| `g_lift_blackbox` | 101槽×20 ms，已有足够运行时间时约2 s；before/trigger/post 各自标注周期；首次原因锁存后保持 |
| `g_lift_diag.event_count[11]` | 新增 REQUEST_REJECT 原因上升沿计数；旧0..10原因索引保持；饱和行为沿用P0 |
| `g_lift_control_debug` | 布局/标定确认、fault_recovery_confirmed脉冲、sequence_reset；旧Watch字段保留 |
| `g_lift_control_status.request_reject*` | 平铺兼容观察；完整一致周期证据以 g_lift_diag 为准 |
| `g_lift_test_snapshot.reason` | JOG结束原因，新增9=主链故障禁入 |
| `g_runtime_monitor_status` | 任务最低栈余量、heap_free/min_free，仅实机运行时才有测量意义 |

新增 `LIFT_TOP_FAULT_STATE=6`。诊断只追加原因和 safety 字段；原成员名、旧原因编号及 source 帧布局保留，但内部结构尺寸发生变化，外部按裸内存偏移读取的脚本必须重新按本版 ELF/结构解释，不能沿用 P0/P1-A 固定地址。

`reject` 保留最后一条证据，`reject_cycle_id` 区分历史与当前；只有当拍拒绝进入当拍黑匣子原因，不把旧请求当新事件。故障原因、触发轴和周期主要查看 trigger；post 是下一完成周期，不能冒充同拍反馈。

### 定位和恢复流程

1. 出现异常先确保物理支撑和隔离，保存黑匣子，不先 reload/zero/reset。确认 cycle_id、trigger 原因、轴号和 datum/行程确认是否有效。
2. 对同一轴比较 PID需求、限幅、Guard、protected_current、submitted_raw；查 gate、stop_inhibited、tests_inhibited。若请求被拒，按 reject_cycle_id 查当前证据。
3. 再比较缓存与打包：先核对 cache/tx generation 和 cycle_id，它们异步刷新，不能要求现场快照总相等。看发送返回、失败数和最近成功时间；最后看每轴反馈年龄/计数/电流。**控制器算过、缓存写过、CAN接受请求、看到新反馈是四件事；DJI反馈没有本控制周期ACK。**
4. 在机械支撑下排除原因，撤销所有运动和测试请求；确认四轴反馈新鲜且静止。显式发送支撑确认脉冲和相应层级 reset，检查 blocked 位/故障未残留；复位周期输出为零。确认脉冲每拍消费，必须与请求在同一服务周期到达；Keil逐字段手工写入可能错拍而被拒绝。可在机构已有独立机械支撑且所有运动源禁用时暂停CPU，写好这几个请求再恢复执行；运行期接口应在短事务内成组提交，不能把确认长期置位代替当次确认。
5. 零点/配置变化后完成真实找零和测量，重新确认行程；检查布局，再显式重新使能/启动。不得只恢复连接就让危险运动自行恢复。
6. 保存原记录后才置 `g_lift_diag_rearm=true` 重建历史。它只重装诊断，不清控制故障；原因仍存在时会立即再锁存。启动拒绝也可能先冻结黑匣子，完成准备后应在无原因状态重新装载并积累历史。

### VOFA 兼容性

USART1、115200、8N1、JustFloat 帧尾 `00 00 80 7F` 保持；source=0 为8通道/36B，1/2/3为28通道/116B。P1-A DMA忙丢帧、不覆盖在途缓冲的机制原样保留，P1-B不改遥测实现。

source=2 看四轴位置/目标/提交电流/实际与目标速度/限幅/反馈电流，每轴7通道。source=3 配 `diagnostic_axis=0..3` 看完整电流链：通道8..17为PID、前馈、需求、限幅、Guard、保护后、提交、缓存、打包、反馈；18反馈年龄、19门禁、20原因位、21/22缓存/TX周期、23发送结果、24失败数、25最近接受请求年龄。新 safety 详细枚举保留在 Watch/黑匣子中，未挤占旧通道。长时间周期ID超过float精确整数范围时应以Watch整数记录对齐，不能仅依赖波形浮点ID。

## 5. 验证结果

### 可执行软件测试：通过

完整运行 `Tests/Lift/run_tests.ps1`，GCC C11、`-Wall -Wextra -Wpedantic -Werror`：**19组可执行回归 + 7项固件源码严格编译检查全部通过**。完整记录见 `Tests/Lift/Results/P1B/host-regression.txt`。

| 类别 | 覆盖 |
|---|---|
| 新增故障注入 | 零/负上限、NULL配置、非法动作/top/sequence状态、非法状态同时暂停、离线/Guard、故障输出软边界、反馈恢复不重启故障电流 |
| 新增恢复边界 | 无支撑声明、原命令仍启用、反馈离线、轴仍运动、同拍start/reset均拒绝；合格复位零输出周期、无旧使能自动重启；门禁不能冒充禁用 |
| 新增基准/行程 | 自动软件零点无效、软件零点不homed；布局未确认/标定未确认/旧扩展开关/200mm越含余量边界拒绝；配置不被扩展；增益/完整应用/置零不能清故障 |
| 新增通道测试 | 故障撤销租约；claim/override/may_write拒绝；旧JOG缓存不能穿过TX；活动CAN测试取消；JOG结束快照原因9、清待执行请求；恢复后不自动再跑 |
| 保留P0/P1-A | 120000次所有权竞争、错误/重复释放、超时/回绕/旧时间戳、嵌套事务；并发live/history/rearm/cache/TX快照；DMA忙/TC未完成/发送错误恢复、source0..3布局和200连续遥测帧 |
| 保留运动回归 | homing/guard/sync/kinematics/safety/control/system/top/sequence/geometry/properties、轴链、负载模型、台架、底盘上台阶等全部原套件 |

原来用软件置零建立 homed、依赖200mm补偿或自动故障重试的正向夹具，改为通过真实 top 更新路径模拟已确认的机械基准、显式确认布局/行程和合格复位。198mm仅用于软件包络测试；未删除整套回归、未关闭保护、未用模拟器收敛代替真实承载验证。

### Keil 全量构建：通过

实际工具：`D:\keil5_32\UV4\UV4.exe`，目标 `lift_infantry`，工程 `MDK-ARM/lift_infantry.uvprojx`。使用 `-r` 全量重编译，日志明确：`Using Compiler 'V6.21'`；**0 Error(s), 0 Warning(s)**，AXF/HEX生成成功。记录见 `Tests/Lift/Results/P1B/keil-build.txt`。

最终 Program Size：Code **143612 B**、RO **3584 B**、RW **276 B**、ZI **173180 B**；RAM合计 **173456 B**，ROM合计 **147472 B**。日志与源文件/MAP/调用图/AXF哈希见 `source-hashes.json`，不是仅做主机语法检查。

## 6. RAM、栈和5ms实时性

### 实际链接分布

| 区域/对象 | 当前地址 | P1-B大小/使用 | 与P1-A比较 |
|---|---|---:|---|
| 普通SRAM RW_IRAM1，128KiB | 0x20000000 | 119496 B，未分配11576 B | +1512 B |
| CCM RW_IRAM2，64KiB | 0x10000000 | 53960 B，未分配11576 B | +1520 B |
| 合计 | — | 173456 B，未分配23152 B | +3032 B |
| 黑匣子 | 0x200009E4 | 61176 B | 原58264，+2912 B |
| live Watch诊断 | 0x1000374C | 640 B | 原608，+32 B |
| transport | 0x1000024C | 92 B | 不变 |
| 控制任务诊断工作帧 | 0x20010878 | 588 B | 原560，+28 B |
| 遥测快照 | 0x1000ABC0 | 588 B | 原560，+28 B |
| FreeRTOS ucHeap | 0x20010FF0 | 49152 B，已计入ZI | 配置不变，实际空闲堆须实机读取 |
| USART3 RX DMA buffer | 0x200008E0 | 36 B，普通SRAM | DMA可访问 |
| USART1 TX DMA frame | 0x20010F10 | 116 B，普通SRAM | DMA可访问 |
| MSP栈 / C库堆保留 | 0x1000CEC8 / 0x2001D0C8 | 1024 / 512 B | 既有配置 |

STM32F407 普通SRAM1/2合计128KiB，CCM为64KiB且CPU专用；DMA不能使用CCM。依据厂商[RM0090内存与总线架构](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405415xx-stm32f407417xx-stm32f427437xx-and-stm32f429439xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)。本次用实际MAP确认DMA缓冲地址，未修改链接配置。未把未分配RAM、ucHeap和诊断子对象重复相加。

诊断模块对象 `lift_diagnostics.o` ZI为61924 B（P1-A为58980 B）。本版ARM诊断帧588 B、101历史槽及before/trigger/post结构不变；主机64位编译帧为592 B、黑匣子61592 B，不拿主机 sizeof 充当目标RAM值。

### 静态栈检查：通过，动态水位未验证

沿用 `check_p1a_budget.py` 检查全部17个任务，再运行 `check_stack_budget.ps1`。最终抬升栈分配8192 B、Keil调用图最大深度4296 B，另留1024 B后余2872 B；P1-A深度4256 B，本次增加40 B。任务栈配置未改。

JOG：3072/656 B；TX：2048/200 B；telemetry：2048/128 B。启动任务恢复为2048/1352 B，未提高栈或放宽审计：初次构建因运行期反馈校验被纳入初始化调用链，静态预算失败；分离运行期校验入口后已解决。最终详细17任务表在 `memory-stack.txt`。

既有窄余量仍保留风险：link任务1024/500 B，另留512 B后仅12 B；启动任务另留512后184 B；Idle另留256后184 B。静态图不含所有ISR嵌套、未解析回调、真实FPU上下文与运行时路径，不能据此宣称不溢出。硬件上应读 `g_runtime_monitor_status.tasks[].stack_min_free_bytes` 及堆最低水位并留证据。

### 实时性评估与测量方法

新增稳态路径是固定四轴/枚举检查、短锁标量和约28 B的诊断帧增量；参数校验只在请求时做。控制周期仍5 ms、TX仍2 ms、黑匣子仍20 ms；没有新动态分配、高频日志、阻塞等待或整段历史复制临界区。**不能由代码规模、主机运行速度或编译通过推断5ms WCET通过。**

机械支撑、车轮隔离并清空危险动作后，用临时测量构建在任务唤醒/反馈快照前、最终缓存提交后及诊断发布后各取DWT CYCCNT或GPIO边沿。用实际 `SystemCoreClock` 将无符号周期差换算为时间，同时记录唤醒间隔、执行时间、缓存提交延迟和CAN线上帧时间。覆盖正常运动、20ms记录点、首次故障before/trigger/post复制、请求拒绝、参数应用、DMA忙、CAN高负载和中断并发。检查总执行时间/调度延迟是否满足5ms及所需裕量；GPIO/总线分析仪校验DWT结果，观察全系统最高负载下的栈最低余量。调试器暂停时的结果不用于证明实时性。

## 7. 必须在机械支撑条件下进行的实机验证

1. **逐轴测量，先不带危险载荷**：核对电机ID/前后布局、原始与逻辑方向；分别记录真实最低端、前轴接触参考点、最高可用位置、每段已知位移对应计数、干涉区及回差。加载变形/接触参考变化后重测有效区间，按实际制动/惯性留足余量；不得用历史150000/750或临时补偿推导当前全行程。记录未测项为空，不预填为通过。
2. **边界试验**：先以受约束的小位移核对命令、参考、保护后、缓存和线上电流方向；验证未确认布局/行程不能启动，软件零点不homed，找零/配置变化清标定确认。任何机构扩展参数均先评审测量证据，不能靠置flag代替标定。
3. **故障矩阵**：在能独立承受整车载荷的支撑/防坠装置下，逐轴断反馈、断CAN、阻塞发送邮箱、挂起控制任务、挂起TX任务，注入Guard/堵转/中止。保存trigger/post、总线帧、实际位移/速度、载荷/温度和时间；恢复连接不能重启故障停机电流/测试通道，未确认复位必须被拒绝。
4. **电调和动力丢失**：验证最后电流帧之后，电调掉线、CAN bus-off、MCU复位、断电时究竟如何动作；独立测量下坠/制动距离和支撑承载，不依赖主机继续发送“保持”或“零电流”。
5. **恢复/移交**：禁用所有远程/Watch/测试请求，在新鲜静止反馈和物理支撑下显式确认、复位；确认复位当拍零输出、旧请求不复活、测试必须新启动。核对遥控持续上档等来源，避免它继续发送新的enabled；验证COMPLETE移交手动、PARKED和低位窗口不会被误当作机械支撑成立。
6. **独立急停/防坠**：由机械与电气安全设计确认额定载荷、失效姿态、制动器失电状态、保持/防坠冗余及验证方法。软件现有abort和电机电流不能替代这些措施。未完成该验证前，不批准人员进入机构下方或无防坠的悬空载荷运行。

## 8. 修改清单、范围检查和剩余风险

本次修改既有文件18个，另新增本报告及结果文件；按开始P1-B时的工作区快照比较，避免把P0/P1-A改动误算成本次内容。

| 文件 | 修改内容 |
|---|---|
| `Application/Lift/Inc/lift_safety.h`、`Src/lift_safety.c` | 恢复拒绝位；停机幅值边界/无效输入；测试所有者故障禁入，复用P1-A锁 |
| `Application/Lift/Inc/lift_top.h`、`Src/lift_top.c` | datum类型、软件零点资格、故障复位前置条件、停机边界/失效锁存、非法状态、电气能力与物理能力分离 |
| `Application/Lift/Inc/lift_seq.h`、`Src/lift_seq.c` | 显式恢复输入/拒绝位、活动前置条件持续检查、非法状态优先报故；未改阶段运动算法和默认阈值 |
| `Application/Lift/Inc/lift_control_task.h`、`Src/lift_control_task.c` | 去掉测试绕过/临时扩展/自动复位，机械行程声明与目标包络、参数入口保护、原命令复位校验、诊断采集、分离运行期校验调用链 |
| `Application/Lift/Inc/lift_diagnostics.h` | 追加REQUEST_REJECT原因及safety字段，旧原因/遥测布局保留 |
| `Devices/RobotMotors/Src/robot_motor_commands.c` | 抬升CAN测试故障取消和最终旧测试缓存/覆写拦截，保留100ms超时归零 |
| `Diagnostics/MotorTest/Inc/lift_jog_task.h`、`Src/lift_jog_task.c` | 故障禁入结束原因9、清待执行请求和故障通道退出 |
| `Tests/Lift/test_lift_axes.c` | P1-B故障注入/恢复/布局/行程/参数绕过/非法状态及正向夹具更新 |
| `Tests/Lift/test_lift_transport.c`、`test_lift_jog.c` | 测试通道故障注入、旧缓存、取消、恢复不重启 |
| `Tests/Lift/test_lift_safety.c`、`test_lift_seq.c`、`test_lift_top.c` | 与新输入/失效输出契约一致的回归及合格复位夹具 |
| `LIFT_P1B_REPORT.md` | 本报告 |
| `Tests/Lift/Results/P1B/` | host-regression.txt、keil-build.txt、memory-stack.txt、scope-check.txt、source-hashes.json |

范围检查基线：`D:\rm\lift_infantry\_staging\p1b_baseline_20261019`。SHA256核对确认 PID、lift_control（含前馈/轨迹/正常限幅）、sync、guard、homing、kinematics、geometry、FreeRTOS配置/任务栈、工程链接配置、USART、遥测实现、P1-A诊断实现及P0/P1-A报告未改。legacy策略和sequence默认配置去注释后等价；5ms/100ms及当前电流/行程/余量常量未改。沿用各文件原有CRLF/LF；仓库`* -text`下用 `git -c core.whitespace=cr-at-eol diff --check` 和额外字节检查验证，未为通过检查批量转换行尾。证据见 `scope-check.txt` 与 `source-hashes.json`。

剩余风险明确保留：真实行程/动力学/承载/防坠未经测量；等幅故障电流没有可靠保持或速度控制证明；阶段支撑推断/人工确认不是安全传感器；CAN接受不等于送达、零命令不等于停稳；控制器/调度器全失效时软件链无独立保障；JOG/CAN测试仍需受控台架条件；Watch直接篡改内部状态可破坏所有软件约定；栈/实时性必须实机测量。机械补偿重设计、三模态/独立轴架构、失电制动/安全硬件方案和系统级故障监护列入后续任务，本次未擅自启用。
