# LIFT_P1A_REPORT

日期：2026-10-19。工程：`MDK-ARM/lift_infantry.uvprojx`，STM32F407，Arm Compiler 6.21。

## 1. 结果与修改边界

P1-A 已完成代码修改和软件验证：所有权状态/租约操作互斥，JOG 不再代 CONTROL 释放令牌，最终所有权检查与电流缓存写入成为短事务；CAN1 0x200 测试与零保持阶段显式持有 CAN_TEST 所有权；USART1 遥测在确认前一 DMA 和 UART 发送均完成后才构造下一帧；P0 实时帧、发送状态和黑匣子采用原子共享访问及原有序列校验。

保持 PID、前馈、轨迹规划、限幅/保护阈值、故障停机策略、机械行程补偿、上台阶状态机、控制周期及正常电流数值不变。保留 P0 Watch 变量、结构体布局、快照接口、黑匣子容量和 VOFA source=0/1/2/3 布局。没有新增动态分配或高频阻塞日志。

**最终软件结果：19组可执行测试、7项严格编译检查通过；Keil 6.21 全量重建0错误、0警告。未上板，5ms最坏响应时间、真实中断延迟、栈高水位及DMA/CAN硬件行为尚未验证。**

工作区已有 P0 及用户修改，未覆盖、未提交。开始 P1-A 时建立了工作区文件哈希及源码基线，范围复核针对该基线进行，不将已有 Git diff 误算为本次修改。P0 报告保持原样，本报告覆盖其中相关风险的最新状态。

## 2. 修改清单

| 文件 | 修改 |
| --- | --- |
| `Application/Lift/Inc/lift_lock.h`（新增） | 保存/恢复PRIMASK的短、可嵌套排他；主机使用真实递归原子锁 |
| `Application/Lift/Inc/lift_safety.h`、`Src/lift_safety.c` | 互斥保护claim/release/current/timeout/tick/reset；新增显式原子交接API；修复旧时间戳误过期 |
| `Application/Lift/Src/lift_control_task.c` | CONTROL认领→缓存写入→归还在同一短事务；原有冲突后的PID复位移到锁外，执行条件不变 |
| `Diagnostics/MotorTest/Src/lift_jog_task.c` | JOG显式交接、授权与缓存写入同事务；零输出也检查所有权；删除代CONTROL释放路径 |
| `Devices/RobotMotors/Src/robot_motor_commands.c` | 抬升CAN测试及零保持期间取得/刷新CAN_TEST租约，结束只释放自身；非抬升组不占用抬升所有权 |
| `Application/Lift/Inc/lift_diagnostics.h`、`Src/lift_diagnostics.c` | 原子共享数据/元数据访问；内部串行化缓存/TX写入；原子消费rearm，新增任务用请求API |
| `Diagnostics/Telemetry/Src/telemetry_task.c` | 构帧前检查UART/DMA状态；source=0/1短字段快照；source=2/3共享560B暂存区 |
| `Tests/Lift/test_lift_owner_concurrency.c`（新增） | 三角色真实线程竞争、错误/重复释放、交接、租约及写事务 |
| `Tests/Lift/test_lift_diag_concurrency.c`（新增） | 发布者、缓存写入者、TX写入者、两个读取者并发压力测试 |
| `Tests/Lift/test_lift_telemetry.c`、`telemetrystubs/*.h`（新增） | 运行真实遥测任务，模拟异步DMA/UART状态及协议数据 |
| `Tests/Lift/test_lift_jog.c`、`test_lift_transport.c` | 验证真实JOG零输出和CAN测试/零保持所有权路径 |
| `Tests/Lift/run_tests.ps1` | 注册3组新测试；并发测试使用-pthread |
| `Tests/Lift/check_p1a_budget.py`（新增） | 从实际MAP/调用图检查SRAM/CCM、DMA地址及17个任务栈预算 |
| `Tests/Lift/Results/P1A/*.txt`（新增） | 完整构建、测试、内存/栈和代码范围记录 |
| `LIFT_P1A_REPORT.md`（本文件） | 设计、验证、内存及上板测量方法 |

与P1-A前基线逐字节对比确认：`pid.c`、`lift_control.c`、`lift_guard.c`、`lift_homing.c`、`lift_top.c`、`lift_seq.c`、`lift_sync.c`、scatter、FreeRTOS配置、USART初始化及P0报告未改。`lift_safety_policy_for`及其后的全部停机策略实现与基线一致。

## 3. 所有权修复

### 原子操作和最终写入

此前单次32位读写原子不能保证“检查当前所有者→更新所有者/租约”原子。现所有相关状态都通过同一个 `lift_lock_enter/exit` 访问。

目标端用PRIMASK保存/恢复，不依赖调度器已启动；内层退出恢复原掩码，不会提前打开外层禁止的中断。主机端用静态atomic_flag和线程局部嵌套深度提供真实互斥，因此并发测试不会因RTOS空桩而假通过。主机实现不进入目标固件。

CONTROL最终认领、四轴缓存写入、归还共用外层短锁；JOG认领/交接和四轴缓存写入也同锁。JOG最终写零使用同样的检查+写入事务。锁内只包含身份/时间戳、4轴命令及小型诊断缓存更新，不包含控制数学、PID复位、HAL发送、历史复制或阻塞调用。

`lift_owner_may_write/current` 仅为观察接口，不单独作为稍后写缓存的授权。普通claim不抢占；release只有指定身份与当前所有者相同才生效，空闲/重复/错误身份释放不影响新所有者。代码中已移除JOG调用 `release(CONTROL)` 的路径。

### 显式交接与CAN测试

新增 `lift_owner_claim_override(owner, now, &previous)` 原子完成交接，无中间NONE窗口，策略为：

| 请求者 | 可接管 | 不可接管 |
| --- | --- | --- |
| CONTROL | 普通claim：NONE或自身 | JOG、CAN_TEST |
| JOG | NONE、自身、CONTROL | CAN_TEST |
| CAN_TEST | NONE、自身、CONTROL、JOG | 无效身份/参数 |

保留原有JOG优先于主链、CAN测试整帧替换优先的调试语义，取消通过冒充另一个角色释放实现抢占。JOG的既有丢失/抢占计数继续工作。

只有CAN1 0x200测试取得抬升CAN_TEST租约；活动和零保持期间每个发送周期刷新；零保持结束只释放CAN_TEST。其他总线/组测试不抢抬升令牌。原测试时长、电流、槽位、帧替换、零保持时长和缓存超时规则未改变。

### 超时边界

保留 `age > timeout` 才过期及timeout=0禁用规则。新增对旧采样时间的识别：若调用者取now后被抢占，其他操作已刷新为更新租约，不能将负时间差当成接近2^32ms的超时。时间比较采用模运算，要求有效间隔小于2^31ms；正常50ms租约行为不变。

每个角色仍由工程中的单一任务使用。旧身份枚举API保留，调用点必须传自身身份；未来若同一角色增加多个独立任务或异步延迟释放，应另行引入带代际的租约句柄。

## 4. VOFA DMA完整性

当前工程中USART1 TX唯一生产者是遥测任务。复用HAL已有生命周期：DMA未完成，或DMA结束但UART最后字节TC尚未完成，均不构造下一帧，也不写 `telemetry_frame`。只有 `huart1.gState == HAL_UART_STATE_READY` 且 `HAL_DMA_GetState(huart1.hdmatx) == HAL_DMA_STATE_READY` 才构帧并提交。

检查在任何构帧、源切换和帧尾写入之前执行。忙时增加既有busy计数并等待下个20ms周期，不阻塞等待完成，不另建回调或超时强行复用缓冲区；HAL错误或DMA未恢复时不会越过所有权边界重写数据。

保持USART1、115200、8N1、JustFloat及帧尾 `00 00 80 7F`：source=0仍8通道/36B，source=1/2/3仍28通道/116B，所有索引含义沿用P0。source=0/1字段采集置于短临界区，防止其任务写入者在读取期间更新；source=2/3继续读取P0一致性快照。两个互斥使用的诊断暂存帧合并为一个，减少560B常驻RAM。

这里依赖“仅一个USART1 TX生产者”的工程约束。若以后加入其他USART1发送者，必须统一提交接口，不能仅凭HAL READY检查共享该缓冲区。

## 5. P0并发访问修复

原P0序列号能够拒绝混合周期，但volatile结构体复制与并发写入仍缺少语言/编译器层面的原子共享访问；transport写入还依赖调用方正确持锁。

本次保留结构体及序列接口，改为：

- 帧数据按对齐32位原子字复制，使用GCC/Armclang支持的may_alias表示访问；标量元数据使用对应宽度的原子读写；序列发布使用原子操作和内存屏障。目标帧和transport大小有编译期字对齐检查，目标代码无需原子运行库锁。
- 缓存和TX hook内部使用同一短锁串行化，独立写入者不会互相破坏transport序列；原子读接口保持最多3次尝试。
- 活跃帧、before/trigger/post、环形历史读写及黑匣子元数据均遵循该协议。帧/历史复制不关中断；读到更新中的序列即失败/重试，不等待低优先级写入者完成。
- `lift_diag_request_rearm()`供任务原子请求；发布者通过atomic exchange消费。交换之后的新请求保留到下个周期，修复原先“先读后清”可能吞掉请求的窗口。原Watch布尔变量仍可写1；重复请求仍按布尔语义合并，不是请求计数队列。

`lift_diag_publish`仍限定一个控制任务写入；`lift_diag_init`仅启动初始化时调用。工程启动任务在调度锁内创建/初始化，因此初始化不会与业务读写并发。运行中其他任务应调用快照API，不直接读写共享Watch结构；调试器逐字段刷新的画面本身仍不构成原子快照。

## 6. 测试与编译

执行：`powershell -ExecutionPolicy Bypass -File Tests/Lift/run_tests.ps1`。

**19组可执行测试全部通过，7个固件源严格编译检查通过**，沿用 `-std=c11 -Wall -Wextra -Wpedantic -Werror`。新增并发用例运行真实源文件和真实主机线程，不用顺序调用代替竞争。

| 测试 | 验证内容 |
| --- | --- |
| owner_concurrency | 三身份共120000次竞争；同时持有计数不超过1；错误/重复释放；无效身份；CONTROL→JOG→CAN_TEST交接；拒绝反向抢占；50ms边界、32位回绕、续约后的旧now、禁用超时；可嵌套最终缓存事务 |
| diag_concurrency | 16000次周期发布、16000次缓存写入、16000次TX写入和两个读取线程并发；活跃帧、历史/触发记录、rearm、transport字段及计数一致性 |
| telemetry | 60个忙周期和任意source切换期间DMA数据逐字节不变；DMA完成但UART TC未到；UART/DMA状态不一致；HAL_BUSY/ERROR后恢复；source=0/1/2/3既有布局；200帧连续轮换；disabled及无有效诊断快照 |
| 真实JOG/发送任务补充 | 未拥有通道时不写零/不误释放；CAN测试及零保持持续持有自身；结束释放；其他总线不抢抬升所有权；缓存/打包/限幅原有断言仍通过 |
| 原16组回归 | P0黑匣子、缓存、控制任务、PID、Guard、找零、流程、运动学、仿真性质、台架等全部通过 |

软件线程压力测试覆盖实际并发读写，但不是所有可能调度交错的形式化证明，也不是Cortex-M中断时序测试。

使用 `D:\keil5_32\UV4\UV4.exe -r <工程绝对路径> -t lift_infantry -o <日志绝对路径>` 全量重建。日志确认 `Using Compiler 'V6.21'`，**0 Error(s), 0 Warning(s)**，生成AXF/HEX。

| 项目 | P0 | P1-A |
| --- | ---: | ---: |
| Code | 135908 B | 139188 B |
| RO-data | 3520 B | 3520 B |
| RW-data | 276 B | 276 B |
| ZI-data | 170708 B | 170148 B |
| 总RW+ZI | 170984 B | 170424 B |

日志：[host-tests.txt](Tests/Lift/Results/P1A/host-tests.txt)、[keil-build.txt](Tests/Lift/Results/P1A/keil-build.txt)、[static-scope.txt](Tests/Lift/Results/P1A/static-scope.txt)。

## 7. MAP内存与栈分析

执行 `python Tests/Lift/check_p1a_budget.py`，同时继续执行已有 `check_stack_budget.ps1`。两者通过；脚本中的预留是静态筛查标准，不是实机栈安全保证。

### SRAM/CCM实际分布

scatter保持不变，DMA专用段显式进入SRAM，CPU数据/栈由链接器分配；不依赖仅看C变量声明推断地址。

| 区域/对象 | 地址 | 使用/大小 | 余量/说明 |
| --- | --- | ---: | --- |
| SRAM RW_IRAM1，128KiB | 0x20000000 | 117984 B | 13088 B未分配 |
| CCM RW_IRAM2，64KiB | 0x10000000 | 52440 B | 13096 B未分配 |
| 总192KiB | — | 170424 B | 26184 B未分配，比P0多560 B |
| USART3遥控RX DMA | 0x200006E8 | 36 B | SRAM，DMA可访问 |
| USART1遥测TX DMA | 0x2001040C | 116 B | SRAM，DMA可访问 |
| 黑匣子 g_lift_blackbox | 0x200008B0 | 58264 B | 101历史+before/trigger/post，未缩减 |
| g_lift_diag | 0x2000EE54 | 608 B | Watch实时帧与计数 |
| g_lift_diag_transport | 0x200002C4 | 92 B | 缓存/发送证据 |
| 控制诊断暂存 | 0x10004D30 | 560 B | CCM，仅CPU使用 |
| 遥测快照暂存 | 0x2000FE78 | 560 B | CPU暂存，不直接交DMA |
| FreeRTOS ucHeap | 0x200105B0 | 49152 B | 已包含在总ZI，不能再与未分配RAM相加当可用堆 |
| 主异常栈MSP | 0x1000C8D8 | 1024 B | IRQ嵌套/FPU需实测 |
| C运行库堆 | 0x2001CAE0 | 512 B | 未新增使用 |

`lift_diagnostics.o`自身ZI为58980B；主控制和遥测各另有560B暂存，已有旧单次快照另计。本次未增加固件动态内存申请；工程原有动态RTOS对象、启动任务和USB相关分配仍使用现有堆。MAP的未分配空间与 `xPortGetFreeHeapSize()` 是不同指标。

### FreeRTOS栈预算

| 任务 | 分配B | Keil调用链B | 筛查预留B | 预留后剩余B |
| --- | ---: | ---: | ---: | ---: |
| lift_control_task | 8192 | 4256 | 1024 | 2912 |
| lift_test_task/JOG | 3072 | 664 | 512 | 1896 |
| robot_motor_tx_task | 2048 | 200 | 512 | 1336 |
| telemetry_task | 2048 | 128 | 512 | 1408 |
| can_rx_task | 2048 | 136 | 512 | 1400 |
| chassis_control_task | 2048 | 616 | 512 | 920 |
| friction_task | 2048 | 600 | 512 | 936 |
| feed_task | 2048 | 600 | 512 | 936 |
| shooter_task | 2048 | 608 | 512 | 928 |
| yaw_task/pitch_task（各） | 2048 | 616 | 512 | 920 |
| remote_task | 2048 | 128 | 512 | 1408 |
| runtime_monitor_task | 1536 | 128 | 512 | 896 |
| link_task | 1024 | 500 | 512 | **12，偏紧** |
| StartSystemTask | 2048 | 1352 | 512 | **184，初始化路径需测** |
| Idle | 512 | 72 | 256 | **184，需留意** |
| Timer | 1024 | 136 | 512 | 376 |

共17个任务预算项（含启动任务、Idle、Timer）。未改变任何任务栈、优先级、FreeRTOS配置或周期。启动任务分配来自堆并在初始化后退出；应在退出前采集其高水位。静态调用图可能低估间接调用/回调及运行环境需求，预留不能替代运行验证。

完整记录：[memory-stack.txt](Tests/Lift/Results/P1A/memory-stack.txt)、[lift-stack.txt](Tests/Lift/Results/P1A/lift-stack.txt)。

## 8. 5ms实时性评估及实机测量方法

### 当前可以确认的静态边界

控制周期仍为5ms，未加入等待锁释放的目标端循环、动态分配或打印。目标短锁是固定指令的掩码保存/恢复，不是阻塞互斥量；所有权和小型transport事务之外的计算可被正常中断。

诊断每周期固定4轴、11类原因；普通周期复制560B实时帧，20ms采样周期再复制560B历史。首次故障最多复制before、实时帧、trigger，共1680B；下一周期另存post，后续不复制整个历史区。每次快照最多3次尝试。原子字访问增加固定指令和屏障成本，但没有历史长度相关的关中断复制。它们是有界工作的证据，**不能单凭这些计数证明满足5ms截止时间**。

当前时钟配置为12MHz HSE，PLLM=6、PLLN=168、PLLP=2，名义HCLK=168MHz；5ms对应840000个CPU周期，实际换算须以实机 `SystemCoreClock` 为准。116B遥测帧在115200/8N1约10.07ms，20ms周期约占50.35%线路时间；DMA/UART中断延迟仍可能导致忙帧被跳过。

### 上板测量步骤（本次未执行）

1. **建立非带载基线。** 使用本次AXF/MAP，保存优化选项与时钟；先在可靠机械支撑/无运动条件下测量。不要靠带载断点判断实时性，暂停内核会影响既有超时与CAN反馈。
2. **DWT测完整任务响应。** 一次性设置 `CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk`、清CYCCNT并使能 `DWT_CTRL_CYCCNTENA_Msk`。在控制循环入口读取start，在调用 `vTaskDelayUntil` 前读取end，以无符号差累计last/max，另记录相邻入口差与超840000周期次数。结果放静态Watch计数，不在循环打印。DWT时间包含期间抢占，适合检查从释放到完成的墙钟响应；若要纯CPU执行时间，另用任务切换追踪分离。168MHz下CYCCNT约25.6秒回绕，单次间隔必须短于回绕周期。
3. **用GPIO验证时序。** 经原理图确认空闲引脚后，用BSRR在入口置位、结束清零。示波器/逻辑分析仪记录脉宽（含抢占）和相邻上升沿周期。单独测所有权事务及transport短锁：在已屏蔽中断后、恢复前取时间，避免将恢复后立刻执行的待处理中断误算为屏蔽时长；测量代码自身开销也要记录。
4. **覆盖最坏组合。** 分别测正常四轴控制、20ms历史采样、首次故障、post、rearm、JOG/CAN测试交接，以及遥测source=0/1/2/3、CAN RX突发、遥控DMA、USB及自瞄同时运行。记录max而非只看平均；检查最大完整响应是否小于5ms，并确认既有反馈年龄/接收队列丢弃计数无异常增长。保护/堵转场景可先用软件注入离线或测试输入，不以硬顶机械机构制造测试条件。
5. **栈和堆实测。** 从 `g_runtime_monitor_status.tasks[]` 读取任务名及 `stack_min_free_bytes`，核查lift、JOG、motor_tx、telemetry、link和Timer/Idle。启动任务会退出，需在初始化完成、退出前单独读取其高水位。观察 `heap_free_bytes/heap_min_free_bytes`；FPU上下文、ISR嵌套及MSP需另外评估，任务高水位不覆盖MSP。
6. **验证真实DMA生命周期。** 用逻辑分析仪连续解码USART1，确认36/116B帧及JustFloat尾标完整；切换源、制造受控DMA忙碌/延迟TC中断，核查busy计数增加且正在发送的数据保持不变。恢复错误后需同时确认DMA状态和UART状态再恢复构帧。本次主机HAL模型不能替代该测量。

本阶段交付的是修复、静态预算和可执行测量方法；没有填写虚构的WCET、IRQ延迟、栈高水位或抬升成功率。

## 9. 剩余风险与后续范围

- **待实机验证**：最坏控制响应、短PRIMASK区的实际时长、实际ISR/FPU嵌套、DMA错误恢复及物理CAN/电机行为。现有栈溢出hook为空也需后续单独处理，未借本次扩大到系统故障策略。
- **栈余量重点项**：link_task、启动任务和Idle在静态预留后余量偏小，已提示复核；未未经测量调整与抬升无关任务的栈配置。
- **接口使用边界**：共享Watch结构运行中应只读；任务用快照和rearm API；init/reset仅启动阶段。三种所有权身份仍对应各自单一任务，未来多实例需要代际句柄。UART增加第二生产者时必须统一所有权。
- **内存及历史**：仍有约58KiB黑匣子常驻；上电/重布防不足2秒、任务停摆、掉电时不保证完整2秒或post记录；无持久化。未来扩展需重新运行MAP/栈审计。
- **协议证据边界**：CAN返回成功仍只表示HAL接受请求，未新增邮箱完成/ACK/bus-off事件流；电机反馈不是逐命令ACK。VOFA float大整数精度限制和字符串指针跨固件解释限制沿用P0。
- **明确未改**：故障停机策略、支撑/保持能力、机械行程补偿、旧三模态控制和其他控制策略问题，保留到后续任务。
