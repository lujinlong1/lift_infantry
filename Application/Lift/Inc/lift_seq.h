#ifndef LIFT_SEQ_H
#define LIFT_SEQ_H

/*
 * 分阶段上台阶流程状态机（对应议题 LIFT-011）。
 *
 * ── 为什么重写 ─────────────────────────────────────────────────────────
 * 上一版 `lift_seq` 是一个通用的"动作表执行器"（MOVE / WAIT / STOP），
 * 动作表是一组数据常量。它有两个问题，而且是致命的：
 *
 *   1. **零调用**。全固件没有任何地方调 `lift_seq_update()`，
 *      "一键上台阶"从来没有进入过运行路径（只有主机测试在调它）。
 *   2. **判断太弱**。它只看"到位标志 + 延时"，而到位标志只看位置误差；
 *      没有前置检查、没有支撑确认、没有同步/电流/限位判据、没有故障策略。
 *
 * 本模块改成一台**显式状态机**，覆盖任务要求的 15 个阶段，每一阶段的推进
 * 条件都由【可观测量】决定，而不是"定时到了就算完成"。
 *
 * ── 量纲约定（这是本模块唯一需要小心的地方）──────────────────────────
 *
 *   输入输出【全部在毫米域】，正 = 抬升（齿条伸出把车顶起）：
 *     - `in->position_mm[ch]`、`in->speed_mm_s[ch]` 每轴独立换算后的实际值；
 *     - `status->axis_target_mm[ch]` 每轴目标；
 *     - 调用方用 `lift_kinematics` 把毫米折成计数，**每轴独立零点与系数**。
 *
 *   为什么模块自己不拿 `lift_kin_config_t`：那样它就要复制一份标定参数，
 *   而"同一个参数在三个文件里各有一份"正是本项目反复出错的根源
 *   （见 lift_control_task.c 里关于 4 倍陷阱的记录）。调用方是唯一持有
 *   标定表的地方，换算也应该在那里做。
 *
 * ── ⚠️ 一个必须先人工确认的机械问题（本模块不替你做主）──────────────
 *
 * 底盘前/后轮组的定义在现有资料里【互相矛盾】：
 *   - HANDOFF 10.23：抬升 3/4（ch2/ch3）连麦轮，抬升 1/2（ch0/ch1）连小辅助轮
 *   - 调试手册 / 早期描述：前部小辅助轮、后部麦轮
 * 也就是"远端 (ch2,ch3) 到底是车头还是车尾"没有定论。
 *
 * 这件事决定"先收哪一组"，收错组 = 把还在承担重量的支撑收掉 = 翻车。
 * 所以本模块【不猜】：
 *   - `front_mask` / `rear_mask` 是显式配置项，默认按 CAN 通道顺序
 *     （ch0,ch1 = 前）。`layout_confirmed` 为 false 时 PRECHECK 报
 *     `UNSUPPORTED` 并【拒绝启动】。
 *   - 确认方法（2 分钟，架空或垫住车做）：单轴给一点抬升力，看是车头还是
 *     车尾在动。记下哪两个通道是车头，填 `front_mask`，把 `layout_confirmed` 置 1。
 *
 * ── ⚠️ 能力缺口：没有接触/负载/姿态传感器 ────────────────────────────
 *
 * 这台车【没有】台阶接触开关、没有轮组负载传感器、没有 IMU 参与抬升。
 * 所以"前轮组已可靠接触台阶并具备支撑条件"这件事**无法自动判定**。
 * 本模块把缺口显式化，而不是用一个假判据糊过去：
 *
 *   - 每个 VERIFY_*_SUPPORT 阶段必须拿到【人工确认脉冲】(confirm_request)；
 *   - 另外必须满足一条【电气必要条件】：本组至少有一根轴的电流达到
 *     `load_current_min`（四轴电流全为 0 不可能"撑住了"）；
 *   - 必要条件不满足 → 立即 `UNSUPPORTED` 故障，不往下走；
 *   - 条件满足但没人确认 → 停在原地等，直到该阶段超时。
 *
 * 也就是说："定时到了"永远不会被当成"轮子已经可靠支撑"。
 *
 * ⚠️ `load_current_min` 默认是 **0 = 该判据关闭**，因为它的正确取值必须由
 *    实测的保持电流（P3 标定表的 `gravity_feedforward_current`）来定。
 *    标定完成后把它设成"每轴保持电流 × 0.5"左右，这条判据才开始有意义。
 *    在那之前，支撑确认【完全依赖人工】——这一点必须让操作手知道。
 *
 * 如果以后装了接触开关，把它接到 `contact_front` / `contact_rear` 即可：
 * 判据会自动变成"人工确认 或 接触信号"。
 *
 * ── 状态流程 ───────────────────────────────────────────────────────────
 *
 *   IDLE ──start──► PRECHECK ──ok──► LIFT_CLEARANCE ──稳定──► VERIFY_CLEARANCE
 *                                                                   │
 *                              ┌────────────────────────────────────┘
 *                              ▼
 *                        APPROACH_STEP ──► VERIFY_FRONT_SUPPORT ──确认──►
 *                        SEEK_FRONT_CONTACT ──确认/接触──► RETRACT_FRONT ──►
 *                        VERIFY_FRONT_RETRACT ──►
 *                        ADVANCE_MIDDLE ──► VERIFY_REAR_SUPPORT ──确认──►
 *                        RETRACT_REAR ──► VERIFY_REAR_RETRACT ──►
 *                        FINAL_ADVANCE ──► COMPLETE
 *
 *   任何阶段：判据不满足 / 超时 / 中止 / 上游故障 → FAULT（锁存，须 reset）
 *
 * ── 本模块不做的事 ─────────────────────────────────────────────────────
 *   - 不直接驱动电机。它只产出"四轴目标高度"，由调用方（lift_control_task）
 *     交给既有的 lift_top → lift_control → lift_sync → lift_guard 链路。
 *   - 不做底盘运动。上台阶时底盘该往前走多少由人推摇杆或由底盘模块决定；
 *     本模块只通过 `status->drive_request` 声明"现在该往前走了"。
 *   - 不做故障后的电流决策。那是 lift_safety 的 `lift_stop_action_t`。
 *
 * 设计约束：纯逻辑，时间由 now_ms 注入，不依赖 RTOS/HAL，不做动态分配。
 */

#include "lift_safety.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    LIFT_SEQ_STATE_IDLE = 0,          /* 等待启动 */
    LIFT_SEQ_STATE_PRECHECK,          /* 电机在线/零点/标定/限位/同步 前置检查 */
    LIFT_SEQ_STATE_LIFT_CLEARANCE,    /* 四轴抬到标定过的安全高度（200 mm） */
    LIFT_SEQ_STATE_VERIFY_CLEARANCE,  /* 确认高度/同步/姿态/电流都满足 */
    LIFT_SEQ_STATE_APPROACH_STEP,     /* 低速靠近台阶（人/底盘执行前进） */
    LIFT_SEQ_STATE_VERIFY_FRONT_SUPPORT,  /* 确认前部轮组已可靠支撑 */
    LIFT_SEQ_STATE_SEEK_FRONT_CONTACT,/* 前部降到"前轮刚好压上台阶"的接触点 */
    LIFT_SEQ_STATE_RETRACT_FRONT,     /* 载荷转移确认后，前部继续收到收回位 */
    LIFT_SEQ_STATE_VERIFY_FRONT_RETRACT,
    LIFT_SEQ_STATE_ADVANCE_MIDDLE,    /* 底盘继续前进 */
    LIFT_SEQ_STATE_VERIFY_REAR_SUPPORT,   /* 确认后部轮组已可靠支撑 */
    LIFT_SEQ_STATE_RETRACT_REAR,      /* 收回后部升降机构 */
    LIFT_SEQ_STATE_VERIFY_REAR_RETRACT,
    LIFT_SEQ_STATE_FINAL_ADVANCE,     /* 完成剩余前进动作 */
    LIFT_SEQ_STATE_COMPLETE,          /* 完成一次上台阶 */
    LIFT_SEQ_STATE_FAULT,             /* 故障锁存，等人工复位 */
    LIFT_SEQ_STATE_TEST_HOLD,         /* 16: lift-only test, hold target */
    LIFT_SEQ_STATE_TEST_PAUSED,       /* 17: hold measured encoder positions */
    LIFT_SEQ_STATE_TEST_LOWER,        /* 18: controlled return to retracted_mm */
    LIFT_SEQ_STATE_TEST_PARKED,       /* 19: low position, still powered */
    LIFT_SEQ_STATE_COUNT
} lift_seq_state_t;

typedef struct
{
    /* 车头一侧的通道掩码（bit ch）。见文件头"必须先人工确认的机械问题"。 */
    uint8_t front_mask;
    /* 车尾一侧的通道掩码。必须与 front_mask 互补，否则配置非法。 */
    uint8_t rear_mask;

    /* 底盘要抬升的净高度（mm，单轴逻辑量）。任务目标 200 mm。 */
    int32_t clearance_mm;
    /*
     * 前部「找接触」的目标高度（mm）。
     *
     * ★ 为什么 RETRACT_FRONT 要分两段：收回前部不是"一路降到底"，
     *   而是【先降到前轮刚好压上台阶，再继续降到收回位】。
     *
     *   两段之间那一下停在接触点，正是整车载荷从"后部两根齿条独自承受
     *   全部重量"转移到"前轮 + 后部齿条分担"的时刻。跳过它直接降到底，
     *   在真实机构上表现为前部突然塌下去（姿态误差瞬间超限 → SYNC 故障），
     *   而在仿真里则会被误判成"前轮没压住"。
     *
     *   接触的确认条件仍然是【人工确认 / 接触开关】：这台车没有接触传感器，
     *   所以"降到接触点"只能由人看着说"到了"。到了之后再继续降，
     *   此时若前轮其实没压住（`front_unsupported`），姿态会变 → SYNC 故障。
     *
     * <= 0 表示【不找接触】：确认之后直接一路降到 retracted_mm。
     * 用于还没有台阶可上的台架测试。
     */
    int32_t contact_mm;
    /* 收回后停在的高度（mm）：留一点余量，不要贴死点。 */
    int32_t retracted_mm;
    /* 允许的最大四轴高度差（mm，极差）。超过 → SYNC 故障。 */
    int32_t max_sync_error_mm;
    /* 允许的最大单轴偏离平均高度（mm）。超过 → SYNC 故障。 */
    int32_t max_deviation_mm;
    /* 单轴高度到位容差（mm）。 */
    int32_t position_tolerance_mm;
    /* 判"停住了"的单轴速度阈值（mm/s）。 */
    int32_t settled_speed_mm_s;
    /* 判"电流异常"的单轴电流上限（C620 量纲）。 */
    int32_t max_current;
    /* SUPPORT 阶段"电流确实在承担负载"的下限；0 = 关闭该判据（见文件头）。 */
    int32_t load_current_min;
    /* 判"这根轴卡住了"的速度阈值与持续时间。 */
    int32_t stall_speed_mm_s;
    uint32_t stall_ms;
    /* 各阶段超时（ms）。0 = 不限时（只允许出现在终态）。 */
    uint32_t precheck_timeout_ms;
    uint32_t lift_timeout_ms;
    uint32_t verify_timeout_ms;
    uint32_t advance_timeout_ms;
    uint32_t retract_timeout_ms;
    uint32_t support_timeout_ms;
    bool lift_test_only; /* No chassis advance or waiting timeout. */
} lift_seq_config_t;

typedef struct
{
    /* ---- 命令面（全部是单周期脉冲）---- */
    bool start_request;
    bool abort_request;         /* 中止：进入 FAULT，按停机策略处理 */
    bool confirm_request;       /* 人工确认：支撑条件 / 前进动作已完成 */
    bool release_request;       /* 人力解除：已确认机械支撑，允许 FAULT 直接断电 */
    bool reset_request;         /* 复位故障 */
    bool test_pause_request;
    bool test_resume_request;
    bool test_lower_request;

    /* ---- 上游状态（来自 lift_top / lift_control_status）---- */
    bool homed;                 /* 四轴都有可信零点 */
    bool calibrated;            /* 运动学标定完成（未标定不得走绝对位置） */
    bool layout_confirmed;      /* 前后轮组布局已人工确认 */
    bool axis_online[LIFT_CHANNEL_COUNT];
    /*
     * 每轴软限位（已找零逻辑【计数】）。前置检查用它判"现在是不是已经在界外"。
     * min >= max 表示该轴没配限位。
     */
    int32_t limit_min_ecd[LIFT_CHANNEL_COUNT];
    int32_t limit_max_ecd[LIFT_CHANNEL_COUNT];
    /* Startup-only allowance below the zero datum at min=0. The task supplies
     * 1000 encoder counts for all four axes, independent of counts/mm.
     * Normal command and output limits remain unchanged. */
    int32_t contact_zero_recovery_ecd[LIFT_CHANNEL_COUNT];

    /* ---- 反馈（逻辑坐标，正 = 抬升；长度一律毫米）---- */
    int32_t position_mm[LIFT_CHANNEL_COUNT];
    int32_t speed_mm_s[LIFT_CHANNEL_COUNT];
    int32_t current[LIFT_CHANNEL_COUNT];     /* 本周期实际下发电流 */
    /*
     * 每轴【已找零逻辑计数】。前置检查用它判断"现在是不是已经在软限位外" ——
     * 限位本身是按计数配的（与 lift_guard / lift_control 同一套边界），
     * 所以这一项必须与 position_mm 一起给，不能互相推算。
     */
    int32_t position_ecd[LIFT_CHANNEL_COUNT];
    int32_t sync_error_mm;                   /* 四轴高度极差 */
    int32_t deviation_max_mm;                /* max |每轴 − 其余轴参考| */
    bool tracking_deviation_valid;
    int32_t tracking_deviation_mm; /* reference-relative, for unequal group targets */

    /* ---- 上游故障标志 ---- */
    bool guard_fault;
    bool limit_blocked[LIFT_CHANNEL_COUNT];
    bool drive_done;            /* 底盘/操作手声明"这一前进段已到位" */

    /* 可选接触信号（没有传感器时恒 false）。 */
    bool contact_front;
    bool contact_rear;

    uint32_t now_ms;
} lift_seq_input_t;

typedef struct
{
    lift_seq_state_t state;
    lift_safety_fault_t fault;
    /* 故障停机策略（由 lift_safety_policy_for() 决定）。 */
    lift_stop_action_t stop_action;
    /* 是否已收到人工解除（机械支撑已确认）。 */
    bool released;
    /* 当前阶段的支撑前提（停机策略的输入，发布出来便于判读）。 */
    lift_support_t support;

    /* ---- 给控制链的命令 ---- */
    bool    command_valid;      /* 真 = 调用方须让 lift_top 认为命令新鲜 */
    bool    command_enabled;    /* 真 = 允许出力 */
    /*
     * 每轴目标高度（mm，逻辑量）。调用方负责用 lift_kinematics 折成
     * 【每轴独立的】编码器计数，再经 lift_sync 正变换折成模态目标。
     */
    int32_t axis_target_mm[LIFT_CHANNEL_COUNT];
    bool command_ecd_valid; /* Exact measured pose for TEST_PAUSED. */
    bool command_zero_output; /* Latched low-position cut, not high-position pause. */
    int32_t axis_target_ecd[LIFT_CHANNEL_COUNT];
    /*
     * FAULT 且停机策略为 HOLD 时，控制链应把四轴停在【当前实际位置】。
     * 这个值给出"当前四轴平均高度"的毫米表示，供控制链生成保持目标。
     */
    int32_t hold_mm;

    /* ---- 可观测进度 ---- */
    uint32_t step_index;        /* 0..14，与 lift_seq_state_t 一一对应 */
    uint32_t step_elapsed_ms;
    uint32_t elapsed_ms;        /* 整轮流程耗时 */
    bool     waiting_confirm;   /* 本阶段是否在等人工确认（Watch 最该看的一项） */
    /*
     * 本阶段要求底盘往前开。
     *
     * ★ 抬升模块【没有权限动底盘】，也不应该动（那会绕过底盘自己的电流
     *   限制与跟随逻辑）。所以这里只声明"现在该往前走了"，由人或底盘模块
     *   执行，执行完通过 `drive_done` 或人工确认回报。
     */
    bool     drive_request;
    /* 需要人做什么（静态字符串，不分配）。 */
    const char *operator_action;
    /* FAULT 的成因与相关轴，方便现场定位。 */
    const char *fault_detail;
    int32_t  fault_axis;        /* -1 = 与单轴无关 */
    uint32_t confirm_count;     /* 累计收到的确认次数 */
} lift_seq_status_t;

typedef struct
{
    lift_seq_config_t config;

    lift_seq_state_t state;
    lift_safety_fault_t fault;
    lift_stop_action_t stop_action;
    bool released;

    lift_seq_state_t test_resume_state;
    int32_t test_pause_ecd[LIFT_CHANNEL_COUNT];
    int32_t test_stall_axis; /* -1 manual pause/no stalled axis */

    uint32_t step_start_ms;
    uint32_t run_start_ms;
    uint32_t settle_start_ms;
    bool     settle_active;
    /*
     * ★ 人工确认的【定向挂起】。
     *
     * 为什么必须有它：确认是单周期脉冲，而状态转换也发生在同一周期。
     * 操作手是看着当前画面按的，所以这一次按键的含义由【按下时的阶段】
     * 决定；但状态机可能在同一个周期里已经换阶段了，于是：
     *   - 要么这次按键被转换吃掉（新阶段永远等不到确认，看起来像卡住）
     *   - 要么更糟：它被下一个毫不相干的阶段消费掉
     *     （实测：进 VERIFY_REAR_SUPPORT 的瞬间就自动过了，
     *       用的还是操作手在前一步按下的那一次确认）
     *
     * 所以挂起必须是【定向】的：记下"这一次确认是给哪个阶段的"，
     * 只有那个阶段能消费它；进入任何其他阶段时挂起自动作废。
     *
     * 代价（刻意接受）：每个需要确认的阶段都要单独按一次。
     * 支撑确认是人命关天的一步，宁可多按一次，也不要一次按键被解释成两件事。
     */
    lift_seq_state_t confirm_for;

    /*
     * 本周期若收到确认上升沿，它应当被"定向"给哪个阶段。
     *
     * 每周期在处理输入之前设为 IDLE，然后由【发起转换的那个阶段】在调用
     * seq_enter 之前设成目标阶段；seq_enter 会把它写进 confirm_for。
     *
     * 设计意图：**一次按键正好确认一件事，而且是有名字的那一件**。
     * 不设它的话，要么按键被转换吃掉（阶段卡住），要么被下一个阶段
     * 顺手消费（两件事被一次按键确认）。
     */
    lift_seq_state_t pending_confirm_target;
    uint32_t stall_start_ms[LIFT_CHANNEL_COUNT];
    bool     stall_active[LIFT_CHANNEL_COUNT];

    /* 边沿检测 */
    bool start_last;
    bool abort_last;
    bool confirm_last;
    bool release_last;
    bool reset_last;

    /* 本阶段的目标（每轴 mm）。 */
    int32_t axis_target_mm[LIFT_CHANNEL_COUNT];
    bool    targets_valid;

    /* 故障明细（静态字符串 + 轴号；不做动态分配、不存指针到栈上） */
    const char *fault_detail;
    int32_t fault_axis;

    uint32_t confirm_count;
} lift_seq_t;

/*
 * 初始化。config 非法时仍然初始化，但 PRECHECK 会拒绝启动 ——
 * 宁可"起不来"，也不要"按错的地图动起来"。
 */
void lift_seq_init(lift_seq_t *self, const lift_seq_config_t *config);

/* 推进一个控制周期。任一指针为 NULL 时直接返回、不修改状态。 */
void lift_seq_update(
    lift_seq_t *self,
    const lift_seq_input_t *in,
    lift_seq_status_t *status);

/* 清故障并回到 IDLE（调用方在确认安全后调用）。 */
void lift_seq_reset(lift_seq_t *self);

/* 流程是否正在控制抬升（调用方据此决定命令来源）。 */
bool lift_seq_is_active(const lift_seq_t *self);

/* 配置是否自洽（前后掩码互补且非空、阈值有序、超时非零）。 */
bool lift_seq_config_is_valid(const lift_seq_config_t *config);

/* 状态名（Watch / 日志用，静态字符串，不分配）。 */
const char *lift_seq_state_name(lift_seq_state_t state);

/* 填一份保守的出厂默认配置：前后按 CAN 通道顺序，200 mm 目标，未确认布局。 */
void lift_seq_config_default(lift_seq_config_t *config);

/* 默认布局是否已被人工确认（编译期开关，见文件头说明）。 */
#ifndef LIFT_SEQ_LAYOUT_CONFIRMED
#define LIFT_SEQ_LAYOUT_CONFIRMED 0
#endif

#endif
