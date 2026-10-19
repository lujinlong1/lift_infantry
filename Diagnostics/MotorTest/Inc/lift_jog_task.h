#ifndef LIFT_JOG_TASK_H
#define LIFT_JOG_TASK_H

#include "lift_control_task.h"
#include "lift_guard.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * 抬升台架测试模块（诊断用，不是控制方案）。
 *
 * ── 三个模式，各自一个结构体 ────────────────────────────────────────
 *
 *   LIFT_TEST_MODE_PER_AXIS   每轴独立闭环：四轴各自目标位置 +
 *                             **各自一套串级 PID**（位置环→速度环→电流，
 *                             结构体数组 position_pid[4] / speed_pid[4]）
 *   LIFT_TEST_MODE_LENGTH     四轴定长运行：一个 length_ecd 控制四根
 *                             正 = 伸出（把车顶起来），负 = 缩回
 *   LIFT_TEST_MODE_FORCE      四轴恒定力（**开环**）：标定"多少力能抬起/锁住车"
 *
 * ── 闭环控制链（2026-10-09 由单环改为串级）────────────────────────────
 *
 *   目标位置 ──斜坡──> 位置环 ──目标速度──> 速度环 ──电流──> 电流斜坡 ──> 限幅 ──> 电机
 *              ↑          ↑               ↑            ↑
 *         target_rate_ecd_s  position_pid  speed_pid  current_ramp_per_s
 *
 *   ① 目标斜坡：`target_rate_ecd_s`（计数/秒，两个闭环模式共用）
 *   ② 位置环  ：输入误差(ecd) → 输出目标速度(ecd/s)，output_limit = 最高速度
 *   ③ 速度环  ：输入误差(ecd/s) → 输出电流，output_limit = 最大电流
 *   ④ 电流斜坡：`current_ramp_per_s`（电流/秒），防"一瞬间到 16000"
 *   ⑤ 摩擦前馈：`feedforward_current`，按位置误差方向叠加
 *   ⑥ 硬限幅  ：`LIFT_TEST_MAX_CURRENT`（16000）
 *
 * ── 为什么闭环模式不需要找零 ────────────────────────────────────────
 *
 * 三个模式的量都是【相对量】：
 *   - LENGTH 的 length_ecd 是"从当前位置走多远"
 *   - PER_AXIS 的 target_ecd[ch] 是"相对使能那一刻的偏移"
 * 所以置 enabled 时先把四轴当前位置记成起点，之后一切都相对它算 ——
 * **不需要零点、不需要找零**。这对台架调试正好。
 *
 * ── 方向语义（全模块统一，靠 axis_direction 折算）────────────────────
 *
 *   逻辑正 = 伸出 = 把车顶起来
 *   逻辑负 = 缩回 = 车落下（找零方向）
 * 四根轴共用这一套语义，所以"正数伸出"对四根都成立。
 *
 * ── 安全 ────────────────────────────────────────────────────────────
 *
 * - 本机构不自锁：`enabled = 0` 就是断力，车会落回去。手别在下面。
 * - `g_lift_test.abort` 是**一键全停**：置 1 立刻停止所有模式并零电流。
 * - 闭环模式的 PID 输出会被 `LIFT_TEST_MAX_CURRENT` 硬钳位。
 */

/* ============================ 模式选择 ============================ */

typedef enum
{
    LIFT_TEST_MODE_OFF = 0,       /* 不输出（读数照样刷新） */
    LIFT_TEST_MODE_PER_AXIS,      /* 每轴独立闭环 */
    LIFT_TEST_MODE_LENGTH,        /* 四轴定长运行 */
    LIFT_TEST_MODE_FORCE          /* 四轴恒定力（开环） */
} lift_test_mode_t;

/*
 * 一条 PID 的参数。per-axis 时按轴放进**结构体数组**（pid[4]），
 * 于是每根轴可以单独调、单独限幅。
 */
typedef struct
{
    volatile float kp;              /* 比例 */
    volatile float ki;              /* 积分 */
    volatile float kd;              /* 微分 */
    volatile float integral_limit;  /* 原始误差积分限幅；0 会禁用积分，I项限幅=ki×该值 */
    volatile float output_limit;    /* 位置环：速度 ecd/s；速度环：电流 */
} lift_test_pid_t;

/* ==================== 模式 1：每轴独立闭环 ==================== */
typedef struct
{
    volatile bool     enabled;
    /*
     * 四轴各自的**相对目标**（逻辑坐标，正 = 伸出，单位：编码器计数）。
     * 它是"相对使能那一刻位置"的偏移，所以 0 = 原地不动。
     */
    volatile int32_t  target_ecd[LIFT_CHANNEL_COUNT];
    /* 目标斜坡见顶层的 target_rate_ecd_s（2026-10-10 合并，原来这里各存一份） */
} lift_test_per_axis_t;

/* ==================== 模式 2：四轴定长运行 ==================== */
typedef struct
{
    volatile bool     enabled;
    /*
     * 要走多远（逻辑坐标，编码器计数）：**正 = 伸出，负 = 缩回**。
     * 四根一起走同一个数，所以这是一条"标定运行长度"的指令。
     */
    volatile int32_t  length_ecd;
    /* 目标斜坡见顶层的 target_rate_ecd_s */
    /* 到位稳定时间：误差进入容差并保持这么久，就算走完（ms） */
    volatile uint16_t settle_ms;
    /* 到位容差 = 顶层的 brake_deadband_ecd（2026-10-10 合并，原来这里另有一个） */
    /* 走完最长允许时间（ms），超时就报超时并停止 */
    volatile uint32_t timeout_ms;
} lift_test_length_t;

/* ==================== 模式 3：四轴恒定力（开环） ==================== */
typedef struct
{
    volatile bool     enabled;
    /* 抬升力（N，四轴合计，正 = 抬起）。换算见 .c 里的系数说明 */
    volatile float    force_n;
    /* 抬升时长（ms）；到点后转入保持 */
    volatile uint32_t duration_ms;
    /* 抬完之后用来锁住车的力（N，四轴合计）；0 = 断力 */
    volatile float    hold_force_n;
    /* 每轴力缩放（默认 1.0）：负载不一样时用它配平 */
    volatile float    axis_scale[LIFT_CHANNEL_COUNT];
    /* 判"顶到头"：|转速| <= 该值并持续 stall_hold_ms 就结束抬升 */
    volatile int16_t  stall_speed_rpm;
    volatile uint32_t stall_hold_ms;
} lift_test_force_t;

/* ================== 顶层：三个模式装在一起，只有一个全局 ================== */
/*
 * 三个模式各自的结构体都【嵌在这里】，对外只暴露一个 g_lift_test。
 *
 * 为什么这么做：Watch 窗口里展开 g_lift_test 就能看到三个模式，
 * 要用哪个再展开哪个，不用在全局变量列表里翻找。
 */
typedef struct
{
    /* ---- 选择与总开关 ---- */
    volatile uint8_t mode;     /* lift_test_mode_t：当前用哪个模式 */
    volatile bool    abort;    /* 一键全停：置 1 立刻零电流并退出所有模式 */

    /*
     * ★★ 每轴一套【串级】PID：位置环（外）→ 速度环（内）→ 电流。
     *
     * 2026-10-09 用户指出"固定到指定位置应该是位置环+速度环" —— 对，已改。
     * 单环位置 PID 的问题：输出直接是电流，误差一大就瞬间顶到限幅，
     * 机构要么不动要么猛冲，中间没有"按速度走"的过程。
     *
     * ⚠️⚠️ 这两条环的【量纲完全不同】，填错就是飞车：
     *
     *   position_pid[ch]  输入 = 位置误差(ecd)   输出 = **目标速度**(ecd/s)
     *                     → 它的 output_limit 填【最高速度】，不是电流！
     *                     4000 ecd/s ÷ 725 计数/mm ≈ 5.5 mm/s（参考值）
     *
     *   speed_pid[ch]     输入 = 速度误差(ecd/s) 输出 = **电流**(C620 量纲)
     *                     → 它的 output_limit 填【最大电流】，默认 16000
     *
     * 这就是 yaw 的做法（`yaw_angle_pid.output_limit = max_speed_rpm`、
     * `yaw_speed_pid.output_limit = max_current`），照抄那个约定。
     *
     * ── 出厂默认（以 lift_jog_task_create() 为准）─────────────────
     *   位置 kp/ki/kd = 5/1/0；速度 kp/ki/kd = 2/0.5/0。
     *   两环原始积分限幅均为 4000，速度环 I 项最多贡献 2000 电流。
     *   即使 PID 增益清零，跟踪/保持前馈仍可能输出电流。
     *
     * ── ★ 改完立刻生效，不用重开 enabled ──────────────────────────
     *   增益**每周期映射**进 pid 控制器（只覆盖参数，保留积分/微分历史）；
     *   积分/微分历史只在 enabled 上升沿清一次。
     *   实现见 lift_jog_task.c 的 lift_test_pid_map_one() / lift_test_pid_load()。
     *
     * 两个闭环模式（PER_AXIS / LENGTH）**共用这两套**，不在各自结构体里重复：
     * 调 PID 是调"这根轴在这个机构上"的特性，与"这次走定长还是走各自目标"无关。
     */
    lift_test_pid_t position_pid[LIFT_CHANNEL_COUNT];
    lift_test_pid_t speed_pid[LIFT_CHANNEL_COUNT];

    /*
     * ★★ 电流斜坡（di/dt 限幅）：按轴，单位 **电流/秒**。<= 0 = 关掉斜坡。
     *
     * 为什么必需：位置环误差一大，速度环 P 项立刻算出上万电流，
     * 机构"一瞬间到 16000"（用户实测）。斜坡把电流按 电流/秒 慢慢放上去，
     * 冲击立刻消失。它是**最后一道、也是唯一作用在电流上的软处理**，
     * 在硬限幅（LIFT_TEST_MAX_CURRENT）之前生效。
     *
     * 参考 `remote_control_task.c` 的 remote_slew_rate_limit()（Pitch 用的同一套思路）。
     *
     * 建议起点：20000 电流/秒 → 5 ms 周期每周期走 100 → 0→16000 约 0.8 s。
     * 太小（<5000）会明显"肉"，太大（>100000）等于没加。
     * 斜坡状态**每次 enabled 上升沿清零**，不会残留上一轮的电流。
     */
    volatile int32_t current_ramp_per_s[LIFT_CHANNEL_COUNT];

    /*
     * ★ 目标斜坡速率（计数/秒 = ecd/s），**两个闭环模式共用**。
     *
     * 2026-10-10 从 `per_axis.max_speed_ecd_s` 与 `length.max_speed_ecd_s`
     * 合并上来：两处语义完全相同（"目标最多走多快"），各存一份只会让人
     * 改错地方。现在只有这一个。
     *
     * 默认 10000，照抄 pitch 的 `max_target_rate_deg_s`。**默认开着**：
     * 目标是阶跃时四层会同时饱和（位置环→速度环→电流→斜坡）→ 极限环。
     * 填 0 = 关掉斜坡（要纯阶跃响应调 PID 时才这么干）。
     */
    volatile int32_t target_rate_ecd_s;

    /*
     * ★ 摩擦力前馈已删除（2026-10-10）。
     *
     * 原来有一对 `feedforward_current[4]` + `feedforward_deadband`：
     * 按误差方向给恒定电流抵消静摩擦。删掉的理由是**语义重叠**：
     * `hold_feedforward_current` 现在**任何状态都加**（照 pitch 的 gravity_torque），
     * 已经承担了"克服静摩擦 + 托住车"这件事；再加一个按误差方向的偏置，
     * 两个前馈会互相打架，是典型的调参陷阱。
     * 而且它从上线起就一直是 0（没人用过）。
     */

    /*
     * ★★★ 减速 / 刹停 / 保持 三段（2026-10-09 用户实测后要求）
     *
     * 用户实测：1/2 两根轴在 **空载**、电流上限给到 16000 时会**大幅摆动**。
     * 所以不能一路满电流冲到目标，要"逐级开放 + 接近减速 + 到位刹停 + 停后托住"。
     *
     *   远端                approach_zone_ecd            brake_deadband_ecd
     *   │◄─── 满速满流 ───►│◄── 目标速度线性收窄 ──────►│◄─ 刹停/保持 ─►│ 目标
     *        (限流逐级放开)      5000 → 500              1000
     *
     * ⚠️⚠️ 收窄的是【目标速度】，**不是电流上限**。
     *     收窄电流上限会让机构在目标附近推不过"重力+静摩擦"而**停半路**
     *     （host 仿真实测：停在误差 -2501 处、电流正好贴收窄后的限幅）。
     *
     * 状态机（每轴独立，三态）：
     *   APPROACH  |误差| > brake_deadband
     *             → 正常串级；目标速度被压到 ≤ 按距离插值出的上限
     *   BRAKE     |误差| ≤ brake_deadband，但 |转速| > hold_speed_rpm
     *             → **速度指令强制为 0**，让速度环主动刹车（不是断力！）
     *   HOLD      |误差| ≤ brake_deadband 且 |转速| ≤ hold_speed_rpm
     *             → 只输出 hold_feedforward_current[ch]（托住车），PID 不再参与
     *             误差一旦超出 brake_deadband 就退回 APPROACH
     */

    /* 减速区起点（计数）：|误差| 小于它，目标速度开始线性收窄。0 = 不减速 */
    volatile int32_t approach_zone_ecd;

    /*
     * ★ 减速区的**最低目标速度**（ecd/s），默认 500。
     *
     * ⚠️ 为什么不能一路收到 0：收到 0 的话，机构在贴近死区边缘时目标速度
     * 只剩几十 ecd/s，而它要克服"重力 + 静摩擦"（≈6000 电流）才动得起来，
     * 结果就是**卡在死区外面进不去**（host 仿真实测：停在误差 -1092 处，
     * 峰峰值 93 计数，就是进不了 ±1000）。
     *
     * 留一个下限，最后一段用这个速度爬进去，进死区后立刻刹停。
     * 它同时决定刹车距离：从 v_min 刹到 0 大约走 v_min × 0.2 s 的计数。
     */
    volatile int32_t approach_min_speed_ecd_s;

    /* 刹停死区（计数）：最终目标已走完且误差进入死区后刹车/保持。
     * 到位还要求进入 HOLD；跟上斜坡中间目标不算到达最终位置。 */
    volatile int32_t brake_deadband_ecd;

    /*
     * ★ 逐级开放电流上限（用户实测要求："启动电流也得限制"）。
     *
     * 上限不是一上来就是 16000，而是从 current_limit_start 开始，
     * 按 current_limit_ramp_per_s 慢慢放开到 speed_pid.output_limit。
     * 每次 enabled 上升沿重新从 current_limit_start 开始。
     *
     *   默认 8000 → 20000 电流/秒 → 8000 涨到 16000 约 0.4 s
     * 空载大幅摆动就是"一上来就允许满电流"造成的。
     */
    volatile int32_t current_limit_start;
    volatile int32_t current_limit_ramp_per_s;

    /* HOLD 判据：|转速| ≤ 它才算"停住了"（rpm，转子侧） */
    volatile int32_t hold_speed_rpm;

    /*
     * ★ 速度反馈一阶低通时间常数（ms）。默认 20；0 = 关闭。
     *
     * 为什么需要它：C620 回的 speed_rpm 是**整数**，1 rpm = 8192/60 = 136.5 ecd/s。
     * 内环 kp = 120 时，**±1 rpm 的量化台阶就是 ±16380 电流 = 满量程**。
     * 位置环算出的目标速度本身也可能带台阶，于是内环一直在"追台阶"，
     * 表现为电机嗡嗡/波形毛刺。
     *
     * 滤波会把量化噪声按 1/(1+τ·2πf) 压下去，代价是相位滞后。
     * 建议：内环怎么调都毛刺就试 **10~20 ms**；此时才有余量去用 kd。
     * 太大（>50 ms）会让内环变慢、外环跟着变差。
     */
    volatile int32_t speed_filter_tau_ms;

    /*
     * ★ 刹停后的重力前馈（**有符号**电流，按轴）。
     *
     * 本机构不自锁：HOLD 状态如果只是"断力"，车会自己落下去。
     * 所以停住以后必须持续给一个电流把车托住 —— 这就是这一项。
     *
     * 方向：**正 = 伸出方向 = 把车顶起来**（托住车的那个方向）。
     *
     * 怎么测：用模式 3（FORCE 开环）慢慢加 hold_force_n，找到"刚好不下滑"的点，
     * 换算成**每轴电流** = hold_force_n / LIFT_TEST_FORCE_PER_CURRENT_N，
     * 填到四根上（该常量 2026-10-11 实测重标为 0.8160，见 .c 里的推导）。
     * 0 = 不补偿（台架空载可以先填 0）。
     *
     * ⚠️ 更稳的做法是**按电流记录**：直接读 `applied_current` 找门槛，
     *    标定完再回填系数。这样系数错了也不会把标定结果带偏。
     */
    volatile float   hold_feedforward_current[LIFT_CHANNEL_COUNT];

    /* ---- 模式 1：每轴独立闭环 ---- */
    lift_test_per_axis_t per_axis;

    /* ---- 模式 2：四轴定长运行 ---- */
    lift_test_length_t   length;

    /* ---- 模式 3：四轴恒定力（开环，标定用）---- */
    lift_test_force_t    force;
} lift_test_control_t;

extern lift_test_control_t g_lift_test;

/* 每轴的运行状态（看 axis_hold_state[4]） */
typedef enum
{
    LIFT_TEST_HOLD_APPROACH = 0,   /* 远离目标：正常串级 + 电流上限收窄 */
    LIFT_TEST_HOLD_BRAKE,          /* 进了死区但还在动：速度指令归零，主动刹车 */
    LIFT_TEST_HOLD_HOLD            /* 已停住：只输出 hold_feedforward_current */
} lift_test_hold_state_t;

/* ============================ 状态面 ============================ */
typedef struct
{
    /* ---- 通用 ---- */
    volatile bool     running;            /* 1 = 正在出力（控制任务靠它让路） */
    volatile uint8_t  active_mode;        /* 当前实际在跑哪个模式 */
    volatile uint32_t run_count;          /* 执行过几次；每次 enabled 上升沿 +1 */
    volatile uint32_t elapsed_ms;         /* 本次已运行时长 */
    volatile uint32_t remaining_ms;       /* 还剩多久（有时限的模式） */
    volatile bool     all_online;         /* 四轴是否全部在线 */

    /*
     * ★★ 掉线保护（2026-10-10 质检补的安全缺口）。
     *
     * 补之前：任务循环只判 abort / enabled / mode，**完全不看在线状态**。
     * 后果：四根里掉一根，另外三根照样满电流推 → 车歪、机构被扭。
     *
     * 现在的行为：
     *   - 四轴没全在线 → **拒绝启动**（enabled 置 1 也不动）
     *   - 运行中掉线   → **立刻零电流退出**
     *   - 两种情况都置 offline_abort = 1，并在 Watch 里看得见
     *   - 自动清对应 enabled；offline_abort 保留到下次成功启动
     */
    volatile bool     offline_abort;      /* 1 = 因为某轴掉线被拒绝启动/中止 */

    /* ---- 四轴实时反馈（任何模式都刷新）---- */
    volatile int32_t  axis_position_ecd[4];   /* 原始累计位置（转子侧计数） */
    volatile int32_t  axis_offset_ecd[4];     /* 相对起点的偏移（逻辑坐标，正=伸出） */
    volatile int16_t  axis_speed_rpm[4];      /* 转子侧转速（电调原始值） */
    volatile int32_t  axis_speed_ecd_s[4];    /* ★逻辑速度(ecd/s)：速度环的反馈，正=伸出方向 */
    volatile int16_t  axis_given_current[4];  /* 电调回显的电流：命令到了没有 */
    volatile int16_t  axis_output_current[4]; /* 我们下发的原始电流 */
    volatile uint8_t  axis_temperature[4];
    volatile float    axis_position_pid_output[4]; /* ★位置 PID 限幅后的输出，速度 ecd/s */
    volatile float    axis_speed_pid_output[4];    /* ★速度 PID 限幅后的输出，尚未加保持前馈 */
    volatile int16_t  axis_ramped_current[4];      /* ★动态限流与斜坡后的逻辑电流 */
    volatile int16_t  axis_limit_now[4];           /* ★本周期逐级开放的电流上限；接近减速只限速度 */
    volatile uint8_t  axis_hold_state[4];          /* ★0=APPROACH 1=BRAKE 2=HOLD，见 lift_test_hold_state_t */
    volatile bool     axis_online[4];

    /* ---- 闭环模式（PER_AXIS / LENGTH）---- */
    volatile int32_t  axis_target_offset[4];  /* 逻辑目标偏移（已过目标斜坡） */
    volatile int32_t  axis_target_speed_ecd_s[4]; /* ★位置环给出的目标速度（速度环的给定） */
    volatile int32_t  axis_error[4];          /* 目标 − 实际（逻辑，位置） */
    volatile bool     axis_at_target[4];      /* 最终目标已收敛、位置进容差且进入 HOLD */
    volatile bool     at_target;              /* 四轴全部到位 */
    volatile bool     timed_out;              /* LENGTH：超时结束 */

    /* ---- FORCE 模式 ---- */
    volatile bool     active;                 /* 1 = 抬升段，0 = 已转保持 */
    volatile float    applied_force_n;        /* 实际下发的抬升力（四轴合计） */
    volatile float    applied_hold_force_n;   /* 实际保持力 */
    volatile int16_t  applied_current;        /* 折算出的每轴电流 */
    volatile bool     stall_latched;          /* 1 = 被"顶到头"结束 */
    volatile uint32_t stall_timer_ms;         /* 已连续不动多久 */

    /* ---- 编码器诊断（上电累加，不会自动清零）---- */
    volatile uint32_t axis_wrap_count[4];         /* 跨 0/8191 边界次数（正常事件） */
    volatile uint32_t axis_wrap_max_raw_delta[4]; /* 跨圈时最大原始差（≈8192 正常） */
    volatile uint32_t axis_gap_count[4];          /* 帧间隔 > 20ms 的次数（预警） */
    volatile uint32_t axis_gap_max_ms[4];         /* 最大帧间隔 */
    volatile uint32_t axis_feedback_count[4];     /* 收到反馈帧数：一直涨 = 通信正常 */

    /*
     * ---- 命令通道所有权诊断（2026-10-11 补）------------------------------
     *
     * 为什么要有这几个：台架"闭环在跑、目标也在、电流却是 0"这个现象，
     * 根因可能是"抢不到 CAN 电流通道"，而这件事**原来在 Watch 里完全看不见** ——
     * 只能靠猜是 PID 问题还是通道问题，白折腾。现在一眼就能分辨。
     */
    /* 本周期是否因为拿不到通道而没出力（1 = 没出力） */
    volatile bool     own_lost;
    /* 累计"没抢到通道"的次数。一直在涨 = 有人在持续占着通道 */
    volatile uint32_t own_lost_count;
    volatile uint32_t own_lost_tick_ms; /* diagnostic event time, not a control cycle */
    /* 累计"从主链手里接管通道"的次数。正常应该只涨几次（每次使能一两次） */
    volatile uint32_t own_preempt_count;
    /* 通道现在在谁手里（lift_owner_t：0=NONE 1=CONTROL 2=JOG 3=CAN_TEST） */
    volatile uint8_t  own_holder;
} lift_jog_status_t;

extern lift_jog_status_t g_lift_jog_status;

/* 结束原因；Watch 中 reason 的数值与此枚举一致。 */
typedef enum
{
    LIFT_TEST_END_NONE = 0,
    LIFT_TEST_END_COMPLETE = 1,
    LIFT_TEST_END_TIMEOUT = 2,
    LIFT_TEST_END_DISABLED = 3,
    LIFT_TEST_END_ABORT = 4,
    LIFT_TEST_END_OFFLINE = 5,
    LIFT_TEST_END_MODE_CHANGED = 6,
    LIFT_TEST_END_MODE_OFF = 7,
    LIFT_TEST_END_STALL = 8,
    LIFT_TEST_END_FAULT_INHIBIT = 9
} lift_test_end_reason_t;

/*
 * 最近一次结束/离线拒绝启动的 RAM 快照，清电流和 enabled 之前保存。
 * valid 最后置 1；sequence 每次保存 +1。实时采样和下一次启动不覆盖它，
 * 下一次结束覆盖；复位丢失。status.running 表示停止前状态，不是当前运行状态。
 * 实际结束时自动清本模式 enabled；PER_AXIS 到位和 FORCE 非零保持不算结束。
 */
typedef struct
{
    volatile bool valid;
    volatile uint32_t sequence;
    volatile uint8_t reason;
    volatile uint8_t mode;
    volatile uint32_t end_tick;
    lift_jog_status_t status;
    lift_test_control_t parameters;
} lift_test_snapshot_t;

extern lift_test_snapshot_t g_lift_test_snapshot;

/* 创建静态内存形式的台架测试任务。 */
bool lift_jog_task_create(void);

#endif
