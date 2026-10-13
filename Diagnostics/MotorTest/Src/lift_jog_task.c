#include "lift_jog_task.h"

#include "cmsis_os2.h"
#include "dji_motor.h"
#include "FreeRTOS.h"
#include "lift_safety.h"
#include "pid.h"
#include "robot_motor_commands.h"
#include "robot_motors.h"
#include "task.h"

#include <stddef.h>
#include <string.h>

#define LIFT_TEST_TASK_STACK_WORDS    768U
#define LIFT_TEST_PERIOD_MS           5U
#define LIFT_TEST_ONLINE_TIMEOUT_MS   100U
#define LIFT_TEST_DEFAULT_TOLERANCE   200
#define LIFT_TEST_DEFAULT_SETTLE_MS   200U
#define LIFT_TEST_DEFAULT_TIMEOUT_MS  10000U
#define LIFT_TEST_DEFAULT_LIFT_MS     2000U
#define LIFT_TEST_DEFAULT_STALL_RPM   20
#define LIFT_TEST_DEFAULT_STALL_MS    1000U

/* 闭环模式输出电流的硬上限（最后一道闸）。 */
#define LIFT_TEST_MAX_CURRENT         16000

/* 电流斜坡的每周期步长：满量程的 1% ≈ 160 计数/周期 ≈ 32000 计数/秒 */
#define LIFT_TEST_RAMP_STEP           (LIFT_TEST_MAX_CURRENT / 100)

/* 一圈的编码器计数（转子侧），用于 rpm → ecd/s 换算（速度环的反馈量纲）。 */
#define LIFT_TEST_ECD_PER_REV         8192

/*
 * 目标斜坡的默认速率（ecd/s）。**默认开着**，照抄 pitch 的 max_target_rate_deg_s。
 *
 * ⚠️ 为什么默认非 0：目标是【阶跃】时，位置环输出瞬间饱和 → 速度环饱和 →
 * 电流饱和 → 电流斜坡也饱和，四层同时饱和就是极限环（用户实测的大幅摆动）。
 * 有斜坡之后两条环始终工作在线性区。
 * 2000 ecd/s ≈ 2.76 mm/s；再配合下面的"跟踪前馈"，斜坡期间位置误差几乎为 0。
 * 想要阶跃响应（纯调 PID 用）就把它填 0。
 */
#define LIFT_TEST_DEFAULT_TARGET_RATE_ECD_S  2000

/*
 * 力 ↔ 电流 的换算系数（N 每 电流单位）。
 *   四轴合计 vs 一根轴相差 4 倍，所以分两个常量，从命名上杜绝混用。
 *
 * ★ 2026-10-11 实测重标：0.5705 → 0.8160。
 *
 * 推导（与机构扭矩链同构）：
 *   力 ∝ 电流 · 减速比 · 齿轮半径 · 效率 · 2π
 *   驱动齿轮分度圆是实测反推的（46.693 mm），与旧值同一个齿轮，
 *   所以**只有减速比变了**：
 *
 *       0.5705 × (19.2 ÷ 13.43) = 0.8156 ≈ 0.8160
 *
 *   旧值 0.5705 建立在图纸的 19.2:1 上；实测减速比是 13.43（齿轮一圈
 *   = 110000 计数），于是旧值**偏小约 43%** —— 用它标定会把"需要多少电流"
 *   低估四成，抬不起来还以为是增益问题。
 *
 * ⚠️ 效率（旧推导取 0.7）仍是估算，所以这个新值也还是**估算**。
 *    要变成实测值，用弹簧秤给恒定电流拉齿条、反推 N/电流 即可 ——
 *    只改本常量，不动任何控制逻辑。
 *
 * ⚠️ 它对标定的直接影响：
 *       hold_feedforward_current = hold_force_n ÷ 本常量
 *    本常量错了，标出来的保持电流就错。G3 请**按电流记录**，
 *    标定完再回填系数，不要反过来。
 */
#define LIFT_TEST_FORCE_PER_CURRENT_N      0.8160F

/*
 * 每轴力缩放的有效范围（LIFT_TEST_SCALE_MAX 是上限）。
 *   0 或负数 / 超上限 / NaN  → 【关掉这根轴】（输出 0），见 lift_test_clamp_scale()。
 * 2026-10-11 修正：以前非法值一律回落 1.0 = 满力，于是"填 0 想关一根轴"
 * 得到的是满力。LIFT_TEST_SCALE_MIN 不再作为下限使用，只留作参考下限值。
 */
#define LIFT_TEST_SCALE_MIN                0.05F
#define LIFT_TEST_SCALE_MAX                4.0F

lift_test_control_t g_lift_test;
lift_jog_status_t   g_lift_jog_status;
lift_test_snapshot_t g_lift_test_snapshot;

static osThreadId_t  lift_test_task_handle;
static StaticTask_t  lift_test_task_control;
static StackType_t   lift_test_task_stack[LIFT_TEST_TASK_STACK_WORDS];

/* ---- 内部状态 ---- */
static bool     test_running;                         /* 模式是否已开始 */
static uint8_t  test_active_mode;
/*
 * 启动用的边沿检测（2026-10-11 修 LENGTH 自动重跑）：
 *   test_enabled_last = 上一次本模式的 enabled
 *   test_seen_mode    = 上一次服务的 mode 变量
 * 只有"enabled 由 0 变 1"或"mode 变量变了"才启动新的一次运行。
 * 不能用 test_running 当判据：end() 会把它清 0，下一次循环立刻又启动。
 */
static bool     test_enabled_last;
static uint8_t  test_seen_mode = (uint8_t)LIFT_TEST_MODE_OFF;
static uint32_t test_start_tick;
static uint32_t test_duration_ticks;
static int32_t  test_start_raw[LIFT_CHANNEL_COUNT];   /* 使能那一刻的原始位置 */
/* 串级两条环，按轴各一个控制器实例（同 yaw：angle_pid / speed_pid） */
static pid_controller_t test_position_pid[LIFT_CHANNEL_COUNT];
static pid_controller_t test_speed_pid[LIFT_CHANNEL_COUNT];
/* 电流斜坡的"缓存"：上一周期已经放出去的逻辑电流（float 保存小数步长） */
static float    test_current_ramp[LIFT_CHANNEL_COUNT];
/* 逐级开放的电流上限：本周期已经放到多少 */
static int32_t  test_limit_open[LIFT_CHANNEL_COUNT];
/* 每轴的三态：APPROACH / BRAKE / HOLD */
static uint8_t  test_hold_state[LIFT_CHANNEL_COUNT];
/* 速度反馈低通的状态（浮点保存，避免每周期取整丢精度） */
static float    test_speed_filtered[LIFT_CHANNEL_COUNT];
static bool     test_all_at_target_last;
static uint32_t test_settle_tick;
static int16_t  test_force_ramped;
static int16_t  test_force_target;
static uint32_t test_force_stall_tick;

/* ==================================================================== */
/*  工具                                                                 */
/* ==================================================================== */

static int32_t lift_test_abs_i32(int32_t v)
{
    return (v < 0) ? -v : v;
}

static int16_t lift_test_clamp_current(int32_t v)
{
    if (v > LIFT_TEST_MAX_CURRENT)
    {
        return LIFT_TEST_MAX_CURRENT;
    }
    if (v < -LIFT_TEST_MAX_CURRENT)
    {
        return -LIFT_TEST_MAX_CURRENT;
    }
    return (int16_t)v;
}

/** float → int32，四舍五入（正负都对）。 */
static int32_t lift_test_round_i32(float v)
{
    return (int32_t)((v >= 0.0F) ? (v + 0.5F) : (v - 0.5F));
}

/** 逻辑电流 → 原始电流：原始电流 = direction × 逻辑电流。 */
static int16_t lift_test_logical_to_raw(uint32_t ch, int32_t logical)
{
    const int8_t direction = g_lift_control_debug.axis_direction[ch];

    return lift_test_clamp_current((direction < 0) ? -logical : logical);
}

/** 四轴合计力(N) → 每轴电流。 */
static int16_t lift_test_force_to_current_total(float force_n)
{
    return lift_test_clamp_current(
        lift_test_round_i32(force_n / LIFT_TEST_FORCE_PER_CURRENT_N));
}

static float lift_test_clamp_scale(float scale)
{
    /*
     * ⚠️ 2026-10-11 修正语义：以前"非法值（含 0 与负数）一律回落 1.0 = 满力"，
     *    本意是"别让笔误把某根轴送飞"，实际效果正相反——想用
     *    `axis_scale = 0` 关掉一根轴，得到的是【满力】。
     *    现在：<= 0 或超上限（含 NaN）一律视为"关掉这根轴"，输出 0。
     *    要减弱一根轴就填一个 (0, LIFT_TEST_SCALE_MAX] 里的正数。
     */
    if (!(scale > 0.0F) || (scale > LIFT_TEST_SCALE_MAX))
    {
        return 0.0F;
    }
    return scale;
}

/* ==================================================================== */
/*  采样：每周期都做，与 enabled 无关                                     */
/* ==================================================================== */

static void lift_test_sample(void)
{
    robot_motor_state_t motors;
    const uint32_t now = osKernelGetTickCount();
    bool all_online = true;
    uint32_t ch;

    robot_motors_get_snapshot(&motors);

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        const dji_motor_feedback_t *motor = &motors.lift.motor[ch];
        const int8_t direction = g_lift_control_debug.axis_direction[ch];
        const bool online = motor->initialized &&
            dji_motor_feedback_is_online(
                motor, now, pdMS_TO_TICKS(LIFT_TEST_ONLINE_TIMEOUT_MS));

        g_lift_jog_status.axis_position_ecd[ch] = motor->total_ecd;
        g_lift_jog_status.axis_speed_rpm[ch] = motor->speed_rpm;
        g_lift_jog_status.axis_given_current[ch] = motor->given_current;
        g_lift_jog_status.axis_temperature[ch] = motor->temperature;
        g_lift_jog_status.axis_online[ch] = online;
        g_lift_jog_status.axis_feedback_count[ch] = motor->update_count;
        g_lift_jog_status.axis_wrap_count[ch] = motor->ecd_wrap_count;
        g_lift_jog_status.axis_wrap_max_raw_delta[ch] =
            motor->ecd_wrap_max_raw_delta;
        g_lift_jog_status.axis_gap_count[ch] = motor->ecd_long_gap_count;
        g_lift_jog_status.axis_gap_max_ms[ch] = motor->ecd_long_gap_max_ms;

        /* 相对起点的偏移，逻辑坐标（正 = 伸出） */
        g_lift_jog_status.axis_offset_ecd[ch] =
            (int32_t)direction * (motor->total_ecd - test_start_raw[ch]);

        /*
         * 逻辑速度(ecd/s)：速度环的反馈。
         * C620 回的 speed_rpm 是【转子侧】rpm，所以 × 8192 ÷ 60 就是转子侧 ecd/s；
         * 再乘方向，保证"位置正方向 = 速度正方向"，否则速度环会变成正反馈。
         */
        {
            const int32_t raw =
                (int32_t)direction *
                (((int32_t)motor->speed_rpm * LIFT_TEST_ECD_PER_REV) / 60);
            const int32_t tau_ms = g_lift_test.speed_filter_tau_ms;

            if (tau_ms > 0)
            {
                /* 一阶低通：α = dt/(τ+dt)，dt = 5 ms */
                const float alpha =
                    (float)LIFT_TEST_PERIOD_MS / (float)(tau_ms +
                        (int32_t)LIFT_TEST_PERIOD_MS);

                test_speed_filtered[ch] +=
                    alpha * ((float)raw - test_speed_filtered[ch]);
                g_lift_jog_status.axis_speed_ecd_s[ch] =
                    lift_test_round_i32(test_speed_filtered[ch]);
            }
            else
            {
                test_speed_filtered[ch] = (float)raw;
                g_lift_jog_status.axis_speed_ecd_s[ch] = raw;
            }
        }

        if (!online)
        {
            all_online = false;
        }
    }

    g_lift_jog_status.all_online = all_online;
}

/* ==================================================================== */
/*  PID 实时映射：Watch 里改完立刻生效，不用重开 enabled                  */
/* ==================================================================== */

/*
 * ⚠️ 关键区分：**映射**和**装载**是两件事。
 *
 *   map  ：只把 kp/ki/kd/限幅 拷进 pid->config，**保留** integral、
 *          previous_error、initialized。
 *   load ：map + pid_reset()，把积分和微分历史一起清掉。
 *
 * 为什么不能每周期都调 pid_init()：pid_init() 内部会调 pid_reset()，
 * 那会把积分清零 —— 于是积分永远攒不起来、微分永远是 0（因为
 * initialized 每周期都被打回 false）。表现就是"ki 和 kd 填了完全没用"。
 * 所以：
 *   每周期          → map（实时生效，状态不断）
 *   enabled 上升沿  → load（清历史，从干净状态开始）
 */
static void lift_test_pid_map_one(
    pid_controller_t *pid,
    const lift_test_pid_t *cfg)
{
    pid->config.kp = cfg->kp;
    pid->config.ki = cfg->ki;
    pid->config.kd = cfg->kd;
    pid->config.integral_limit = cfg->integral_limit;
    pid->config.output_limit = cfg->output_limit;
}

/** enabled 上升沿调用：映射 + 清积分/微分历史。 */
static void lift_test_pid_load(const lift_test_control_t *cfg)
{
    uint32_t ch;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        lift_test_pid_map_one(&test_position_pid[ch], &cfg->position_pid[ch]);
        lift_test_pid_map_one(&test_speed_pid[ch], &cfg->speed_pid[ch]);
        pid_reset(&test_position_pid[ch]);
        pid_reset(&test_speed_pid[ch]);
    }
}

/*
 * 电流斜坡（di/dt 限幅）：把逻辑电流按 current_ramp_per_s 慢慢逼近目标。
 * 状态是"上一周期放出去的值"，所以叫斜坡缓存。
 *   rate <= 0  → 关掉斜坡，直接跟到目标
 * 单位：电流/秒。5 ms 周期下 rate=20000 → 每周期 100 → 0→16000 约 0.8 s。
 */
static int32_t lift_test_ramp_current(uint32_t ch, int32_t target)
{
    const int32_t rate = g_lift_test.current_ramp_per_s[ch];
    float *state = &test_current_ramp[ch];
    float max_step;

    if (rate <= 0)
    {
        *state = (float)target;
        return target;
    }

    max_step = (float)rate * ((float)LIFT_TEST_PERIOD_MS / 1000.0F);

    if ((float)target > (*state + max_step))
    {
        *state += max_step;
    }
    else if ((float)target < (*state - max_step))
    {
        *state -= max_step;
    }
    else
    {
        *state = (float)target;
    }

    return lift_test_round_i32(*state);
}

/* ==================================================================== */
/*  进入 / 退出                                                          */
/* ==================================================================== */

static void lift_test_zero_output(void)
{
    uint32_t ch;

    /*
     * ★★ 2026-10-11 第二次修：判据从 "may_write" 改成 "自己是不是所有者"。
     *
     * 第一版（同一天早些时候）用的是 `lift_owner_may_write(JOG)`。那个判据
     * 在【通道空闲】时也返回 true，于是出现这条时序：
     *
     *   T+0ms  主链：claim → 写电流 → release        （owner 立刻变回 NONE）
     *   T+1ms  台架空闲（mode = 0）：zero_output()
     *            → may_write(JOG) 看到 owner == NONE → true
     *            → 写 0，把主链刚写的电流抹掉
     *
     * 结果：主链「能算出电流、也确实写了、然后每 5 ms 被抹一次」——
     * 现象是"能抬升但抬不起来"。这是本项目第二次栽在同一个地方：
     * 第一版是"台架空闲时无条件写 0"，第二版是"用错了判据，等于换个方式
     * 无条件写 0"。两次的根因相同：**允许一个不拥有通道的模块去写它**。
     *
     * 正确规则：只有【自己当前是所有者】才写。
     *   · 台架在跑（或刚跑完的最后那一拍）→ 它是所有者 → 写 0 是对的
     *   · 台架空闲 → 它不是所有者 → **不碰通道**，主链独占
     *
     * 写 0 还有两个时刻需要，但那不是台架的活：
     *   · 参数未生效 / 掉线 → 主链的门禁会主动写 0（那是保护动作）
     */
    if (lift_owner_current() != LIFT_OWNER_JOG)
    {
        /* 通道不是自己的：只把状态清干净，绝不碰 CAN 电流 */
        g_lift_jog_status.running = false;
        g_lift_jog_status.active = false;
        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            g_lift_jog_status.axis_output_current[ch] = 0;
            g_lift_jog_status.axis_ramped_current[ch] = 0;
            g_lift_jog_status.axis_target_offset[ch] = 0;
            g_lift_jog_status.axis_target_speed_ecd_s[ch] = 0;
            g_lift_jog_status.axis_position_pid_output[ch] = 0.0F;
            g_lift_jog_status.axis_speed_pid_output[ch] = 0.0F;
            g_lift_jog_status.axis_at_target[ch] = false;
            test_current_ramp[ch] = 0.0F;
        }
        g_lift_jog_status.at_target = false;
        return;
    }

    robot_lift_set_currents(0, 0, 0, 0);
    g_lift_jog_status.running = false;
    g_lift_jog_status.active = false;

    /*
     * ⚠️ 状态里的这几个量必须跟着清零，否则停了以后还留着最后一拍的值：
     *   - VOFA 波形会显示"没使能但还有电流"，看波形的人会被骗
     *   - axis_output_current 是"我们下发了多少"，停了就是 0
     * 位置/速度/回显电流不动：那是真实的传感器读数，不是命令。
     */
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        g_lift_jog_status.axis_output_current[ch] = 0;
        g_lift_jog_status.axis_ramped_current[ch] = 0;
        g_lift_jog_status.axis_target_offset[ch] = 0;
        g_lift_jog_status.axis_target_speed_ecd_s[ch] = 0;
        g_lift_jog_status.axis_position_pid_output[ch] = 0.0F;
        g_lift_jog_status.axis_speed_pid_output[ch] = 0.0F;
        g_lift_jog_status.axis_at_target[ch] = false;
        test_current_ramp[ch] = 0.0F;
    }
    g_lift_jog_status.at_target = false;
}

/** 记起点：所有模式都是相对量，所以不需要零点。 */
static void lift_test_capture_start(void)
{
    robot_motor_state_t motors;
    uint32_t ch;

    robot_motors_get_snapshot(&motors);
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        test_start_raw[ch] = motors.lift.motor[ch].total_ecd;
        g_lift_jog_status.axis_target_offset[ch] = 0;
        g_lift_jog_status.axis_error[ch] = 0;
    }
}

static void lift_test_begin(uint8_t mode, uint32_t now)
{
    uint32_t ch;

    test_running = true;
    test_active_mode = mode;
    test_start_tick = now;
    test_all_at_target_last = false;
    test_settle_tick = now;
    test_force_ramped = 0;
    test_force_target = 0;
    test_force_stall_tick = now;

    if (mode == (uint8_t)LIFT_TEST_MODE_FORCE)
    {
        test_duration_ticks = pdMS_TO_TICKS((g_lift_test.force.duration_ms == 0U)
            ? LIFT_TEST_DEFAULT_LIFT_MS : g_lift_test.force.duration_ms);
    }
    else
    {
        test_duration_ticks = pdMS_TO_TICKS(LIFT_TEST_DEFAULT_TIMEOUT_MS);
    }

    g_lift_jog_status.run_count++;
    g_lift_jog_status.running = true;
    g_lift_jog_status.active_mode = mode;
    g_lift_jog_status.elapsed_ms = 0U;
    g_lift_jog_status.remaining_ms = 0U;
    g_lift_jog_status.at_target = false;
    g_lift_jog_status.timed_out = false;
    g_lift_jog_status.offline_abort = false;
    g_lift_jog_status.stall_latched = false;
    g_lift_jog_status.stall_timer_ms = 0U;
    g_lift_jog_status.applied_force_n = 0.0F;
    g_lift_jog_status.applied_hold_force_n = 0.0F;
    g_lift_jog_status.applied_current = 0;

    /*
     * 斜坡缓存清零：每次使能都从 0 电流开始放。
     * 不清的话上一轮残留的值会让这一轮的第一拍直接跳上去，
     * 正好抵消掉加斜坡的意义。
     */
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        test_current_ramp[ch] = 0.0F;
        test_limit_open[ch] = (g_lift_test.current_limit_start > 0)
            ? g_lift_test.current_limit_start : 0;
        test_hold_state[ch] = (uint8_t)LIFT_TEST_HOLD_APPROACH;
        test_speed_filtered[ch] = 0.0F;
        g_lift_jog_status.axis_ramped_current[ch] = 0;
        g_lift_jog_status.axis_limit_now[ch] = 0;
        g_lift_jog_status.axis_hold_state[ch] =
            (uint8_t)LIFT_TEST_HOLD_APPROACH;
        g_lift_jog_status.axis_target_speed_ecd_s[ch] = 0;
        g_lift_jog_status.axis_target_offset[ch] = 0;
    }

    lift_test_capture_start();
}

static bool lift_test_mode_enabled(uint8_t mode)
{
    switch (mode)
    {
    case LIFT_TEST_MODE_PER_AXIS: return g_lift_test.per_axis.enabled;
    case LIFT_TEST_MODE_LENGTH: return g_lift_test.length.enabled;
    case LIFT_TEST_MODE_FORCE: return g_lift_test.force.enabled;
    default: return false;
    }
}

static void lift_test_clear_enabled(uint8_t mode)
{
    switch (mode)
    {
    case LIFT_TEST_MODE_PER_AXIS: g_lift_test.per_axis.enabled = false; break;
    case LIFT_TEST_MODE_LENGTH: g_lift_test.length.enabled = false; break;
    case LIFT_TEST_MODE_FORCE: g_lift_test.force.enabled = false; break;
    default: break;
    }
    if (test_seen_mode == mode)
    {
        test_enabled_last = false;
    }
}

static void lift_test_save_snapshot(lift_test_end_reason_t reason, uint8_t mode)
{
    g_lift_test_snapshot.valid = false;
    g_lift_test_snapshot.reason = (uint8_t)reason;
    g_lift_test_snapshot.mode = mode;
    g_lift_test_snapshot.end_tick = osKernelGetTickCount();
    g_lift_test_snapshot.status = g_lift_jog_status;
    g_lift_test_snapshot.parameters = g_lift_test;
    if (test_running)
    {
        g_lift_test_snapshot.status.elapsed_ms =
            (uint32_t)(g_lift_test_snapshot.end_tick - test_start_tick) *
            (uint32_t)portTICK_PERIOD_MS;
    }
    g_lift_test_snapshot.sequence++;
    g_lift_test_snapshot.valid = true;
}

static void lift_test_end(lift_test_end_reason_t reason)
{
    if (!test_running)
    {
        return; /* 空闲周期不能覆盖上次快照。 */
    }
    lift_test_save_snapshot(reason, test_active_mode);
    lift_test_clear_enabled(test_active_mode);
    test_running = false;
    test_active_mode = (uint8_t)LIFT_TEST_MODE_OFF;
    g_lift_jog_status.active_mode = (uint8_t)LIFT_TEST_MODE_OFF;
    /*
     * 先写 0（此时自己还是所有者），再归还通道。
     * 顺序不能反：归还之后 lift_test_zero_output() 就不会写 CAN 了
     * （那不是自己的通道），最后那一下"收尾清零"就丢了。
     */
    lift_test_zero_output();
    lift_owner_release(LIFT_OWNER_JOG);
}

/*
 * @brief 启动判定（纯逻辑，单独抽出来便于主机单测）。
 *
 * 只有两种情况算"一次新的运行"：
 *   ① 本模式的 enabled 由 0 变 1（上升沿）；
 *   ② mode 变量本身变了（切模式）。
 * 结束时 end() 会把 test_running 清 0，但这里【不看】test_running ——
 * 否则下一拍就会自己重新启动（LENGTH 会一直重复爬升、距离累加）。
 * 结束自动清 enabled 和对应边沿记录，再跑一次只需重新置 1。
 */
static bool lift_test_should_start(bool enabled, uint8_t mode)
{
    const bool start = enabled &&
        (!test_enabled_last || (mode != test_seen_mode));

    test_enabled_last = enabled;
    test_seen_mode = mode;
    return start;
}

/*
 * 目标斜坡：把 ramped 往 final 靠，每周期最多走 max_step。
 * 结果写回 axis_target_offset[ch]，既当 PID 目标也方便在 Watch 里看。
 *
 * 附加输出 *out_rate：本拍目标实际走了多少（ecd/s）。
 * 它是【跟踪前馈】的来源 —— 见 lift_test_cascade_axis 里对照 pitch 的说明。
 */
static void lift_test_ramp_target(
    uint32_t ch,
    int32_t final,
    int32_t max_step,
    int32_t *out_rate)
{
    int32_t ramped = g_lift_jog_status.axis_target_offset[ch];
    const int32_t before = ramped;
    const int32_t diff = final - ramped;

    if (max_step <= 0)
    {
        ramped = final;
    }
    else if (diff > max_step)
    {
        ramped += max_step;
    }
    else if (diff < -max_step)
    {
        ramped -= max_step;
    }
    else
    {
        ramped = final;
    }
    g_lift_jog_status.axis_target_offset[ch] = ramped;

    if (out_rate != NULL)
    {
        /*
         * ⚠️ 斜坡关掉时（max_step <= 0）前馈必须是 **0**，不能算 (ramped-before)/dt：
         * 那时目标是阶跃，算出来的"速率"是无穷大（host 测试里实测到 2000 万 ecd/s），
         * 一下子就把内环顶到满电流 —— 正好是我们要消灭的那个冲击。
         * pitch 那边对应的是 `pitch_clamp(..., ±max_target_rate_deg_s)`，同一个道理。
         */
        if (max_step > 0)
        {
            *out_rate = lift_test_round_i32(
                (float)(ramped - before) *
                (1000.0F / (float)LIFT_TEST_PERIOD_MS));
        }
        else
        {
            *out_rate = 0;
        }
    }
}


/* ==================================================================== */
/*  ★ 串级的一根轴：位置环 → 速度环 → 前馈 → 电流斜坡 → 限幅             */
/*                                                                    */
/*  两个闭环模式（PER_AXIS / LENGTH）都走这一个函数，所以它们的响应        */
/*  完全可比，调好的参数也直接通用。返回【原始电流】（已按方向折算）。      */
/* ==================================================================== */

/*
 * 逐级开放的电流上限（用户实测要求）。
 * 每次 enabled 上升沿从 current_limit_start 重新开始，按 ramp 放开到 full_limit。
 *
 * 注意顺序：**先返回"这一拍允许的值"，再把下一拍的值推进**。
 * 反过来写会让使能后的第一拍就变成 start + rate·dt，语义上不对。
 */
static int32_t lift_test_open_limit(uint32_t ch, int32_t full_limit)
{
    const int32_t start = (g_lift_test.current_limit_start > 0)
        ? g_lift_test.current_limit_start : 0;
    const int32_t rate = (g_lift_test.current_limit_ramp_per_s > 0)
        ? g_lift_test.current_limit_ramp_per_s : 0;
    int32_t open = test_limit_open[ch];

    if (rate <= 0)
    {
        open = full_limit;   /* ramp <= 0 = 不逐级开放，直接满 */
    }
    if (open < start)
    {
        open = start;
    }
    if (open > full_limit)
    {
        open = full_limit;
    }

    /* 推进下一拍 */
    test_limit_open[ch] = open + (rate * (int32_t)LIFT_TEST_PERIOD_MS / 1000);
    return open;
}

/*
 * 接近目标时的目标速度上限（ecd/s）。返回值 < 0 = 不减速（用位置环自己的限幅）。
 *
 *   |误差| >= approach_zone            → 不减速
 *   deadband < |误差| < approach_zone  → 线性收窄到 approach_min_speed_ecd_s
 *   |误差| <= deadband                 → 0（刹停）
 *
 * ⚠️ 收到 approach_min_speed_ecd_s 就到底，**不能再往 0 收**：
 * 机构要克服"重力 + 静摩擦"（≈6000 电流）才动得起来，最后几十个计数
 * 如果只给几十 ecd/s 的目标速度，会**卡在死区外面进不去**。
 * host 仿真实测过：收到 0 时停在误差 -1092 处、峰峰值 93 计数，就是进不了 ±1000。
 */
static int32_t lift_test_approach_vmax(uint32_t ch, int32_t error, int32_t deadband)
{
    const int32_t zone = g_lift_test.approach_zone_ecd;
    const int32_t vmax = lift_test_round_i32(
        g_lift_test.position_pid[ch].output_limit);
    const int32_t magnitude = lift_test_abs_i32(error);
    const int32_t vmin = (g_lift_test.approach_min_speed_ecd_s > 0)
        ? g_lift_test.approach_min_speed_ecd_s : 0;

    if ((zone <= 0) || (magnitude >= zone) || (zone <= deadband))
    {
        return -1;   /* 不减速 */
    }
    if (magnitude <= deadband)
    {
        return 0;    /* 刹停 */
    }
    if (vmax <= vmin)
    {
        return vmax;
    }
    return vmin + ((vmax - vmin) * (magnitude - deadband) / (zone - deadband));
}

static int16_t lift_test_cascade_axis(
    uint32_t ch,
    int32_t final_target,
    int32_t max_step,
    int32_t tolerance)
{
    const float dt = (float)LIFT_TEST_PERIOD_MS / 1000.0F;
    /*
     * 刹停死区 = 调用方给的容差；容差 <= 0 时回落到模块级 brake_deadband_ecd。
     * ⚠️ 这一个数必须同时决定三件事，否则会出现"控制上停住了、但 at_target
     *    永远不为真" → LENGTH 一路等到超时：
     *      ① 减速收窄的终点
     *      ② 什么时候转 BRAKE / HOLD
     *      ③ axis_at_target 的判据
     */
    const int32_t deadband = (tolerance > 0)
        ? tolerance
        : ((g_lift_test.brake_deadband_ecd > 0)
               ? g_lift_test.brake_deadband_ecd : 0);
    int32_t target;
    int32_t position;
    int32_t position_error;
    int32_t speed;
    int32_t speed_target;
    float position_out;
    float speed_out;
    int32_t current;
    int32_t limit;
    int32_t approach_vmax;
    int32_t ramp_rate = 0;
    int32_t hold_ff;
    bool target_settled;

    /*
     * ★ PID 实时映射。放在这里（而不是只在 enabled 上升沿）是为了让
     * Watch 里改完增益**立刻生效**，不用"关掉再打开"。
     * 映射只覆盖参数、不动积分/微分历史，所以每周期反复调用是安全的；
     * 顺手放在这条必经之路上，就不存在"忘了映射"的可能。
     */
    lift_test_pid_map_one(&test_position_pid[ch], &g_lift_test.position_pid[ch]);
    lift_test_pid_map_one(&test_speed_pid[ch], &g_lift_test.speed_pid[ch]);

    /* ---- ① 目标斜坡：把目标位置按 target_rate_ecd_s 慢慢挪过去 ---- */
    lift_test_ramp_target(ch, final_target, max_step, &ramp_rate);
    target = g_lift_jog_status.axis_target_offset[ch];
    position = g_lift_jog_status.axis_offset_ecd[ch];
    position_error = target - position;
    speed = g_lift_jog_status.axis_speed_ecd_s[ch];
    /* 目标是否已经走到头（斜坡收敛）。刹车/保持必须等它，见 ③。 */
    target_settled = (target == final_target);

    /*
     * ---- ② 电流上限：逐级开放 ----
     * 这一层是用户实测"空载大幅摆动"的解药：不允许一上来就 16000。
     */
    limit = lift_test_open_limit(ch, LIFT_TEST_MAX_CURRENT);
    g_lift_jog_status.axis_limit_now[ch] = (int16_t)limit;

    /*
     * ---- ③ 减速区：收窄的是【速度指令】，不是电流上限 ----
     *
     * ⚠️ 这里踩过两个坑，都是 host 仿真抓出来的：
     *   ① 收窄**电流上限** → 机构停在目标附近**推不过去**：
     *      收窄后的上限刚好等于"重力 + 静摩擦"，实测停在误差 -2501 处、电流贴限幅。
     *   ② 速度指令一路收到 **0** → 最后几十个计数目标速度只剩几十 ecd/s，
     *      更推不动，卡在死区外进不去（实测停在误差 -1092，就是进不了 ±1000）。
     * 所以：收窄的是速度指令，而且**只收到 approach_min_speed_ecd_s**。
     */
    approach_vmax = lift_test_approach_vmax(ch, position_error, deadband);

    /*
     * ---- ③ 三态：APPROACH / BRAKE / HOLD ----
     *
     * ⚠️ 转 BRAKE 的条件里有 **target_settled**：
     * 目标还在斜坡上走的时候，位置误差本来就只有几十计数（跟踪前馈把它压住了），
     * 如果只看 |误差| ≤ 死区就刹车，会出现"目标还在走、轴却刹停了"的抖振。
     * 必须等目标自己走到头，才允许刹停。目标斜坡关掉时 target_settled 恒为真，
     * 行为与以前一致。
     */
    if ((lift_test_abs_i32(position_error) > deadband) || !target_settled)
    {
        if (test_hold_state[ch] != (uint8_t)LIFT_TEST_HOLD_APPROACH)
        {
            /*
             * 从 BRAKE/HOLD 退回 APPROACH：先把两条环的历史清掉。
             * HOLD 期间根本不调用 PID，previous_error 是走进 HOLD 之前那一拍的，
             * 直接接着算微分会得到一个巨大的假导数 → **退出瞬间踢一脚**
             * （kd = 0 时看不出来，一加 kd 就会炸）。
             */
            pid_reset(&test_position_pid[ch]);
            pid_reset(&test_speed_pid[ch]);
        }
        test_hold_state[ch] = (uint8_t)LIFT_TEST_HOLD_APPROACH;
    }
    else if (test_hold_state[ch] == (uint8_t)LIFT_TEST_HOLD_APPROACH)
    {
        test_hold_state[ch] = (uint8_t)LIFT_TEST_HOLD_BRAKE;
    }

    if ((test_hold_state[ch] == (uint8_t)LIFT_TEST_HOLD_BRAKE) &&
        (lift_test_abs_i32(speed) <=
         ((int32_t)LIFT_TEST_ECD_PER_REV * g_lift_test.hold_speed_rpm / 60)))
    {
        test_hold_state[ch] = (uint8_t)LIFT_TEST_HOLD_HOLD;
    }

    /* 状态发布到状态结构（Watch 直接看 axis_hold_state[4]） */
    g_lift_jog_status.axis_hold_state[ch] = test_hold_state[ch];

    if (test_hold_state[ch] == (uint8_t)LIFT_TEST_HOLD_HOLD)
    {
        /*
         * HOLD：PID 不再参与，只给重力前馈把车托住。
         * 机构不自锁，断力就会自己落下去，所以这一项不能是 0（除非台架空载）。
         */
        current = lift_test_round_i32(g_lift_test.hold_feedforward_current[ch]);
        g_lift_jog_status.axis_position_pid_output[ch] = 0.0F;
        g_lift_jog_status.axis_speed_pid_output[ch] = 0.0F;
        g_lift_jog_status.axis_target_speed_ecd_s[ch] = 0;
        current = lift_test_ramp_current(ch, current);
        g_lift_jog_status.axis_ramped_current[ch] =
            (int16_t)lift_test_clamp_current(current);
        g_lift_jog_status.axis_error[ch] = position_error;
        g_lift_jog_status.axis_at_target[ch] = true;
        return lift_test_logical_to_raw(ch, current);
    }

    /*
     * ★ 重力/保持前馈 —— **任何状态都加**，不只是 HOLD。
     * 照抄 pitch 的 `current += gravity_torque`（pitch 在哪个模式都加）。
     *
     * 为什么：重力是**单向恒定负载**。只在 HOLD 加的话，运动过程中
     * 这 4000 电流全得由 PID 靠误差攒出来 —— 误差 = 4000/(kp_pos·kp_spd)，
     * 白白吃掉一段行程精度，而且更容易顶到限幅。
     * 常开之后 PID 只需要处理"剩下的那一点"，两条环更不容易饱和。
     */
    hold_ff = lift_test_round_i32(g_lift_test.hold_feedforward_current[ch]);

    /* ---- ④ 位置环：输入误差(ecd) → 输出目标速度(ecd/s) ---- */
    if (test_hold_state[ch] == (uint8_t)LIFT_TEST_HOLD_BRAKE)
    {
        /* 刹车：速度指令强制 0，让速度环主动把速度拉下来（不是断力） */
        speed_target = 0;
        position_out = 0.0F;
    }
    else
    {
        position_out = pid_calculate(
            &test_position_pid[ch], (float)target, (float)position, dt);
        speed_target = lift_test_round_i32(position_out);
        /* 减速区：把位置环给的目标速度按距离压小（只压上限，不压下限） */
        if (approach_vmax >= 0)
        {
            if (speed_target > approach_vmax)
            {
                speed_target = approach_vmax;
            }
            else if (speed_target < -approach_vmax)
            {
                speed_target = -approach_vmax;
            }
        }

        /*
         * ---- ★ 跟踪前馈（照抄 pitch 的 `target_speed += logical_velocity`）----
         *
         * 目标以速率 R 在走的时候，理想的速度指令就是 R。
         * 不给这个前馈的话，位置环必须靠"攒出 R/kp 的误差"才能命令出 R ——
         * 那就是**跟踪滞后**：目标走多快，轴就落后多少，误差一直存在，
         * 位置环的积分/输出就一直在往外顶。
         *
         * 加上之后，斜坡期间位置误差≈0，两条环始终工作在线性区，
         * 不会出现"一给目标就饱和 → 电流饱和 → 斜坡饱和 → 极限环"。
         *
         * 放在减速收窄【之后】加，所以前馈不会被收窄砍掉
         * （pitch 也是这个顺序：先 pid_calculate，再 += logical_velocity）。
         */
        speed_target += ramp_rate;
    }
    g_lift_jog_status.axis_position_pid_output[ch] = position_out;
    g_lift_jog_status.axis_target_speed_ecd_s[ch] = speed_target;

    /* ---- ⑤ 速度环：输入误差(ecd/s) → 输出电流 ---- */
    speed_out = pid_calculate(
        &test_speed_pid[ch], (float)speed_target, (float)speed, dt);
    g_lift_jog_status.axis_speed_pid_output[ch] = speed_out;

    /* ---- ⑥ 前馈：重力/保持（单向恒定，任何状态都加）---- */
    current = lift_test_round_i32(speed_out) + hold_ff;

    /* ---- ⑦ 按【本周期生效的上限】限幅（不是硬上限 16000）---- */
    if (current > limit)
    {
        current = limit;
    }
    else if (current < -limit)
    {
        current = -limit;
    }

    /* ---- ⑧ 电流斜坡（di/dt 限幅）：防"一瞬间到 16000" ---- */
    current = lift_test_ramp_current(ch, current);
    g_lift_jog_status.axis_ramped_current[ch] =
        (int16_t)lift_test_clamp_current(current);

    /* ---- ⑨ 硬限幅 + 方向折算 ---- */
    g_lift_jog_status.axis_error[ch] = position_error;
    /* 跟上斜坡不是到达最终目标；BRAKE 期间也尚未停稳。只有 HOLD
     * 分支可以报告到位，避免 LENGTH 在斜坡还没走完时结束并断力。 */
    g_lift_jog_status.axis_at_target[ch] = false;

    return lift_test_logical_to_raw(ch, current);
}

/* ==================================================================== */
/*  ★★ 掉线保护                                                         */
/*                                                                    */
/*  抽成独立函数是为了**能被 host 测试直接调用**（任务循环是 for(;;)，      */
/*  测试进不去）。安全逻辑必须可测，否则等于没写。                        */
/* ==================================================================== */

/**
 * @brief 四轴在线检查。
 * @return true = 允许出力；false = 已经零电流退出（并把 offline_abort 置起）。
 * @note  启动前和运行中都调它：
 *          启动前挡住 → 拒绝启动
 *          运行中挡住 → 立刻停下
 */
static bool lift_test_offline_guard(void)
{
    if (g_lift_jog_status.all_online)
    {
        return true;
    }

    g_lift_jog_status.offline_abort = true;
    if (test_running)
    {
        lift_test_end(LIFT_TEST_END_OFFLINE);
    }
    else
    {
        /*
         * 没在跑（例如启动前就被挡住）：还没认领过通道，
         * 这时只清状态，不碰 CAN —— 主链可能正在用。
         */
        lift_test_zero_output();
        if (lift_test_mode_enabled(g_lift_test.mode))
        {
            lift_test_save_snapshot(LIFT_TEST_END_OFFLINE, g_lift_test.mode);
        }
    }
    lift_test_clear_enabled(g_lift_test.mode);
    return false;
}

/**
 * @brief 把四轴的原始电流一次性下发，并刷新状态里的下发值。
 *
 * ★★ 这里是**唯一的下发出口**，所以掉线保护也钉在这里 ——
 *    三个模式（PER_AXIS / LENGTH / FORCE）无论走哪条路，
 *    最后都必经这一个函数，**不可能绕过**。
 *    （host 测试直接调 lift_test_run_per_axis() 也能被挡住，
 *      这就是上一版把守卫只放在任务循环里时漏掉的那条路。）
 */
static void lift_test_apply(const int16_t currents[LIFT_CHANNEL_COUNT])
{
    uint32_t ch;

    if (!lift_test_offline_guard())
    {
        return;   /* 守卫内部已经把 CAN 电流写成 0 了 */
    }

    /*
     * ★★ 认领命令通道（2026-10-11）。
     *
     * 台架要出力，就必须先拿到这一组电流的所有权 —— 否则主链可能在同一
     * 个 2 ms 发送窗口里也写一次，两边互相覆盖。
     *
     * ── 2026-10-11 第二次修：改成"可以抢占" ────────────────────────────
     *
     * 第一版是"抢不到就算了（静默 return）"。那是错的：主链每 5 ms 也会
     * 认领同一个通道，两个任务同周期同优先级，于是台架会**频繁抢不到** ——
     * 现象是"闭环在跑、目标也在、电流却是 0"，而且时好时坏（取决于调度顺序），
     * 看起来像 PID 没调好，实际上是**命令通道被主链占着**。
     *
     * 现在：台架是**显式的人在调试**，优先级应该高于主链的自动输出。
     * 所以先看通道在谁手里：
     *   · 空着 / 是自己的        → 直接认领
     *   · 主链占着              → **先让主链放手**（release），再认领
     *   · CAN 测试占着          → 不抢（那是发送层整帧替换，抢了也没用），
     *                             记一笔 own_lost_count，让操作手看得见
     *
     * ⚠️ 让主链放手是安全的：主链每周期都会重新判断"现在该不该我写"，
     *    它下一拍自然会拿回去；而台架认领后一直持有到收尾
     *    （见 lift_test_end），期间主链本来就不该写。
     */
    {
        const lift_owner_t owner_now = lift_owner_current();

        if ((owner_now != LIFT_OWNER_NONE) && (owner_now != LIFT_OWNER_JOG) &&
            (owner_now != LIFT_OWNER_CONTROL))
        {
            /* CAN 测试占着：这是发送层的整帧替换，台架抢不到 */
            g_lift_jog_status.own_lost = true;
            g_lift_jog_status.own_lost_count++;
            g_lift_jog_status.own_holder = (uint8_t)owner_now;
            g_lift_jog_status.running = false;
            return;
        }

        if (owner_now == LIFT_OWNER_CONTROL)
        {
            /* 主链占着：让位（它不是人，可以下一拍再拿回去） */
            lift_owner_release(LIFT_OWNER_CONTROL);
            g_lift_jog_status.own_preempt_count++;
        }

        if (!lift_owner_claim(LIFT_OWNER_JOG, osKernelGetTickCount()))
        {
            g_lift_jog_status.own_lost = true;
            g_lift_jog_status.own_lost_count++;
            g_lift_jog_status.own_holder = (uint8_t)lift_owner_current();
            g_lift_jog_status.running = false;
            return;
        }
        g_lift_jog_status.own_lost = false;
        g_lift_jog_status.own_holder = (uint8_t)LIFT_OWNER_JOG;
    }

    robot_lift_set_currents(
        currents[0], currents[1], currents[2], currents[3]);
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        g_lift_jog_status.axis_output_current[ch] = currents[ch];
    }
}

/* ==================================================================== */
/*  模式 1：每轴独立闭环（串级）                                          */
/* ==================================================================== */

static void lift_test_run_per_axis(void)
{
    const int32_t max_step = g_lift_test.target_rate_ecd_s *
                             (int32_t)LIFT_TEST_PERIOD_MS / 1000;
    int16_t currents[LIFT_CHANNEL_COUNT];
    bool all_at_target = true;
    uint32_t ch;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        currents[ch] = lift_test_cascade_axis(
            ch, g_lift_test.per_axis.target_ecd[ch], max_step,
            (g_lift_test.brake_deadband_ecd > 0)
                ? g_lift_test.brake_deadband_ecd
                : LIFT_TEST_DEFAULT_TOLERANCE);

        if (!g_lift_jog_status.axis_at_target[ch])
        {
            all_at_target = false;
        }
    }

    lift_test_apply(currents);
    if (!test_running)
    {
        return;
    }
    g_lift_jog_status.at_target = all_at_target;
}

/* ==================================================================== */
/*  模式 2：四轴定长运行（串级）                                          */
/* ==================================================================== */

static void lift_test_run_length(uint32_t now)
{
    const int32_t final = g_lift_test.length.length_ecd;
    const int32_t max_step = g_lift_test.target_rate_ecd_s *
                             (int32_t)LIFT_TEST_PERIOD_MS / 1000;
    /* 到位容差 = 模块级刹停死区（2026-10-10 合并，原来 length 里另有一个） */
    const int32_t tolerance = (g_lift_test.brake_deadband_ecd > 0)
        ? g_lift_test.brake_deadband_ecd
        : LIFT_TEST_DEFAULT_TOLERANCE;
    const uint32_t settle_ticks = pdMS_TO_TICKS(
        (g_lift_test.length.settle_ms == 0U)
            ? LIFT_TEST_DEFAULT_SETTLE_MS : g_lift_test.length.settle_ms);
    const uint32_t timeout_ticks = pdMS_TO_TICKS(
        (g_lift_test.length.timeout_ms == 0U)
            ? LIFT_TEST_DEFAULT_TIMEOUT_MS : g_lift_test.length.timeout_ms);
    const uint32_t elapsed = (uint32_t)(now - test_start_tick);
    int16_t currents[LIFT_CHANNEL_COUNT];
    bool all_at_target = true;
    uint32_t ch;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        currents[ch] = lift_test_cascade_axis(ch, final, max_step, tolerance);

        if (!g_lift_jog_status.axis_at_target[ch])
        {
            all_at_target = false;
        }
    }

    lift_test_apply(currents);
    if (!test_running)
    {
        return;
    }
    g_lift_jog_status.at_target = all_at_target;

    /* 到位：全部进容差并稳定 settle_ms → 走完 */
    if (all_at_target)
    {
        if (!test_all_at_target_last)
        {
            test_all_at_target_last = true;
            test_settle_tick = now;
        }
        else if ((uint32_t)(now - test_settle_tick) >= settle_ticks)
        {
            g_lift_jog_status.remaining_ms = 0U;
            lift_test_end(LIFT_TEST_END_COMPLETE);
            return;
        }
    }
    else
    {
        test_all_at_target_last = false;
    }

    /* 超时保护 */
    if (elapsed >= timeout_ticks)
    {
        g_lift_jog_status.timed_out = true;
        g_lift_jog_status.remaining_ms = 0U;
        lift_test_end(LIFT_TEST_END_TIMEOUT);
        return;
    }

    g_lift_jog_status.remaining_ms =
        (timeout_ticks - elapsed) * (uint32_t)portTICK_PERIOD_MS;
}

/* ==================================================================== */
/*  模式 3：四轴恒定力（开环）                                            */
/* ==================================================================== */

/*
 * FORCE 抬升段结束（到时间或堵转）。
 *
 * 若保持力折算出来是 0，就【彻底交还】这一组电流：结束本次运行
 * （running 清 0、写一次 0），主抬升链才能重新接管这一组。
 * 否则只结束抬升段，继续按 hold_force_n 保持 —— 那是"松手不掉"的用法，
 * 此时仍然由本模块独占，主链保持被门禁挡住。
 *
 * 为什么以前会卡住：FORCE 只把 active 清 0，从不结束运行，于是
 * running 一直是 1，主抬升链被永久门禁，现象是"点动做完了、遥控/主链推不动"。
 */
static void lift_test_force_finish(lift_test_end_reason_t reason)
{
    g_lift_jog_status.remaining_ms = 0U;
    if (lift_test_force_to_current_total(g_lift_test.force.hold_force_n) == 0)
    {
        lift_test_end(reason);
    }
    else
    {
        g_lift_jog_status.active = false;
    }
}

static void lift_test_run_force(uint32_t now)
{
    int16_t currents[LIFT_CHANNEL_COUNT];
    int16_t min_rpm = 32767;
    int16_t max_rpm = -32768;
    uint32_t ch;
    int32_t base;

    if (g_lift_jog_status.active)
    {
        /* ---- 抬升段 ---- */
        if (test_force_target == 0)
        {
            test_force_target = lift_test_force_to_current_total(
                g_lift_test.force.force_n);
        }
        base = test_force_target;

        if (lift_test_abs_i32(base - (int32_t)test_force_ramped) <=
            LIFT_TEST_RAMP_STEP)
        {
            test_force_ramped = (int16_t)base;
        }
        else if (base > (int32_t)test_force_ramped)
        {
            test_force_ramped = (int16_t)((int32_t)test_force_ramped +
                                          LIFT_TEST_RAMP_STEP);
        }
        else
        {
            test_force_ramped = (int16_t)((int32_t)test_force_ramped -
                                          LIFT_TEST_RAMP_STEP);
        }
        base = test_force_ramped;

        g_lift_jog_status.applied_current = (int16_t)base;
        g_lift_jog_status.applied_force_n =
            (float)base * LIFT_TEST_FORCE_PER_CURRENT_N;
    }
    else
    {
        /* ---- 保持段 ---- */
        base = lift_test_force_to_current_total(
            g_lift_test.force.hold_force_n);
        g_lift_jog_status.applied_current = (int16_t)base;
        g_lift_jog_status.applied_hold_force_n =
            (float)base * LIFT_TEST_FORCE_PER_CURRENT_N;
    }

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        const float scale = lift_test_clamp_scale(
            g_lift_test.force.axis_scale[ch]);
        const int32_t scaled = lift_test_clamp_current(
            lift_test_round_i32((float)base * scale));

        currents[ch] = lift_test_logical_to_raw(ch, scaled);

        if (g_lift_jog_status.axis_speed_rpm[ch] < min_rpm)
        {
            min_rpm = g_lift_jog_status.axis_speed_rpm[ch];
        }
        if (g_lift_jog_status.axis_speed_rpm[ch] > max_rpm)
        {
            max_rpm = g_lift_jog_status.axis_speed_rpm[ch];
        }
    }

    lift_test_apply(currents);

    if (!g_lift_jog_status.active)
    {
        g_lift_jog_status.remaining_ms = 0U;
        return;
    }

    /* 堵转：四轴【最快】的也接近 0 才算顶到头 */
    {
        const uint32_t elapsed = (uint32_t)(now - test_start_tick);
        const int16_t stall_rpm = (g_lift_test.force.stall_speed_rpm > 0)
            ? g_lift_test.force.stall_speed_rpm : LIFT_TEST_DEFAULT_STALL_RPM;
        const uint32_t stall_ticks = pdMS_TO_TICKS(
            (g_lift_test.force.stall_hold_ms == 0U)
                ? LIFT_TEST_DEFAULT_STALL_MS
                : g_lift_test.force.stall_hold_ms);
        const bool stalled =
            (lift_test_abs_i32((int32_t)max_rpm) <= (int32_t)stall_rpm) &&
            (lift_test_abs_i32((int32_t)min_rpm) <= (int32_t)stall_rpm);

        if (stalled)
        {
            g_lift_jog_status.stall_timer_ms =
                (uint32_t)(now - test_force_stall_tick) *
                (uint32_t)portTICK_PERIOD_MS;
            if ((uint32_t)(now - test_force_stall_tick) >= stall_ticks)
            {
                g_lift_jog_status.stall_latched = true;
                lift_test_force_finish(LIFT_TEST_END_STALL);
                return;
            }
        }
        else
        {
            test_force_stall_tick = now;
            g_lift_jog_status.stall_timer_ms = 0U;
        }

        if (elapsed >= test_duration_ticks)
        {
            lift_test_force_finish(LIFT_TEST_END_COMPLETE);
            return;
        }

        g_lift_jog_status.remaining_ms = (elapsed >= test_duration_ticks)
            ? 0U
            : (test_duration_ticks - elapsed) * (uint32_t)portTICK_PERIOD_MS;
    }
}

/* ==================================================================== */
/*  任务主体                                                             */
/* ==================================================================== */

/* 单周期服务函数：任务和主机回归测试执行同一条采样/门禁/输出路径。 */
static void lift_test_update(uint32_t now)
{
    const uint8_t mode = g_lift_test.mode;
    const bool enabled = lift_test_mode_enabled(mode);

    /* ---- 先采样：与 enabled 无关 ---- */
    lift_test_sample();

    /*
     * PID 的实时映射不在这里做，而在 lift_test_cascade_axis() 里 ——
     * 那条路是闭环的必经之路，放那儿就不存在"忘了映射"的可能。
     */

    /* ---- 一键全停 / 关掉 / 换模式：立刻退出 ---- */
    if (g_lift_test.abort || !enabled ||
        (mode == (uint8_t)LIFT_TEST_MODE_OFF))
    {
        if (test_running)
        {
            const lift_test_end_reason_t reason = g_lift_test.abort
                ? LIFT_TEST_END_ABORT
                : ((mode == (uint8_t)LIFT_TEST_MODE_OFF)
                    ? LIFT_TEST_END_MODE_OFF
                    : ((mode != test_active_mode)
                        ? LIFT_TEST_END_MODE_CHANGED : LIFT_TEST_END_DISABLED));
            lift_test_end(reason);
        }
        /* abort 时也撤销尚未启动的当前请求，解除 abort 不会自行起动。 */
        lift_test_clear_enabled(mode);
        lift_test_zero_output();
        (void)lift_test_should_start(false, mode);
        return;
    }

    if (test_running && (mode != test_active_mode))
    {
        /* 保存被切走的模式，只清旧模式 enabled，随后按原有语义启动新模式。 */
        lift_test_end(LIFT_TEST_END_MODE_CHANGED);
    }

    /*
     * ---- ★★ 掉线保护（2026-10-10 质检补的安全缺口）----
     *
     * 放在 enabled 判断【之后】、装载/分派【之前】，所以它同时管两件事：
     *   - 启动前：某轴不在线 → 直接拒绝启动（enabled 置 1 也不出力）
     *   - 运行中：某轴掉线   → 立刻零电流退出
     *
     * 为什么必需：四根里掉一根，另外三根照样满电流推 → 车歪、机构被扭。
     * 掉线判据用 dji_motor_feedback_is_online()（超时 LIFT_TEST_ONLINE_TIMEOUT_MS），
     * 上电后 CAN 还没上来时也会被它挡住 —— 这是对的，宁可不动。
     */
    if (!lift_test_offline_guard())
    {
        return;
    }

    /*
     * ---- 启动条件：本模式 enabled 的【上升沿】，或 mode 变量刚变过 ----
     *
     * ⚠️ 2026-10-11 修正：原判据是 `!test_running || 换了模式`，而 end()
     *    会把 test_running 清 0 → 下一拍立刻又 begin()：LENGTH 一跑完就
     *    自动重来、起点重记、距离一直接着累加；FORCE 同理会被反复重启。
     *    而且"切到一个 enabled 已经是 1 的模式"会立刻启动，与注释写的
     *    "enabled 上升沿"不符。
     * 现在只认两件事：enabled 由 0 变 1，或者 mode 变量本身变了。
     * 结束自动清 enabled，再跑一次只需置 1（或换一次模式）。
     */
    if (lift_test_should_start(enabled, mode))
    {
        /* 两个闭环模式共用同一套串级参数，所以判断合并成一条 */
        if ((mode == (uint8_t)LIFT_TEST_MODE_PER_AXIS) ||
            (mode == (uint8_t)LIFT_TEST_MODE_LENGTH))
        {
            lift_test_pid_load(&g_lift_test);
        }
        lift_test_begin(mode, now);
        g_lift_jog_status.active = true;   /* FORCE：进入抬升段 */
    }

    /* 完成/超时/掉线结束后，等待新的使能边沿；不再计算或写电流。 */
    if (!test_running)
    {
        return;
    }

    if (mode != (uint8_t)LIFT_TEST_MODE_FORCE)
    {
        g_lift_jog_status.elapsed_ms =
            (uint32_t)(now - test_start_tick) * (uint32_t)portTICK_PERIOD_MS;
    }
    else if (g_lift_jog_status.active)
    {
        g_lift_jog_status.elapsed_ms =
            (uint32_t)(now - test_start_tick) * (uint32_t)portTICK_PERIOD_MS;
    }
    else
    {
        /* 保持段：计时不动 */
    }

    /* ---- 分派 ---- */
    if (mode == (uint8_t)LIFT_TEST_MODE_PER_AXIS)
    {
        lift_test_run_per_axis();
    }
    else if (mode == (uint8_t)LIFT_TEST_MODE_LENGTH)
    {
        lift_test_run_length(now);
    }
    else
    {
        lift_test_run_force(now);
    }

}

static void lift_test_task(void *argument)
{
    TickType_t last_wake_tick;

    (void)argument;
    last_wake_tick = xTaskGetTickCount();
    for (;;)
    {
        lift_test_update(osKernelGetTickCount());
        vTaskDelayUntil(
            &last_wake_tick, pdMS_TO_TICKS(LIFT_TEST_PERIOD_MS));
    }
}

/* ==================================================================== */
/*  初始化                                                               */
/* ==================================================================== */

bool lift_jog_task_create(void)
{
    const osThreadAttr_t attributes = {
        .name = "lift_test",
        .cb_mem = &lift_test_task_control,
        .cb_size = sizeof(lift_test_task_control),
        .stack_mem = lift_test_task_stack,
        .stack_size = sizeof(lift_test_task_stack),
        .priority = osPriorityNormal
    };
    uint32_t ch;

    (void)memset(&g_lift_test, 0, sizeof(g_lift_test));
    (void)memset(&g_lift_jog_status, 0, sizeof(g_lift_jog_status));
    (void)memset(&g_lift_test_snapshot, 0, sizeof(g_lift_test_snapshot));

    g_lift_test.mode = (uint8_t)LIFT_TEST_MODE_OFF;

    /*
     * ★★★ 减速 / 刹停 / 保持（2026-10-11 按实车重设，取代 10-09 的那一套）
     *
     * 10-09 那套是按"占位行程 34 mm"和"没实测过 K"配的：
     *   ① ±5000 开始降电流    → approach_zone_ecd = 5000
     *   ② 启动电流要限制        → current_limit_start = 2000
     *   ③ 逐级开放电流限制      → current_limit_ramp_per_s = 10000
     *   ④ ±1000 就刹停 + 停后重力前馈 → brake_deadband_ecd = 1000
     *
     * 2026-10-11 实测踩到的坑：**current_limit_start = 2000 起步太慢**。
     * 它要 1.4 秒才爬到 16000，而用户的起步门槛是 3000 —— 头一两秒
     * 一直在门槛附近打转，看起来像"没使劲"。现在起点直接给 8000。
     */
    g_lift_test.approach_zone_ecd = 10000;
    g_lift_test.brake_deadband_ecd = 750;
    g_lift_test.current_limit_start = 8000;
    g_lift_test.current_limit_ramp_per_s = 20000;
    g_lift_test.approach_min_speed_ecd_s = 750;
    g_lift_test.hold_speed_rpm = 20;
    /*
     * 速度反馈低通默认**开着 20 ms**（2026-10-09）：
     * C620 回的 rpm 是整数，1 rpm = 136.5 ecd/s；内环 kp=60 时 ±1 rpm
     * 就是 ±8190 电流的台阶。开着能把量化噪声压掉，代价是相位滞后。
     * 想对比就把 0 填回去试。
     */
    g_lift_test.speed_filter_tau_ms = 20;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        test_start_raw[ch] = 0;

        /*
         * ★★ 串级两条环的**出厂默认增益** ★★
         *
         * 历史：2026-10-09 曾按"用户把两环 P 填成 60 / 320 后大幅极限环"
         * 定过一组保守值（60 / 5）。但那组建立在**两个没验证过的前提**上：
         *   · K ≈ 0.5 (ecd/s) 每电流  —— 从没实测过
         *   · 行程 34 mm             —— 占位值
         * 2026-10-11 实测后两个前提都不成立（750 计数/mm、200 mm 行程），
         * 所以重新扫参，见下。
         *
         * ── 量纲（填错就是飞车）──
         *   position_pid[ch].output_limit  = **速度 ecd/s**（"最高速度"）
         *   speed_pid[ch].output_limit     = **电流**（C620 量纲）
         * 两条环的 output_limit **量纲不同**，这是最容易填反的地方。
         *
         * ── 扫参结果（_staging/tune_lift_jog.c，K = 0.5 / 2 / 9 三档）──
         *
         *   kp=0.5 ki=0.05 : 三档都**到不了 HOLD**（误差 -2237 就顶住不动）
         *   kp=2   ki=0.5  : 三档都进 HOLD，超调 <=0.8%，误差 <=750  ★ 采用
         *   kp=5   ki=1    : K=9 时进不了 HOLD
         *   kp=10  ki=2    : K=9 时进不了 HOLD
         *
         *   规律：内环太软会在"重力+静摩擦"负载下**停在半路** —— 纯 P 的输出
         *   恒定（误差被钉死），永远顶不过静摩擦。这正是用户实测"不动"的机理。
         *   太硬则 K 小时振荡（GH = kp × K × h 太大）。
         *   kp=2 / ki=0.5 是唯一在三档 K 下都收敛的。
         *   K 未知（机械估算 2~9，旧模型假设 0.5），所以按最坏情况取。
         *
         * ── 实时生效 ──
         *   这两组增益**每周期映射**，Watch 里改完立刻生效，不用重开 enabled。
         *   （积分/微分历史只在 enabled 上升沿清一次，见 lift_test_pid_load()）
         *
         * ── 实车怎么复核 ──
         *   跑起来后读 K = axis_speed_ecd_s ÷ axis_ramped_current。
         *   K 若落在 2 附近说明内环正好；远大于 2 说明内环偏软，可把 kp 往上加。
         */
        /*
         * ★★★ 2026-10-11 按实测机械常数重标（用户要求"输出给满 16000"）★★★
         *
         * ── 为什么旧值（60 / 5）大错 ────────────────────────────────────
         *
         * 旧值是按"K ≈ 0.5 ecd/s 每电流"这个**没有实测过**的假设配的。
         * 实测后重新算（依据见 lift_kinematics.h：750 计数/mm、13.43:1）：
         *
         *   速度环被控增益 K = 速度(ecd/s) ÷ 电流
         *     M3508 每电流约 0.0015 N·m；经 13.43:1、齿轮 46.693 mm
         *     → 位移速度 ≈ 9 ecd/s 每电流
         *     最坏情况（效率差、电压低）取 K ≈ 2 ecd/s 每电流
         *
         *   内环开环增益 GH = kp × K × h（h = 5 ms）
         *     旧值 kp = 60  →  GH = 60 × 2 × 0.005 = 0.6      ← 偏高，会振
         *     新值 kp = 0.5 →  GH = 0.5 × 2 × 0.005 = 0.005   ← 很稳
         *
         *   外环（位置）kp：GH = kp × h = 5 × 0.005 = 0.025
         *
         * ⚠️ 这是**估算**，不是实测标定。K 必须靠实车反推（看
         *    axis_speed_ecd_s ÷ axis_ramped_current）。所以下面的值是
         *    "能安全动起来"的起点，不是最优值。
         *
         * ── 为什么速度环必须有 ki ───────────────────────────────────────
         *
         * 纯 P 速度环有个**数学上的死锁**（2026-10-11 实测踩到）：
         *   电机需要 3000 电流才动 → 不动 → 速度恒为 0 → 误差恒为 5600
         *   → 纯 P 输出 = 5600 × kp，**恒定不变**，永远到不了 3000 → 永远不动
         * 加 ki 之后积分才会把电流顶上去。所以 ki **不能是 0**。
         */
        g_lift_test.speed_pid[ch].kp = 2.0F;
        g_lift_test.speed_pid[ch].ki = 0.5F;
        g_lift_test.speed_pid[ch].kd = 0.0F;
        /*
         * ⚠️ integral_limit 的 0 = 【积分恒为 0】，不是"不限"！
         *    pid.c 的 pid_clamp() 是正负对称钳位，钳位到 0 就是把积分项清零。
         *    所以想用 ki 就必须先给 integral_limit 一个正数，否则怎么调 ki
         *    都毫无反应（2026-10-11 修：原来这里是 0，等于把 ki 焊死）。
         *    单位是"误差×秒"：速度环误差是 ecd/s，4000 ≈ 满速误差 0.25 s 的量。
         */
        g_lift_test.speed_pid[ch].integral_limit = 4000.0F;
        /* ★ 用户 2026-10-11 指定：输出给满量程，不再掐 */
        g_lift_test.speed_pid[ch].output_limit = 16000.0F;

        g_lift_test.position_pid[ch].kp = 5.0F;
        g_lift_test.position_pid[ch].ki = 1.0F;
        g_lift_test.position_pid[ch].kd = 0.0F;
        /*
         * 同理：0 会把位置环积分焊死。积分最多贡献 4000 ecd/s
         * （= 输出上限 16000 的 25%），够消稳态误差，不抢戏。
         */
        g_lift_test.position_pid[ch].integral_limit = 4000.0F;
        /*
         * ★ 速度量纲：最高 16000 ecd/s ≈ 21.3 mm/s（750 计数/mm）。
         *
         * 2026-10-11 由 4000 提到 16000：4000 ecd/s 只有 5.3 mm/s，
         * 走 100000 计数（133 mm）要 25 秒，太慢。
         * 注意它**不是电流**，填错量纲就是飞车。
         */
        g_lift_test.position_pid[ch].output_limit = 16000.0F;

        /*
         * ★ 电流斜坡：40000 电流/秒 → 5 ms 每周期走 200 → 0→16000 约 0.4 s。
         *
         * 它一头防"PID 一瞬间到 16000"（用户实测的冲击），
         * 另一头还**参与稳定**：斜坡太慢（20000）时电流跟不上、
         * 刹不住车，过冲反而从 1.9% 涨到 19%。这两个方向都别调过头。
         */
        g_lift_test.current_ramp_per_s[ch] = 40000;

        /*
         * 刹停后的重力前馈。**必须实测**（它取决于整车重量，代码里推不出来）。
         *
         * 2026-10-11 实测：**3000 电流/轴是"车开始动"的门槛**（用户手测）。
         * 所以先给 4000（留 33% 余量）作为可用的起点：
         *   · 停住后缓慢下沉 → 往上加（4000 → 5000 → 6000）
         *   · 停住后往上顶   → 往下减
         *
         * ⚠️ 它**任何状态都加**（不只 HOLD），所以它同时承担"起步助力"和
         *    "停住托住"两件事。方向：**正 = 伸出 = 把车顶起来**。
         */
        g_lift_test.hold_feedforward_current[ch] = 4000.0F;

        /* 模式 1：每轴独立闭环 —— 各轴目标先给 0（不动） */
        g_lift_test.per_axis.target_ecd[ch] = 0;

        /* 模式 3：四轴恒定力 —— 每轴缩放先给 1.0 */
        g_lift_test.force.axis_scale[ch] = 1.0F;
    }

    /*
     * 目标斜坡默认**开着**（照抄 pitch 的 max_target_rate_deg_s）：
     * 顶层定义处有完整理由 —— 阶跃目标会让四层同时饱和 → 极限环。
     *
     * 2026-10-11：2000 → 10000 ecd/s（= 13.3 mm/s，750 计数/mm）。
     *   2000 太慢：走 100000 计数（133 mm）要 50 秒。
     *   10000 走完约 10 秒，同时仍然比"阶跃"柔和得多。
     */
    g_lift_test.target_rate_ecd_s = 10000;
    /*
     * 默认就给一次"能直接跑"的行程：100000 计数 = 133.3 mm。
     * 用户 2026-10-11 指定。**使能仍然是关的**（length.enabled = 0），
     * 所以上电不会自己动 —— 要跑必须显式把 enabled 置 1。
     */
    g_lift_test.length.length_ecd = 100000;
    g_lift_test.length.settle_ms = LIFT_TEST_DEFAULT_SETTLE_MS;
    /* 13.3 mm/s 走 133 mm 约 10 s；给 60 s 余量（含起步/收尾） */
    g_lift_test.length.timeout_ms = 60000U;

    g_lift_test.force.force_n = 0.0F;
    g_lift_test.force.duration_ms = LIFT_TEST_DEFAULT_LIFT_MS;
    g_lift_test.force.hold_force_n = 0.0F;
    g_lift_test.force.stall_speed_rpm = LIFT_TEST_DEFAULT_STALL_RPM;
    g_lift_test.force.stall_hold_ms = LIFT_TEST_DEFAULT_STALL_MS;

    if (lift_test_task_handle != NULL)
    {
        return true;
    }

    lift_test_task_handle = osThreadNew(lift_test_task, NULL, &attributes);
    return lift_test_task_handle != NULL;
}
