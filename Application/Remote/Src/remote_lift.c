#include "remote_lift.h"

#include "lift_control_task.h"

#include <string.h>

remote_lift_debug_t g_remote_lift_debug;
remote_lift_status_t g_remote_lift_status;

typedef struct
{
    bool     enabled_last;
    bool     homed_last;           /* 找零完成那一拍要重新吸附目标 */
    bool     homing_armed;         /* 本次上电已在下档清过零点 */
    bool     clear_requested;      /* 本轮的清零点请求已发（只发一次） */
    uint32_t left_down_since_ms;   /* 下档起始时刻 */
    uint32_t homing_start_ms;      /* 中档起始时刻 */
    uint32_t last_command_ms;      /* 最近一次真正发出抬升命令的时刻 */
    float    filtered_stick;
} remote_lift_state_t;

static remote_lift_state_t lift_state;

/* 摇杆量化噪声的死区（在可配置死区之上再滤掉最末端的台阶）。 */
#define REMOTE_LIFT_STICK_DEADBAND 10

/* 遥控任务周期，用于摇杆滤波。与 remote_control_task.c 的 REMOTE_PERIOD_MS 一致。 */
#define REMOTE_LIFT_PERIOD_S 0.01F

/** 四舍五入到整数计数（命令是整数，浮点只在本模块内部使用）。 */
static int32_t remote_lift_round_i32(float value)
{
    return (int32_t)((value >= 0.0F) ? (value + 0.5F) : (value - 0.5F));
}

static float remote_lift_abs_f(float value)
{
    return (value < 0.0F) ? -value : value;
}

static int32_t remote_lift_abs_i32(int32_t value)
{
    if (value == (-2147483647 - 1))
    {
        return 2147483647;
    }
    return (value < 0) ? -value : value;
}

static int16_t remote_lift_abs_i16(int16_t value)
{
    if (value == (-32767 - 1))
    {
        return 32767;
    }
    return (value < 0) ? (int16_t)(-value) : value;
}

static int32_t remote_lift_clamp_i32(
    int32_t value,
    int32_t minimum,
    int32_t maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static int16_t remote_lift_deadband(int16_t value, int16_t deadband)
{
    if ((value < deadband) && (value > -deadband))
    {
        return 0;
    }
    return value;
}

/** 一阶低通：削弱摇杆抖动与采样噪声，时间常数可配。 */
static float remote_lift_low_pass(
    float previous,
    float input,
    float tau_s,
    float dt_s)
{
    float alpha;

    if ((tau_s <= 0.0F) || (dt_s <= 0.0F))
    {
        return input;
    }
    alpha = dt_s / (tau_s + dt_s);
    return previous + (alpha * (input - previous));
}

/** 撤销使能并清掉本地的接管状态（下次使能会重新吸附当前位置）。 */
static void remote_lift_disable(uint32_t now_ms)
{
    g_remote_lift_status.enabled = false;
    g_remote_lift_status.need_homing = false;
    g_remote_lift_status.command_age_ms = 0U;
    lift_state.enabled_last = false;
    /* 时间戳往后推，避免"刚撤销"立刻又被自己的超时看门狗判一次超时。 */
    lift_state.last_command_ms = now_ms;
    lift_state.filtered_stick = 0.0F;
    lift_control_enable(false);
}

void remote_lift_init(void)
{
    (void)memset(&g_remote_lift_debug, 0, sizeof(g_remote_lift_debug));
    (void)memset(&g_remote_lift_status, 0, sizeof(g_remote_lift_status));
    (void)memset(&lift_state, 0, sizeof(lift_state));

    /*
     * ⚠️ 默认【关闭】：抬升目前只在 debug 里调（Watch 直接写
     *    g_lift_task_command，或用台架模块 lift_jog_task）。
     * 打开之前必须先定两件事（见 remote_lift.h 文件头）：
     *   ① 用哪个操作件抬升——stick_channel 默认 1 与 Pitch 抢同一个通道；
     *   ② 找零/清零点的三段拨杆时序从没在实机上跑通过。
     */
    g_remote_lift_debug.enabled = false;
    g_remote_lift_debug.stick_channel = 1;
    g_remote_lift_debug.stick_deadband = 30;
    /*
     * 摇杆量 → 每周期目标增量（【模态高度】计数，目标量纲见 lift_control_task.h）。
     * 0.91 × 满杆 660 ≈ 600 模态计数/周期，正好等于 max_target_step_ecd，
     * 于是【全行程都是线性的】，不会一推过死区就顶到限幅。
     *
     * 换算：600 模态计数/周期 = 150 单轴计数/周期 @10 ms = 15000 单轴计数/s
     *       ÷ 725 计数/mm ≈ 20.7 mm/s（单轴齿条速度）。
     * （旧注释写"20 计数/周期 ≈ 98 mm/s"两处都不对：20 是"每摇杆单位"不是
     *   满杆，而 98 mm/s 既没乘 660 也没做模态→单轴的 4 倍折算。）
     */
    g_remote_lift_debug.stick_scale = 0.91F;
    g_remote_lift_debug.stick_filter_tau_s = 0.05F;
    g_remote_lift_debug.stick_direction = 1;
    g_remote_lift_debug.max_target_step_ecd = 600;
    g_remote_lift_debug.level_tolerance_ecd = 2500;
    g_remote_lift_debug.command_timeout_ms = 40U;
    g_remote_lift_debug.homing_clear_hold_ms = 500U;
    g_remote_lift_debug.homing_arm_ms = 700U;

    /*
     * 上电时"已清零点"是 false：这意味着每次上电都必须先在下档保持够久，
     * 再切中档保持够久，才会发出找零请求（ADR-007 的两步重新找零）。
     */
}

void remote_lift_map(
    const int16_t channel[5],
    bool dbus_online,
    bool lift_enabled,
    remote_lift_left_t left,
    uint32_t now_ms)
{
    /*
     * 命令面用【模态高度】边界（= 四轴之和，已含余量），不是单轴行程：
     * 以前拿 limit_height_*_ecd[0] 夹模态目标，可达高度只有配置行程的 1/4，
     * 而且与 lift_apply_safety 用的边界不是同一套（推到头会被改写回起点）。
     */
    const int32_t height_min = g_lift_control_status.command_height_min_ecd;
    const int32_t height_max = g_lift_control_status.command_height_max_ecd;
    const int32_t limits_valid = height_max - height_min;
    const int32_t tolerance = remote_lift_abs_i32(
        g_remote_lift_debug.level_tolerance_ecd);
    const uint32_t timeout_ms = g_remote_lift_debug.command_timeout_ms;
    const bool homed = g_lift_control_status.homed &&
                       !g_lift_control_status.configuration_rejected;
    int32_t actual_height = g_lift_control_status.actual_height;

    g_remote_lift_status.command_age_ms =
        (uint32_t)(now_ms - lift_state.last_command_ms);

    /* ---- 1. 失联 / 关掉 / 不在档位：一律撤销使能 ---- */
    if (!g_remote_lift_debug.enabled || !dbus_online || !lift_enabled)
    {
        if (lift_state.enabled_last)
        {
            remote_lift_disable(now_ms);
        }
        return;
    }

    /* ---- 2. 清零点 / 找零时序（左拨杆两段保持）---- */
    if (left == REMOTE_LIFT_LEFT_DOWN)
    {
        if (lift_state.left_down_since_ms == 0U)
        {
            lift_state.left_down_since_ms = now_ms;
        }
        if ((uint32_t)(now_ms - lift_state.left_down_since_ms) >=
            g_remote_lift_debug.homing_clear_hold_ms)
        {
            if (!lift_state.clear_requested)
            {
                /*
                 * 下档保持够久 = 操作手明确要求"清零点 / 清故障"。送三样：
                 *   1. 撤销使能：抬升立刻不再接受运动命令；
                 *   2. 清零点：两步重新找零的第一步（ADR-007），
                 *      禁止在已找零状态下直接重新找零；
                 *   3. 复位锁存故障：锁存故障只能被显式清除。
                 * 只发一次：单周期脉冲由抬升任务消费。
                 */
                lift_control_enable(false);
                lift_control_request_clear_homing();
                lift_control_request_reset();
                lift_state.clear_requested = true;
                lift_state.homing_armed = true;
            }
        }
        g_remote_lift_status.homing_armed = lift_state.homing_armed;
        if (lift_state.enabled_last)
        {
            remote_lift_disable(now_ms);
        }
        return;
    }

    lift_state.left_down_since_ms = 0U;
    lift_state.clear_requested = false;

    if (left == REMOTE_LIFT_LEFT_MID)
    {
        if (lift_state.homing_start_ms == 0U)
        {
            lift_state.homing_start_ms = now_ms;
        }
        /*
         * 只有"清过零点"且在中档保持够久才允许找零。保持而不是点按，
         * 是为了让一次误碰不可能启动找零——找零会让机构一路下行到底部。
         */
        if (lift_state.homing_armed &&
            ((uint32_t)(now_ms - lift_state.homing_start_ms) >=
             g_remote_lift_debug.homing_arm_ms) &&
            !homed)
        {
            lift_control_request_homing();
        }
        g_remote_lift_status.homing_armed = lift_state.homing_armed;
        if (lift_state.enabled_last)
        {
            remote_lift_disable(now_ms);
        }
        return;
    }

    /* ---- 上档：正常操作，抬升保持高度 ---- */
    lift_state.homing_start_ms = 0U;
    g_remote_lift_status.homing_armed = lift_state.homing_armed;

    if (!homed)
    {
        /* 未找零不发运动命令。这里只提示，不"帮忙"去动机构。 */
        if (lift_state.enabled_last)
        {
            remote_lift_disable(now_ms);
        }
        g_remote_lift_status.need_homing = true;
        g_remote_lift_status.no_homing_count++;
        g_remote_lift_status.block_count++;
        return;
    }
    g_remote_lift_status.need_homing = false;

    /*
     * 本模块自己的刷新看门狗。它比抬升护栏的超时更早动作：先撤销使能，
     * 让护栏把"没人给命令"解释成受控缓降（ADR-006），而不是靠设备层的
     * "命令超时归零"去断力——对不自锁的抬升机构，断力就是掉下来。
     */
    if ((timeout_ms > 0U) &&
        ((uint32_t)(now_ms - lift_state.last_command_ms) > timeout_ms))
    {
        remote_lift_disable(now_ms);
        g_remote_lift_status.block_count++;
        return;
    }

    /*
     * 找零刚完成的那一拍：目标还停在"未找零"时的起点值，而机构此刻在
     * 底部回退点上。先把目标吸附到实际位置再积分，避免这一拍就把机构
     * 往一个凭空的坐标上拽。
     */
    if (homed && !lift_state.homed_last)
    {
        lift_state.homed_last = true;
        if (lift_state.enabled_last)
        {
            actual_height = g_lift_control_status.actual_height;
            if (limits_valid > 0)
            {
                actual_height = remote_lift_clamp_i32(
                    actual_height, height_min, height_max);
            }
            lift_control_set_target_from(LIFT_DIAG_SOURCE_REMOTE, actual_height, 0, 0);
            g_remote_lift_status.target_height_ecd = actual_height;
            g_remote_lift_status.actual_height_ecd = actual_height;
            g_remote_lift_status.refresh_count++;
            lift_state.last_command_ms = now_ms;
            return;
        }
    }
    else if (!homed)
    {
        lift_state.homed_last = false;
    }
    else
    {
        /* 已找零且上一拍也已找零：正常路径。 */
    }

    /* ---- 3. 首次使能：吸附当前位置，姿态目标归零 ---- */
    if (!lift_state.enabled_last)
    {
        lift_state.enabled_last = true;
        lift_state.filtered_stick = 0.0F;
        g_remote_lift_status.enable_count++;
        actual_height = g_lift_control_status.actual_height;

        if (limits_valid > 0)
        {
            actual_height = remote_lift_clamp_i32(
                actual_height, height_min, height_max);
        }

        lift_control_set_target_from(LIFT_DIAG_SOURCE_REMOTE, actual_height, 0, 0);
        lift_control_enable(true);
        g_remote_lift_status.enabled = true;
        g_remote_lift_status.target_height_ecd = actual_height;
        g_remote_lift_status.actual_height_ecd = actual_height;
        g_remote_lift_status.refresh_count++;
        lift_state.last_command_ms = now_ms;
        return;
    }

    /* ---- 4. 正常跟随：增量 -> 绝对目标 -> 每周期刷新 ---- */
    {
        int32_t target = g_lift_control_status.target_height_ecd;
        int32_t step;
        int16_t raw;

        raw = (int16_t)remote_lift_deadband(
            channel[g_remote_lift_debug.stick_channel],
            g_remote_lift_debug.stick_deadband);
        if (remote_lift_abs_i16(raw) <= REMOTE_LIFT_STICK_DEADBAND)
        {
            raw = 0;
        }

        lift_state.filtered_stick = remote_lift_low_pass(
            lift_state.filtered_stick,
            (float)raw,
            g_remote_lift_debug.stick_filter_tau_s,
            REMOTE_LIFT_PERIOD_S);
        if (remote_lift_abs_f(lift_state.filtered_stick) <=
            (float)REMOTE_LIFT_STICK_DEADBAND)
        {
            lift_state.filtered_stick = 0.0F;
        }
        g_remote_lift_status.stick_filtered = lift_state.filtered_stick;

        step = remote_lift_round_i32(
            lift_state.filtered_stick *
            g_remote_lift_debug.stick_scale *
            (float)g_remote_lift_debug.stick_direction);

        if (remote_lift_abs_i32(g_remote_lift_debug.max_target_step_ecd) > 0)
        {
            step = remote_lift_clamp_i32(
                step,
                -remote_lift_abs_i32(g_remote_lift_debug.max_target_step_ecd),
                remote_lift_abs_i32(g_remote_lift_debug.max_target_step_ecd));
        }

        /*
         * 车身歪了就先往下走把它放平：如果继续往上顶，只有一个角受力，
         * 机构内应力会全落在翘曲模态上（ADR-008：翘曲不加 PID，它是机械
         * 问题，只能靠"先放平再抬"来避免）。
         */
        if ((remote_lift_abs_i32(g_lift_control_status.actual_roll) >
             tolerance) ||
            (remote_lift_abs_i32(g_lift_control_status.actual_pitch) >
             tolerance))
        {
            if (step > 0)
            {
                step = 0;
            }
            if (target > actual_height)
            {
                target = actual_height;
            }
            g_remote_lift_status.level_block_count++;
        }

        target += step;

        if (limits_valid > 0)
        {
            target = remote_lift_clamp_i32(target, height_min, height_max);
        }

        /*
         * 姿态目标每周期写 0：落地歪车时不靠姿态 PID 去"拧"回来，而是让
         * 四条通道各自回到同一高度（水平由机械与地面决定，不靠内力硬顶）。
         */
        lift_control_set_target_from(LIFT_DIAG_SOURCE_REMOTE, target, 0, 0);

        g_remote_lift_status.enabled = true;
        g_remote_lift_status.target_height_ecd = target;
        g_remote_lift_status.actual_height_ecd = actual_height;
        g_remote_lift_status.refresh_count++;
        lift_state.last_command_ms = now_ms;
    }
}
