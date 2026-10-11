#include "lift_jog_task.h"

#include "cmsis_os2.h"
#include "dji_motor.h"
#include "FreeRTOS.h"
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
 * 依据：M3508 原装减速箱 19.2:1 + 驱动齿轮分度圆 69 mm + 效率取 0.7。
 * ⚠️ 效率是估算，真实值有 ±30% 量级误差；用弹簧秤实测可反推校准。
 */
#define LIFT_TEST_FORCE_PER_CURRENT_N      0.5705F

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

static void lift_test_end(void)
{
    test_running = false;
    test_active_mode = (uint8_t)LIFT_TEST_MODE_OFF;
    g_lift_jog_status.active_mode = (uint8_t)LIFT_TEST_MODE_OFF;
    lift_test_zero_output();
}

/*
 * @brief 启动判定（纯逻辑，单独抽出来便于主机单测）。
 *
 * 只有两种情况算"一次新的运行"：
 *   ① 本模式的 enabled 由 0 变 1（上升沿）；
 *   ② mode 变量本身变了（切模式）。
 * 结束时 end() 会把 test_running 清 0，但这里【不看】test_running ——
 * 否则下一拍就会自己重新启动（LENGTH 会一直重复爬升、距离累加）。
 * 想再跑一次：先把 enabled 清 0 再置 1，或者换一次模式。
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
static int32_t lift_test_approach_vmax(int32_t error, int32_t deadband)
{
    const int32_t zone = g_lift_test.approach_zone_ecd;
    const int32_t vmax = lift_test_round_i32(
        g_lift_test.position_pid[0].output_limit);
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
    approach_vmax = lift_test_approach_vmax(position_error, deadband);

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
    g_lift_jog_status.axis_at_target[ch] =
        (lift_test_abs_i32(position_error) <= tolerance);

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

    if (test_running)
    {
        lift_test_end();
    }
    lift_test_zero_output();
    g_lift_jog_status.offline_abort = true;
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
            lift_test_end();
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
        lift_test_end();
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
static void lift_test_force_finish(void)
{
    g_lift_jog_status.active = false;

    if (lift_test_force_to_current_total(g_lift_test.force.hold_force_n) == 0)
    {
        lift_test_end();
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
                lift_test_force_finish();
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
            lift_test_force_finish();
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

static void lift_test_task(void *argument)
{
    TickType_t last_wake_tick;

    (void)argument;
    last_wake_tick = xTaskGetTickCount();

    for (;;)
    {
        const uint32_t now = osKernelGetTickCount();
        const uint8_t mode = g_lift_test.mode;
        bool enabled;

        /* ---- 先采样：与 enabled 无关 ---- */
        lift_test_sample();

        /*
         * PID 的实时映射不在这里做，而在 lift_test_cascade_axis() 里 ——
         * 那条路是闭环的必经之路，放那儿就不存在"忘了映射"的可能。
         */

        if (mode == (uint8_t)LIFT_TEST_MODE_PER_AXIS)
        {
            enabled = g_lift_test.per_axis.enabled;
        }
        else if (mode == (uint8_t)LIFT_TEST_MODE_LENGTH)
        {
            enabled = g_lift_test.length.enabled;
        }
        else if (mode == (uint8_t)LIFT_TEST_MODE_FORCE)
        {
            enabled = g_lift_test.force.enabled;
        }
        else
        {
            enabled = false;
        }

        /* ---- 一键全停 / 关掉 / 换模式：立刻退出 ---- */
        if (g_lift_test.abort || !enabled ||
            (mode == (uint8_t)LIFT_TEST_MODE_OFF))
        {
            if (test_running)
            {
                lift_test_end();
            }
            lift_test_zero_output();
            /*
             * 关掉了 → 清除掉线标记。
             * 标记是"这一次使能尝试"的结论，要重新使能才会重新判定，
             * 这样用户能在 Watch 里看到"上次为什么没动"。
             */
            g_lift_jog_status.offline_abort = false;
            /*
             * 边沿状态跟着实际值走：enabled 已经是 0 → 下次置 1 才算上升沿；
             * abort 还是 1 而 enabled 仍为 1 → 不产生新的上升沿，避免
             * "解除全停时悄悄自己又跑一次"。返回值在这里不用。
             */
            (void)lift_test_should_start(enabled, mode);
            vTaskDelayUntil(
                &last_wake_tick, pdMS_TO_TICKS(LIFT_TEST_PERIOD_MS));
            continue;
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
            vTaskDelayUntil(
                &last_wake_tick, pdMS_TO_TICKS(LIFT_TEST_PERIOD_MS));
            continue;
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
         * 想再跑一次：先把 enabled 清 0 再置 1（或换一次模式）。
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

    g_lift_test.mode = (uint8_t)LIFT_TEST_MODE_OFF;

    /*
     * ★★★ 减速 / 刹停 / 保持（2026-10-09 用户实测后的四条要求）
     *
     *   ① ±5000 开始降电流    → approach_zone_ecd = 5000
     *   ② 启动电流要限制        → current_limit_start = 2000
     *   ③ 逐级开放电流限制      → current_limit_ramp_per_s = 10000
     *   ④ ±1000 就刹停 + 停后重力前馈 → brake_deadband_ecd = 1000
     *
     * 为什么要这三层：用户实测 **1/2 两根轴在空载、限流 16000 时大幅摆动**。
     * 一上来就允许满电流 + 贴着目标还满电流，是摆动的直接来源。
     */
    g_lift_test.approach_zone_ecd = 5000;
    g_lift_test.brake_deadband_ecd = 1000;
    g_lift_test.current_limit_start = 2000;
    g_lift_test.current_limit_ramp_per_s = 10000;
    g_lift_test.approach_min_speed_ecd_s = 500;
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
         * ★★ 串级两条环的**出厂默认增益**（2026-10-09 按用户实测重设）。
         *
         * 用户实测把两环 P 填成 **60 / 320** 后出现大幅极限环：
         *   GH = speed_kp × K × h = 320 × 0.5 × 0.005 = 0.80
         *   而主抬升模块用的是 GH = 0.30（留 6 倍余量）。
         * 所以出厂取保守值，用户再往上加：
         *
         *   position_pid.kp = 5     （外环；仿真扫描推荐 10，先给一半）
         *   speed_pid.kp    = 60    （内环；GH = 0.15，推荐 120）
         *
         * ⚠️ 两条环的 kp **都非 0 才会动**：位置环出速度、速度环出电流。
         *    只填一个 → 另一个输出 0 → 整条链还是 0。
         *
         * ── 量纲（填错就是飞车）──
         *   position_pid[ch].output_limit  = **速度 ecd/s**（"最高速度"）
         *     4000 ecd/s ÷ 725 计数/mm ≈ 5.5 mm/s
         *   speed_pid[ch].output_limit     = **电流**（C620 量纲）
         *     16000 是硬上限；**台架建议先填 5000**（用户实测 5000 就够，
         *     空载轴在 16000 下容易摆）
         *
         * ── 调参顺序（照这个走）──
         *   ① 先只加 speed_pid.kp：60 → 90 → 120 → …，加到刚要振退回一档
         *   ② 内环定了再加 position_pid.kp：5 → 8 → 10 → …
         *   ③ 全程盯 VOFA 通道 8~11（电流）：**只要还在跑满就说明在饱和区，别加**
         *
         * ── 实时生效 ──
         *   这两组增益**每周期映射**，Watch 里改完立刻生效，不用重开 enabled。
         *   （积分/微分历史只在 enabled 上升沿清一次，见 lift_test_pid_load()）
         */
        g_lift_test.speed_pid[ch].kp = 60.0F;
        g_lift_test.speed_pid[ch].ki = 0.0F;
        g_lift_test.speed_pid[ch].kd = 0.0F;
        /*
         * ⚠️ integral_limit 的 0 = 【积分恒为 0】，不是"不限"！
         *    pid.c 的 pid_clamp() 是正负对称钳位，钳位到 0 就是把积分项清零。
         *    所以想用 ki 就必须先给 integral_limit 一个正数，否则怎么调 ki
         *    都毫无反应（2026-10-11 修：原来这里是 0，等于把 ki 焊死）。
         *    单位是"误差×秒"：速度环误差是 ecd/s，20000 ≈ 满速误差 2 s 的量。
         */
        g_lift_test.speed_pid[ch].integral_limit = 20000.0F;
        g_lift_test.speed_pid[ch].output_limit = 16000.0F;

        g_lift_test.position_pid[ch].kp = 5.0F;
        g_lift_test.position_pid[ch].ki = 0.0F;
        g_lift_test.position_pid[ch].kd = 0.0F;
        /* 同理：0 会把位置环积分焊死。4000 = 位置环满输出速度 × 1 s。 */
        g_lift_test.position_pid[ch].integral_limit = 4000.0F;
        /* ★ 速度量纲：最高 4000 ecd/s ≈ 5.5 mm/s。它不是电流！ */
        g_lift_test.position_pid[ch].output_limit = 4000.0F;

        /*
         * ★ 电流斜坡：60000 电流/秒 → 5 ms 每周期走 300 → 0→16000 约 0.27 s。
         *
         * 它一头防"PID 一瞬间到 16000"（用户实测的冲击），
         * 另一头还**参与稳定**：斜坡太慢（20000）时电流跟不上、
         * 刹不住车，过冲反而从 1.9% 涨到 19%。这两个方向都别调过头。
         */
        g_lift_test.current_ramp_per_s[ch] = 60000;

        /* 摩擦力前馈：先给 0（关闭），调参时按注释里的建议打开 */
        /*
         * 刹停后的重力前馈。**必须实测**，先给 0（台架空载可以先 0）。
         * 测法：模式 3 慢慢加 hold_force_n 到"刚好不下滑"，
         *       每轴电流 = hold_force_n ÷ 0.5705，四根都填这个数。
         */
        g_lift_test.hold_feedforward_current[ch] = 0.0F;

        /* 模式 1：每轴独立闭环 —— 各轴目标先给 0（不动） */
        g_lift_test.per_axis.target_ecd[ch] = 0;

        /* 模式 3：四轴恒定力 —— 每轴缩放先给 1.0 */
        g_lift_test.force.axis_scale[ch] = 1.0F;
    }

    /*
     * 目标斜坡默认**开着**（照抄 pitch 的 max_target_rate_deg_s）：
     * 顶层定义处有完整理由 —— 阶跃目标会让四层同时饱和 → 极限环。
     * 想让目标也慢慢走，就填一个速度（ecd/s）；要纯阶跃就填 0。
     */
    g_lift_test.target_rate_ecd_s = LIFT_TEST_DEFAULT_TARGET_RATE_ECD_S;
    g_lift_test.length.length_ecd = 0;
    g_lift_test.length.settle_ms = LIFT_TEST_DEFAULT_SETTLE_MS;
    g_lift_test.length.timeout_ms = LIFT_TEST_DEFAULT_TIMEOUT_MS;

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
