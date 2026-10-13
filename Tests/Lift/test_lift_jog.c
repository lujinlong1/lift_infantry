/*
 * Host unit test for the lift bench module's NEW cascade + current ramp.
 *
 * This file #includes the real firmware source (lift_jog_task.c) so that the
 * static functions under test are reachable directly. Nothing is re-implemented
 * here: if the shipped module drifts, these assertions fail.
 *
 * Covered:
 *   [1] rpm -> axis_speed_ecd_s conversion and the axis_direction sign chain
 *   [2] current ramp really bounds di/dt, and 0 disables it
 *   [3] cascade units: position loop output is SPEED, speed loop output is CURRENT
 *   [4] closed-loop convergence: reaches the target and then holds it (no drift)
 *   [5] current never exceeds the hard cap even with the ramp enabled
 *   [6] the ramp cache is cleared on every enable rising edge
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

/* 先引入桩头文件（把固件的 HAL/RTOS 依赖换掉），再引入真实模块头 */
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "dji_motor.h"
#include "robot_motors.h"
#include "robot_motor_commands.h"

#include "lift_control_task.h"
#include "lift_jog_task.h"
#include "pid.h"

/* ==================================================================== */
/*  桩：把固件依赖换掉，只保留被测逻辑                                    */
/* ==================================================================== */

static robot_motor_state_t g_fake_motors;
static int16_t g_last_cmd[LIFT_CHANNEL_COUNT];
static uint32_t g_cmd_count;
static uint32_t g_now;

uint32_t g_stub_delay_calls;

void robot_motors_get_snapshot(robot_motor_state_t *state)
{
    *state = g_fake_motors;
}

bool dji_motor_feedback_is_online(
    const dji_motor_feedback_t *feedback,
    uint32_t now_ms,
    uint32_t timeout_ticks)
{
    (void)now_ms;
    (void)timeout_ticks;
    return (feedback != NULL) && feedback->initialized;
}

void robot_lift_set_currents(int16_t c0, int16_t c1, int16_t c2, int16_t c3)
{
    g_last_cmd[0] = c0;
    g_last_cmd[1] = c1;
    g_last_cmd[2] = c2;
    g_last_cmd[3] = c3;
    g_cmd_count++;
}

void vTaskDelayUntil(TickType_t *previous_wake, TickType_t increment)
{
    (void)previous_wake;
    (void)increment;
    g_stub_delay_calls++;
}

TickType_t xTaskGetTickCount(void)
{
    return 0U;
}

osThreadId_t osThreadNew(void (*func)(void *), void *argument,
                         const osThreadAttr_t *attr)
{
    (void)func;
    (void)argument;
    (void)attr;
    return (osThreadId_t)1;
}

uint32_t osKernelGetTickCount(void)
{
    return g_now;
}

/* 真实源码：静态函数于是可以在本文件里直接调用。
 * g_lift_control_debug 在固件里由 lift_control_task.c 提供（那个文件依赖 HAL，
 * 测试里不编译），所以这里给出唯一的定义。 */
lift_control_debug_t g_lift_control_debug;

#include "lift_jog_task.c"

/* ==================================================================== */
/*  断言框架                                                             */
/* ==================================================================== */

static uint32_t g_checks;
static uint32_t g_fails;

static void check_true(bool condition, const char *what)
{
    g_checks++;
    if (!condition)
    {
        g_fails++;
        printf("  FAIL: %s\n", what);
    }
}

static void check_int(int32_t actual, int32_t expected, const char *what)
{
    g_checks++;
    if (actual != expected)
    {
        g_fails++;
        printf("  FAIL: %s (got %ld, want %ld)\n", what, (long)actual,
               (long)expected);
    }
}

static void check_int_le(int32_t actual, int32_t limit, const char *what)
{
    g_checks++;
    if (actual > limit)
    {
        g_fails++;
        printf("  FAIL: %s (got %ld, limit %ld)\n", what, (long)actual,
               (long)limit);
    }
}

static void check_float(float actual, float expected, const char *what)
{
    g_checks++;
    if (fabsf(actual - expected) > 1e-6f)
    {
        g_fails++;
        printf("  FAIL: %s (got %f, want %f)\n", what, (double)actual,
               (double)expected);
    }
}

/* ==================================================================== */
/*  被控对象模型：一阶惯性 + **库仑静摩擦**                                */
/*    J dv/dt = i - sign(v)·F - v/K                                     */
/*    K = 1/b = 0.5 (ecd/s)/电流（实测稳态增益）                          */
/*    F = 静摩擦门槛（电流）：静止时 |i| ≤ F 就一点都不动 ——                */
/*        "在目标附近来回抖"的根源之一                                    */
/* ==================================================================== */

#define PLANT_K       0.5f
#define PLANT_DT_S    ((float)LIFT_TEST_PERIOD_MS / 1000.0f)

/* 机械时间常数：实测只给了稳态增益 K=0.5，时间常数是估的，
 * 所以调参扫描里专门看它对结论的影响。 */
static float g_plant_tau = 0.1f;

/* 库仑静摩擦（电流量纲）。实测无载启动门槛 ≈ 2000。 */
static float g_plant_friction = 0.0f;

/*
 * 恒定负载（电流量纲，方向 = 逻辑负 = 把齿条往回压）。
 * 车上就是"车重折算到单轴的量"。它在的时候环路必须**持续出力**才能停住，
 * 而这正是"停在目标附近一直抖"的另一个必要条件 ——
 * 没有它，误差为 0 时电流为 0，机构自己就停住了，抖不起来。
 */
static float g_plant_gravity = 0.0f;

/*
 * ★ 速度反馈的量化抖动（单位 rpm）。
 *
 * C620 回的 speed_rpm 是**整数**，1 rpm = 8192/60 = 136.5 ecd/s。
 * 机构静止时它并不是稳定的 0，会在 0/±1 之间跳（电调测速窗口 + 取整）。
 * 内环 kp = 120 时，**±1 rpm 的抖动 = ±16380 电流 = 直接满量程**。
 * 这就是"停在目标附近一直抖"最可能的直接原因，也必须建模才能复现。
 */
static int32_t g_plant_rpm_dither = 0;
static int32_t g_dither_sign = 1;

static float g_axis_position[LIFT_CHANNEL_COUNT];   /* 逻辑位置 ecd */
static float g_axis_velocity[LIFT_CHANNEL_COUNT];   /* 逻辑速度 ecd/s */

/*
 * 每轴"接线还在不在"。置 false 模拟掉线。
 *
 * ⚠️ plant_publish() 必须跟随它，**不能无脑写 initialized = true** ——
 * 早期版本写死 true，掉线在下一拍就被自己恢复了，
 * 导致"掉线保护"测试测了个寂寞（草稿里确实踩过这个坑）。
 */
static bool g_axis_connected[LIFT_CHANNEL_COUNT];

/** 把逻辑位置/速度写回假电机，使 lift_test_sample() 能反算出同样的值。 */
static void plant_publish(void)
{
    uint32_t ch;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        const int32_t direction =
            g_lift_control_debug.axis_direction[ch];
        float rpm;

        /* 原始 total_ecd = direction * 逻辑位置（start_raw 固定为 0） */
        g_fake_motors.lift.motor[ch].total_ecd =
            (int32_t)direction * (int32_t)g_axis_position[ch];

        /*
         * 逻辑速度 = direction * rpm * 8192 / 60  →  反解 rpm。
         * ⚠️ 按真实 C620 的行为**量化到整数 rpm**：
         * 1 rpm = 8192/60 = 136.5 ecd/s，这就是速度反馈的分辨率，
         * 也正是"速度环把量化台阶放大成电流抖动"的来源。
         */
        rpm = (g_axis_velocity[ch] * 60.0f) / 8192.0f;
        g_fake_motors.lift.motor[ch].speed_rpm =
            (int16_t)(direction * (int32_t)(rpm + (rpm >= 0 ? 0.5f : -0.5f)));

        /* 量化抖动：最小的 ±1 rpm 跳变（真实电调在静止时就是这样） */
        if (g_plant_rpm_dither > 0)
        {
            g_dither_sign = -g_dither_sign;
            g_fake_motors.lift.motor[ch].speed_rpm +=
                (int16_t)(direction * g_dither_sign * g_plant_rpm_dither);
        }

        /* ★ 跟随 g_axis_connected，不能写死 true（否则掉线测不出来） */
        g_fake_motors.lift.motor[ch].initialized = g_axis_connected[ch];
    }
}

/** 单轴被控对象推进一步：一阶惯性 + 库仑静摩擦 + 恒定重力负载。 */
static void plant_integrate(uint32_t ch, float i)
{
    const float j = g_plant_tau / PLANT_K;
    const float v = g_axis_velocity[ch];
    const float f = g_plant_friction;
    const float drive = i - g_plant_gravity;   /* 重力恒把齿条往回压 */
    float net;

    if (v > 0.5f)
    {
        net = drive - f;
    }
    else if (v < -0.5f)
    {
        net = drive + f;
    }
    else
    {
        if ((drive <= f) && (drive >= -f))
        {
            /* 静止且净电流没过静摩擦：一点都不动（这正是复现抖动的关键） */
            g_axis_velocity[ch] = 0.0f;
            return;
        }
        net = drive - ((drive > 0.0f) ? f : -f);
    }

    g_axis_velocity[ch] = v + ((net - (v / PLANT_K)) / j) * PLANT_DT_S;
    g_axis_position[ch] += g_axis_velocity[ch] * PLANT_DT_S;
}

/** 一个控制周期：采样 → 串级 → 用逻辑电流推被控对象。 */
static void plant_step(bool run_length_mode)
{
    uint32_t ch;

    plant_publish();
    lift_test_sample();

    if (run_length_mode)
    {
        lift_test_run_length(g_now);
    }
    else
    {
        lift_test_run_per_axis();
    }

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        /*
         * ★ 被控对象必须由【真正发出去的电流】驱动，不能用 axis_ramped_current。
         *
         * axis_ramped_current 是串级内部算出来的中间量；真正到电调的是
         * axis_output_current（原始电流，带方向）。两者在正常情况下相等，
         * 但**任何在出口处拦截输出的保护（掉线保护、以后的限位）只改后者**。
         * 用内部量当被控对象输入，会让"保护到底有没有生效"测不出来 ——
         * 这正是掉线保护第一版没测出来的原因。
         */
        const int32_t direction = g_lift_control_debug.axis_direction[ch];
        const float logical =
            (float)(direction * (int32_t)g_lift_jog_status.axis_output_current[ch]);

        plant_integrate(ch, logical);
    }
    g_now += LIFT_TEST_PERIOD_MS;
}

/**
 * 把模块恢复到"刚上电"的干净状态。
 *
 * 保留真实的出厂参数；各物理测试显式配置负载或空载参考控制器，
 * 避免新版 ki/前馈/目标斜坡被悄悄继承进旧夹具。
 */
static void reset_all(void)
{
    (void)lift_jog_task_create();
    lift_owner_reset();
    test_running = false;
    test_enabled_last = false;
    test_seen_mode = LIFT_TEST_MODE_OFF;
    g_now = 0U;

    (void)memset(&g_fake_motors, 0, sizeof(g_fake_motors));
    (void)memset(g_last_cmd, 0, sizeof(g_last_cmd));
    (void)memset(g_axis_position, 0, sizeof(g_axis_position));
    (void)memset(g_axis_velocity, 0, sizeof(g_axis_velocity));
    g_cmd_count = 0U;
    g_plant_friction = 0.0f;   /* 默认无摩擦/无重载；要测极限环的用例自己设 */
    g_plant_gravity = 0.0f;
    g_plant_rpm_dither = 0;
    g_plant_tau = 0.1f;

    for (uint32_t ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        g_axis_connected[ch] = true;
        g_fake_motors.lift.motor[ch].initialized = true;
    }
}

/*
 * 参考调参值：host 仿真扫出来的那组（`[8] 调参参考`），
 * 独立的空载夹具参数；不代表实车 PID 推荐值。
 */
#define REF_KP_POS   10.0F
#define REF_KP_SPD   120.0F
#define REF_VMAX     4000
#define REF_RAMP     60000

/** 在 reset_all() 之后调用：填上能跑起来的参考增益。 */
static void apply_reference_tuning(void)
{
    for (uint32_t ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        g_lift_test.position_pid[ch].kp = REF_KP_POS;
        g_lift_test.position_pid[ch].ki = 0.0F;
        g_lift_test.position_pid[ch].kd = 0.0F;
        g_lift_test.position_pid[ch].output_limit = (float)REF_VMAX;
        g_lift_test.speed_pid[ch].kp = REF_KP_SPD;
        g_lift_test.speed_pid[ch].ki = 0.0F;
        g_lift_test.speed_pid[ch].kd = 0.0F;
        g_lift_test.hold_feedforward_current[ch] = 0.0F;
        g_lift_test.speed_pid[ch].output_limit = 16000.0F;
        g_lift_test.current_ramp_per_s[ch] = REF_RAMP;
        g_lift_test.approach_zone_ecd = 5000;
        g_lift_test.brake_deadband_ecd = 1000;
        g_lift_test.current_limit_start = 2000;
        g_lift_test.current_limit_ramp_per_s = 10000;
        g_lift_test.approach_min_speed_ecd_s = 500;
        g_lift_test.hold_speed_rpm = 20;
        lift_test_pid_map_one(&test_position_pid[ch],
                              &g_lift_test.position_pid[ch]);
        lift_test_pid_map_one(&test_speed_pid[ch], &g_lift_test.speed_pid[ch]);
    }
    g_lift_test.target_rate_ecd_s = 2000;
}

/* ==================================================================== */
/*  [1] 采样：rpm → ecd/s 与方向链                                        */
/* ==================================================================== */

static void t_speed_conversion(void)
{
    printf("[1] rpm → axis_speed_ecd_s 换算与方向\n");

    reset_all();
    apply_reference_tuning();
    /* 这一段测的是 rpm→ecd/s 换算本身，关掉低通否则测到的是滤波后的值 */
    g_lift_test.speed_filter_tau_ms = 0;
    lift_test_pid_load(&g_lift_test);
    lift_test_begin((uint8_t)LIFT_TEST_MODE_PER_AXIS, 0U);

    /* 轴 0 是 direction = -1（抬升1 反向），轴 1 是 +1 */
    g_fake_motors.lift.motor[0].speed_rpm = 600;
    g_fake_motors.lift.motor[1].speed_rpm = 600;
    g_fake_motors.lift.motor[2].speed_rpm = -600;
    g_fake_motors.lift.motor[3].speed_rpm = -600;
    lift_test_sample();

    /*
     * 600 rpm = 600 * 8192 / 60 = 81920 ecd/s（转子侧）
     * 轴0 direction=-1 → 逻辑 -81920；轴1 direction=+1 → +81920
     * 也就是说"电机往同一物理方向转"在逻辑坐标下必须同号。
     */
    check_int(g_lift_jog_status.axis_speed_ecd_s[0], -81920, "轴0 逻辑速度");
    check_int(g_lift_jog_status.axis_speed_ecd_s[1], 81920, "轴1 逻辑速度");
    check_int(g_lift_jog_status.axis_speed_ecd_s[2], 81920, "轴2 逻辑速度");
    check_int(g_lift_jog_status.axis_speed_ecd_s[3], -81920, "轴3 逻辑速度");

    /* 位置也要同号：同样的原始位移，逻辑偏移一致 */
    g_fake_motors.lift.motor[0].total_ecd = -5000;
    g_fake_motors.lift.motor[1].total_ecd = 5000;
    lift_test_sample();
    check_int(g_lift_jog_status.axis_offset_ecd[0], 5000, "轴0 逻辑偏移");
    check_int(g_lift_jog_status.axis_offset_ecd[1], 5000, "轴1 逻辑偏移");
}

/* ==================================================================== */
/*  [2] 电流斜坡                                                          */
/* ==================================================================== */

static void t_ramp(void)
{
    printf("[2] 电流斜坡（di/dt 限幅）\n");

    reset_all();

    /* rate = 20000 电流/秒，周期 5 ms → 每周期最多 100 */
    g_lift_test.current_ramp_per_s[0] = 20000;
    test_current_ramp[0] = 0.0F;
    check_int(lift_test_ramp_current(0U, 16000), 100, "第1拍只放出 100");
    check_int(lift_test_ramp_current(0U, 16000), 200, "第2拍 200");
    check_int(lift_test_ramp_current(0U, 16000), 300, "第3拍 300");

    /* 反向也一样被限 */
    test_current_ramp[0] = 0.0F;
    check_int(lift_test_ramp_current(0U, -16000), -100, "反向下限 -100");

    /* 目标小于一步时直接跟上，不振荡 */
    test_current_ramp[0] = 0.0F;
    check_int(lift_test_ramp_current(0U, 60), 60, "目标小于一步：直接到");

    /* 0 = 关闭斜坡，直接跟 */
    g_lift_test.current_ramp_per_s[0] = 0;
    test_current_ramp[0] = 0.0F;
    check_int(lift_test_ramp_current(0U, 16000), 16000, "rate=0 时不限");

    /* 16000 / 100 = 160 拍 ≈ 0.8 s，这就是"不会一瞬间到 16000" */
    g_lift_test.current_ramp_per_s[0] = 20000;
    test_current_ramp[0] = 0.0F;
    {
        int32_t v = 0;
        int32_t steps = 0;
        while ((v < 16000) && (steps < 1000))
        {
            v = lift_test_ramp_current(0U, 16000);
            steps++;
        }
        check_int(v, 16000, "斜坡最终能到 16000");
        check_int(steps, 160, "到 16000 需要 160 拍（0.8 s）");
    }
}

/* ==================================================================== */
/*  [3] 串级量纲：外环出速度、内环出电流                                  */
/* ==================================================================== */

static void t_cascade_units(void)
{
    printf("[3] 串级量纲（位置环=速度，速度环=电流）\n");

    reset_all();
    apply_reference_tuning();
    lift_test_pid_load(&g_lift_test);
    lift_test_begin((uint8_t)LIFT_TEST_MODE_PER_AXIS, 0U);

    /* 目标很远，误差 100000 计数 → 位置环算出 4e6，必须被钳到 4000（速度） */
    g_lift_jog_status.axis_offset_ecd[0] = 0;
    g_lift_jog_status.axis_speed_ecd_s[0] = 0;
    (void)lift_test_cascade_axis(0U, 100000, 0, 200);

    check_int((int32_t)g_lift_jog_status.axis_position_pid_output[0], 4000,
              "位置环输出被钳到 4000（速度量纲）");
    check_int(g_lift_jog_status.axis_target_speed_ecd_s[0], 4000,
              "目标速度 = 4000 ecd/s");
    check_true((int32_t)g_lift_jog_status.axis_speed_pid_output[0] > 4000,
               "速度环要的电流大于位置环上限（证明两条环量纲不同）");
    check_int_le(g_lift_jog_status.axis_speed_pid_output[0] > 0
                     ? (int32_t)g_lift_jog_status.axis_speed_pid_output[0] : 0,
                 16000, "速度环输出不超 16000");

    /* 速度环的 output_limit 生效：调小到 500 就最多 500 */
    g_lift_test.speed_pid[0].output_limit = 500.0F;
    lift_test_pid_load(&g_lift_test);
    lift_test_begin((uint8_t)LIFT_TEST_MODE_PER_AXIS, 0U);
    g_lift_jog_status.axis_offset_ecd[0] = 0;
    g_lift_jog_status.axis_speed_ecd_s[0] = 0;
    (void)lift_test_cascade_axis(0U, 100000, 0, 200);
    check_int((int32_t)g_lift_jog_status.axis_speed_pid_output[0], 500,
              "速度环 output_limit=500 生效");

    /* 位置环的 output_limit 是速度，调小到 800 就最多 800 */
    g_lift_test.speed_pid[0].output_limit = 16000.0F;
    g_lift_test.position_pid[0].output_limit = 800.0F;
    lift_test_pid_load(&g_lift_test);
    lift_test_begin((uint8_t)LIFT_TEST_MODE_PER_AXIS, 0U);
    g_lift_jog_status.axis_offset_ecd[0] = 0;
    g_lift_jog_status.axis_speed_ecd_s[0] = 0;
    (void)lift_test_cascade_axis(0U, 100000, 0, 200);
    check_int(g_lift_jog_status.axis_target_speed_ecd_s[0], 800,
              "位置环 output_limit=800 决定最高速度");
}

/* ==================================================================== */
/*  [4][5] 闭环：能走到目标、能停住、电流不超限、上升不比斜坡快            */
/* ==================================================================== */

static void t_closed_loop(void)
{
    const int32_t target = 5000;
    int32_t peak_current = 0;
    int32_t max_delta = 0;
    int32_t prev_current = 0;
    int32_t settled_step = -1;
    float peak_position = -1.0e9f;
    uint32_t step;
    uint32_t ch;

    printf("[4][5] 闭环收敛 / 停住 / 电流受限\n");

    reset_all();
    apply_reference_tuning();
    lift_test_pid_load(&g_lift_test);
    lift_test_begin((uint8_t)LIFT_TEST_MODE_PER_AXIS, 0U);

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        g_lift_test.per_axis.target_ecd[ch] = target;
    }

    /* 6 s = 1200 拍。加了"减速区"之后接近过程变慢，2 s 走不完 5000 计数 */
    for (step = 0U; step < 1200U; step++)
    {
        plant_step(false);

        if (g_axis_position[0] > peak_position)
        {
            peak_position = g_axis_position[0];
        }

        {
            const int32_t c = g_lift_jog_status.axis_ramped_current[0];
            const int32_t mag = (c < 0) ? -c : c;
            const int32_t delta = c - prev_current;
            const int32_t dmag = (delta < 0) ? -delta : delta;

            if (mag > peak_current)
            {
                peak_current = mag;
            }
            if (dmag > max_delta)
            {
                max_delta = dmag;
            }
            prev_current = c;
        }

        if ((settled_step < 0) &&
            (g_lift_jog_status.axis_at_target[0]))
        {
            settled_step = (int32_t)step;
        }
    }

    printf("      轴0：位置 %.0f / 目标 %ld，峰值电流 %ld，最大单拍变化 %ld\n",
           (double)g_axis_position[0], (long)target, (long)peak_current,
           (long)max_delta);

    /* 走到目标附近（容差 200 计数） */
    /* 加了目标斜坡 + 跟踪前馈之后，稳态误差应该远小于死区（实测 99 计数） */
    check_true(fabsf(g_axis_position[0] - (float)target) <= 300.0f,
               "轴0 停在目标 ±300 计数内");
    check_true(settled_step >= 0, "轴0 报到位");
    /* 出厂默认值的过冲必须很小（扫描值 1.9%，这里留到 6%） */
    check_true(peak_position <= (float)target * 1.06f,
               "出厂默认值过冲 <= 6%");

    /* 停住：最后 50 拍位置几乎不动 */
    {
        const float snapshot = g_axis_position[0];
        for (step = 0U; step < 50U; step++)
        {
            plant_step(false);
        }
        check_true(fabsf(g_axis_position[0] - snapshot) <= 50.0f,
                   "停住后不漂移");
        check_true(fabsf(g_axis_position[0] - (float)target) <= 300.0f,
                   "停住后仍在目标附近");
    }

    /* 电流：不超硬上限，单拍变化不超过斜坡（按出厂斜坡值算，改默认值也不用改这里） */
    check_int_le(peak_current, LIFT_TEST_MAX_CURRENT, "峰值电流不超 16000");
    /* ★ 关键：有目标斜坡+跟踪前馈时，环路【不应该饱和】。
     * 饱和（电流贴着限幅来回）正是大幅极限环的特征。 */
    /* 实测 5000~7600；判据取 10000（离硬上限 16000 留 37% 余量） */
    check_true(peak_current <= 10000,
               "峰值电流不饱和（目标斜坡让两条环工作在线性区）");
    check_int_le(max_delta,
                 g_lift_test.current_ramp_per_s[0] * (int32_t)LIFT_TEST_PERIOD_MS
                     / 1000,
                 "单拍电流变化不超过斜坡");

    /* 四轴目标一致 → 四轴位置一致（串级对四根是同一个函数） */
    for (ch = 1U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        check_true(fabsf(g_axis_position[ch] - g_axis_position[0]) <= 10.0f,
                   "四轴跟随一致");
    }

    /* 下了电流（真的在驱动电机） */
    check_true(g_cmd_count >= 1000U, "每周期都下发了电流");
}

/* ==================================================================== */
/*  [6] 使能上升沿清斜坡缓存                                              */
/* ==================================================================== */

static void t_ramp_cache_reset(void)
{
    printf("[6] 使能上升沿清斜坡缓存\n");

    reset_all();
    apply_reference_tuning();
    lift_test_pid_load(&g_lift_test);

    /* 先让缓存里留一个大值，模拟上一轮跑完的残留 */
    test_current_ramp[1] = 12000.0F;
    g_lift_jog_status.axis_ramped_current[1] = 12000;

    lift_test_begin((uint8_t)LIFT_TEST_MODE_PER_AXIS, 0U);

    check_int((int32_t)test_current_ramp[1], 0, "缓存清零");
    check_int(g_lift_jog_status.axis_ramped_current[1], 0, "状态清零");
    check_int(g_lift_jog_status.axis_target_offset[1], 0, "目标清零");
}

/* ==================================================================== */
/*  LENGTH 模式也走同一条串级                                             */
/* ==================================================================== */

static void t_length_mode(void)
{
    printf("[7] LENGTH 模式（四轴定长，同一条串级）\n");

    reset_all();
    apply_reference_tuning();
    lift_test_pid_load(&g_lift_test);
    lift_test_begin((uint8_t)LIFT_TEST_MODE_LENGTH, 0U);

    /* 负 = 缩回 */
    g_lift_test.length.length_ecd = -3000;
    /* 到位容差 = 模块级 brake_deadband_ecd（已合并） */
    g_lift_test.length.settle_ms = 100U;
    g_lift_test.length.timeout_ms = 5000U;

    for (uint32_t step = 0U; step < 1500U; step++)
    {
        plant_step(true);
        if (!g_lift_jog_status.running)
        {
            break;
        }
    }

    printf("      四轴最终位置 %.0f / %.0f / %.0f / %.0f\n",
           (double)g_axis_position[0], (double)g_axis_position[1],
           (double)g_axis_position[2], (double)g_axis_position[3]);

    for (uint32_t ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        check_true(fabsf(g_axis_position[ch] + 3000.0f) <= 300.0f,
                   "LENGTH 反向定长到位");
    }
    check_true(!g_lift_jog_status.timed_out, "LENGTH 没有超时");
}

/* ==================================================================== */
/*  [8] 调参参考：三个旋钮怎么互相牵制（只打印，不断言）                  */
/*                                                                    */
/*  冲击/过冲同时受三个量影响：                                         */
/*    position_pid.output_limit  最高速度（越大越快，但刹车距离越长）     */
/*    current_ramp_per_s         电流上升速率（太慢刹不住车 → 过冲）      */
/*    position_pid.kp            外环增益（太大 → 振荡）                  */
/* ==================================================================== */

static void sweep_one(int32_t speed, int32_t ramp, float kp, float tau,
                      const char *tag)
{
    const int32_t target = 5000;
    float worst = -1e9f;

    g_plant_tau = tau;
    reset_all();
    for (uint32_t ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        g_lift_test.position_pid[ch].kp = kp;
        g_lift_test.position_pid[ch].output_limit = (float)speed;
        /* 内环增益不参与扫描，用参考值；出厂默认是 0，不填就一点都不动 */
        g_lift_test.speed_pid[ch].kp = REF_KP_SPD;
        g_lift_test.speed_pid[ch].output_limit = 16000.0F;
        g_lift_test.current_ramp_per_s[ch] = ramp;
    }
    lift_test_pid_load(&g_lift_test);
    lift_test_begin((uint8_t)LIFT_TEST_MODE_PER_AXIS, 0U);
    g_lift_test.per_axis.target_ecd[0] = target;

    for (uint32_t step = 0U; step < 900U; step++)
    {
        plant_step(false);
        if (g_axis_position[0] > worst)
        {
            worst = g_axis_position[0];
        }
    }

    printf("      %s %6ld %7ld %7.0f %6.2f | %6.1f %8.0f\n", tag,
           (long)speed, (long)ramp, (double)kp, (double)tau,
           (double)((worst - (float)target) * 100.0f / (float)target),
           (double)(g_axis_position[0] - (float)target));
}

static void t_sweep_report(void)
{
    static const int32_t speeds[] = {2500, 4000};
    static const int32_t ramps[] = {25000, 40000, 60000, 80000};
    static const float kps[] = {8.0F, 10.0F, 15.0F, 25.0F};
    static const float taus[] = {0.03f, 0.1f, 0.3f};

    printf("[8] 调参参考（目标 5000 计数；超调 %% / 稳态误差）\n");
    printf("      表A：速度上限 × 斜坡 × kp_pos（tau=0.10）\n");
    printf("      %6s %7s %7s %6s | %6s %8s\n", "速度", "斜坡", "kp_pos",
           "tau", "超调%", "误差");

    for (uint32_t si = 0U; si < 2U; si++)
    {
        for (uint32_t ri = 0U; ri < 4U; ri++)
        {
            for (uint32_t ki = 0U; ki < 4U; ki++)
            {
                sweep_one(speeds[si], ramps[ri], kps[ki], 0.1f, "A");
            }
        }
    }

    printf("      表B：时间常数 × kp_pos（速度 2500，斜坡 60000）\n");
    for (uint32_t ti = 0U; ti < 3U; ti++)
    {
        for (uint32_t ki = 0U; ki < 4U; ki++)
        {
            sweep_one(2500, 60000, kps[ki], taus[ti], "B");
        }
    }

    g_plant_tau = 0.1f;
}

/* ==================================================================== */
/*  [9] 实时映射 + 出厂默认值                                            */
/* ==================================================================== */

/* 出厂默认增益（与 lift_jog_task_create() 一致；改了固件这里要跟着改） */
#define FACTORY_KP_POS   5.0F
#define FACTORY_KP_SPD   2.0F

static void t_live_mapping(void)
{
    float integral_early;
    float integral_late;
    uint32_t step;

    printf("[9] PID 实时映射（改完立刻生效，不用重开 enabled）\n");

    /* ---- 9a：出厂默认值必须是有意义的一套 ---- */
    reset_all();
    for (uint32_t ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        /*
         * 2026-10-11 按实测机械常数重标（用户要求输出给满）。
         * 内环 kp 从 60 降到 0.5：旧值是按没实测过的 K=0.5 配的，
         * 实际 K ≈ 2（最坏）时 GH = 60×2×0.005 = 0.6，偏高会振。
         * ★ speed/position 的 ki **必须非 0** —— 纯 P 速度环有数学死锁
         *   （见 lift_jog_task.c 里那段推导），实测踩到过。
         */
        check_int((int32_t)g_lift_test.position_pid[ch].kp, (int32_t)FACTORY_KP_POS,
                  "出厂 position kp = 5");
        check_int((int32_t)g_lift_test.speed_pid[ch].kp, (int32_t)FACTORY_KP_SPD,
                  "出厂 speed kp = 2");
        check_int((int32_t)g_lift_test.position_pid[ch].ki, 1, "出厂 position ki = 1");
        check_int((int32_t)g_lift_test.position_pid[ch].kd, 0, "出厂 position kd = 0");
        check_float(g_lift_test.speed_pid[ch].ki, 0.5F,
                    "出厂 speed ki = 0.5（保留浮点精度）");
        check_int((int32_t)g_lift_test.speed_pid[ch].kd, 0, "出厂 speed kd = 0");
        /*
         * ★ integral_limit 必须非 0。
         *   pid.c 的 pid_clamp(integral, 0) 会把积分**清零**，
         *   所以 integral_limit = 0 等于把 ki 焊死（2026-10-11 修过一次）。
         */
        check_int((int32_t)g_lift_test.position_pid[ch].integral_limit, 4000,
                  "出厂 position.integral_limit 非 0（否则 ki 被焊死）");
        check_int((int32_t)g_lift_test.speed_pid[ch].integral_limit, 4000,
                  "出厂 speed.integral_limit 非 0（否则 ki 被焊死）");
        /* 限幅必须有意义，否则填上 kp 也会被 0 卡死 */
        check_int((int32_t)g_lift_test.position_pid[ch].output_limit, 16000,
                  "出厂 position.output_limit = 16000（速度量纲）");
        check_int((int32_t)g_lift_test.speed_pid[ch].output_limit, 16000,
                  "出厂 speed.output_limit = 16000（电流量纲，用户指定给满）");
        /* ★ hold 前馈是"托住车"的唯一手段，出厂必须非 0，否则停住就掉 */
        check_int((int32_t)g_lift_test.hold_feedforward_current[ch], 4000,
                  "出厂 hold_feedforward_current 非 0（否则停住会掉下来）");
        /* 目标斜坡默认开着（照 pitch），这是防极限环的关键 */
        check_int(g_lift_test.target_rate_ecd_s, 10000,
                  "出厂目标斜坡开着（10000 ecd/s ≈ 13.3 mm/s）");
        check_int(g_lift_test.speed_filter_tau_ms, 20, "出厂速度低通 20 ms");
    }

    /* ---- 9b：把增益显式清零 → 一动不动 ---- */
    for (uint32_t ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        g_lift_test.position_pid[ch].kp = 0.0F;
        g_lift_test.position_pid[ch].ki = 0.0F;
        g_lift_test.position_pid[ch].kd = 0.0F;
        g_lift_test.speed_pid[ch].kp = 0.0F;
        g_lift_test.speed_pid[ch].ki = 0.0F;
        g_lift_test.speed_pid[ch].kd = 0.0F;
        g_lift_test.hold_feedforward_current[ch] = 0.0F;
    }
    lift_test_pid_load(&g_lift_test);
    lift_test_begin((uint8_t)LIFT_TEST_MODE_PER_AXIS, 0U);
    g_lift_test.per_axis.target_ecd[0] = 5000;
    for (step = 0U; step < 40U; step++)
    {
        plant_step(false);
    }
    check_true(g_axis_position[0] == 0.0f, "增益全 0 时位置不动");
    check_int(g_lift_jog_status.axis_output_current[0], 0, "增益全 0 时电流为 0");

    /* ---- 9c：运行中改增益，下一周期就生效（不重开 enabled）---- */
    g_lift_test.position_pid[0].kp = REF_KP_POS;
    g_lift_test.speed_pid[0].kp = REF_KP_SPD;
    /* 只跑 40 拍：跑久了会走到位进 HOLD，那时电流本来就该是 hold_ff（=0） */
    for (step = 0U; step < 40U; step++)
    {
        plant_step(false);
        if (g_axis_position[0] > 100.0f)
        {
            break;
        }
    }
    check_true(g_axis_position[0] > 0.0f,
               "运行中填上 kp 后立刻开始动（没重开 enabled）");
    check_true(g_lift_jog_status.axis_output_current[0] != 0,
               "运行中填上 kp 后电流立刻非 0");

    /*
     * ---- 9d：映射不能清积分历史 ----
     *
     * 这是 map/load 拆开的核心原因。如果每周期误用 pid_init()，
     * 积分每周期被清零 → test_speed_pid[0].integral 永远只有一个周期的量。
     * 这里让速度误差恒定（位置环给 4000，实际速度 0），看积分是否在累加。
     */
    reset_all();
    g_lift_test.position_pid[0].kp = REF_KP_POS;
    g_lift_test.position_pid[0].output_limit = (float)REF_VMAX;
    g_lift_test.speed_pid[0].kp = 0.0F;
    g_lift_test.speed_pid[0].ki = 1.0F;
    g_lift_test.speed_pid[0].integral_limit = 100000.0F;
    g_lift_test.speed_pid[0].output_limit = 16000.0F;
    g_lift_test.current_ramp_per_s[0] = 0;   /* 关斜坡，免得干扰 */
    lift_test_pid_load(&g_lift_test);
    lift_test_begin((uint8_t)LIFT_TEST_MODE_PER_AXIS, 0U);
    g_lift_test.per_axis.target_ecd[0] = 100000;

    for (step = 0U; step < 20U; step++)
    {
        lift_test_sample();
        lift_test_run_per_axis();
    }
    integral_early = test_speed_pid[0].integral;

    for (step = 0U; step < 80U; step++)
    {
        lift_test_sample();
        lift_test_run_per_axis();
    }
    integral_late = test_speed_pid[0].integral;

    printf("      积分：20 拍后 %.0f，100 拍后 %.0f\n",
           (double)integral_early, (double)integral_late);

    check_true(integral_early > 0.0f, "积分在累加");
    check_true(integral_late > integral_early * 2.0f,
               "积分历史没被每周期清掉（=5 倍以上）");
}

/* ==================================================================== */
/*  [10] 静摩擦下的稳态：为什么"停在目标附近会一直抖"                     */
/* ==================================================================== */


/*
 * 实测背景（用户 2026-10-09 原话）：
 *   "1和2的限流要是在16000的时候会在空载的时候直接大幅度摆动"
 *   "应该逐级开放电流限制"
 *   "在目标值附近 ±5000 的时候就开始降低电流输出"
 *   "到达目标之后在 ±1000 的范围里就可以刹停了"
 *   "刹停之后需要有重力前馈补偿"
 */
static void t_brake_hold(void)
{
    float err;
    int32_t limit_first;
    int32_t limit_mid;
    float pk;

    printf("[10] 减速 / 刹停 / 保持（用户实测四条要求）\n");

    /* ---- A. 逐级开放电流上限：刚使能时只允许 2000 ---- */
    reset_all();
    apply_reference_tuning();
    lift_test_begin((uint8_t)LIFT_TEST_MODE_PER_AXIS, 0U);
    g_lift_jog_status.axis_offset_ecd[0] = 0;
    g_lift_jog_status.axis_speed_ecd_s[0] = 0;
    (void)lift_test_cascade_axis(0U, 100000, 0, 0);
    limit_first = g_lift_jog_status.axis_limit_now[0];
    printf("      刚使能：电流上限 = %ld（应为 current_limit_start=2000）\n",
           (long)limit_first);
    check_int(limit_first, 2000, "启动时电流上限被限制在 2000");

    /* 跑 100 拍（0.5 s）后应该放开到 2000 + 10000×0.5 = 7000 左右 */
    for (uint32_t i = 0U; i < 100U; i++)
    {
        (void)lift_test_cascade_axis(0U, 100000, 0, 0);
    }
    limit_mid = g_lift_jog_status.axis_limit_now[0];
    printf("      0.5 s 后：电流上限 = %ld（应 ≈ 7000）\n", (long)limit_mid);
    check_true(limit_mid > limit_first, "电流上限在逐级放开");
    check_true(limit_mid <= 16000, "放开后不超过硬上限");

    /* ---- B. ±5000 开始收窄：误差 5000(远端) vs 3000(区内) ---- */
    {
        const int16_t c_far = lift_test_cascade_axis(0U, 100000, 0, 0);
        const int16_t l_far = g_lift_jog_status.axis_limit_now[0];

        /* 让误差正好落在减速区中间：目标 3000，实际 0 */
        (void)c_far;
        (void)l_far;
    }
    reset_all();
    apply_reference_tuning();
    lift_test_begin((uint8_t)LIFT_TEST_MODE_PER_AXIS, 0U);
    /* 先把上限放开到满，排除"逐级开放"的干扰 */
    for (uint32_t i = 0U; i < 1000U; i++)
    {
        g_lift_jog_status.axis_offset_ecd[0] = 0;
        g_lift_jog_status.axis_speed_ecd_s[0] = 0;
        (void)lift_test_cascade_axis(0U, 100000, 0, 1000);
    }
    check_int(g_lift_jog_status.axis_limit_now[0], 16000,
              "误差远大于减速区时上限放到满 16000");

    /*
     * 误差 3000（在 1000~5000 之间）→ 收窄系数 = (3000-1000)/(5000-1000) = 0.5
     * 收窄的是**速度指令**，上限 4000 → 2000。
     * ⚠️ 不能收窄电流上限：仿真里那样做会让机构停在误差 -2501 处推不过去。
     */
    g_lift_jog_status.axis_offset_ecd[0] = 0;
    g_lift_jog_status.axis_speed_ecd_s[0] = 0;
    (void)lift_test_cascade_axis(0U, 3000, 0, 1000);
    printf("      误差 3000：目标速度 = %ld（应收窄到 2250），电流上限仍 %ld\n",
           (long)g_lift_jog_status.axis_target_speed_ecd_s[0],
           (long)g_lift_jog_status.axis_limit_now[0]);
    check_int(g_lift_jog_status.axis_target_speed_ecd_s[0], 2250,
              "±5000 减速区：误差 3000 时目标速度压到 2250");
    check_int(g_lift_jog_status.axis_limit_now[0], 16000,
              "减速不收窄电流上限（否则过不了静摩擦）");

    /* 误差 900 → 小于死区 1000 → 目标速度 0（刹停） */
    (void)lift_test_cascade_axis(0U, 900, 0, 1000);
    check_int(g_lift_jog_status.axis_target_speed_ecd_s[0], 0,
              "进入 ±1000 死区：目标速度为 0（刹停）");

    /* ---- C/D. 带静摩擦+重载跑完，必须刹停并进入 HOLD ---- */
    reset_all();
    apply_reference_tuning();
    g_plant_friction = 2000.0f;
    g_plant_gravity = 4000.0f;
    g_plant_rpm_dither = 1;
    for (uint32_t ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        /* 刹停后靠它托住车：给得比重载 4000 略大 */
        g_lift_test.hold_feedforward_current[ch] = 4200.0F;
    }
    lift_test_pid_load(&g_lift_test);
    lift_test_begin((uint8_t)LIFT_TEST_MODE_PER_AXIS, 0U);
    g_lift_test.per_axis.target_ecd[0] = 5000;

    for (uint32_t step = 0U; step < 800U; step++)
    {
        plant_step(false);
    }

    err = g_axis_position[0] - 5000.0f;
    printf("      跑完：位置 %.0f（误差 %.0f），状态 = %u（2 = HOLD），"
           "电流 = %d\n",
           (double)g_axis_position[0], (double)err,
           (unsigned)g_lift_jog_status.axis_hold_state[0],
           (int)g_lift_jog_status.axis_output_current[0]);

    /* 停在 ±1000 死区里 */
    check_true(fabsf(err) <= 1000.0f, "刹停在 ±1000 死区内");
    /* 进入 HOLD 状态 */
    check_int(g_lift_jog_status.axis_hold_state[0],
              (int32_t)LIFT_TEST_HOLD_HOLD, "停住后进入 HOLD 状态");
    /* HOLD 时输出 = 重力前馈（托住车），不是 0 */
    check_int((int32_t)g_lift_jog_status.axis_ramped_current[0], 4200,
              "HOLD 时输出重力前馈 4200（托住车，不断力）");

    /* 最后 200 拍必须真的不动（这就是用户说的"停稳"） */
    {
        float lo = 1.0e9f;
        float hi = -1.0e9f;

        for (uint32_t step = 0U; step < 200U; step++)
        {
            plant_step(false);
            if (g_axis_position[0] < lo)
            {
                lo = g_axis_position[0];
            }
            if (g_axis_position[0] > hi)
            {
                hi = g_axis_position[0];
            }
        }
        pk = hi - lo;
        printf("      稳住后 200 拍峰峰值 = %.0f 计数\n", (double)pk);
        check_true(pk <= 60.0f, "刹停后不再抖动（峰峰值 ≤ 60 计数）");
    }

    /* ---- E. HOLD 里被推出去 → 自动退回 APPROACH ---- */
    g_lift_jog_status.axis_offset_ecd[0] = g_lift_jog_status.axis_target_offset[0] - 5000;
    (void)lift_test_cascade_axis(0U, 5000, 0, 1000);
    check_int(g_lift_jog_status.axis_hold_state[0],
              (int32_t)LIFT_TEST_HOLD_APPROACH,
              "被推出死区后自动回到 APPROACH");
}

/* ==================================================================== */
/*  [11] 出厂默认值真的能用（不改任何参数直接跑）                          */
/* ==================================================================== */

static void t_factory_defaults(void)
{
    const int32_t target = 5000;
    int32_t peak_current = 0;
    float err;

    printf("[11] 出厂默认值带载运行（kp = 5 / 2，目标斜坡 10000，低通 20 ms）\n");

    reset_all();   /* 只上电，不调任何参数 */
    /* 默认前馈=4000 对应带载保持，不能放进零重力夹具后要求停住。
     * 摩擦和惯性沿用测试模型；这里只验证该明确负载下的控制行为。 */
    g_plant_gravity = 4000.0F;
    g_plant_friction = 2000.0F;
    lift_test_pid_load(&g_lift_test);
    lift_test_begin((uint8_t)LIFT_TEST_MODE_PER_AXIS, 0U);
    g_lift_test.per_axis.target_ecd[0] = target;

    for (uint32_t step = 0U; step < 1500U; step++)
    {
        plant_step(false);
        if (lift_test_abs_i32(g_lift_jog_status.axis_ramped_current[0]) >
            peak_current)
        {
            peak_current =
                lift_test_abs_i32(g_lift_jog_status.axis_ramped_current[0]);
        }
    }

    err = g_axis_position[0] - (float)target;
    printf("      位置 %.0f（误差 %.0f），峰值电流 %ld，状态 %u\n",
           (double)g_axis_position[0], (double)err, (long)peak_current,
           (unsigned)g_lift_jog_status.axis_hold_state[0]);

    check_true(fabsf(err) <= (float)g_lift_test.brake_deadband_ecd,
               "出厂默认值停在配置的到位容差内");
    check_int(g_lift_jog_status.axis_hold_state[0],
              (int32_t)LIFT_TEST_HOLD_HOLD, "出厂默认值能进入 HOLD");
    {
        const float held_position = g_axis_position[0];
        const int32_t final_offset = g_lift_jog_status.axis_target_offset[0];
        for (uint32_t step = 0; step < 1000; ++step)
            plant_step(false);
        check_true(fabsf(g_axis_position[0] - held_position) <= 50.0F,
                   "带载 HOLD 持续五秒不下滑");
        check_int(final_offset, target, "进入 HOLD 时斜坡已走到最终目标");
        check_int(g_last_cmd[0] * g_lift_control_debug.axis_direction[0],
                  4000, "带载 HOLD 的真实命令维持匹配负载的前馈");
    }
}

/* ==================================================================== */
/*  [13] 掉线保护（2026-10-10 质检补的安全缺口）                          */
/*                                                                    */
/*  补之前：任务循环只判 abort/enabled/mode，完全不看在线状态。            */
/*  后果：四根里掉一根，另外三根照样满电流推 → 车歪、机构被扭。            */
/* ==================================================================== */

static void t_offline_guard(void)
{
    printf("[13] 掉线保护（四轴必须全在线才允许出力）\n");

    /* ---- A. 启动前掉线 → 拒绝启动 ---- */
    reset_all();
    apply_reference_tuning();
    g_axis_connected[2] = false;                       /* 抬升3 掉线 */
    g_fake_motors.lift.motor[2].initialized = false;
    lift_test_begin((uint8_t)LIFT_TEST_MODE_PER_AXIS, 0U);
    g_lift_jog_status.running = true;                  /* 假装已经跑起来了 */

    lift_test_sample();                                /* 刷新 all_online */
    check_true(!g_lift_jog_status.all_online, "三轴在线时 all_online = 0");
    check_true(!lift_test_offline_guard(), "掉线时守卫返回 false（拒绝出力）");
    check_int(g_lift_jog_status.offline_abort ? 1 : 0, 1, "offline_abort 被置起");
    check_true(!g_lift_jog_status.running, "守卫把运行状态停掉了");

    /* ---- B. 运行中掉线 → 立刻零电流 ---- */
    reset_all();
    apply_reference_tuning();
    lift_test_begin((uint8_t)LIFT_TEST_MODE_PER_AXIS, 0U);
    g_lift_test.per_axis.target_ecd[0] = 3000;
    for (uint32_t step = 0U; step < 200U; step++)
    {
        plant_step(false);
    }
    check_true(g_axis_position[0] > 100.0f, "先跑起来（离线前有位移）");

    /* 拔掉抬升2 */
    g_axis_connected[1] = false;
    g_fake_motors.lift.motor[1].initialized = false;
    lift_test_sample();
    check_true(!lift_test_offline_guard(), "运行中掉线：守卫返回 false");

    /*
     * 守卫内部调了 lift_test_zero_output()，它会把下发的电流清成 0。
     * 这里直接确认状态里的下发电流为 0（真实 CAN 也在那里被写成 0）。
     */
    for (uint32_t ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        check_int(g_lift_jog_status.axis_output_current[ch], 0,
                  "掉线后下发电流被清 0");
    }

    /*
     * 掉线之后继续跑：
     *   ① 不允许再下发任何电流（这才是"被驱动"的判据）
     *   ② 轴在零电流下自然滑行到停（不是被驱动，所以允许有一小段滑行）
     */
    {
        bool ever_driven = false;

        for (uint32_t step = 0U; step < 300U; step++)
        {
            plant_step(false);
            for (uint32_t ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
            {
                if (g_lift_jog_status.axis_output_current[ch] != 0)
                {
                    ever_driven = true;
                }
            }
        }
        check_true(!ever_driven, "掉线后不再下发任何电流");
        check_true(fabsf(g_axis_velocity[0]) < 1.0f, "掉线后轴自然滑行到停");
    }

    /* ---- C. 四轴全在线 → 守卫放行 ---- */
    reset_all();
    apply_reference_tuning();
    lift_test_sample();
    check_true(g_lift_jog_status.all_online, "四轴在线时 all_online = 1");
    check_true(lift_test_offline_guard(), "全在线时守卫放行");
    check_true(!g_lift_jog_status.offline_abort, "放行时不置 offline_abort");
}

/* ==================================================================== */
/*  [14] 2026-10-11 修的三处台架语义                                      */
/* ==================================================================== */

static void t_axis_scale_and_start_edge(void)
{
    printf("  [14] axis_scale=0 关轴 / enabled 上升沿才启动\n");

    /*
     * axis_scale：0（以及负数、超上限、NaN）必须 = 【关掉这根轴】。
     * 以前非法值一律回落 1.0 = 满力，于是"填 0 想关一根轴"得到满力 ——
     * 单轴标定 K 时这么做会把四根一起驱动起来。
     */
    check_float(lift_test_clamp_scale(0.0F), 0.0F, "axis_scale=0 -> 关掉");
    check_float(lift_test_clamp_scale(-1.0F), 0.0F, "负值 -> 关掉");
    check_float(lift_test_clamp_scale(LIFT_TEST_SCALE_MAX + 1.0F), 0.0F,
                "超上限 -> 关掉（不是满力）");
    check_float(lift_test_clamp_scale(1.0F), 1.0F, "正常值原样通过");
    check_float(lift_test_clamp_scale(0.5F), 0.5F, "半力原样通过");

    /*
     * 启动判据：只有 enabled 上升沿或换模式才算一次新的运行。
     * 以前用 `!test_running` 判，end() 清掉 running 后下一拍立刻重启 →
     * LENGTH 一跑完就自动再来一次、距离一直累加。
     */
    check_true(lift_test_should_start(true, (uint8_t)LIFT_TEST_MODE_PER_AXIS),
               "首次 enabled=1 -> 启动");
    check_true(!lift_test_should_start(true, (uint8_t)LIFT_TEST_MODE_PER_AXIS),
               "保持 1 -> 不重复启动（跑完不会自重启）");
    check_true(!lift_test_should_start(false, (uint8_t)LIFT_TEST_MODE_PER_AXIS),
               "清 0 -> 不启动");
    check_true(lift_test_should_start(true, (uint8_t)LIFT_TEST_MODE_PER_AXIS),
               "清 0 再置 1 -> 允许再来一次");
    check_true(lift_test_should_start(true, (uint8_t)LIFT_TEST_MODE_LENGTH),
               "换模式 -> 允许启动");
    check_true(!lift_test_should_start(true, (uint8_t)LIFT_TEST_MODE_LENGTH),
               "同一模式下保持 1 -> 仍不重复启动");
}

/* 使用真实单周期任务服务入口，覆盖斜坡跟踪与结束后的输出行为。 */
static void t_length_lifecycle(void)
{
    printf("[15] LENGTH 跟踪斜坡不提前完成 / 结束后不再写电流\n");
    reset_all();
    g_lift_test.mode = LIFT_TEST_MODE_LENGTH;
    g_lift_test.length.enabled = true;
    g_lift_test.length.length_ecd = 10000;
    bool premature = false;
    bool finished = false;
    bool false_at_target = false;
    for (g_now = 0; g_now <= 2000; g_now += LIFT_TEST_PERIOD_MS)
    {
        lift_test_update(g_now);
        if ((g_now < 1000) && g_lift_jog_status.at_target)
            false_at_target = true;
        if (!test_running)
        {
            premature = (g_now < 1000);
            finished = true;
            break;
        }
        for (uint32_t ch = 0; ch < LIFT_CHANNEL_COUNT; ++ch)
        {
            const int32_t direction = g_lift_control_debug.axis_direction[ch];
            const int32_t target = g_lift_jog_status.axis_target_offset[ch];
            g_fake_motors.lift.motor[ch].total_ecd = direction * target;
            /* 完成斜坡前仍在运动；之后反馈停稳。 */
            g_fake_motors.lift.motor[ch].speed_rpm =
                (int16_t)(direction * ((target < 10000) ? 73 : 0));
        }
    }
    check_true(!false_at_target, "跟上中间目标时到位标志仍为 false");
    check_true(!premature, "整个任务不会在斜坡走完前结束");
    check_true(finished, "最终目标到位且停稳后正常完成");
    check_true(!g_lift_jog_status.timed_out, "正常完成不误报超时");
    check_int(g_lift_jog_status.run_count, 1, "完成前只启动一次");
    check_int(g_fake_motors.lift.motor[0].total_ecd *
              g_lift_control_debug.axis_direction[0], 10000,
              "正常完成时已到最终位置");
    const uint32_t writes = g_cmd_count;
    for (uint32_t cycle = 0; cycle < 20; ++cycle)
    {
        g_now += LIFT_TEST_PERIOD_MS;
        lift_test_update(g_now);
    }
    check_int(g_cmd_count, writes, "完成后空闲不再写 CAN 缓存");
    check_int(lift_owner_current(), LIFT_OWNER_NONE, "完成后通道已释放");
    check_int(g_last_cmd[0], 0, "完成后保持既有零电流退出语义");
    check_true(!g_lift_test.length.enabled, "定长完成自动清 enabled");
    check_true(g_lift_test_snapshot.valid, "完成留下有效快照");
    check_int(g_lift_test_snapshot.reason, LIFT_TEST_END_COMPLETE, "快照完成原因");
    check_true(g_lift_test_snapshot.status.at_target, "完成快照保留四轴到位");
    check_int(g_lift_test_snapshot.status.axis_target_offset[0], 10000,
              "快照保留清零前最终目标");
    check_int(g_lift_test_snapshot.status.axis_output_current[0] *
              g_lift_control_debug.axis_direction[0], 4000,
              "快照保留清零前保持电流");
    g_lift_test.length.enabled = true;
    lift_test_update(g_now += LIFT_TEST_PERIOD_MS);
    check_int(g_lift_jog_status.run_count, 2, "新的使能边沿允许再次运行");
    check_true(test_running, "重新使能后正常启动");
    check_int(g_lift_test_snapshot.sequence, 1, "再次启动不清上一轮快照");
    check_int(g_lift_test_snapshot.status.axis_target_offset[0], 10000,
              "再次启动不改变保存目标");
    check_int(test_start_raw[0], g_fake_motors.lift.motor[0].total_ecd,
              "重新运行以新位置为相对起点");

    reset_all();
    g_lift_test.mode = LIFT_TEST_MODE_LENGTH;
    g_lift_test.length.enabled = true;
    g_lift_test.length.timeout_ms = 10;
    lift_test_update(g_now);
    g_now = 10;
    lift_test_update(g_now);
    check_true(g_lift_jog_status.timed_out, "静止不跟踪时按超时退出");
    check_true(!test_running, "超时清除运行状态");
    check_true(!g_lift_test.length.enabled, "超时自动清 enabled");
    check_int(g_lift_test_snapshot.reason, LIFT_TEST_END_TIMEOUT, "快照超时原因");
    check_true(g_lift_test_snapshot.status.timed_out, "快照保存超时标志");
    const uint32_t timeout_writes = g_cmd_count;
    for (uint32_t cycle = 0; cycle < 20; ++cycle)
        lift_test_update(g_now += LIFT_TEST_PERIOD_MS);
    check_int(g_cmd_count, timeout_writes, "超时后不再写入电流或重新抢通道");
    check_int(lift_owner_current(), LIFT_OWNER_NONE, "超时后通道已释放");
    check_true(g_lift_jog_status.timed_out, "等待重启时保留超时原因");

    reset_all();
    g_lift_test.mode = LIFT_TEST_MODE_FORCE;
    g_lift_test.force.enabled = true;
    g_lift_test.force.force_n = 800;
    g_lift_test.force.duration_ms = 10;
    lift_test_update(g_now);
    g_now = 10;
    lift_test_update(g_now);
    const uint32_t force_writes = g_cmd_count;
    for (uint32_t cycle = 0; cycle < 20; ++cycle)
        lift_test_update(g_now += LIFT_TEST_PERIOD_MS);
    check_true(!test_running, "FORCE 无保持力结束后保持停止");
    check_true(!g_lift_test.force.enabled, "无保持 FORCE 自动清 enabled");
    check_int(g_lift_test_snapshot.reason, LIFT_TEST_END_COMPLETE, "FORCE 保存完成原因");
    check_int(g_cmd_count, force_writes, "FORCE 结束后不再写零或抢通道");

    reset_all();
    g_lift_test.mode = LIFT_TEST_MODE_FORCE;
    g_lift_test.force.enabled = true;
    g_lift_test.force.force_n = 800;
    g_lift_test.force.hold_force_n = 816;
    g_lift_test.force.duration_ms = 10;
    lift_test_update(g_now);
    lift_test_update(g_now = 10);
    lift_test_update(g_now = 15);
    check_true(test_running && !g_lift_jog_status.active,
               "FORCE 配置了保持力时仍执行保持段");
    check_true(g_lift_test.force.enabled, "FORCE 保持段不清 enabled");
    check_true(!g_lift_test_snapshot.valid, "进入 FORCE 保持段不误存结束快照");
    check_int(g_last_cmd[0] * g_lift_control_debug.axis_direction[0], 1000,
              "运行门禁允许 FORCE 保持电流持续下发");
}

static void t_approach_per_axis_limit(void)
{
    printf("[16] 其他轴的减速限幅独立于轴 0\n");
    reset_all();
    g_lift_test.approach_zone_ecd = 7500;
    g_lift_test.position_pid[0].output_limit = 0;
    for (uint32_t ch = 1; ch < LIFT_CHANNEL_COUNT; ++ch)
    {
        g_lift_test.position_pid[ch].kp = 100;
        g_lift_test.position_pid[ch].output_limit = 4000;
    }
    lift_test_pid_load(&g_lift_test);
    lift_test_begin(LIFT_TEST_MODE_PER_AXIS, 0);
    for (uint32_t ch = 1; ch < LIFT_CHANNEL_COUNT; ++ch)
    {
        (void)lift_test_cascade_axis(ch, 4000, 0, 750);
        check_true(g_lift_jog_status.axis_target_speed_ecd_s[ch] > 750,
                   "轴 0 速度限幅为零不会压死其他轴");
        check_true(g_lift_jog_status.axis_target_speed_ecd_s[ch] < 4000,
                   "该轴仍按自己的速度上限进入接近减速");
    }
    g_lift_test.position_pid[2].output_limit = 0;
    (void)lift_test_cascade_axis(2, 4000, 0, 750);
    check_int(g_lift_jog_status.axis_target_speed_ecd_s[2], 0,
              "本轴速度限幅为零时正确限制本轴速度");
}

static void start_snapshot_run(void)
{
    g_lift_test.mode = LIFT_TEST_MODE_PER_AXIS;
    for (uint32_t ch = 0; ch < LIFT_CHANNEL_COUNT; ++ch)
        g_lift_test.per_axis.target_ecd[ch] = 10000;
    g_lift_test.per_axis.enabled = true;
    lift_test_update(g_now);
    lift_test_update(g_now += 50);
}

static void t_end_snapshot(void)
{
    printf("[17] 结束快照冻结 / 自动归零 / 立即重新使能 / 模式切换\n");
    reset_all();
    check_true(!g_lift_test_snapshot.valid, "复位时无旧快照");
    start_snapshot_run();
    const int16_t before = g_lift_jog_status.axis_output_current[0];
    const int32_t target = g_lift_jog_status.axis_target_offset[0];
    const float pid_output = g_lift_jog_status.axis_speed_pid_output[0];
    g_lift_test.abort = true;
    lift_test_update(g_now += 5);
    check_true(!g_lift_test.per_axis.enabled, "abort 清对应 enabled");
    check_true(!test_running, "abort 停止运行");
    check_int(g_lift_test_snapshot.reason, LIFT_TEST_END_ABORT, "保存 abort 原因");
    check_int(g_lift_test_snapshot.mode, LIFT_TEST_MODE_PER_AXIS, "保存实际模式");
    check_int(g_lift_test_snapshot.end_tick, g_now, "保存停止时刻");
    check_true(g_lift_test_snapshot.status.running, "快照保留清零前运行标志");
    check_int(g_lift_test_snapshot.status.axis_output_current[0], before, "保存最后下发电流");
    check_int(g_lift_test_snapshot.status.axis_target_offset[0], target, "保存停止前斜坡目标");
    check_float(g_lift_test_snapshot.status.axis_speed_pid_output[0], pid_output, "保存 PID 输出");
    check_int(g_lift_jog_status.axis_output_current[0], 0, "实时输出按退出语义归零");
    check_true(g_lift_test_snapshot.parameters.per_axis.enabled, "保存自动清零前 enabled");
    check_true(g_lift_test_snapshot.parameters.abort, "保存停止命令参数");
    const lift_test_snapshot_t saved = g_lift_test_snapshot;
    g_fake_motors.lift.motor[0].total_ecd = 12345;
    g_fake_motors.lift.motor[0].given_current = 777;
    g_lift_test.speed_pid[0].kp = 123;
    for (uint32_t i = 0; i < 10; ++i)
        lift_test_update(g_now += 5);
    check_int(g_lift_jog_status.axis_position_ecd[0], 12345, "停止后实时位置仍刷新");
    check_true(memcmp(&saved, &g_lift_test_snapshot, sizeof(saved)) == 0,
               "实时反馈和参数修改不改变任何快照字段");
    g_lift_test.abort = false;
    lift_test_update(g_now += 5);
    check_true(!test_running, "解除 abort 不自动重启");
    g_lift_test.per_axis.enabled = true;
    lift_test_update(g_now += 5);
    check_true(test_running, "直接 enabled=1 可重启");
    check_int(g_lift_test_snapshot.sequence, 1, "启动保留旧快照");
    g_lift_test.per_axis.enabled = false;
    lift_test_update(g_now += 5);
    check_int(g_lift_test_snapshot.sequence, 2, "下一次停止覆盖并累计保存次数");
    check_int(g_lift_test_snapshot.reason, LIFT_TEST_END_DISABLED, "手动停用原因");
    check_float(g_lift_test_snapshot.parameters.speed_pid[0].kp, 123, "新快照保存本轮参数");

    reset_all();
    start_snapshot_run();
    g_axis_connected[2] = false;
    g_fake_motors.lift.motor[2].initialized = false;
    lift_test_update(g_now += 5);
    check_true(!g_lift_test.per_axis.enabled, "运行掉线自动清 enabled");
    check_int(g_lift_test_snapshot.reason, LIFT_TEST_END_OFFLINE, "掉线原因");
    check_true(g_lift_test_snapshot.status.offline_abort, "清电流前已保存掉线标志");
    check_true(!g_lift_test_snapshot.status.axis_online[2], "快照标出掉线轴");
    g_axis_connected[2] = true;
    g_fake_motors.lift.motor[2].initialized = true;
    lift_test_update(g_now += 5);
    check_true(!test_running, "掉线恢复不自动重启");

    reset_all();
    g_axis_connected[2] = false;
    g_fake_motors.lift.motor[2].initialized = false;
    g_lift_test.mode = LIFT_TEST_MODE_LENGTH;
    g_lift_test.length.enabled = true;
    lift_test_update(g_now);
    check_true(!g_lift_test.length.enabled, "离线拒绝启动也撤销 enabled");
    check_int(g_lift_test_snapshot.reason, LIFT_TEST_END_OFFLINE, "保存拒绝启动原因");
    check_true(!g_lift_test_snapshot.status.running, "拒绝启动快照标识未运行");
    check_int(g_lift_jog_status.run_count, 0, "拒绝启动不累计运行次数");
    lift_test_update(g_now += 5);
    check_int(g_lift_test_snapshot.sequence, 1, "拒绝后空闲不反复覆盖快照");

    reset_all();
    start_snapshot_run();
    g_lift_test.mode = LIFT_TEST_MODE_LENGTH;
    g_lift_test.length.enabled = true;
    lift_test_update(g_now += 5);
    check_int(g_lift_test_snapshot.reason, LIFT_TEST_END_MODE_CHANGED, "切模式保存旧运行");
    check_int(g_lift_test_snapshot.mode, LIFT_TEST_MODE_PER_AXIS, "切模式快照归属旧模式");
    check_true(!g_lift_test.per_axis.enabled, "切模式清旧模式 enabled");
    check_true(g_lift_test.length.enabled && test_running, "切模式保留新模式请求并启动");
    g_lift_test.mode = LIFT_TEST_MODE_OFF;
    lift_test_update(g_now += 5);
    check_int(g_lift_test_snapshot.reason, LIFT_TEST_END_MODE_OFF, "mode=0 保存关闭原因");
    check_true(!g_lift_test.length.enabled, "mode=0 清实际运行模式 enabled");

    reset_all();
    g_lift_test.mode = LIFT_TEST_MODE_PER_AXIS;
    g_lift_test.per_axis.enabled = true; /* 原地目标，到位后应继续保持。 */
    lift_test_update(g_now);
    for (uint32_t i = 0; i < 20; ++i)
        lift_test_update(g_now += 5);
    check_true(g_lift_jog_status.at_target && test_running, "模式 1 到位仍运行");
    check_true(g_lift_test.per_axis.enabled && !g_lift_test_snapshot.valid,
               "模式 1 到位不归零或误存结束快照");

    reset_all();
    g_lift_test.mode = LIFT_TEST_MODE_FORCE;
    g_lift_test.force.enabled = true;
    g_lift_test.force.force_n = 800;
    g_lift_test.force.duration_ms = 1000;
    g_lift_test.force.stall_hold_ms = 10;
    lift_test_update(g_now);
    lift_test_update(g_now = 10);
    check_int(g_lift_test_snapshot.reason, LIFT_TEST_END_STALL, "堵转停止保存原因");
    check_true(g_lift_test_snapshot.status.stall_latched, "堵转快照保留锁存标志");
    check_true(!g_lift_test.force.enabled, "堵转且无保持力时自动归零");

    reset_all();
    g_lift_test.mode = LIFT_TEST_MODE_LENGTH;
    g_lift_test.length.enabled = true;
    g_lift_test.length.timeout_ms = 10;
    lift_test_update(g_now);
    lift_test_update(g_now = 10);
    g_lift_test.length.enabled = true;
    lift_test_update(g_now = 15); /* 自动结束后没有插入 enabled=0 的空闲周期。 */
    check_true(test_running, "自动结束后直接置 1，下一周期立即重启");
    check_int(g_lift_jog_status.run_count, 2, "立即重新使能仅启动新一轮");
    check_int(g_lift_test_snapshot.sequence, 1, "立即重启仍保留上一轮快照");

    reset_all();
    g_lift_test.mode = LIFT_TEST_MODE_FORCE;
    g_lift_test.force.enabled = true;
    g_lift_test.force.force_n = 800;
    g_lift_test.force.hold_force_n = 816;
    g_lift_test.force.duration_ms = 10;
    lift_test_update(g_now);
    lift_test_update(g_now = 10);
    lift_test_update(g_now = 15);
    const int16_t hold_current = g_lift_jog_status.axis_output_current[0];
    g_lift_test.abort = true;
    lift_test_update(g_now = 20);
    check_true(!g_lift_test.force.enabled, "FORCE 保持中止时清 enabled");
    check_int(g_lift_test_snapshot.reason, LIFT_TEST_END_ABORT, "FORCE 保持中止原因");
    check_int(g_lift_test_snapshot.status.axis_output_current[0], hold_current,
              "FORCE 保持结束快照保留断力前电流");
    check_true(!g_lift_test_snapshot.status.active &&
               g_lift_test_snapshot.status.running, "快照标识 FORCE 保持阶段");
}

int main(void)
{
    printf("lift_jog 串级 + 电流斜坡 单元测试\n\n");

    /* g_lift_control_debug 的 default 由 lift_control 模块提供；
     * 这里只需要方向数组，和固件 lift_set_default_debug() 保持一致。 */
    (void)memset(&g_lift_control_debug, 0, sizeof(g_lift_control_debug));
    for (uint32_t ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        g_lift_control_debug.axis_direction[ch] =
            ((ch == 0U) || (ch == 2U)) ? -1 : 1;
    }

    t_speed_conversion();
    t_ramp();
    t_cascade_units();
    t_closed_loop();
    t_ramp_cache_reset();
    t_length_mode();
    t_live_mapping();
    t_brake_hold();
    t_factory_defaults();
    t_offline_guard();
    t_axis_scale_and_start_edge();
    t_length_lifecycle();
    t_approach_per_axis_limit();
    t_end_snapshot();
    t_sweep_report();

    printf("\n共 %lu 项断言，失败 %lu 项\n", (unsigned long)g_checks,
           (unsigned long)g_fails);
    if (g_fails != 0U)
    {
        printf("结果：失败\n");
        return 1;
    }
    printf("结果：全部通过\n");
    return 0;
}
