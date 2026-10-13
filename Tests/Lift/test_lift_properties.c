/*
 * 抬升模块的性质测试（property-based / randomized invariants）。
 *
 * 为什么要有这一套：
 *   前面九个套件都是【手写用例】——它们只覆盖我想到的情况。
 *   这一套改成对【不变量】做出断言，然后随机撒输入，让机器去找反例。
 *
 * 每条不变量都是"无论输入是什么都必须成立"的命题，例如：
 *   · 反变换的输出，翘曲分量必须恒为 0
 *   · 安全闸的输出绝对值不得超过 current_limit
 *   · 一旦进入 FAULT，输出必须全为 0
 *   · 找零模块只在 SEEK/BACKOFF 状态下才允许输出非零电流
 *   · 状态机只有在"正在控制"时才允许 command_valid 为真
 *   · 停机动作只有在"车确实有支撑"时才允许退化成 CUT
 *
 * 随机源是确定性的（xorshift + 固定种子），所以失败可复现：
 * 把种子填回 SEED 就能重放同一条序列。
 *
 * 编译：见 run_tests.ps1。该文件只用于主机验证，不进 Keil 固件工程。
 */

#include "lift_control.h"
#include "lift_geometry.h"
#include "lift_guard.h"
#include "lift_homing.h"
#include "lift_seq.h"
#include "lift_sync.h"
#include "lift_top.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

static int g_checks;
static int g_failures;

#define CHECK(cond, msg)                                            \
    do                                                              \
    {                                                               \
        g_checks++;                                                 \
        if (!(cond))                                                \
        {                                                           \
            g_failures++;                                           \
            printf("    FAIL  %s   (line %d)\n", (msg), __LINE__);  \
        }                                                           \
    } while (0)

#define N LIFT_CHANNEL_COUNT
#define SEED 0x1A2B3C4DU

static uint32_t g_rng;

static void rnd_seed(uint32_t seed)
{
    g_rng = seed;
}

static uint32_t rnd_u32(void)
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng << 5;
    return g_rng;
}

static int32_t rnd_i32(int32_t lo, int32_t hi)
{
    const uint32_t span = (uint32_t)(hi - lo + 1);
    return lo + (int32_t)(rnd_u32() % span);
}

static int32_t i32_abs(int32_t v)
{
    return v < 0 ? -v : v;
}

/* ---------------------------------------------------------------- sync */

static void test_sync_invariants(void)
{
    lift_sync_config_t cfg;
    lift_sync_effort_t u;
    lift_sync_modes_t back;
    int32_t cur[N];
    int i;

    printf("  [P1] lift_sync：反变换的翘曲分量恒为 0，且正反变换互为伴随\n");
    lift_sync_config_rect(&cfg);
    CHECK(lift_sync_config_is_orthogonal(&cfg), "矩形权重始终正交");

    for (i = 0; i < 4000; i++)
    {
        u.height = rnd_i32(-1000000, 1000000);
        u.roll = rnd_i32(-100000, 100000);
        u.pitch = rnd_i32(-100000, 100000);

        lift_sync_distribute(&cfg, &u, cur);
        CHECK(lift_sync_warp_component(&cfg, cur) == 0,
              "任意模态控制量下翘曲分量严格为 0");

        lift_sync_forward(&cfg, cur, &back);
        CHECK(back.height == (int32_t)N * u.height, "高度往返 = N x U");
        CHECK(back.roll == (int32_t)N * u.roll, "横滚往返 = N x U");
        CHECK(back.pitch == (int32_t)N * u.pitch, "俯仰往返 = N x U");
        CHECK(back.warp == 0, "翘曲往返仍为 0");
    }
}

/* ---------------------------------------------------------------- guard */

static void test_guard_invariants(void)
{
    lift_guard_t g;
    lift_guard_config_t cfg;
    lift_guard_input_t in;
    lift_guard_output_t out;
    uint32_t ch;
    int i;

    printf("  [P2] lift_guard：输出不超限；故障必全零；未找零必全零\n");

    for (ch = 0U; ch < N; ch++)
    {
        cfg.position_min_ecd[ch] = 0;
        cfg.position_max_ecd[ch] = 20000;
        cfg.direction[ch] = 1;
        cfg.descent_current[ch] = -1500;
    }
    cfg.current_limit = 3000;
    cfg.descent_stall_speed_rpm = 50;
    cfg.descent_stall_ms = 100U;
    cfg.descent_timeout_ms = 5000U;

    for (i = 0; i < 4000; i++)
    {
        lift_guard_init(&g, &cfg);

        for (ch = 0U; ch < N; ch++)
        {
            in.position_ecd[ch] = rnd_i32(-5000, 25000);
            in.speed_rpm[ch] = (int16_t)rnd_i32(-3000, 3000);
            in.requested_current[ch] = (int16_t)rnd_i32(-30000, 30000);
            in.online[ch] = (rnd_u32() % 10U) != 0U;
        }
        in.homed = (rnd_u32() % 4U) != 0U;
        in.command_fresh = (rnd_u32() % 2U) != 0U;
        in.reset_request = false;
        in.now_ms = (uint32_t)i * 5U;

        lift_guard_update(&g, &in, &out);

        for (ch = 0U; ch < N; ch++)
        {
            CHECK(i32_abs((int32_t)out.output_current[ch]) <=
                      (int32_t)cfg.current_limit,
                  "输出绝对值不超过 current_limit");
        }

        if (out.state == LIFT_GUARD_STATE_FAULT)
        {
            for (ch = 0U; ch < N; ch++)
            {
                CHECK(out.output_current[ch] == 0, "故障时输出全为 0");
            }
        }

        if (!in.homed)
        {
            for (ch = 0U; ch < N; ch++)
            {
                CHECK(out.output_current[ch] == 0, "未找零时输出全为 0");
            }
        }

        if (!in.online[0] && (out.state != LIFT_GUARD_STATE_FAULT))
        {
            CHECK(false, "任一轴离线却不报故障");
        }
    }
}

/* ---------------------------------------------------------------- homing */

static void test_homing_invariants(void)
{
    lift_homing_t h;
    lift_homing_config_t cfg;
    lift_homing_input_t in;
    lift_homing_output_t out;
    int run;

    printf("  [P3] lift_homing：只有工作状态才允许输出，且不超限\n");

    cfg.seek_current = -800;
    cfg.backoff_current = 800;
    /*
     * 前置段（PREPOSITION）在这里显式关掉：随机性质测试只考察
     * SEEK/BACKOFF 的不变量；前置段另有专门用例覆盖。
     * 不初始化会让这两个字段取栈上垃圾值，测试就变成随机的了。
     */
    cfg.preposition_current = 0;
    cfg.preposition_ecd = 0;
    cfg.preposition_timeout_ms = 3000U;
    cfg.current_limit = 2000;
    cfg.stall_speed_rpm = 50;
    cfg.seek_min_travel_ecd = 1000;
    cfg.stall_hold_ms = 100U;
    cfg.backoff_ecd = 500;
    cfg.seek_timeout_ms = 3000U;
    cfg.backoff_timeout_ms = 3000U;

    for (run = 0; run < 300; run++)
    {
        int32_t pos = rnd_i32(-4000, 40000);
        uint32_t t = 0U;
        int step;

        lift_homing_init(&h, &cfg);

        for (step = 0; step < 200; step++)
        {
            in.position_ecd = pos;
            in.speed_rpm = (int16_t)rnd_i32(-2000, 2000);
            in.online = (rnd_u32() % 8U) != 0U;
            in.start_request = (step == 0);
            in.abort_request = (step == 50) && ((rnd_u32() % 5U) == 0U);
            in.now_ms = t;

            lift_homing_update(&h, &in, &out);

            CHECK(i32_abs((int32_t)out.commanded_current) <=
                      (int32_t)cfg.current_limit,
                  "找零电流不超过 current_limit");

            if ((out.state != LIFT_HOMING_STATE_PREPOSITION) &&
                (out.state != LIFT_HOMING_STATE_SEEK) &&
                (out.state != LIFT_HOMING_STATE_BACKOFF))
            {
                CHECK(out.commanded_current == 0, "非工作状态必须零输出");
            }

            if (out.state == LIFT_HOMING_STATE_DONE)
            {
                CHECK(out.commanded_current == 0, "完成后必须零输出");
            }

            /* 模拟机械按命令运动，撞到死点则速度归零。 */
            pos += (int32_t)out.commanded_current * 2;
            if (pos < -3000) { pos = -3000; }
            if (pos > 40000) { pos = 40000; }

            t += 5U;
        }
    }
}

/* ---------------------------------------------------------------- geometry */

static void test_geometry_invariants(void)
{
    lift_geometry_t g;
    int i;

    printf("  [P4] lift_geometry：往返一致；非法配置一律返回 0\n");

    for (i = 0; i < 3000; i++)
    {
        const int32_t lead = rnd_i32(100, 20000);
        const int32_t um = rnd_i32(-500000, 500000);

        lift_geometry_config_3508_direct(&g, lead);
        g.drive_num = rnd_i32(1, 5);
        g.drive_den = rnd_i32(1, 5);

        if (!lift_geometry_validate(&g))
        {
            CHECK(false, "随机生成的合法配置不应被判非法");
            continue;
        }

        {
            const int32_t ecd = lift_geometry_um_to_ecd(&g, um);
            const int32_t back = lift_geometry_ecd_to_um(&g, ecd);
            /* 两次整除截断，误差上界与导程/传动比有关，给一个宽松但有限的界。 */
            CHECK(i32_abs(back - um) <= (lead * 5 + 5),
                  "往返误差有界");
        }
    }

    lift_geometry_config_3508_direct(&g, 0);
    CHECK(lift_geometry_ecd_to_um(&g, 12345) == 0, "导程为 0 时返回 0");
    CHECK(lift_geometry_um_to_ecd(&g, 12345) == 0, "导程为 0 时反向返回 0");
    CHECK(lift_geometry_speed_to_um_per_s(&g, 100) == 0, "导程为 0 时速度返回 0");
}

/* ---------------------------------------------------------------- control */

static void test_control_invariants(void)
{
    lift_control_t ctl;
    lift_control_config_t cfg = {0};
    lift_control_command_t cmd = {0};
    lift_control_status_t st;
    lift_sync_effort_t effort;
    int32_t pos[N];
    int16_t spd[N];
    uint32_t ch;
    int i;
    bool enabled = false;

    printf("  [P5] lift_control：模态控制量不超 PID 输出上限；到位标志与误差一致\n");

    cfg.pid[LIFT_CONTROL_LOOP_POSITION].kp = 0.1F;
    cfg.pid[LIFT_CONTROL_LOOP_POSITION].ki = 0.0F;
    cfg.pid[LIFT_CONTROL_LOOP_POSITION].kd = 0.0F;
    cfg.pid[LIFT_CONTROL_LOOP_POSITION].integral_limit = 0.0F;
    cfg.pid[LIFT_CONTROL_LOOP_POSITION].output_limit = 3000.0F;
    cfg.pid[LIFT_CONTROL_LOOP_SPEED].kp = 1.0F;
    cfg.pid[LIFT_CONTROL_LOOP_SPEED].ki = 0.0F;
    cfg.pid[LIFT_CONTROL_LOOP_SPEED].kd = 0.0F;
    cfg.pid[LIFT_CONTROL_LOOP_SPEED].integral_limit = 0.0F;
    cfg.pid[LIFT_CONTROL_LOOP_SPEED].output_limit = 3000.0F;
    lift_sync_config_rect(&cfg.sync);
    cfg.ecd_per_rev = 8192;
    cfg.gear_ratio = 1.0F;
    cfg.speed_is_rotor = false;
    cfg.max_target_step_ecd = 0;
    cfg.period_s = 0.005F;
    cfg.settle_tolerance_ecd = 200;

    lift_control_init(&ctl, &cfg);

    for (i = 0; i < 4000; i++)
    {
        for (ch = 0U; ch < N; ch++)
        {
            pos[ch] = rnd_i32(-5000, 30000);
            spd[ch] = (int16_t)rnd_i32(-3000, 3000);
        }
        cmd.enabled = enabled;
        cmd.target_height_ecd = rnd_i32(-40000, 120000);
        cmd.target_roll_ecd = rnd_i32(-20000, 20000);
        cmd.target_pitch_ecd = rnd_i32(-20000, 20000);

        lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);

        if (!cmd.enabled)
        {
            CHECK(effort.height == 0 && effort.roll == 0 && effort.pitch == 0,
                  "未使能时零输出");
        }
        else
        {
            CHECK(i32_abs(effort.height) <= 3001, "高度控制量不超限");
            CHECK(i32_abs(effort.roll) <= 3001, "横滚控制量不超限");
            CHECK(i32_abs(effort.pitch) <= 3001, "俯仰控制量不超限");

            if (st.at_target)
            {
                CHECK(i32_abs(st.error_height_ecd) <= 200, "到位则高度误差在容差内");
                CHECK(i32_abs(st.error_roll_ecd) <= 200, "到位则横滚误差在容差内");
                CHECK(i32_abs(st.error_pitch_ecd) <= 200, "到位则俯仰误差在容差内");
            }
        }

        /* 随机切换使能，专门捶打"首次使能吸附当前位置"这条路径。 */
        if ((rnd_u32() % 20U) == 0U)
        {
            enabled = !enabled;
        }
    }
}

/* ---------------------------------------------------------------- top */

static void test_top_invariants(void)
{
    lift_top_t top;
    lift_top_config_t cfg = {0};
    lift_top_status_t st;
    lift_control_command_t cmd = {0};
    lift_top_input_t in = {0};
    uint32_t ch;
    int i;

    printf("  [P6] lift_top：未找零且未找零中则零输出；故障则零输出\n");

    cfg.homing.seek_current = -800;
    cfg.homing.backoff_current = 800;
    cfg.homing.current_limit = 2000;
    cfg.homing.stall_speed_rpm = 50;
    cfg.homing.seek_min_travel_ecd = 1000;
    cfg.homing.stall_hold_ms = 100U;
    cfg.homing.backoff_ecd = 500;
    cfg.homing.seek_timeout_ms = 30000U;
    cfg.homing.backoff_timeout_ms = 30000U;

    for (ch = 0U; ch < N; ch++)
    {
        cfg.guard.position_min_ecd[ch] = 0;
        cfg.guard.position_max_ecd[ch] = 30000;
        cfg.guard.direction[ch] = 1;
        cfg.guard.descent_current[ch] = -1500;
    }
    cfg.guard.current_limit = 3000;
    cfg.guard.descent_stall_speed_rpm = 50;
    cfg.guard.descent_stall_ms = 100U;
    cfg.guard.descent_timeout_ms = 5000U;

    cfg.control.ecd_per_rev = 8192;
    cfg.control.gear_ratio = 1.0F;
    cfg.control.speed_is_rotor = false;
    cfg.control.pid[LIFT_CONTROL_LOOP_POSITION].kp = 0.1F;
    cfg.control.pid[LIFT_CONTROL_LOOP_POSITION].ki = 0.0F;
    cfg.control.pid[LIFT_CONTROL_LOOP_POSITION].kd = 0.0F;
    cfg.control.pid[LIFT_CONTROL_LOOP_POSITION].integral_limit = 0.0F;
    cfg.control.pid[LIFT_CONTROL_LOOP_POSITION].output_limit = 3000.0F;
    cfg.control.pid[LIFT_CONTROL_LOOP_SPEED].kp = 1.0F;
    cfg.control.pid[LIFT_CONTROL_LOOP_SPEED].ki = 0.0F;
    cfg.control.pid[LIFT_CONTROL_LOOP_SPEED].kd = 0.0F;
    cfg.control.pid[LIFT_CONTROL_LOOP_SPEED].integral_limit = 0.0F;
    cfg.control.pid[LIFT_CONTROL_LOOP_SPEED].output_limit = 3000.0F;
    lift_sync_config_rect(&cfg.control.sync);
    cfg.control.max_target_step_ecd = 0;
    cfg.control.period_s = 0.005F;
    cfg.control.settle_tolerance_ecd = 200;

    cmd.enabled = true;
    cmd.target_height_ecd = 20000;
    cmd.target_roll_ecd = 0;
    cmd.target_pitch_ecd = 0;

    for (i = 0; i < 3000; i++)
    {
        lift_top_init(&top, &cfg);

        {
            int step;
            for (step = 0; step < 40; step++)
            {
                for (ch = 0U; ch < N; ch++)
                {
                    in.position_ecd[ch] = rnd_i32(-4000, 40000);
                    in.speed_rpm[ch] = (int16_t)rnd_i32(-2000, 2000);
                    in.online[ch] = (rnd_u32() % 12U) != 0U;
                }
                in.homing_request = (step == 0);
                in.clear_homing_request = false;
                in.reset_request = false;
                in.command_fresh = (rnd_u32() % 3U) != 0U;
                in.now_ms = (uint32_t)step * 5U;

                lift_top_update(&top, &in, &cmd, &st);

                for (ch = 0U; ch < N; ch++)
                {
                    CHECK(i32_abs((int32_t)st.output_current[ch]) <= 3000,
                          "顶层输出不超过安全闸上限");
                }

                if (st.state == LIFT_TOP_STATE_FAULT)
                {
                    for (ch = 0U; ch < N; ch++)
                    {
                        CHECK(st.output_current[ch] == 0, "顶层故障时零输出");
                    }
                }

                if ((st.state == LIFT_TOP_STATE_IDLE) && !st.homed)
                {
                    for (ch = 0U; ch < N; ch++)
                    {
                        CHECK(st.output_current[ch] == 0,
                              "既未找零也未找零中必须零输出");
                    }
                }
            }
        }
    }
}

/* ---------------------------------------------------------------- seq */

/*
 * 随机化不变量：不管喂什么输入序列，状态机都不能出现"自相矛盾"的组合。
 *
 * 这里【刻意】不假设任何具体流程：全部输入都是随机的（在线/找零/标定/
 * 布局/确认/中止/复位任意翻转），只看三个不变量是否始终成立。
 * 具体的 15 阶段流程由 test_lift_seq.c 的定向用例负责。
 */
static void test_seq_invariants(void)
{
    lift_seq_t s;
    lift_seq_status_t st;
    lift_seq_config_t cfg;
    int i;

    printf("  [P7] lift_seq：command_valid 只在工作中为真；故障必须锁存\n");

    lift_seq_config_default(&cfg);

    for (i = 0; i < 2000; i++)
    {
        int step;

        lift_seq_init(&s, &cfg);

        for (step = 0; step < 120; step++)
        {
            lift_seq_input_t in;

            (void)memset(&in, 0, sizeof(in));
            in.start_request = (step == 0) || ((rnd_u32() % 50U) == 0U);
            in.abort_request = (rnd_u32() % 40U) == 0U;
            in.confirm_request = (rnd_u32() % 6U) == 0U;
            in.release_request = (rnd_u32() % 50U) == 0U;
            in.reset_request = (rnd_u32() % 30U) == 0U;
            in.homed = (rnd_u32() % 8U) != 0U;
            in.calibrated = (rnd_u32() % 4U) != 0U;
            in.layout_confirmed = (rnd_u32() % 4U) != 0U;
            in.guard_fault = (rnd_u32() % 100U) == 0U;
            in.drive_done = (rnd_u32() % 3U) == 0U;
            in.contact_front = (rnd_u32() % 20U) == 0U;
            in.contact_rear = (rnd_u32() % 20U) == 0U;
            in.sync_error_mm = rnd_i32(0, 40);
            in.deviation_max_mm = rnd_i32(0, 40);
            in.now_ms = (uint32_t)step * 5U;
            for (uint32_t ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
            {
                in.axis_online[ch] = (rnd_u32() % 16U) != 0U;
                in.limit_blocked[ch] = (rnd_u32() % 40U) == 0U;
                in.limit_min_ecd[ch] = 0;
                in.limit_max_ecd[ch] = 25000;
                in.position_mm[ch] = rnd_i32(-200, 400);
                in.speed_mm_s[ch] = rnd_i32(-200, 200);
                in.current[ch] = rnd_i32(-16000, 16000);
            }

            lift_seq_update(&s, &in, &st);

            /*
             * 不变量 1：command_valid ⟺ 状态机正在【下发运动命令】。
             *
             * ⚠️ 注意"在控制中"（lift_seq_is_active）与"在下发命令"
             *    不是同一件事：PRECHECK 属于活动状态，但它刻意不下发命令
             *    （没有检查完就不许出力）。所以判据必须用下面这个
             *    "会下发命令的阶段集合"，不能用 is_active。
             */
            {
                const bool issuing =
                    (st.state != LIFT_SEQ_STATE_IDLE) &&
                    (st.state != LIFT_SEQ_STATE_PRECHECK) &&
                    (st.state != LIFT_SEQ_STATE_COMPLETE) &&
                    (st.state != LIFT_SEQ_STATE_FAULT);

                if (st.command_valid)
                {
                    CHECK(issuing,
                          "command_valid 为真时状态机必须在下发命令");
                    CHECK(st.command_enabled, "命令有效时必须使能");
                }
                else
                {
                    CHECK(!issuing,
                          "会下发命令的阶段里 command_valid 不得为假");
                }
            }

            /* 不变量 2：状态索引永远在合法范围内。 */
            CHECK(st.step_index < (uint32_t)LIFT_SEQ_STATE_COUNT,
                  "状态索引不得越界");
            CHECK(st.state == (lift_seq_state_t)st.step_index,
                  "step_index 必须与 state 一致");

            /* 不变量 3：FAULT 必须有非 NONE 的故障码；非 FAULT 必须为 NONE。 */
            if (st.state == LIFT_SEQ_STATE_FAULT)
            {
                CHECK(st.fault != LIFT_SAFETY_FAULT_NONE,
                      "FAULT 状态必须有故障码");
                CHECK(st.fault_axis >= -1, "故障轴号不得小于 -1");
            }
            else
            {
                CHECK(st.fault == LIFT_SAFETY_FAULT_NONE,
                      "非 FAULT 状态不得带故障码");
            }

            /* 不变量 4：只有 FAULT + 悬空 + 未解除时才允许 HOLD/LOWER。 */
            if ((st.stop_action != LIFT_STOP_CUT) &&
                (st.state == LIFT_SEQ_STATE_FAULT))
            {
                CHECK(st.support != LIFT_SUPPORT_GROUND,
                      "有地面支撑时停机动作必须是 CUT");
                CHECK(st.support != LIFT_SUPPORT_MECHANICAL,
                      "有机械支撑时停机动作必须是 CUT");
            }
        }
    }
}

int main(void)
{
    printf("lift 性质测试（随机化不变量，种子 0x%08X）\n", SEED);

    rnd_seed(SEED);

    test_sync_invariants();
    test_guard_invariants();
    test_homing_invariants();
    test_geometry_invariants();
    test_control_invariants();
    test_top_invariants();
    test_seq_invariants();

    printf("\n共 %d 项断言，失败 %d 项\n", g_checks, g_failures);
    if (g_failures == 0)
    {
        printf("结果：全部通过\n");
        return 0;
    }
    printf("结果：存在失败（种子 0x%08X 可复现）\n", SEED);
    return 1;
}
