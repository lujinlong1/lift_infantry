/*
 * lift_top 编排单元测试。
 *
 * 覆盖：未找零门禁 / 四轴找零编排 / 找零失败锁存 / 复位与重新找零 /
 *       已找零后的控制链路 / 命令超时缓降 / 两步重新找零约定。
 *
 * 编译：见 run_tests.ps1。该文件只用于主机验证，不进 Keil 固件工程。
 */

#include "lift_top.h"

#include <stddef.h>
#include <stdio.h>

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

#define STEP_MS 5U
#define N LIFT_CHANNEL_COUNT

#define PLANT_START_ECD    10000
#define PLANT_STOP_MIN     (-3000)
#define PLANT_STOP_MAX     (40000)
/*
 * 机械增益（计数/秒 每电流单位）：正电流 -> 绝对计数增大。
 * 与 test_lift_system 用同一套量级，理由见那边的注释。
 */
#define PLANT_GAIN_ECD_S_PER_CURRENT 200.0
#define PLANT_FULL_CURRENT           16384.0
#define PLANT_SPEED_TIME_CONSTANT_MS 4U
#define PLANT_MAX_STEP     300

typedef struct
{
    int32_t position_ecd[N];
    int16_t speed_rpm[N];
    double  position_accum[N];
    double  speed_ecd_s[N];
    double  ecd_s_per_rpm;
} plant_t;

/*
 * 台架模型：一阶速度环节 + 连续位置积分。
 * 速度反馈按【控制器自己那套换算】折成 rpm 再回报，两者共用
 * lift_control_speed_to_ecd_s()，否则内环会被隐式放大几千倍。
 */
static void plant_init(plant_t *p, const lift_control_config_t *control)
{
    uint32_t ch;
    const float ref = lift_control_speed_to_ecd_s(control, 1000);

    *p = (plant_t){0};
    for (ch = 0U; ch < N; ch++)
    {
        p->position_ecd[ch] = PLANT_START_ECD;
        p->position_accum[ch] = (double)PLANT_START_ECD;
        p->speed_rpm[ch] = 0;
        p->speed_ecd_s[ch] = 0.0;
    }
    p->ecd_s_per_rpm = (ref > 0.0F) ? ((double)ref / 1000.0) : 1.0;
}

/* 一阶速度环节 + 连续位置积分；碰到机械死点则速度归零。 */
static void plant_step(plant_t *p, const int16_t cmd[N])
{
    const double dt = (double)STEP_MS / 1000.0;
    const double alpha =
        dt / ((double)PLANT_SPEED_TIME_CONSTANT_MS / 1000.0);
    uint32_t ch;

    for (ch = 0U; ch < N; ch++)
    {
        const double target_ecd_s =
            (double)cmd[ch] * PLANT_GAIN_ECD_S_PER_CURRENT;
        double step;

        if (alpha >= 1.0)
        {
            p->speed_ecd_s[ch] = target_ecd_s;
        }
        else
        {
            p->speed_ecd_s[ch] +=
                (target_ecd_s - p->speed_ecd_s[ch]) * alpha;
        }

        step = p->speed_ecd_s[ch] * dt;
        if (step > (double)PLANT_MAX_STEP)
        {
            step = (double)PLANT_MAX_STEP;
        }
        if (step < -(double)PLANT_MAX_STEP)
        {
            step = -(double)PLANT_MAX_STEP;
        }

        p->position_accum[ch] += step;
        p->position_ecd[ch] = (int32_t)p->position_accum[ch];
        p->speed_rpm[ch] =
            (int16_t)((p->speed_ecd_s[ch] / p->ecd_s_per_rpm) + 0.5);

        if (p->position_ecd[ch] <= PLANT_STOP_MIN)
        {
            p->position_ecd[ch] = PLANT_STOP_MIN;
            p->position_accum[ch] = (double)PLANT_STOP_MIN;
            p->speed_rpm[ch] = 0;
            p->speed_ecd_s[ch] = 0.0;
        }
        else if (p->position_ecd[ch] >= PLANT_STOP_MAX)
        {
            p->position_ecd[ch] = PLANT_STOP_MAX;
            p->position_accum[ch] = (double)PLANT_STOP_MAX;
            p->speed_rpm[ch] = 0;
            p->speed_ecd_s[ch] = 0.0;
        }
        else
        {
            /* 正常运动。 */
        }
    }
}

static void fill_input(lift_top_input_t *in, const plant_t *p, uint32_t now)
{
    uint32_t ch;

    *in = (lift_top_input_t){0};

    for (ch = 0U; ch < N; ch++)
    {
        in->position_ecd[ch] = p->position_ecd[ch];
        in->speed_rpm[ch] = p->speed_rpm[ch];
        in->online[ch] = true;
    }
    in->homing_request = false;
    in->homing_known_zero_confirmed_mask = 0U;
    in->homing_contact_confirmed_mask = 0U;
    in->clear_homing_request = false;
    in->reset_request = false;
    in->command_fresh = true;
    in->now_ms = now;
}

static bool output_is_zero(const lift_top_status_t *st)
{
    uint32_t ch;
    for (ch = 0U; ch < N; ch++)
    {
        if (st->output_current[ch] != 0)
        {
            return false;
        }
    }
    return true;
}

static lift_top_config_t top_config(void)
{
    lift_top_config_t c = {0};
    uint32_t ch;

    c.homing.seek_current = -2000;   /* 负电流 -> 位置减小 -> 顶向机械下限 */
    c.homing.backoff_current = 2000;
    c.homing.current_limit = 2000;
    c.homing.stall_speed_rpm = 50;
    c.homing.seek_min_travel_ecd = 1000;
    c.homing.stall_hold_ms = 100U;
    c.homing.backoff_ecd = 500;
    c.homing.seek_timeout_ms = 30000U;
    c.homing.backoff_timeout_ms = 30000U;

    for (ch = 0U; ch < N; ch++)
    {
        c.guard.position_min_ecd[ch] = 0;
        c.guard.position_max_ecd[ch] = 30000;
        c.guard.direction[ch] = 1;             /* 正电流使已找零坐标增大 */
        c.guard.descent_current[ch] = -2500;   /* 缓降方向：向下 */
        /* 交替模拟左右两轴的电机/编码器安装方向。 */
        c.axis_direction[ch] = ((ch % 2U) == 0U) ? 1 : -1;
    }
    c.guard.current_limit = 3000;
    c.guard.descent_stall_speed_rpm = 50;
    c.guard.descent_stall_ms = 100U;
    c.guard.descent_timeout_ms = 5000U;

    /*
     * 增益按台架机械常数推导，与 test_lift_system 同一套：
     *   控制器的速度环输入是 ecd/s，因此内环单位增益为 1/G；
     *   模态正变换会把四轴速度相加，纯高度时再除以 4：
     *       speed_kp = 1 / (4G)
     *   外环输出也是 ecd/s，位置误差是 ecd，带宽直接按 1/(5τ) 推导。
     *   用于穿过编码器量化死区。
     */
    {
        const double tau = (double)PLANT_SPEED_TIME_CONSTANT_MS / 1000.0;
        const double pos_kp = 1.0 / (5.0 * tau);
        const double speed_kp = 1.0 /
                                (4.0 * PLANT_GAIN_ECD_S_PER_CURRENT);

        c.control.pid[LIFT_CONTROL_LOOP_POSITION].kp = (float)pos_kp;
        c.control.pid[LIFT_CONTROL_LOOP_POSITION].ki = (float)(pos_kp * 0.5);
        c.control.pid[LIFT_CONTROL_LOOP_POSITION].kd = 0.0F;
        c.control.pid[LIFT_CONTROL_LOOP_POSITION].integral_limit = 1.0e9F;
        c.control.pid[LIFT_CONTROL_LOOP_POSITION].output_limit = 30000.0F;
        c.control.pid[LIFT_CONTROL_LOOP_SPEED].kp = (float)speed_kp;
        c.control.pid[LIFT_CONTROL_LOOP_SPEED].ki = 0.0F;
        c.control.pid[LIFT_CONTROL_LOOP_SPEED].kd = 0.0F;
        c.control.pid[LIFT_CONTROL_LOOP_SPEED].integral_limit = 0.0F;
        c.control.pid[LIFT_CONTROL_LOOP_SPEED].output_limit =
            (float)PLANT_FULL_CURRENT;
    }
    lift_sync_config_rect(&c.control.sync);
    /* 齿轮比与每圈计数是模型的物理参数，按真值填，不当调节旋钮。 */
    c.control.ecd_per_rev = 8192;
    c.control.gear_ratio = 1.0F;
    c.control.speed_is_rotor = false;
    c.control.max_target_step_ecd = 0;
    c.control.period_s = (float)STEP_MS / 1000.0F;
    c.control.settle_tolerance_ecd = 200;

    return c;
}

int main(void)
{
    plant_t plant;
    lift_top_t top;
    lift_top_config_t tcfg = top_config();
    lift_top_status_t st;
    lift_control_command_t cmd = {0};
    lift_top_input_t in;
    uint32_t ch;
    uint32_t t;
    bool reached;

    printf("lift_top 编排单元测试\n");

    cmd.enabled = true;
    cmd.target_height_ecd = 0;
    cmd.target_roll_ecd = 0;
    cmd.target_pitch_ecd = 0;

    plant_init(&plant, &tcfg.control);
    lift_top_init(&top, &tcfg);

    /* ---------- [1] 未找零时，命令再新鲜也不得输出 ---------- */
    printf("  [1] 未找零门禁\n");
    fill_input(&in, &plant, 0U);
    lift_top_update(&top, &in, &cmd, &st);
    CHECK(st.state == LIFT_TOP_STATE_IDLE, "初始为 IDLE");
    CHECK(!st.homed, "尚未找零");
    CHECK(output_is_zero(&st), "输出全零");

    cmd.target_height_ecd = 20000;   /* 上层硬要给目标 */
    fill_input(&in, &plant, STEP_MS);
    lift_top_update(&top, &in, &cmd, &st);
    CHECK(output_is_zero(&st), "未找零时即使有目标也不输出");
    cmd.target_height_ecd = 0;

    /* ---------- [2] 四轴找零编排 ---------- */
    printf("  [2] 四轴找零\n");
    reached = false;
    for (t = 0U; t < 4000U; t += STEP_MS)
    {
        fill_input(&in, &plant, t);
        in.homing_request = (t == 0U);
        lift_top_update(&top, &in, &cmd, &st);
        plant_step(&plant, st.output_current);

        if (st.state == LIFT_TOP_STATE_READY)
        {
            reached = true;
            printf("      READY at t=%u:\n", t);
            for (ch = 0U; ch < N; ch++)
            {
                printf("        ch%u raw=%d zero=%d homed=%d\n",
                       ch, in.position_ecd[ch], st.zero_ecd[ch],
                       st.homed_position_ecd[ch]);
            }
            break;
        }
        if (st.state == LIFT_TOP_STATE_FAULT)
        {
            break;
        }
    }
    CHECK(reached, "找零完成并进入 READY");
    CHECK(st.homed, "homed 置位");
    CHECK(st.fault == LIFT_TOP_FAULT_NONE, "无故障");
    CHECK(lift_top_is_operational(&top), "is_operational 为真");
    for (ch = 0U; ch < N; ch++)
    {
        /*
         * 语义（固件 2026-10-09 变更）：top.zero_ecd 保存在【原始编码器域】，
         * 不经过 axis_direction 折算。逻辑方向只在形成相对位置、或把模态
         * 电流映射回原始电机域时才用。
         *
         * 本台架里 axis_direction=+1 的轴，逻辑向【负】找零 → 原始位置
         * 减小 → 停在原始最小值；direction=-1 的轴原始位置增大 → 停在
         * 原始最大值。
         */
        const int32_t expected_zero = (tcfg.axis_direction[ch] > 0)
                                           ? PLANT_STOP_MIN
                                           : PLANT_STOP_MAX;
        CHECK(top.zero_ecd[ch] == expected_zero, "零点落在原始域的死点上");
        /*
         * 找零最后一步是"退出死点"：机构停在离死点约 backoff_ecd 处。
         * 回退判定量的是"相对回退起点的位移"，转状态那一拍还会再走一步，
         * 所以允许 backoff_ecd ~ backoff_ecd + 一步 的区间。
         */
        CHECK((st.homed_position_ecd[ch] >= (int32_t)tcfg.homing.backoff_ecd) &&
              (st.homed_position_ecd[ch] <=
               (int32_t)tcfg.homing.backoff_ecd + PLANT_MAX_STEP),
              "找零完成时已找零坐标≈回退距离（含一拍余量）");
    }

    /* ---------- [3] 已找零后的控制链路 ---------- */
    printf("  [3] 抬升收敛\n");
    cmd.target_height_ecd = 20000;
    {
        bool checked_current_direction = false;
        bool checked_speed_direction = false;

    for (t = 0U; t < 20000U; t += STEP_MS)
    {
        fill_input(&in, &plant, t);
        lift_top_update(&top, &in, &cmd, &st);

        if (!checked_current_direction && !st.at_target)
        {
            CHECK(st.output_current[0] > 0,
                  "正方向轴上升时输出正电流");
            CHECK(st.output_current[1] < 0,
                  "负方向轴上升时输出反向电流");
            checked_current_direction = true;
        }

        plant_step(&plant, st.output_current);

        if (checked_current_direction && !checked_speed_direction &&
            (st.control.actual_speed.height > 0))
        {
            CHECK(st.control.actual_speed.height > 0,
                  "混合方向轴的逻辑速度反馈仍为正");
            checked_speed_direction = true;
        }
        if (st.at_target)
        {
            break;
        }
    }
    CHECK(checked_current_direction, "完成混合方向电流检查");
    CHECK(checked_speed_direction, "完成混合方向速度反馈检查");
    }
    CHECK(st.at_target, "到达目标");
    CHECK(st.guard_state == LIFT_GUARD_STATE_READY, "安全闸处于 READY");
    for (ch = 0U; ch < N; ch++)
    {
        const int32_t homed = (tcfg.axis_direction[ch] > 0)
                                  ? (plant.position_ecd[ch] - top.zero_ecd[ch])
                                  : (top.zero_ecd[ch] - plant.position_ecd[ch]);
        const int32_t dev = homed - 5000;
        if ((dev > 200) || (dev < -200))
        {
            printf("      ch%u homed=%d (zero_ecd=%d pos=%d dev=%d)\n",
                   ch, homed, top.zero_ecd[ch], plant.position_ecd[ch], dev);
        }
        CHECK((dev <= 200) && (dev >= -200), "每轴收敛到 5000 附近");
    }

    /* ---------- [4] 命令超时 -> 受控缓降 ---------- */
    printf("  [4] 命令超时受控缓降\n");
    for (t = 0U; t < 20000U; t += STEP_MS)
    {
        fill_input(&in, &plant, t);
        in.command_fresh = false;
        lift_top_update(&top, &in, &cmd, &st);
        plant_step(&plant, st.output_current);
        if (st.guard_state == LIFT_GUARD_STATE_PARKED)
        {
            break;
        }
    }
    CHECK(st.guard_state == LIFT_GUARD_STATE_PARKED, "缓降到底并 PARKED");
    CHECK(st.state == LIFT_TOP_STATE_READY, "顶层不因缓降报故障");
    CHECK(st.fault == LIFT_TOP_FAULT_NONE, "无顶层故障");
    for (ch = 0U; ch < N; ch++)
    {
        /*
         * 缓降把机构送回【找零那一端的死点】——也就是原始域里
         * direction=+1 的最小值 / direction=-1 的最大值。
         */
        const int32_t expected_stop = (tcfg.axis_direction[ch] > 0)
                                          ? PLANT_STOP_MIN
                                          : PLANT_STOP_MAX;
        if (plant.position_ecd[ch] != expected_stop)
        {
            printf("      ch%u dir=%d pos=%d expect_stop=%d\n",
                   ch, (int)tcfg.axis_direction[ch],
                   plant.position_ecd[ch], expected_stop);
        }
        CHECK(plant.position_ecd[ch] == expected_stop, "落回逻辑下限对应的死点");
    }

    /* ---------- [5] 已找零时不得直接重新找零（必须两步） ---------- */
    printf("  [5] 重新找零必须两步\n");
    fill_input(&in, &plant, 0U);
    in.homing_request = true;
    lift_top_update(&top, &in, &cmd, &st);
    CHECK(st.state == LIFT_TOP_STATE_READY, "已找零时直接重新找零被忽略");
    CHECK(st.homed, "homed 保持不变");

    fill_input(&in, &plant, STEP_MS);
    in.clear_homing_request = true;
    lift_top_update(&top, &in, &cmd, &st);
    CHECK(!st.homed, "clear 使找零结果失效");
    CHECK(st.state == LIFT_TOP_STATE_IDLE, "回到 IDLE");

    /* ---------- [6] 找零途中单轴离线 -> 整组锁存故障 ---------- */
    printf("  [6] 找零失败必须整组锁存\n");
    plant_init(&plant, &tcfg.control);
    lift_top_init(&top, &tcfg);

    for (t = 0U; t < 2000U; t += STEP_MS)
    {
        fill_input(&in, &plant, t);
        in.homing_request = (t == 0U);
        if (t >= (10U * STEP_MS))
        {
            in.online[2] = false;   /* 找零途中第三轴反馈丢失 */
        }
        lift_top_update(&top, &in, &cmd, &st);
        plant_step(&plant, st.output_current);

        if (st.state == LIFT_TOP_STATE_FAULT)
        {
            break;
        }
    }
    CHECK(st.state == LIFT_TOP_STATE_FAULT, "进入顶层 FAULT");
    CHECK(st.fault == LIFT_TOP_FAULT_HOMING_FAILED, "故障码为 HOMING_FAILED");
    CHECK(output_is_zero(&st), "故障时输出全零");
    CHECK(!lift_top_is_operational(&top), "故障时不可用");

    /* 故障锁存：只是恢复在线、不给 reset，不得自动恢复 */
    fill_input(&in, &plant, t + STEP_MS);
    lift_top_update(&top, &in, &cmd, &st);
    CHECK(st.state == LIFT_TOP_STATE_FAULT, "未复位则保持锁存");

    fill_input(&in, &plant, t + (2U * STEP_MS));
    in.reset_request = true;
    in.recovery_confirmed = true;
    cmd.enabled = false;
    for (ch = 0U; ch < N; ch++) in.speed_rpm[ch] = 0;
    lift_top_update(&top, &in, &cmd, &st);
    CHECK(st.state == LIFT_TOP_STATE_IDLE, "复位后回到 IDLE（尚未找零）");
    CHECK(st.fault == LIFT_TOP_FAULT_NONE, "故障码清零");

    /* ---------- [7] 复位后可重新找零成功 ---------- */
    printf("  [7] 复位后重新找零\n");
    plant_init(&plant, &tcfg.control);
    reached = false;
    for (t = 0U; t < 4000U; t += STEP_MS)
    {
        fill_input(&in, &plant, t);
        in.homing_request = (t == 0U);
        lift_top_update(&top, &in, &cmd, &st);
        plant_step(&plant, st.output_current);

        if (st.state == LIFT_TOP_STATE_READY)
        {
            reached = true;
            break;
        }
        if (st.state == LIFT_TOP_STATE_FAULT)
        {
            break;
        }
    }
    CHECK(reached, "重新找零成功并回到 READY");
    CHECK(lift_top_is_operational(&top), "恢复可用");

    /* ---------- [8] 空指针 ---------- */
    printf("  [8] 空指针不崩\n");
    fill_input(&in, &plant, 0U);
    lift_top_init(NULL, &tcfg);
    lift_top_init(&top, NULL);
    lift_top_update(NULL, &in, &cmd, &st);
    lift_top_update(&top, NULL, &cmd, &st);
    lift_top_update(&top, &in, NULL, &st);
    lift_top_update(&top, &in, &cmd, NULL);
    CHECK(!lift_top_is_operational(NULL), "is_operational(NULL) 为 false");
    CHECK(true, "空指针路径未崩溃");

    printf("\n共 %d 项断言，失败 %d 项\n", g_checks, g_failures);
    if (g_failures == 0)
    {
        printf("结果：全部通过\n");
        return 0;
    }
    printf("结果：存在失败\n");
    return 1;
}
