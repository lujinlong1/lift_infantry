/*
 * lift_seq 分阶段上台阶状态机单元测试。
 *
 * 覆盖（对应任务的第九节「状态机测试」）：
 *   [1]  正常完整流程：15 个阶段按序走完
 *   [2]  启动前置检查：未找零 / 未标定 / 布局未确认 / 电机离线 / 已在限位外
 *   [3]  前部支撑未确认 → 等在原地直到超时（「定时到了」≠「支撑住了」）
 *   [4]  后部支撑未确认 → 同上
 *   [5]  反馈失效 / 安全闸报故 → 故障
 *   [6]  限位触发 → 故障
 *   [7]  四轴误差超限 → 故障
 *   [8]  状态超时
 *   [9]  中止 + 故障锁存 + 未经复位不得继续动作
 *   [10] 故障停机策略：悬空 HOLD/LOWER，落地/人力解除 CUT
 *   [11] 载荷转移判据：前轮压空 → 收回时姿态变化 → 故障并原地停下
 *   [12] 配置校验 + 边界情形
 *
 * 这里全部使用【模拟反馈】，不代表任何硬件已经验证过。
 *
 * 编译：见 run_tests.ps1。该文件只用于主机验证，不进 Keil 固件工程。
 */

#include "lift_seq.h"

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

/* ---------------------------------------------------------------- */
/*  测试夹具：一个"会听目标话"的四轴模拟器                             */
/* ---------------------------------------------------------------- */

typedef struct
{
    int32_t position_mm[N];
    /* 本周期各轴是否允许跟随目标（用来模拟"某一根不动/卡住"） */
    bool    follow[N];
    /* 前部压空：前组收回时前部不下沉（正常），true 表示会塌 */
    bool    front_unsupported;
    bool    rear_unsupported;
} plant_t;

static void plant_init(plant_t *p, int32_t mm)
{
    uint32_t ch;

    (void)memset(p, 0, sizeof(*p));
    for (ch = 0U; ch < N; ch++)
    {
        p->position_mm[ch] = mm;
        p->follow[ch] = true;
    }
}

/*
 * 把模拟器朝目标推进一步。
 *
 * 模拟规则（刻意保持简单，只体现"会跟目标"与"不跟目标"两种行为）：
 *   - 正常情况下每周期最多走 step_mm（模拟有限速度）
 *   - 载荷转移：前组收回时，若前轮没压在台阶上（front_unsupported），
 *     车会前倾 —— 这里用"前两根只降到一半就下不去了"来体现姿态变化
 */
static void plant_step(
    plant_t *p,
    const lift_seq_status_t *st,
    int32_t step_mm)
{
    uint32_t ch;

    for (ch = 0U; ch < N; ch++)
    {
        int32_t target = st->axis_target_mm[ch];
        int32_t diff;
        int32_t move;

        if (!p->follow[ch])
        {
            continue;
        }

        /*
         * 前部压空：前组明明收到了"降下去"的目标，但车已经压在前轮上，
         * 位移跟不上 —— 表现为 group 目标未达成 + 整车姿态被抬起/压弯。
         * 这里用"前两根只走 1/8"来体现。
         */
        if (p->front_unsupported && ((ch == 0U) || (ch == 1U)) &&
            (target < p->position_mm[ch]))
        {
            diff = target - p->position_mm[ch];
            move = diff / 8;
            if ((move == 0) && (diff != 0))
            {
                move = (diff > 0) ? 1 : -1;
            }
            p->position_mm[ch] += move;
            continue;
        }
        if (p->rear_unsupported && ((ch == 2U) || (ch == 3U)) &&
            (target < p->position_mm[ch]))
        {
            diff = target - p->position_mm[ch];
            move = diff / 8;
            if ((move == 0) && (diff != 0))
            {
                move = (diff > 0) ? 1 : -1;
            }
            p->position_mm[ch] += move;
            continue;
        }

        diff = target - p->position_mm[ch];
        if (diff > step_mm)
        {
            move = step_mm;
        }
        else if (diff < -step_mm)
        {
            move = -step_mm;
        }
        else
        {
            move = diff;
        }
        p->position_mm[ch] += move;
    }
}

static lift_seq_input_t base_input(void)
{
    lift_seq_input_t in;
    uint32_t ch;

    (void)memset(&in, 0, sizeof(in));
    in.homed = true;
    in.calibrated = true;
    in.layout_confirmed = true;
    for (ch = 0U; ch < N; ch++)
    {
        in.axis_online[ch] = true;
        in.limit_min_ecd[ch] = 0;
        /* 行程上限按"200 mm 的量级"给：145 143 计数 ≈ 200 mm。 */
        in.limit_max_ecd[ch] = 150000;
    }
    return in;
}

/* 把模拟器的状态搬进输入，并算出同步/偏差判据。 */
static void fill_feedback(
    lift_seq_input_t *in,
    const plant_t *p,
    int32_t current)
{
    int32_t lowest;
    int32_t highest;
    uint32_t ch;

    for (ch = 0U; ch < N; ch++)
    {
        in->position_mm[ch] = p->position_mm[ch];
        in->speed_mm_s[ch] = 0;
        in->current[ch] = current;
        /* 逻辑计数按 725.7 计数/mm 折算，只用来喂前置检查的限位判据。 */
        in->position_ecd[ch] = p->position_mm[ch] * 725;
    }

    lowest = p->position_mm[0];
    highest = p->position_mm[0];
    for (ch = 1U; ch < N; ch++)
    {
        if (p->position_mm[ch] < lowest)
        {
            lowest = p->position_mm[ch];
        }
        if (p->position_mm[ch] > highest)
        {
            highest = p->position_mm[ch];
        }
    }
    in->sync_error_mm = highest - lowest;

    /*
     * 偏差参考量 = 去掉【一个】最高与【一个】最低之后的平均 ——
     * 与生产代码 lift_kin_axis_deviation_mm 用的是同一个定义。
     * 这里刻意照抄那个算法（而不是"写成差不多的样子"）：夹具与生产代码
     * 对同一个量给出不同数值，是这类测试最容易产生的假通过/假失败。
     */
    {
        int32_t work[N];
        uint32_t count = N;
        int32_t sum = 0;
        int32_t reference;
        uint32_t i;
        uint32_t drop_at;

        for (ch = 0U; ch < N; ch++)
        {
            work[ch] = p->position_mm[ch];
        }

        if (count > 2U)
        {
            /* 剔除一个最大值（按名次，不按值 —— 三根同高时按值会剔错） */
            drop_at = 0U;
            for (i = 1U; i < count; i++)
            {
                if (work[i] > work[drop_at])
                {
                    drop_at = i;
                }
            }
            for (i = drop_at; (i + 1U) < count; i++)
            {
                work[i] = work[i + 1U];
            }
            count--;

            /* 索引必须重新找：上一步的拷贝会挪动下标 */
            drop_at = 0U;
            for (i = 1U; i < count; i++)
            {
                if (work[i] < work[drop_at])
                {
                    drop_at = i;
                }
            }
            for (i = drop_at; (i + 1U) < count; i++)
            {
                work[i] = work[i + 1U];
            }
            count--;
        }

        if (count == 0U)
        {
            reference = (lowest + highest) / 2;
        }
        else
        {
            for (i = 0U; i < count; i++)
            {
                sum += work[i];
            }
            reference = sum / (int32_t)count;
        }

        in->deviation_max_mm = 0;
        for (ch = 0U; ch < N; ch++)
        {
            const int32_t dev = p->position_mm[ch] - reference;
            const int32_t mag = (dev < 0) ? -dev : dev;

            if (mag > in->deviation_max_mm)
            {
                in->deviation_max_mm = mag;
            }
        }
    }
}

/*
 * ====================================================================
 *  测试驱动器
 * ====================================================================
 *
 * 一个【有状态】的驱动器，整轮测试共用。之前每个用例各自拼装
 * run_cycle / run_until，结果反复踩同一类坑：st 是不是刚刷新的、
 * 确认脉冲在哪一拍、自动操作手的冷却从哪算起。合成一个驱动器之后
 * 这些问题只在一个地方解决。
 *
 * 它负责四件事（每一步都按真实调用方的顺序）：
 *   ① 按当前阶段填输入（目标、人工确认、drive_done）
 *   ② 调 lift_seq_update
 *   ③ 让模拟机构朝新目标动一步，再把反馈填回输入
 *   ④ 清掉单周期脉冲位
 *
 * ⚠️ 顺序很重要：`lift_seq_update` 用的是【输入里】的 now_ms 与反馈，
 *    所以必须在调用【之前】把 now 填进去。反过来的话，状态机会拿着上一拍
 *    的时间戳与反馈做判断，而调用方看到的是本拍的 st —— 两边错开一拍，
 *    测试就会在"状态已经切了、但 st 还是旧的"这种缝里读到假数据。
 */

/* 模拟机构每周期最多走多少毫米（≈ 8 mm/s @5 ms，与固件目标斜坡同量级）。 */
#define DRV_STEP_MM 4

/* 自动操作手两次按键之间的最小间隔（拍）。人不会每 5 ms 按一次。 */
#define DRV_CONFIRM_COOLDOWN_TICKS 40U

typedef struct
{
    lift_seq_t *s;
    plant_t *p;
    lift_seq_input_t *in;
    lift_seq_status_t *st;
    uint32_t clock;
    bool auto_operator;         /* 自动按确认 / 自动报 drive_done */
    /*
     * 静音确认（但保留 drive_done）。
     *
     * drive_done 是【另一个信息源】（底盘有没有开到），人工确认是
     * "支撑条件成立没有"。测后者时必须能把两者分开：关掉确认但继续
     * 报"底盘已经到位"，否则流程停在 ADVANCE_MIDDLE 根本走不到
     * VERIFY_*_SUPPORT，测到的就不是被测的那件事了。
     */
    bool hush_confirm;
    /*
     * 指定"哪一个阶段不许自动确认"。
     *
     * 比一个全局静音开关更好用：测试常常需要让前一个阶段的确认照常发生，
     * 只掐掉后面某一个阶段的那一次 —— 而流程是连着跑的，
     * "跑到某处再关"必然慢一拍（本用例第一版就是这么失败的：状态在同一拍
     *  就从 ADVANCE_MIDDLE 冲到了 VERIFY_REAR_SUPPORT，等调用方反应过来
     *  去静音，确认已经发出去了）。
     */
    lift_seq_state_t mute_confirm_state;
    bool mute_confirm_active;
    uint32_t confirm_cooldown;
    int32_t current;            /* 每周期实际下发的电流（喂给判据） */
} drv_t;

static bool drv_state_needs_confirm(lift_seq_state_t state)
{
    return (state == LIFT_SEQ_STATE_VERIFY_FRONT_SUPPORT) ||
           (state == LIFT_SEQ_STATE_VERIFY_REAR_SUPPORT);
}

/*
 * 哪些阶段在"等底盘往前开"。
 *
 * ⚠️ ADVANCE_MIDDLE 必须在这里！漏掉它的症状是"流程永远停在 ADVANCE_MIDDLE
 *    然后 60 秒后超时"，而且看起来像状态机的错 —— 实际上是夹具忘了给
 *    drive_done。（这一条踩过：调试了十几轮才发现是测试驱动器的问题。）
 */
static bool drv_state_wants_drive(lift_seq_state_t state)
{
    return (state == LIFT_SEQ_STATE_APPROACH_STEP) ||
           (state == LIFT_SEQ_STATE_ADVANCE_MIDDLE) ||
           (state == LIFT_SEQ_STATE_FINAL_ADVANCE);
}

static void drv_init(
    drv_t *d,
    lift_seq_t *s,
    plant_t *p,
    lift_seq_input_t *in,
    lift_seq_status_t *st)
{
    (void)memset(d, 0, sizeof(*d));
    d->s = s;
    d->p = p;
    d->in = in;
    d->st = st;
    d->auto_operator = true;
    d->current = 3000;
}

/*
 * 自动操作手：只有在"需要确认"的阶段才按键，而且两次之间至少隔
 * DRV_CONFIRM_COOLDOWN_TICKS 拍。冷却【在进入需要确认的阶段时重置】，
 * 这样每个阶段都有一次干净的按键机会（否则从上一个阶段带过来的冷却
 * 会让新阶段白等一段时间，用例得跟着多跑很多拍）。
 */
static void drv_auto_operator(drv_t *d)
{
    bool muted;

    if (!d->auto_operator)
    {
        return;
    }

    muted = d->hush_confirm ||
            (d->mute_confirm_active &&
             (d->s->state == d->mute_confirm_state));

    if (muted || !drv_state_needs_confirm(d->s->state))
    {
        d->confirm_cooldown = 0U;
    }
    else if (d->confirm_cooldown > 0U)
    {
        d->confirm_cooldown--;
    }
    else
    {
        d->in->confirm_request = true;
        d->confirm_cooldown = DRV_CONFIRM_COOLDOWN_TICKS;
    }

    if (drv_state_wants_drive(d->s->state))
    {
        d->in->drive_done = true;
    }
}

/** 推进一拍。返回本拍结束后的阶段。 */
static lift_seq_state_t drv_tick(drv_t *d)
{
    d->clock += 5U;
    drv_auto_operator(d);
    d->in->now_ms = d->clock;
    lift_seq_update(d->s, d->in, d->st);
    plant_step(d->p, d->st, DRV_STEP_MM);
    fill_feedback(d->in, d->p, d->current);
    /* 脉冲位逐周期清掉（真实调用方由 API 保证单周期脉冲）。 */
    d->in->start_request = false;
    d->in->abort_request = false;
    d->in->confirm_request = false;
    d->in->release_request = false;
    d->in->reset_request = false;
    d->in->drive_done = false;
    d->in->contact_front = false;
    d->in->contact_rear = false;
    d->in->guard_fault = false;
    return d->st->state;
}

/*
 * 跑到指定阶段为止。
 *
 * 契约：返回 true 时 `st` 是【刚跑完的那一拍】的输出。
 *   · 已经在目标阶段 → 再走一拍并检查是否还在（这样 st 被刷新过，
 *     而"只存在一拍"的阶段也测得到）
 *   · 不在目标阶段 → 循环推进，一到就【立刻返回】，绝不多跑
 */
static bool drv_until(drv_t *d, lift_seq_state_t want, uint32_t max_cycles)
{
    uint32_t i;

    if (d->st->state == want)
    {
        (void)drv_tick(d);
        return (d->st->state == want);
    }

    for (i = 0U; i < max_cycles; i++)
    {
        if (d->st->state == LIFT_SEQ_STATE_FAULT)
        {
            return false;
        }
        if (drv_tick(d) == want)
        {
            return true;
        }
    }
    return d->st->state == want;
}

/*
 * 启动流程并让前置检查跑出结论。
 *
 * 前置检查有一个 100 ms 的稳定窗口，所以"按一下启动键、跑两拍看结果"
 * 是不成立的。**所有**前置检查用例都必须走这里，否则测到的是
 * "窗口还没结束"，而不是被测的那道闸门。
 */
static void drv_start_and_settle(drv_t *d)
{
    d->in->start_request = true;
    (void)drv_tick(d);
    d->in->start_request = false;

    while ((d->clock < 400U) &&
           (d->st->state == LIFT_SEQ_STATE_PRECHECK))
    {
        (void)drv_tick(d);
    }
}

/*
 * 跑到 RETRACT_FRONT。中间有两个人确认点（前轮压上台阶、前轮刚接触），
 * 自动操作手会按过去。各个用例用它跳过与自己无关的部分。
 */
static bool drv_walk_to_retract_front(drv_t *d)
{
    return drv_until(d, LIFT_SEQ_STATE_RETRACT_FRONT, 20000U);
}

/*
 * 手动按一次确认（并静音自动确认）。
 * 用于"必须由人按下去才会动"这类用例。
 */
static void drv_press_confirm(drv_t *d)
{
    d->hush_confirm = true;
    d->in->confirm_request = true;
    (void)drv_tick(d);
    d->in->confirm_request = false;
}

/* 一路走到 COMPLETE。用于"正常完整流程"。 */
static bool drv_walk_to_complete(drv_t *d)
{
    return drv_until(d, LIFT_SEQ_STATE_COMPLETE, 80000U);
}

/*
 * 起手姿态就已经歪了：把四轴位置设成"某几根高、某几根低"，然后跑前置检查。
 *
 * ⚠️ 必须【持续】保持这个姿态，不能设一次就放手 —— 夹具每拍都会把
 *    位置朝目标挪一步，只设一次的话第二拍就自己回平了，测到的是
 *    "已经回平所以放行"，而不是"歪着所以拒绝"。
 */
static void drv_start_tilted(drv_t *d, int32_t tilted_mm)
{
    uint32_t i;

    for (i = 0U; i < N; i++)
    {
        d->p->follow[i] = false;   /* 冻住姿态 */
    }
    d->p->position_mm[0] = tilted_mm;

    d->in->start_request = true;
    (void)drv_tick(d);
    d->in->start_request = false;

    for (i = 0U; i < 60U; i++)
    {
        (void)drv_tick(d);
        if (d->st->state != LIFT_SEQ_STATE_PRECHECK)
        {
            break;
        }
    }
}

static lift_seq_config_t test_config(void)
{
    lift_seq_config_t cfg;

    lift_seq_config_default(&cfg);
    /*
     * 用例里允许启动：把"布局已确认"和"标定完成"当作输入给进来
     * （真实固件里它们是两个必须人工完成的闸门，见 lift_seq.h）。
     */
    cfg.clearance_mm = 200;
    cfg.contact_mm = 180;
    cfg.retracted_mm = 40;
    cfg.max_sync_error_mm = 15;
    cfg.max_deviation_mm = 15;
    cfg.position_tolerance_mm = 3;
    cfg.settled_speed_mm_s = 2;
    cfg.max_current = 16000;
    cfg.stall_speed_mm_s = 1;
    cfg.stall_ms = 2000U;
    cfg.precheck_timeout_ms = 2000U;
    cfg.lift_timeout_ms = 60000U;
    cfg.verify_timeout_ms = 5000U;
    cfg.advance_timeout_ms = 60000U;
    cfg.retract_timeout_ms = 30000U;
    cfg.support_timeout_ms = 60000U;
    return cfg;
}

/* ---------------------------------------------------------------- */
/*  [1] 正常完整流程                                                   */
/* ---------------------------------------------------------------- */

static void test_full_sequence(void)
{
    lift_seq_config_t cfg = test_config();
    lift_seq_t s;
    lift_seq_status_t st;
    lift_seq_input_t in = base_input();
    plant_t p;
    drv_t d;

    printf("  [1] 正常完整流程（按序走完全部阶段）\n");

    lift_seq_init(&s, &cfg);
    plant_init(&p, 0);
    drv_init(&d, &s, &p, &in, &st);

    in.start_request = true;
    (void)drv_tick(&d);
    CHECK(st.state == LIFT_SEQ_STATE_PRECHECK, "启动后进入 PRECHECK");
    CHECK(!st.command_valid, "PRECHECK 期间不应下发运动命令");

    /*
     * PRECHECK 有一个 100 ms 的稳定窗口：它把"请求启动"和"开始出力"
     * 分成两个可观察的时刻，现场能先看到检查结果再看到动作。
     */
    in.start_request = false;
    CHECK(drv_until(&d, LIFT_SEQ_STATE_LIFT_CLEARANCE, 100U),
          "前置检查通过后开始抬升");
    CHECK(st.command_valid, "抬升期间命令必须有效");
    CHECK(st.axis_target_mm[0] == 200, "目标是标定过的 200 mm");

    /* 抬到 200 mm 并稳定 */
    CHECK(drv_until(&d, LIFT_SEQ_STATE_VERIFY_CLEARANCE, 4000U),
          "应能抬到安全高度并进入 VERIFY_CLEARANCE");
    CHECK(p.position_mm[0] == 200, "四轴都到达 200 mm");
    CHECK(p.position_mm[3] == 200, "四轴都到达 200 mm");

    /* 接近台阶 */
    CHECK(drv_until(&d, LIFT_SEQ_STATE_APPROACH_STEP, 400U),
          "进入 APPROACH_STEP");
    CHECK(st.drive_request, "APPROACH_STEP 必须声明「该往前走了」");
    CHECK(st.operator_action[0] != '\0', "必须给人一句该做什么");
    CHECK(st.command_valid, "接近台阶期间必须继续撑住");
    CHECK(st.axis_target_mm[0] == 200, "接近台阶期间保持安全高度");

    /* 前部支撑确认 → 找接触（载荷转移点） */
    CHECK(drv_until(&d, LIFT_SEQ_STATE_VERIFY_FRONT_SUPPORT, 400U),
          "前进到位后进入「确认前部支撑」");
    CHECK(st.waiting_confirm, "必须明确报出「在等人工确认」");
    CHECK(st.operator_action[0] != '\0', "必须给人一句该做什么");

    CHECK(drv_until(&d, LIFT_SEQ_STATE_SEEK_FRONT_CONTACT, 4000U),
          "确认后进入「找前部接触」");
    CHECK(st.axis_target_mm[0] == 180, "前部目标降到接触点");
    CHECK(st.axis_target_mm[2] == 200, "后部仍然撑在安全高度");
    CHECK(!st.waiting_confirm,
          "找接触段不需要人工确认（上一阶段刚确认过），不应让人以为卡住");

    /* 接触确认 → 收回前部 */
    CHECK(drv_until(&d, LIFT_SEQ_STATE_RETRACT_FRONT, 4000U),
          "接触确认后进入 RETRACT_FRONT");
    CHECK(st.axis_target_mm[0] == 40, "前部目标降到收回位");
    CHECK(st.axis_target_mm[1] == 40, "前部目标降到收回位");
    CHECK(st.axis_target_mm[2] == 200, "后部仍然撑在安全高度");
    CHECK(st.axis_target_mm[3] == 200, "后部仍然撑在安全高度");

    CHECK(drv_until(&d, LIFT_SEQ_STATE_ADVANCE_MIDDLE, 8000U),
          "前部收回并确认后进入 ADVANCE_MIDDLE");
    CHECK(st.drive_request, "ADVANCE_MIDDLE 必须声明「该往前走了」");

    /* 底盘再前进 → 确认后部支撑 */
    in.drive_done = true;
    (void)drv_tick(&d);
    CHECK(st.state == LIFT_SEQ_STATE_VERIFY_REAR_SUPPORT,
          "前进到位后进入「确认后部支撑」");
    CHECK(st.waiting_confirm, "必须明确报出「在等人工确认」");
    in.drive_done = false;

    CHECK(drv_until(&d, LIFT_SEQ_STATE_RETRACT_REAR, 4000U),
          "确认后部支撑后进入 RETRACT_REAR");
    CHECK(st.axis_target_mm[2] == 40, "后部也降到收回位");

    CHECK(drv_until(&d, LIFT_SEQ_STATE_FINAL_ADVANCE, 8000U),
          "进入 FINAL_ADVANCE");

    CHECK(drv_walk_to_complete(&d), "走到 COMPLETE");
    CHECK(!st.command_valid, "COMPLETE 后必须交还手动控制");
    CHECK(!st.command_enabled, "COMPLETE 后不得再使能");
    CHECK(st.fault == LIFT_SAFETY_FAULT_NONE, "完整流程不应产生故障");
    CHECK(lift_seq_is_active(&s) == false, "COMPLETE 不再是活动状态");
}

/* ---------------------------------------------------------------- */
/*  [2] 启动前置检查                                                   */
/* ---------------------------------------------------------------- */

static void test_precheck_gates(void)
{
    lift_seq_config_t cfg = test_config();
    uint32_t ch;

    printf("  [2] 启动前置检查：五道闸门\n");

/*
 * 每道闸门一个独立的作用域：自己建实例、自己建夹具、自己建驱动器。
 * 用宏避免七次抄同一段样板（抄错一处就会测到别的闸门）。
 */
#define GATE_BEGIN()                                                   \
    lift_seq_t s;                                                      \
    lift_seq_status_t st;                                              \
    lift_seq_input_t in = base_input();                                \
    plant_t p;                                                         \
    drv_t d;                                                           \
    lift_seq_init(&s, &cfg);                                           \
    plant_init(&p, 0);                                                 \
    drv_init(&d, &s, &p, &in, &st)

    /* ① 未找零 */
    {
        GATE_BEGIN();
        in.homed = false;
        drv_start_and_settle(&d);
        CHECK(st.state == LIFT_SEQ_STATE_FAULT, "未找零必须拒绝启动");
        CHECK(st.fault == LIFT_SAFETY_FAULT_UNSUPPORTED, "故障码为 UNSUPPORTED");
        CHECK(!st.command_valid, "拒绝启动后不得下发命令");
    }

    /* ② 未标定运动学 —— 明确拒绝 200 mm 绝对位置命令 */
    {
        GATE_BEGIN();
        in.calibrated = false;
        drv_start_and_settle(&d);
        CHECK(st.state == LIFT_SEQ_STATE_FAULT, "未标定必须拒绝启动");
        CHECK(st.fault == LIFT_SAFETY_FAULT_NOT_CALIBRATED,
              "故障码为 NOT_CALIBRATED");
    }

    /* ③ 前后布局未确认 —— 收错一组就是把承重支撑收掉，必须拒绝 */
    {
        GATE_BEGIN();
        in.layout_confirmed = false;
        drv_start_and_settle(&d);
        CHECK(st.state == LIFT_SEQ_STATE_FAULT, "布局未确认必须拒绝启动");
        CHECK(st.fault == LIFT_SAFETY_FAULT_UNSUPPORTED, "故障码为 UNSUPPORTED");
    }

    /* ④ 任一电机离线 */
    for (ch = 0U; ch < N; ch++)
    {
        GATE_BEGIN();
        in.axis_online[ch] = false;
        drv_start_and_settle(&d);
        CHECK(st.state == LIFT_SEQ_STATE_FAULT, "任一轴离线必须拒绝启动");
        CHECK(st.fault == LIFT_SAFETY_FAULT_OFFLINE, "故障码为 OFFLINE");
        CHECK(st.fault_axis == (int32_t)ch, "必须指出是哪一根轴");
    }

    /* ⑤ 已经在软限位外 */
    {
        GATE_BEGIN();
        p.position_mm[0] = 300;
        p.position_mm[1] = 300;
        p.position_mm[2] = 300;
        p.position_mm[3] = 300;
        fill_feedback(&in, &p, 0);
        in.position_ecd[2] = 200000;   /* 超出 limit_max_ecd = 150000 */
        drv_start_and_settle(&d);
        CHECK(st.state == LIFT_SEQ_STATE_FAULT, "已在限位外必须拒绝启动");
        CHECK(st.fault == LIFT_SAFETY_FAULT_LIMIT, "故障码为 LIMIT");
    }

    /* ⑥ 起手姿态就已经歪了 */
    {
        GATE_BEGIN();
        drv_start_tilted(&d, 40);
        CHECK(st.state == LIFT_SEQ_STATE_FAULT, "起手姿态超限必须拒绝启动");
        CHECK(st.fault == LIFT_SAFETY_FAULT_SYNC, "故障码为 SYNC");
    }

    /* ⑦ 全部通过 → 正常进入抬升（证明上面的拒绝不是"永远拒绝"） */
    {
        GATE_BEGIN();
        drv_start_and_settle(&d);
        CHECK(st.state == LIFT_SEQ_STATE_LIFT_CLEARANCE,
              "六道闸门全过之后必须真的开始抬升");
    }

#undef GATE_BEGIN
}

/* ---------------------------------------------------------------- */
/*  [3][4] 支撑未确认                                                 */
/* ---------------------------------------------------------------- */

static void test_front_support_not_confirmed(void)
{
    lift_seq_config_t cfg = test_config();
    lift_seq_t s;
    lift_seq_status_t st;
    lift_seq_input_t in = base_input();
    plant_t p;
    drv_t d;
    uint32_t i;

    printf("  [3] 前部支撑未确认：等在原地，绝不当成「已经撑住」\n");

    lift_seq_init(&s, &cfg);
    plant_init(&p, 0);
    drv_init(&d, &s, &p, &in, &st);

    /*
     * 先让自动操作手把流程开到 VERIFY_FRONT_SUPPORT 之前（它负责给
     * drive_done），然后在需要确认的那一刻静音。
     *
     * drive_done 是【另一个信息源】（底盘有没有开到），人工确认才是
     * "支撑条件成立没有"。两者必须能分开测。
     */
    drv_start_and_settle(&d);
    CHECK(drv_until(&d, LIFT_SEQ_STATE_APPROACH_STEP, 40000U),
          "应能走到 APPROACH_STEP");
    d.hush_confirm = true;

    CHECK(drv_until(&d, LIFT_SEQ_STATE_VERIFY_FRONT_SUPPORT, 4000U),
          "前进到位后进入 VERIFY_FRONT_SUPPORT");
    CHECK(st.waiting_confirm, "必须把「在等人工确认」报出来");
    CHECK(st.operator_action[0] != '\0', "必须给人一句该做什么");

    /* 再跑很久（但不超过 support_timeout）仍然不往下走 */
    for (i = 0U; i < 1000U; i++)
    {
        (void)drv_tick(&d);
        if (st.state != LIFT_SEQ_STATE_VERIFY_FRONT_SUPPORT)
        {
            break;
        }
    }
    CHECK(st.state == LIFT_SEQ_STATE_VERIFY_FRONT_SUPPORT,
          "没有人确认就绝对不能推进（「定时到了」≠「支撑住了」）");
    CHECK(p.position_mm[0] == 200, "等待期间保持姿态，不动");

    /* 超过 support_timeout → 超时故障 */
    {
        bool faulted = false;

        for (i = 0U; i < 20000U; i++)
        {
            (void)drv_tick(&d);
            if (st.state == LIFT_SEQ_STATE_FAULT)
            {
                faulted = true;
                break;
            }
        }
        CHECK(faulted, "一直没人确认最终必须超时报故障");
        CHECK(st.fault == LIFT_SAFETY_FAULT_TIMEOUT, "故障码为 TIMEOUT");
    }
}

static void test_rear_support_not_confirmed(void)
{
    lift_seq_config_t cfg = test_config();
    lift_seq_t s;
    lift_seq_status_t st;
    lift_seq_input_t in = base_input();
    plant_t p;
    drv_t d;
    uint32_t i;
    bool advanced_too_early = false;

    printf("  [4] 后部支撑未确认：同样等在原地\n");

    lift_seq_init(&s, &cfg);
    plant_init(&p, 0);
    drv_init(&d, &s, &p, &in, &st);

    /*
     * 一路跑到 ADVANCE_MIDDLE 之前都照常，但【VERIFY_REAR_SUPPORT
     * 那一次确认从一开始就掐掉】。
     *
     * 为什么用"指定阶段静音"而不是"跑到某处再关"：状态机在同一拍里
     * 就可能从 ADVANCE_MIDDLE 冲进 VERIFY_REAR_SUPPORT，
     * 等调用方反应过来去关，确认已经发出去了。
     *
     * 这一条专门钉住一个真实缺陷：确认如果只是无差别锁存，下一个需要
     * 确认的阶段会顺手把它消费掉 —— 现象就是"进 VERIFY_REAR_SUPPORT
     * 的瞬间自己就过了"，而支撑其实根本没被检查。
     */
    d.mute_confirm_state = LIFT_SEQ_STATE_VERIFY_REAR_SUPPORT;
    d.mute_confirm_active = true;
    drv_start_and_settle(&d);
    CHECK(drv_until(&d, LIFT_SEQ_STATE_VERIFY_REAR_SUPPORT, 40000U),
          "应能走到 VERIFY_REAR_SUPPORT");

    /*
     * 这一条专门钉住一个真实缺陷：确认如果只是无差别锁存，下一个需要
     * 确认的阶段会顺手把它消费掉 —— 现象就是"进 VERIFY_REAR_SUPPORT
     * 的瞬间自己就过了"，而支撑其实根本没被检查。
     */
    for (i = 0U; i < 2000U; i++)
    {
        (void)drv_tick(&d);
        if (st.state != LIFT_SEQ_STATE_VERIFY_REAR_SUPPORT)
        {
            advanced_too_early = true;
            break;
        }
    }
    CHECK(!advanced_too_early,
          "★ 没有人按确认时，后部支撑阶段绝不能被上一次确认顺带满足");
    CHECK(st.state == LIFT_SEQ_STATE_VERIFY_REAR_SUPPORT, "停在等确认");
    CHECK(st.waiting_confirm, "必须报出在等确认");

    /* 新按一次确认 → 立刻推进 */
    drv_press_confirm(&d);
    CHECK(st.state == LIFT_SEQ_STATE_RETRACT_REAR, "新确认后应立刻推进");
    CHECK(st.axis_target_mm[2] == 40, "后部目标降到收回位");
}

/* ---------------------------------------------------------------- */
/*  [5][6][7] 上下游故障                                              */
/* ---------------------------------------------------------------- */

static void test_guard_fault(void)
{
    lift_seq_config_t cfg = test_config();
    lift_seq_t s;
    lift_seq_status_t st;
    lift_seq_input_t in = base_input();
    plant_t p;
    drv_t d;

    printf("  [5] 安全闸报故（反馈失效）→ 任何阶段都必须停\n");

    lift_seq_init(&s, &cfg);
    plant_init(&p, 0);
    drv_init(&d, &s, &p, &in, &st);

    drv_start_and_settle(&d);
    CHECK(drv_until(&d, LIFT_SEQ_STATE_LIFT_CLEARANCE, 100U),
          "进入抬升阶段");

    /* 抬升途中，安全闸报故 */
    in.guard_fault = true;
    (void)drv_tick(&d);
    CHECK(st.state == LIFT_SEQ_STATE_FAULT, "上游故障必须立刻停");
    CHECK(st.fault == LIFT_SAFETY_FAULT_OFFLINE, "故障码为 OFFLINE");
    CHECK(!st.command_valid, "故障后不得再下发运动命令");
}

static void test_limit_blocked(void)
{
    lift_seq_config_t cfg = test_config();
    lift_seq_t s;
    lift_seq_status_t st;
    lift_seq_input_t in = base_input();
    plant_t p;
    drv_t d;

    printf("  [6] 限位触发 → 故障\n");

    lift_seq_init(&s, &cfg);
    plant_init(&p, 0);
    drv_init(&d, &s, &p, &in, &st);

    drv_start_and_settle(&d);
    CHECK(drv_until(&d, LIFT_SEQ_STATE_LIFT_CLEARANCE, 100U),
          "进入抬升阶段");

    in.limit_blocked[3] = true;
    (void)drv_tick(&d);
    CHECK(st.state == LIFT_SEQ_STATE_FAULT, "限位被拦必须报故障");
    CHECK(st.fault == LIFT_SAFETY_FAULT_LIMIT, "故障码为 LIMIT");
}

static void test_group_divergence_during_retract(void)
{
    lift_seq_config_t cfg = test_config();
    lift_seq_t s;
    lift_seq_status_t st;
    lift_seq_input_t in = base_input();
    plant_t p;
    drv_t d;
    uint32_t i;
    bool faulted = false;

    printf("  [7] 同组一根卡死 → 堵转看门狗必须报故障\n");

    lift_seq_init(&s, &cfg);
    plant_init(&p, 0);
    drv_init(&d, &s, &p, &in, &st);

    drv_start_and_settle(&d);
    CHECK(drv_walk_to_retract_front(&d), "进入 RETRACT_FRONT");

    /*
     * 把收前部这一组的【两根都】冻住：机构顶住了、动不了。
     *
     * 这是真实工况里最危险的一种失败：控制器一直在要它往下走，
     * 机构一点也不动。没有堵转看门狗的话它会一直给电流顶到超时。
     * 判据是"该动却不动够 stall_ms"，所以这里必须两根都不动、
     * 而且目标确实还没到。
     */
    p.follow[0] = false;
    p.follow[1] = false;
    d.hush_confirm = true;

    for (i = 0U; i < 8000U; i++)
    {
        (void)drv_tick(&d);
        if (st.state == LIFT_SEQ_STATE_FAULT)
        {
            faulted = true;
            break;
        }
    }
    CHECK(faulted, "机构卡死必须报故障，不能一直给电流顶下去");
    CHECK((st.fault == LIFT_SAFETY_FAULT_OVERCURRENT) ||
              (st.fault == LIFT_SAFETY_FAULT_SYNC),
          "故障码应为 OVERCURRENT（堵转）或 SYNC（姿态）");
    CHECK(st.fault_axis >= 0, "必须指出是哪一根轴卡住了");
}
/* ---------------------------------------------------------------- */
/*  [8] 超时                                                          */
/* ---------------------------------------------------------------- */

static void test_lift_timeout(void)
{
    lift_seq_config_t cfg = test_config();
    lift_seq_t s;
    lift_seq_status_t st;
    lift_seq_input_t in = base_input();
    plant_t p;
    drv_t d;
    uint32_t i;
    bool faulted = false;

    printf("  [8] 状态超时\n");

    cfg.lift_timeout_ms = 500U;   /* 故意设得很短 */
    lift_seq_init(&s, &cfg);
    plant_init(&p, 0);
    drv_init(&d, &s, &p, &in, &st);
    d.auto_operator = false;

    /* 轴完全不动（follow 全关）→ 抬升阶段必然超时 */
    for (i = 0U; i < N; i++)
    {
        p.follow[i] = false;
    }

    drv_start_and_settle(&d);
    CHECK(drv_until(&d, LIFT_SEQ_STATE_LIFT_CLEARANCE, 100U),
          "进入抬升阶段");

    for (i = 0U; i < 400U; i++)
    {
        (void)drv_tick(&d);
        if (st.state == LIFT_SEQ_STATE_FAULT)
        {
            faulted = true;
            break;
        }
    }
    CHECK(faulted, "抬升超时必须报故障");
    CHECK((st.fault == LIFT_SAFETY_FAULT_TIMEOUT) ||
              (st.fault == LIFT_SAFETY_FAULT_OVERCURRENT),
          "故障码应为 TIMEOUT 或堵转（两者都是「没到位」）");
}

/* ---------------------------------------------------------------- */
/*  [9] 中止与锁存                                                    */
/* ---------------------------------------------------------------- */

static void test_abort_and_latch(void)
{
    lift_seq_config_t cfg = test_config();
    lift_seq_t s;
    lift_seq_status_t st;
    lift_seq_input_t in = base_input();
    plant_t p;
    drv_t d;
    uint32_t i;

    printf("  [9] 中止 → 锁存 → 未复位不得继续动作\n");

    lift_seq_init(&s, &cfg);
    plant_init(&p, 0);
    drv_init(&d, &s, &p, &in, &st);
    d.auto_operator = false;

    drv_start_and_settle(&d);
    CHECK(drv_until(&d, LIFT_SEQ_STATE_LIFT_CLEARANCE, 100U),
          "进入抬升阶段");

    in.abort_request = true;
    (void)drv_tick(&d);
    CHECK(st.state == LIFT_SEQ_STATE_FAULT, "中止 → FAULT");
    CHECK(st.fault == LIFT_SAFETY_FAULT_ABORTED, "故障码为 ABORTED");
    CHECK(!st.command_valid, "中止后不得再下发命令");

    /* ★ 关键：没有人复位，就必须一直锁存，而且不得自己重新动起来 */
    for (i = 0U; i < 500U; i++)
    {
        in.start_request = ((i % 50U) == 0U);   /* 反复按启动键 */
        (void)drv_tick(&d);
        in.start_request = false;
        CHECK(st.state == LIFT_SEQ_STATE_FAULT, "未经复位不得离开 FAULT");
        CHECK(!st.command_valid, "锁存期间不得下发命令");
    }

    /* 复位 → 回到 IDLE；但【不会】自动重新启动 */
    in.reset_request = true;
    (void)drv_tick(&d);
    CHECK(st.state == LIFT_SEQ_STATE_IDLE, "复位后回到 IDLE");
    CHECK(st.fault == LIFT_SAFETY_FAULT_NONE, "复位后故障码清空");

    for (i = 0U; i < 200U; i++)
    {
        (void)drv_tick(&d);
        CHECK(st.state == LIFT_SEQ_STATE_IDLE,
              "复位后没有新的启动沿就不得自己开始");
    }

    /* 复位后可以正常启动 */
    in.start_request = true;
    (void)drv_tick(&d);
    CHECK(st.state == LIFT_SEQ_STATE_PRECHECK, "复位后能重新启动");
}

/* ---------------------------------------------------------------- */
/*  [10] 故障停机策略                                                  */
/* ---------------------------------------------------------------- */

static void test_stop_policy(void)
{
    lift_seq_config_t cfg = test_config();
    lift_seq_t s;
    lift_seq_status_t st;
    lift_seq_input_t in = base_input();
    plant_t p;
    drv_t d;

    printf("  [10] 故障停机策略：悬空不切断，落地/人力解除才切断\n");

    /* 抬升途中中止：四根都撑在空中 → 绝不允许 CUT */
    lift_seq_init(&s, &cfg);
    plant_init(&p, 0);
    drv_init(&d, &s, &p, &in, &st);
    d.auto_operator = false;

    drv_start_and_settle(&d);
    CHECK(drv_until(&d, LIFT_SEQ_STATE_LIFT_CLEARANCE, 100U),
          "进入抬升阶段");
    p.follow[0] = false;
    p.follow[1] = false;
    p.follow[2] = false;
    p.follow[3] = false;
    in.abort_request = true;
    (void)drv_tick(&d);
    CHECK(st.state == LIFT_SEQ_STATE_FAULT, "中止 → FAULT");
    CHECK(st.support == LIFT_SUPPORT_AIRBORNE, "此时四轴都撑在空中");
    CHECK(st.stop_action != LIFT_STOP_CUT,
          "★ 悬空时【绝不能】直接断电 —— 那是摔车，不是保护");

    /* 人力解除：操作手已经确认垫了机械支撑 → 允许断电 */
    in.release_request = true;
    (void)drv_tick(&d);
    CHECK(st.released, "人工解除应生效");
    CHECK(st.stop_action == LIFT_STOP_CUT,
          "确认有机械支撑后允许直接断电");
    CHECK(st.support == LIFT_SUPPORT_MECHANICAL, "支撑状态应为 MECHANICAL");
    CHECK(st.state == LIFT_SEQ_STATE_FAULT, "解除不改变故障锁存");

    /* IDLE / PRECHECK 阶段出故障：车还在地上 → CUT 是对的 */
    {
        lift_seq_t s2;
        lift_seq_status_t st2;
        lift_seq_input_t in2 = base_input();
        plant_t p2;
        drv_t d2;

        lift_seq_init(&s2, &cfg);
        plant_init(&p2, 0);
        drv_init(&d2, &s2, &p2, &in2, &st2);

        in2.start_request = true;
        (void)drv_tick(&d2);
        CHECK(st2.state == LIFT_SEQ_STATE_PRECHECK, "在 PRECHECK");
        CHECK(st2.support == LIFT_SUPPORT_GROUND, "还没抬起来 → 车在地上");

        /* 让稳定窗口走满，但把"未找零"这道闸门关掉 */
        in2.start_request = false;
        in2.homed = false;
        drv_start_and_settle(&d2);
        CHECK(st2.state == LIFT_SEQ_STATE_FAULT, "未找零 → 故障");
        CHECK(st2.stop_action == LIFT_STOP_CUT,
              "车在地上时断电是正确且最快的停机方式");
    }
}

/* ---------------------------------------------------------------- */
/*  [11] 载荷转移判据                                                  */
/* ---------------------------------------------------------------- */

static void test_front_load_transfer_fault(void)
{
    lift_seq_config_t cfg = test_config();
    lift_seq_t s;
    lift_seq_status_t st;
    lift_seq_input_t in = base_input();
    plant_t p;
    drv_t d;
    uint32_t i;
    bool faulted = false;

    printf("  [11] 前轮压空：前部收不动 → 必须报故障并原地停下\n");

    lift_seq_init(&s, &cfg);
    plant_init(&p, 0);
    drv_init(&d, &s, &p, &in, &st);

    drv_start_and_settle(&d);
    CHECK(drv_walk_to_retract_front(&d), "进入 RETRACT_FRONT");
    d.hush_confirm = true;

    /*
     * 模拟"前轮其实没压在台阶上"：
     * 前部两根一开始往下收就被整车重量压住，**完全收不动**。
     *
     * 这是真实机构上最要防的一种情况 —— 操作手以为前轮压上了，
     * 其实压空，前部升降一松就顶不住。此时：
     *   · 前部两轴卡死不动 → 堵转看门狗（OVERCURRENT）
     *   · 姿态回不来       → 姿态判据（SYNC）
     * 两条判据哪一条先报都可以，但【必须报】，而且要停在原地。
     */
    p.follow[0] = false;
    p.follow[1] = false;

    for (i = 0U; i < 8000U; i++)
    {
        (void)drv_tick(&d);
        if (st.state == LIFT_SEQ_STATE_FAULT)
        {
            faulted = true;
            break;
        }
    }
    CHECK(faulted, "前部收不动必须停下来，不能继续走流程");
    CHECK(st.fault != LIFT_SAFETY_FAULT_NONE, "必须给出故障码");
    CHECK(st.fault_detail[0] != '\0', "必须给出故障明细供现场定位");
    CHECK(!st.command_valid, "故障后不得再下发运动命令");
}

/* ---------------------------------------------------------------- */
/*  [12] 配置与边界                                                    */
/* ---------------------------------------------------------------- */

static void test_config_validation(void)
{
    lift_seq_config_t cfg;

    printf("  [12] 配置校验与边界情形\n");

    lift_seq_config_default(&cfg);
    CHECK(lift_seq_config_is_valid(&cfg), "出厂默认配置必须自洽");
    CHECK(!lift_seq_config_is_valid(NULL), "空配置非法");

    /* 前后掩码必须互补且非空 */
    cfg = (lift_seq_config_t){0};
    lift_seq_config_default(&cfg);
    cfg.front_mask = 0U;
    CHECK(!lift_seq_config_is_valid(&cfg), "前组为空非法");
    lift_seq_config_default(&cfg);
    cfg.rear_mask = 0U;
    CHECK(!lift_seq_config_is_valid(&cfg), "后组为空非法");
    lift_seq_config_default(&cfg);
    cfg.rear_mask = 0x03U;
    CHECK(!lift_seq_config_is_valid(&cfg), "前后重叠非法");
    lift_seq_config_default(&cfg);
    cfg.rear_mask = 0x04U;
    CHECK(!lift_seq_config_is_valid(&cfg), "漏掉通道非法");
    lift_seq_config_default(&cfg);
    cfg.front_mask = 0x0FU;
    cfg.rear_mask = 0x0FU;
    CHECK(!lift_seq_config_is_valid(&cfg), "重叠且越界非法");

    /* 高度关系 */
    lift_seq_config_default(&cfg);
    cfg.clearance_mm = 0;
    CHECK(!lift_seq_config_is_valid(&cfg), "目标高度为 0 非法");
    lift_seq_config_default(&cfg);
    cfg.retracted_mm = 300;
    CHECK(!lift_seq_config_is_valid(&cfg), "收回位高于安全高度非法");
    lift_seq_config_default(&cfg);
    cfg.retracted_mm = -1;
    CHECK(!lift_seq_config_is_valid(&cfg), "收回位为负非法");

    /* 阈值 */
    lift_seq_config_default(&cfg);
    cfg.max_sync_error_mm = 0;
    CHECK(!lift_seq_config_is_valid(&cfg), "同步阈值为 0 非法");
    lift_seq_config_default(&cfg);
    cfg.position_tolerance_mm = 0;
    CHECK(!lift_seq_config_is_valid(&cfg), "到位容差为 0 非法");
    lift_seq_config_default(&cfg);
    cfg.load_current_min = cfg.max_current + 1;
    CHECK(!lift_seq_config_is_valid(&cfg), "负载电流门槛超过上限非法");
    lift_seq_config_default(&cfg);
    cfg.stall_ms = 0U;
    CHECK(!lift_seq_config_is_valid(&cfg), "堵转窗口为 0 非法");
    lift_seq_config_default(&cfg);
    cfg.lift_timeout_ms = 0U;
    CHECK(!lift_seq_config_is_valid(&cfg), "抬升不限时非法");

    /* 空指针 */
    lift_seq_init(NULL, &cfg);
    lift_seq_reset(NULL);
    CHECK(lift_seq_is_active(NULL) == false, "is_active(NULL) 为 false");
    CHECK(lift_seq_state_name(LIFT_SEQ_STATE_FAULT)[0] == 'F', "状态名");
    CHECK(lift_seq_state_name((lift_seq_state_t)99)[0] == '?', "非法状态名");

    {
        lift_seq_t s;
        lift_seq_status_t st;
        lift_seq_input_t in = base_input();

        lift_seq_init(&s, &cfg);
        in.now_ms = 5U;
        lift_seq_update(NULL, &in, &st);
        lift_seq_update(&s, NULL, &st);
        lift_seq_update(&s, &in, NULL);
        CHECK(true, "空指针路径不崩");
    }

    /* 默认配置里 load_current_min 必须是 0（判据关闭），否则没标定就全挂 */
    {
        lift_seq_config_t def;
        lift_seq_config_default(&def);
        CHECK(def.load_current_min == 0,
              "★ 出厂默认必须关闭「负载电流」判据 —— 没标定保持电流之前，"
              "随便填一个门槛会让所有支撑阶段直接判故障");
        CHECK(def.clearance_mm == 200, "出厂目标必须是任务的 200 mm");
        CHECK(def.front_mask == 0x03U, "默认前组按通道顺序");
        CHECK(def.rear_mask == 0x0CU, "默认后组按通道顺序");
    }
}

/* ---------------------------------------------------------------- */
/*  [13] 接触传感器的接口（现在没有，但接口要先在）                    */
/* ---------------------------------------------------------------- */

static void test_contact_input(void)
{
    lift_seq_config_t cfg = test_config();
    lift_seq_t s;
    lift_seq_status_t st;
    lift_seq_input_t in = base_input();
    plant_t p;
    drv_t d;

    printf("  [13] 接触信号接口：接了开关之后判据自动升级\n");

    lift_seq_init(&s, &cfg);
    plant_init(&p, 0);
    drv_init(&d, &s, &p, &in, &st);
    d.auto_operator = false;   /* 本用例只用接触信号，不按确认 */

    drv_start_and_settle(&d);
    CHECK(drv_until(&d, LIFT_SEQ_STATE_APPROACH_STEP, 4000U),
          "走到 APPROACH_STEP");
    in.drive_done = true;
    (void)drv_tick(&d);
    CHECK(st.state == LIFT_SEQ_STATE_VERIFY_FRONT_SUPPORT,
          "走到 VERIFY_FRONT_SUPPORT（无人确认）");

    /*
     * 不按确认，但接触信号为真 → 应当推进。
     * 这条钉的是"接口先留着"这件事：现在没有接触开关，但判据已经
     * 写成 "人工确认 || 接触信号"，装了开关就能直接接上。
     */
    in.contact_front = true;
    (void)drv_tick(&d);
    CHECK(st.state == LIFT_SEQ_STATE_SEEK_FRONT_CONTACT,
          "接触信号为真时（替代人工确认）应当推进到找接触");
    in.contact_front = false;

    /* 接触信号是电平量，找接触阶段再给一次就该继续 */
    CHECK(drv_until(&d, LIFT_SEQ_STATE_RETRACT_FRONT, 4000U),
          "找接触阶段的接触信号也应能推进");
}

int main(void)
{
    printf("lift_seq 分阶段上台阶状态机单元测试（全部使用模拟反馈）\n");

    test_full_sequence();
    test_precheck_gates();
    test_front_support_not_confirmed();
    test_rear_support_not_confirmed();
    test_guard_fault();
    test_limit_blocked();
    test_group_divergence_during_retract();
    test_lift_timeout();
    test_abort_and_latch();
    test_stop_policy();
    test_front_load_transfer_fault();
    test_config_validation();
    test_contact_input();

    printf("\n共 %d 项断言，失败 %d 项\n", g_checks, g_failures);
    if (g_failures == 0)
    {
        printf("结果：全部通过\n");
        return 0;
    }
    printf("结果：存在失败\n");
    return 1;
}
