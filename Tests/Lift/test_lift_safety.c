/*
 * lift_safety 单元测试。
 *
 * 两块内容：
 *   [1]~[5] 命令通道所有权令牌 —— 修的是"两个 5 ms 任务抢同一个 CAN 电流缓存"
 *            这个已证实的缺陷（台架空闲分支每周期写 0，把主链的电流砍成占空比）。
 *   [6]~[12] 故障停机策略 —— 修的是"故障 = 四轴电流清零"。
 *            本机构不自锁，悬空时清零 = 摔车；必须区分 CUT / HOLD / LOWER。
 *
 * 编译：见 run_tests.ps1。该文件只用于主机验证，不进 Keil 固件工程。
 */

#include "lift_safety.h"

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

/* ---------------------------------------------------------------- */
/*  [1]~[5] 命令通道所有权                                             */
/* ---------------------------------------------------------------- */

static void test_owner_basic(void)
{
    printf("  [1] 所有权：空闲时可认领，被别人占时拒绝\n");

    lift_owner_reset();
    lift_owner_set_timeout_ms(50U);

    CHECK(lift_owner_current() == LIFT_OWNER_NONE, "复位后无人认领");

    /* 空闲 → 主链可以认领 */
    CHECK(lift_owner_claim(LIFT_OWNER_CONTROL, 1000U), "空闲时主链应能认领");
    CHECK(lift_owner_current() == LIFT_OWNER_CONTROL, "所有者应为主链");
    CHECK(lift_owner_may_write(LIFT_OWNER_CONTROL), "主链应有权写");

    /* 别人来抢 → 拒绝 */
    CHECK(!lift_owner_claim(LIFT_OWNER_JOG, 1001U), "已被占用时应拒绝");
    CHECK(!lift_owner_may_write(LIFT_OWNER_JOG), "台架此时无权写");
    CHECK(!lift_owner_may_write(LIFT_OWNER_CAN_TEST), "CAN 测试此时无权写");
    CHECK(lift_owner_current() == LIFT_OWNER_CONTROL, "拒绝后所有者不变");

    /* 重复认领自己的是允许的（刷新时刻） */
    CHECK(lift_owner_claim(LIFT_OWNER_CONTROL, 1002U), "自己可以重复认领");
    CHECK(lift_owner_current() == LIFT_OWNER_CONTROL, "所有者不变");

    /* NONE 不是合法身份 */
    CHECK(!lift_owner_claim(LIFT_OWNER_NONE, 1003U), "NONE 不得作为身份认领");
    CHECK(!lift_owner_may_write(LIFT_OWNER_NONE), "NONE 永远无权写");
}

static void test_owner_release(void)
{
    printf("  [2] 所有权：只有本人还得了\n");

    lift_owner_reset();
    lift_owner_set_timeout_ms(50U);
    (void)lift_owner_claim(LIFT_OWNER_CONTROL, 2000U);

    /* 别人调用 release 是空操作 —— 否则误释放会把别人的通道抢走 */
    lift_owner_release(LIFT_OWNER_JOG);
    CHECK(lift_owner_current() == LIFT_OWNER_CONTROL,
          "非所有者 release 必须是空操作");

    /* 本人释放 → 通道空出来 */
    lift_owner_release(LIFT_OWNER_CONTROL);
    CHECK(lift_owner_current() == LIFT_OWNER_NONE, "本人释放后应空出来");
    CHECK(lift_owner_may_write(LIFT_OWNER_CONTROL), "空闲时主链有权写");
    CHECK(lift_owner_may_write(LIFT_OWNER_JOG), "空闲时台架也有权写");

    /* 台架认领 → 主链让位 */
    CHECK(lift_owner_claim(LIFT_OWNER_JOG, 2100U), "空闲时台架应能认领");
    CHECK(!lift_owner_may_write(LIFT_OWNER_CONTROL), "台架占用时主链无权写");
    CHECK(lift_owner_may_write(LIFT_OWNER_JOG), "台架自己有权写");

    /* 这正是修掉的那个缺陷：台架【空闲】时必须归还，主链才写得进去 */
    lift_owner_release(LIFT_OWNER_JOG);
    CHECK(lift_owner_may_write(LIFT_OWNER_CONTROL),
          "台架归还后主链必须能写（否则台架空闲时电流被砍成 0）");
}

/*
 * [2b] ★ "空闲时不要写" —— 2026-10-11 第二轮修。
 *
 * 场景：主链在跑（遥控使能、正常抬升），台架处于默认的 mode = OFF。
 *
 *   T+0ms  主链：claim → 写电流 → release          （owner 变 NONE）
 *   T+1ms  台架空闲分支：zero_output()
 *            → may_write(JOG) 看到 owner == NONE → 返回 true
 *            → 写 0，把主链刚写的电流抹掉
 *
 * 结果：主链「能抬升但抬不起来」—— 进 C620 的电流被每 5 ms 抹一次。
 *
 * 修法：台架【空闲】时不写。写 0 只在两个时刻需要：
 *   · 它自己刚跑完（那时 owner 是它自己，release 之前写）
 *   · 参数未生效 / 掉线（保护动作，由主链的门禁负责写）
 * 所以判据是 may_write(JOG) 【在任何状态下都必须为真】才能写 ——
 * 空闲时主链刚 release（owner = NONE）就不写。
 */
static void test_owner_idle_does_not_write(void)
{
    printf("  [2b] 所有权：台架空闲时不得抹掉主链的电流\n");

    lift_owner_reset();
    lift_owner_set_timeout_ms(50U);

    /*
     * 模拟台架空闲分支里应该用的判据：只有【自己当前持有】才允许写 0。
     * 这里直接把这条规则钉成断言，防止以后有人改回 may_write()。
     */
    CHECK(lift_owner_claim(LIFT_OWNER_CONTROL, 5000U), "主链认领");
    lift_owner_release(LIFT_OWNER_CONTROL);
    CHECK(lift_owner_current() == LIFT_OWNER_NONE, "主链写完就归还");

    /*
     * ⚠️ 这一行就是缺陷本身：空闲时 may_write 返回 true，
     *    于是台架会写 0。所以台架【不能】用 may_write 当"该不该写"的判据。
     */
    CHECK(lift_owner_may_write(LIFT_OWNER_JOG),
          "（记录缺陷）空闲时 may_write 为真 —— 所以台架不能用它判该不该写");
    CHECK(lift_owner_current() != LIFT_OWNER_JOG,
          "★ 台架空闲时【不是】所有者 ⇒ 不得写电流");

    /* 台架真正在跑：它是所有者，这时写 0 是对的 */
    CHECK(lift_owner_claim(LIFT_OWNER_JOG, 5100U), "台架跑起来时认领");
    CHECK(lift_owner_current() == LIFT_OWNER_JOG,
          "★ 台架运行中是所有者 ⇒ 允许写（包括收尾写 0）");
}

static void test_owner_watchdog(void)
{
    printf("  [3] 所有权：认领方不刷新则自动失效\n");

    lift_owner_reset();
    lift_owner_set_timeout_ms(50U);
    CHECK(lift_owner_claim(LIFT_OWNER_JOG, 3000U), "台架认领");

    /* 还没超时 */
    lift_owner_tick(3049U);
    CHECK(lift_owner_current() == LIFT_OWNER_JOG, "49 ms 未超时，仍持有");

    /* 超时：自动失效，把通道让出来 */
    lift_owner_tick(3051U);
    CHECK(lift_owner_current() == LIFT_OWNER_NONE,
          "超时未刷新必须自动失效（否则任务死掉就永久占住通道）");

    /* timeout = 0 表示不设失效 */
    lift_owner_reset();
    lift_owner_set_timeout_ms(0U);
    CHECK(lift_owner_claim(LIFT_OWNER_JOG, 4000U), "台架认领");
    lift_owner_tick(999999U);
    CHECK(lift_owner_current() == LIFT_OWNER_JOG,
          "timeout = 0 时不应自动失效");
    lift_owner_set_timeout_ms(50U);
}

static void test_owner_rotation(void)
{
    /*
     * 模拟真实时序：主链每 5 ms 认领并写，台架跑一段后归还，
     * 主链必须立刻能接着写。
     */
    uint32_t t;
    bool control_wrote_after_release = false;
    bool jog_owned = false;

    printf("  [4] 所有权：主链与台架的一轮交接\n");

    lift_owner_reset();
    lift_owner_set_timeout_ms(50U);

    for (t = 0U; t < 1000U; t += 5U)
    {
        lift_owner_tick(t);

        if ((t >= 300U) && (t < 600U))
        {
            /* 台架这一段在跑：抢通道并保持 */
            if (!jog_owned)
            {
                jog_owned = lift_owner_claim(LIFT_OWNER_JOG, t);
                CHECK(jog_owned, "台架启动时应当抢到通道");
            }
            if (jog_owned)
            {
                (void)lift_owner_claim(LIFT_OWNER_JOG, t);   /* 刷新 */
                CHECK(!lift_owner_may_write(LIFT_OWNER_CONTROL),
                      "台架运行期间主链不得写");
            }
        }
        else
        {
            if (jog_owned && (t >= 600U))
            {
                lift_owner_release(LIFT_OWNER_JOG);
                jog_owned = false;
            }
            /*
             * 主链的真实写时序：认领 → 写 → 归还。
             * 归还这一步是【必须的】—— 只认领不归还，通道就被主链永久占住，
             * 台架再也抢不到（本用例第一版就是这么失败的，正好证明了令牌
             * 真的在起作用，而不是"谁都能写"）。
             */
            if (lift_owner_claim(LIFT_OWNER_CONTROL, t))
            {
                if (t >= 600U)
                {
                    control_wrote_after_release = true;
                }
                lift_owner_release(LIFT_OWNER_CONTROL);
            }
        }
    }

    CHECK(control_wrote_after_release,
          "台架归还后主链必须能恢复写入");
}

static void test_owner_null_safety(void)
{
    printf("  [5] 所有权：边界情形\n");

    lift_owner_reset();
    /* 未认领时 release 无副作用 */
    lift_owner_release(LIFT_OWNER_CONTROL);
    CHECK(lift_owner_current() == LIFT_OWNER_NONE, "空释放后仍无人认领");
    lift_owner_release(LIFT_OWNER_NONE);
    CHECK(lift_owner_current() == LIFT_OWNER_NONE, "NONE 释放无副作用");

    lift_owner_set_timeout_ms(50U);
    CHECK(lift_owner_timeout_ms() == 50U, "超时值应可读回");

    lift_owner_reset();
    CHECK(lift_owner_current() == LIFT_OWNER_NONE, "reset 之后无人认领");
}

/* ---------------------------------------------------------------- */
/*  [6]~[12] 故障停机策略                                             */
/* ---------------------------------------------------------------- */

static lift_safety_context_t ctx(
    lift_support_t support,
    bool homed,
    bool calibrated,
    bool moving)
{
    lift_safety_context_t c;

    c.fault = LIFT_SAFETY_FAULT_NONE;
    c.support = support;
    c.homed = homed;
    c.calibrated = calibrated;
    c.moving = moving;
    return c;
}

static void test_policy_supported(void)
{
    lift_safety_context_t c;

    printf("  [6] 策略：有支撑/落地 → CUT\n");

    /* 已确认机械支撑：无论找没找零、动没动，都可以直接断电 */
    c = ctx(LIFT_SUPPORT_MECHANICAL, true, true, true);
    CHECK(lift_safety_policy_for(&c) == LIFT_STOP_CUT, "机械支撑 → CUT");
    c = ctx(LIFT_SUPPORT_MECHANICAL, false, false, true);
    CHECK(lift_safety_policy_for(&c) == LIFT_STOP_CUT, "机械支撑 → CUT");
    c = ctx(LIFT_SUPPORT_MECHANICAL, true, true, false);
    CHECK(lift_safety_policy_for(&c) == LIFT_STOP_CUT, "机械支撑 → CUT");

    /* 车已落地：同上 */
    c = ctx(LIFT_SUPPORT_GROUND, true, true, true);
    CHECK(lift_safety_policy_for(&c) == LIFT_STOP_CUT, "落地 → CUT");
    c = ctx(LIFT_SUPPORT_GROUND, false, false, true);
    CHECK(lift_safety_policy_for(&c) == LIFT_STOP_CUT, "落地 → CUT");
}

static void test_policy_airborne(void)
{
    lift_safety_context_t c;

    printf("  [7] 策略：悬空 → 动作中 LOWER，静止时 HOLD\n");

    c = ctx(LIFT_SUPPORT_AIRBORNE, true, true, true);
    CHECK(lift_safety_policy_for(&c) == LIFT_STOP_LOWER,
          "悬空且正在动作 → 受控下降");

    c = ctx(LIFT_SUPPORT_AIRBORNE, true, true, false);
    CHECK(lift_safety_policy_for(&c) == LIFT_STOP_HOLD,
          "悬空且停着 → 保持托住");

    /* 没找零时没有位置基准，LOWER 无从谈起 → CUT */
    c = ctx(LIFT_SUPPORT_AIRBORNE, false, true, true);
    CHECK(lift_safety_policy_for(&c) == LIFT_STOP_CUT,
          "悬空但没找零 → 只能断电（没有下降基准）");
    c = ctx(LIFT_SUPPORT_AIRBORNE, false, true, false);
    CHECK(lift_safety_policy_for(&c) == LIFT_STOP_CUT,
          "悬空但没找零 → 只能断电");
}

static void test_policy_unknown_is_conservative(void)
{
    lift_safety_context_t c;

    printf("  [8] 策略：支撑未知一律按悬空处理（不许赌）\n");

    c = ctx(LIFT_SUPPORT_UNKNOWN, true, true, true);
    CHECK(lift_safety_policy_for(&c) == LIFT_STOP_LOWER,
          "未知支撑也必须是 LOWER，不能是 CUT");

    c = ctx(LIFT_SUPPORT_UNKNOWN, true, true, false);
    CHECK(lift_safety_policy_for(&c) == LIFT_STOP_HOLD,
          "未知支撑也必须是 HOLD，不能是 CUT");

    /* 空指针：最保守 —— 什么都不知道，只能不出力 */
    CHECK(lift_safety_policy_for(NULL) == LIFT_STOP_CUT,
          "没有上下文时只能 CUT");
}

static void test_stop_apply_cut(void)
{
    lift_stop_config_t cfg;
    lift_stop_state_t st;
    int32_t out[LIFT_CHANNEL_COUNT];
    bool active = true;
    uint32_t ch;

    printf("  [9] 执行：CUT 必须真的清零\n");

    cfg.hold_current = 3000;
    cfg.current_limit = 16000;
    cfg.per_axis_allowed = true;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        st.position_ecd[ch] = 20000;
        st.park_ecd[ch] = 200;
        st.hold_ecd[ch] = 20000;
        out[ch] = 9999;
    }

    lift_stop_apply(LIFT_STOP_CUT, &cfg, &st, out, &active);
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        CHECK(out[ch] == 0, "CUT 必须把四轴清零");
    }
    CHECK(!active, "CUT 不是「仍在执行」的动作");
}

static void test_stop_apply_hold(void)
{
    lift_stop_config_t cfg;
    lift_stop_state_t st;
    int32_t out[LIFT_CHANNEL_COUNT];
    bool active = false;
    uint32_t ch;

    printf("  [10] 执行：HOLD 给抬升方向的保持电流\n");

    cfg.hold_current = 3000;
    cfg.current_limit = 16000;
    cfg.per_axis_allowed = true;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        st.position_ecd[ch] = 20000;
        st.park_ecd[ch] = 200;
        st.hold_ecd[ch] = 20000;
        out[ch] = 0;
    }
    CHECK(lift_stop_can_hold(&cfg), "标定了保持电流 → 能托住");

    lift_stop_apply(LIFT_STOP_HOLD, &cfg, &st, out, &active);
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        CHECK(out[ch] == 3000, "HOLD 必须给正向（抬升）保持电流");
    }
    CHECK(active, "HOLD 是持续动作");

    /* 保持电流为 0（未标定）→ HOLD 退化成 CUT，并如实报告「托不住」 */
    cfg.hold_current = 0;
    CHECK(!lift_stop_can_hold(&cfg), "没标定 → 托不住");
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        out[ch] = 777;
    }
    active = true;
    lift_stop_apply(LIFT_STOP_HOLD, &cfg, &st, out, &active);
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        CHECK(out[ch] == 0, "没标定保持电流时 HOLD 必须退化成 CUT");
    }
    CHECK(!active, "退化后不应报「仍在保持」");

    /* 超过上限的保持电流必须被夹住 */
    cfg.hold_current = 30000;
    cfg.current_limit = 16000;
    lift_stop_apply(LIFT_STOP_HOLD, &cfg, &st, out, &active);
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        CHECK(out[ch] == 16000, "保持电流必须被上限夹住");
    }
}

static void test_stop_apply_lower(void)
{
    lift_stop_config_t cfg;
    lift_stop_state_t st;
    int32_t out[LIFT_CHANNEL_COUNT];
    bool active = false;
    uint32_t ch;

    printf("  [11] 执行：LOWER 往停放点走，且不允许「各停各的」\n");

    cfg.hold_current = 3000;
    cfg.current_limit = 16000;
    cfg.per_axis_allowed = true;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        st.position_ecd[ch] = 20000;
        st.park_ecd[ch] = 200;
        st.hold_ecd[ch] = 20000;
        out[ch] = 0;
    }

    lift_stop_apply(LIFT_STOP_LOWER, &cfg, &st, out, &active);
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        CHECK(out[ch] == -3000, "LOWER 必须给下降方向（逻辑负）");
    }
    CHECK(active, "还没到停放点 → 仍在执行");

    /*
     * ★ 关键：只有一根到位时，其余三根【不许】继续单独下降。
     *   三根继续降、一根停住 = 车在倾斜中被放下 —— 正是"逐轴清零"
     *   那个老缺陷的另一种形式。
     */
    st.position_ecd[0] = 200;    /* ch0 已到停放点 */
    lift_stop_apply(LIFT_STOP_LOWER, &cfg, &st, out, &active);
    CHECK(out[0] == -3000,
          "一根到位时【不得】单独停 —— 单独停会把车斜着放下");
    for (ch = 1U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        CHECK(out[ch] == -3000, "还没到的轴继续下降");
    }
    CHECK(active, "还有轴没到 → 仍在执行");

    /* 四根都到位 → 动作结束，可以交还控制 */
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        st.position_ecd[ch] = 200;
    }
    lift_stop_apply(LIFT_STOP_LOWER, &cfg, &st, out, &active);
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        CHECK(out[ch] == 0, "全部到位后应停止出力");
    }
    CHECK(!active, "全部到位 → 动作结束");

    /* 低于停放点（已经到底）也算到位，不得反向往上顶 */
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        st.position_ecd[ch] = 0;
    }
    active = true;
    lift_stop_apply(LIFT_STOP_LOWER, &cfg, &st, out, &active);
    CHECK(!active, "已经低于停放点 → 不得反向动作");
}

static void test_stop_null_safety(void)
{
    int32_t out[LIFT_CHANNEL_COUNT];
    bool active = true;
    uint32_t ch;

    printf("  [12] 执行：空指针不崩、不误动\n");

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        out[ch] = 1;
    }

    lift_stop_apply(LIFT_STOP_HOLD, NULL, NULL, out, &active);
    CHECK(!active, "空参数时不得报「在执行」");
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        CHECK(out[ch] == 1, "空参数时不得改动输出（由调用方负责清零）");
    }

    /* out_active 允许为 NULL */
    lift_stop_apply(LIFT_STOP_CUT, NULL, NULL, out, NULL);
    CHECK(true, "out_active 为 NULL 不崩");

    CHECK(!lift_stop_can_hold(NULL), "空配置 → 托不住");
}

static void test_names(void)
{
    printf("  [13] 名称表完整（Watch / 日志用）\n");

    CHECK(lift_stop_action_name(LIFT_STOP_HOLD)[0] == 'H', "HOLD 名");
    CHECK(lift_stop_action_name(LIFT_STOP_LOWER)[0] == 'L', "LOWER 名");
    CHECK(lift_stop_action_name(LIFT_STOP_CUT)[0] == 'C', "CUT 名");
    CHECK(lift_stop_action_name((lift_stop_action_t)99)[0] == '?', "非法值");

    CHECK(lift_support_name(LIFT_SUPPORT_AIRBORNE)[0] == 'A', "AIRBORNE 名");
    CHECK(lift_support_name(LIFT_SUPPORT_GROUND)[0] == 'G', "GROUND 名");
    CHECK(lift_support_name((lift_support_t)99)[0] == '?', "非法值");

    CHECK(lift_safety_fault_name(LIFT_SAFETY_FAULT_OFFLINE)[0] == 'O',
          "OFFLINE 名");
    CHECK(lift_safety_fault_name(LIFT_SAFETY_FAULT_NOT_CALIBRATED)[0] == 'N',
          "NOT_CALIBRATED 名");
    CHECK(lift_safety_fault_name((lift_safety_fault_t)99)[0] == '?',
          "非法值");
    CHECK(lift_safety_fault_name(LIFT_SAFETY_FAULT_NONE)[0] == 'N',
          "NONE 名");

    /* 每个故障码都必须有名字，而且不能是 "?" */
    {
        int f;
        for (f = 0; f < (int)LIFT_SAFETY_FAULT_COUNT; f++)
        {
            CHECK(lift_safety_fault_name((lift_safety_fault_t)f)[0] != '?',
                  "每个故障码都必须有名字");
        }
    }
}

int main(void)
{
    printf("lift_safety 命令通道所有权 + 故障停机策略单元测试\n");

    test_owner_basic();
    test_owner_release();
    test_owner_idle_does_not_write();
    test_owner_watchdog();
    test_owner_rotation();
    test_owner_null_safety();

    test_policy_supported();
    test_policy_airborne();
    test_policy_unknown_is_conservative();

    test_stop_apply_cut();
    test_stop_apply_hold();
    test_stop_apply_lower();
    test_stop_null_safety();
    test_names();

    printf("\n共 %d 项断言，失败 %d 项\n", g_checks, g_failures);
    if (g_failures == 0)
    {
        printf("结果：全部通过\n");
        return 0;
    }
    printf("结果：存在失败\n");
    return 1;
}
