/*
 * lift_guard 主机侧单元测试。
 *
 * 编译（与学习仓库 C 练习同一套严格标准）：
 *   gcc -std=c11 -Wall -Wextra -Wpedantic -Werror \
 *       test_lift_guard.c lift_guard.c -o test_lift_guard.exe
 *
 * 该文件只用于主机验证，不进 Keil 固件工程。
 */

#include "lift_guard.h"

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

static lift_guard_config_t base_config(void)
{
    lift_guard_config_t c;
    uint32_t ch;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        c.position_min_ecd[ch] = 0;
        c.position_max_ecd[ch] = 20000;
        c.direction[ch] = 1;
        c.descent_current[ch] = -3000;
        /* 停放点：与 position_min 分开的独立字段（2026-10-11 新增） */
        c.descent_target_ecd[ch] = 0;
    }
    c.current_limit = 8000;
    c.descent_stall_speed_rpm = 30;
    c.descent_stall_ms = 200U;
    c.descent_timeout_ms = 3000U;
    return c;
}

static lift_guard_input_t base_input(void)
{
    lift_guard_input_t in;
    uint32_t ch;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        in.position_ecd[ch] = 10000;
        in.speed_rpm[ch] = 0;
        in.requested_current[ch] = 0;
        in.online[ch] = true;
    }
    in.homed = true;
    in.command_fresh = true;
    in.reset_request = false;
    in.now_ms = 0U;
    return in;
}

static void step(
    lift_guard_t *g,
    lift_guard_output_t *out,
    const lift_guard_input_t *in)
{
    lift_guard_update(g, in, out);
}

static void set_request(lift_guard_input_t *in, int16_t current)
{
    uint32_t ch;
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        in->requested_current[ch] = current;
    }
}

static void set_speed(lift_guard_input_t *in, int16_t rpm)
{
    uint32_t ch;
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        in->speed_rpm[ch] = rpm;
    }
}

static bool output_all_equal(const lift_guard_output_t *out, int16_t value)
{
    uint32_t ch;
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        if (out->output_current[ch] != value)
        {
            return false;
        }
    }
    return true;
}

static void test_disarmed_before_homing(void)
{
    lift_guard_t g;
    lift_guard_output_t out;
    lift_guard_config_t c = base_config();
    lift_guard_input_t in = base_input();

    printf("  [1] 未找零时任何运动命令都不得通过\n");
    lift_guard_init(&g, &c);

    in.homed = false;
    set_request(&in, 5000);
    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_DISARMED, "状态为 DISARMED");
    CHECK(output_all_equal(&out, 0), "输出全为 0");
}

static void test_normal_and_clamp(void)
{
    lift_guard_t g;
    lift_guard_output_t out;
    lift_guard_config_t c = base_config();
    lift_guard_input_t in = base_input();

    printf("  [2] 正常通路与硬限幅\n");
    lift_guard_init(&g, &c);

    set_request(&in, 5000);
    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_READY, "状态为 READY");
    CHECK(output_all_equal(&out, 5000), "请求原样放行");

    set_request(&in, 20000);
    step(&g, &out, &in);
    CHECK(output_all_equal(&out, 8000), "超量请求被硬限幅到 current_limit");
}

static void test_offline_fault(void)
{
    lift_guard_t g;
    lift_guard_output_t out;
    lift_guard_config_t c = base_config();
    lift_guard_input_t in = base_input();

    printf("  [3] 任一轴离线立即归零并锁存\n");
    lift_guard_init(&g, &c);

    set_request(&in, 5000);
    in.online[2] = false;
    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_FAULT, "进入 FAULT");
    CHECK(out.fault == LIFT_GUARD_FAULT_OFFLINE, "故障码为 OFFLINE");
    CHECK(output_all_equal(&out, 0), "输出全为 0");
}

static void test_descent_starts(void)
{
    lift_guard_t g;
    lift_guard_output_t out;
    lift_guard_config_t c = base_config();
    lift_guard_input_t in = base_input();

    printf("  [4] 命令超时不得归零，必须受控缓降\n");
    lift_guard_init(&g, &c);

    in.command_fresh = false;
    set_speed(&in, -500);   /* 全部通道都在动，避免被判堵转 */
    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_DESCENDING, "状态为 DESCENDING");
    CHECK(output_all_equal(&out, -3000), "输出缓降电流而不是 0");
    CHECK(out.descent_elapsed_ms == 0U, "缓降计时起始为 0");

    in.now_ms = 500U;
    step(&g, &out, &in);
    CHECK(out.descent_elapsed_ms == 500U, "缓降计时累计");
    CHECK(out.state == LIFT_GUARD_STATE_DESCENDING, "仍在缓降");
}

static void test_descent_reaches_bottom(void)
{
    lift_guard_t g;
    lift_guard_output_t out;
    lift_guard_config_t c = base_config();
    lift_guard_input_t in = base_input();
    uint32_t ch;

    printf("  [5] 缓降到下限后进入 PARKED\n");
    lift_guard_init(&g, &c);

    in.command_fresh = false;
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        in.position_ecd[ch] = 0;
    }
    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_PARKED, "状态为 PARKED");
    CHECK(output_all_equal(&out, 0), "到位后不再输出");
    CHECK(out.at_target[0], "目标到达标志置位");

    /* 长时间无命令也不得升级为故障：上电找零后就是这种状态。 */
    in.now_ms = 100000U;
    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_PARKED, "持续无命令仍保持 PARKED");
    CHECK(out.fault == LIFT_GUARD_FAULT_NONE, "不产生故障");
}

static void test_descent_target_is_not_position_min(void)
{
    lift_guard_t g;
    lift_guard_output_t out;
    lift_guard_config_t c = base_config();
    lift_guard_input_t in = base_input();
    uint32_t ch;

    printf("  [5b] 缓降目标是【停放点】，不是行程下限\n");

    /*
     * 复现老配置的危险组合：行程下限配在机械死点之下（-2000），
     * 停放点在零点上方 200。若拿 position_min 当缓降目标，
     * 每次命令超时都会一路顶进死点 → DESCENT_STALL → 掉车。
     */
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        c.position_min_ecd[ch] = -2000;
        c.descent_target_ecd[ch] = 200;
        in.position_ecd[ch] = 3000;
    }
    lift_guard_init(&g, &c);

    in.command_fresh = false;
    set_speed(&in, -500);
    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_DESCENDING, "还在往下走");
    CHECK(output_all_equal(&out, -3000), "继续给缓降电流");

    /* 走到停放点就该停住，而不是继续往 -2000 走。 */
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        in.position_ecd[ch] = 200;
    }
    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_PARKED, "到停放点即 PARKED");
    CHECK(output_all_equal(&out, 0), "停放后不再输出电流");

    /* 停放点必须落在行程区间内：配置写到区间外会被夹回来。 */
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        c.descent_target_ecd[ch] = -5000;
    }
    lift_guard_init(&g, &c);
    CHECK(g.config.descent_target_ecd[0] == -2000, "停放点低于下限被夹到下限");
}

static void test_descent_stall_fault(void)
{
    lift_guard_t g;
    lift_guard_output_t out;
    lift_guard_config_t c = base_config();
    lift_guard_input_t in = base_input();

    printf("  [6] 该降却不动 -> 堵转故障\n");

    lift_guard_init(&g, &c);

    in.command_fresh = false;
    in.speed_rpm[0] = 0;

    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_DESCENDING, "首周期先进入缓降");
    CHECK(out.fault == LIFT_GUARD_FAULT_NONE, "未到保持时间不报故障");

    in.now_ms = 100U;
    step(&g, &out, &in);
    CHECK(out.fault == LIFT_GUARD_FAULT_NONE, "100ms 仍未到 200ms");

    in.now_ms = 200U;
    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_FAULT, "持续 200ms 不动 -> FAULT");
    CHECK(out.fault == LIFT_GUARD_FAULT_DESCENT_STALL, "故障码为 DESCENT_STALL");
    CHECK(output_all_equal(&out, 0), "故障时归零");
}

static void test_descent_timeout_fault(void)
{
    lift_guard_t g;
    lift_guard_output_t out;
    lift_guard_config_t c = base_config();
    lift_guard_input_t in = base_input();

    printf("  [7] 一直降不到底 -> 超时故障\n");
    lift_guard_init(&g, &c);

    in.command_fresh = false;
    set_speed(&in, -500);   /* 一直在动，不会触发堵转 */

    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_DESCENDING, "进入缓降");

    in.now_ms = 2999U;
    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_DESCENDING, "2999ms 未超时");

    in.now_ms = 3000U;
    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_FAULT, "3000ms 超时 -> FAULT");
    CHECK(out.fault == LIFT_GUARD_FAULT_DESCENT_TIMEOUT, "故障码为 DESCENT_TIMEOUT");
    CHECK(output_all_equal(&out, 0), "故障时归零");
}

static void test_descent_recovery(void)
{
    lift_guard_t g;
    lift_guard_output_t out;
    lift_guard_config_t c = base_config();
    lift_guard_input_t in = base_input();

    printf("  [8] 缓降途中命令恢复应立刻交还控制权\n");
    lift_guard_init(&g, &c);

    in.command_fresh = false;
    set_speed(&in, -500);
    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_DESCENDING, "先进入缓降");

    in.command_fresh = true;
    in.now_ms = 500U;
    set_request(&in, 2000);
    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_READY, "恢复为 READY");
    CHECK(output_all_equal(&out, 2000), "恢复执行新请求");
    CHECK(out.descent_elapsed_ms == 0U, "缓降计时清零");
}

static void test_limit_at_top(void)
{
    lift_guard_t g;
    lift_guard_output_t out;
    lift_guard_config_t c = base_config();
    lift_guard_input_t in = base_input();
    uint32_t ch;

    printf("  [9] 到上限后向外的电流被拦，向内的放行\n");
    lift_guard_init(&g, &c);

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        in.position_ecd[ch] = 20000;
    }

    set_request(&in, 5000);
    step(&g, &out, &in);
    CHECK(output_all_equal(&out, 0), "向外推被拦为 0");
    CHECK(out.limit_blocked[0], "限位拦截标志置位");

    set_request(&in, -5000);
    step(&g, &out, &in);
    CHECK(output_all_equal(&out, -5000), "向内拉正常放行");
    CHECK(!out.limit_blocked[0], "不再标记拦截");
}

static void test_limit_at_bottom(void)
{
    lift_guard_t g;
    lift_guard_output_t out;
    lift_guard_config_t c = base_config();
    lift_guard_input_t in = base_input();
    uint32_t ch;

    printf("  [10] 到下限后向外的电流被拦，向内的放行\n");
    lift_guard_init(&g, &c);

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        in.position_ecd[ch] = 0;
    }

    set_request(&in, -5000);
    step(&g, &out, &in);
    CHECK(output_all_equal(&out, 0), "向外推被拦为 0");
    CHECK(out.limit_blocked[0], "限位拦截标志置位");

    set_request(&in, 5000);
    step(&g, &out, &in);
    CHECK(output_all_equal(&out, 5000), "向内拉正常放行");
}

static void test_fault_latch(void)
{
    lift_guard_t g;
    lift_guard_output_t out;
    lift_guard_config_t c = base_config();
    lift_guard_input_t in = base_input();

    printf("  [11] 故障锁存，只有 reset 上升沿能清除\n");
    lift_guard_init(&g, &c);

    in.online[0] = false;
    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_FAULT, "离线 -> FAULT");

    in.now_ms = 10U;
    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_FAULT, "仍离线，维持 FAULT");

    in.now_ms = 20U;
    in.online[0] = true;
    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_FAULT, "已恢复在线但未复位，仍锁存");

    in.now_ms = 30U;
    in.reset_request = true;
    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_READY, "reset 上升沿清除故障");
    CHECK(out.fault == LIFT_GUARD_FAULT_NONE, "故障码清零");

    /* reset 保持高电平不应每周期反复复位。 */
    in.now_ms = 40U;
    in.command_fresh = false;
    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_DESCENDING, "reset 保持高不产生额外作用");
}

static void test_reset_with_still_offline(void)
{
    lift_guard_t g;
    lift_guard_output_t out;
    lift_guard_config_t c = base_config();
    lift_guard_input_t in = base_input();

    printf("  [12] 离线未消失时复位应立即重新报故障\n");
    lift_guard_init(&g, &c);

    in.online[1] = false;
    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_FAULT, "离线 -> FAULT");

    in.now_ms = 10U;
    in.reset_request = true;
    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_FAULT, "复位后仍离线 -> 再次 FAULT");
    CHECK(out.fault == LIFT_GUARD_FAULT_OFFLINE, "故障码仍为 OFFLINE");
}

static void test_inverted_direction_channel(void)
{
    lift_guard_t g;
    lift_guard_output_t out;
    lift_guard_config_t c = base_config();
    lift_guard_input_t in = base_input();

    printf("  [13] 方向取反的通道：限位与缓降都要按符号解释\n");
    c.direction[1] = -1;
    c.descent_current[1] = 3000;   /* direction=-1 时，正电流才是下降 */
    lift_guard_init(&g, &c);

    in.position_ecd[1] = 10000;
    in.command_fresh = false;
    in.speed_rpm[1] = -500;
    step(&g, &out, &in);
    CHECK(out.output_current[1] == 3000, "反向通道缓降电流符号正确");
    CHECK(out.at_target[1] == false, "尚未到底");

    /* 该通道已到上限，requested 正电流对它是「向下」，不应被拦。 */
    in.command_fresh = true;
    in.position_ecd[1] = 20000;
    in.requested_current[1] = 5000;
    in.requested_current[0] = 0;
    step(&g, &out, &in);
    CHECK(out.output_current[1] == 5000, "反向通道到上限时向下电流放行");
    CHECK(!out.limit_blocked[1], "不应误判为越界");

    /* 同位置给负电流对它是「向上」，必须拦。 */
    in.requested_current[1] = -5000;
    step(&g, &out, &in);
    CHECK(out.output_current[1] == 0, "反向通道到上限时向上电流被拦");
    CHECK(out.limit_blocked[1], "限位拦截标志置位");
}

static void test_config_normalization(void)
{
    lift_guard_t g;
    lift_guard_output_t out;
    lift_guard_config_t c = base_config();
    lift_guard_input_t in = base_input();

    printf("  [14] 配置写错时自动纠正\n");
    c.current_limit = 2000;
    c.descent_current[0] = -30000;   /* 超限 */
    c.descent_stall_speed_rpm = -30; /* 负值 */
    c.direction[2] = 0;              /* 非法方向 */
    c.position_min_ecd[3] = 5000;    /* 上下界写反 */
    c.position_max_ecd[3] = 100;
    lift_guard_init(&g, &c);

    CHECK(g.config.descent_current[0] == -2000, "缓降电流被硬限幅");
    CHECK(g.config.descent_stall_speed_rpm == 30, "负转速阈值被取正");
    CHECK(g.config.direction[2] == 1, "非法方向回落到 +1");
    CHECK(g.config.position_min_ecd[3] == 100, "上下界写反被交换");
    CHECK(g.config.position_max_ecd[3] == 5000, "上下界写反被交换");

    in.command_fresh = true;
    set_request(&in, 0);
    step(&g, &out, &in);
    CHECK(out.state == LIFT_GUARD_STATE_READY, "修正后仍能正常工作");
}

static void test_null_safety(void)
{
    lift_guard_t g;
    lift_guard_output_t out;
    lift_guard_input_t in = base_input();
    lift_guard_config_t c = base_config();

    printf("  [15] 空指针不崩\n");
    lift_guard_init(NULL, &c);
    lift_guard_init(&g, NULL);
    lift_guard_update(NULL, &in, &out);
    lift_guard_update(&g, NULL, &out);
    lift_guard_update(&g, &in, NULL);
    CHECK(!lift_guard_has_fault(NULL), "has_fault(NULL) 返回 false");
    CHECK(true, "空指针路径未崩溃");
}

int main(void)
{
    printf("lift_guard 单元测试\n");

    test_disarmed_before_homing();
    test_normal_and_clamp();
    test_offline_fault();
    test_descent_starts();
    test_descent_reaches_bottom();
    test_descent_target_is_not_position_min();
    test_descent_stall_fault();
    test_descent_timeout_fault();
    test_descent_recovery();
    test_limit_at_top();
    test_limit_at_bottom();
    test_fault_latch();
    test_reset_with_still_offline();
    test_inverted_direction_channel();
    test_config_normalization();
    test_null_safety();

    printf("\n共 %d 项断言，失败 %d 项\n", g_checks, g_failures);
    if (g_failures == 0)
    {
        printf("结果：全部通过\n");
        return 0;
    }
    printf("结果：存在失败\n");
    return 1;
}
