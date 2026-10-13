/*
 * lift_homing 主机侧单元测试。
 *
 * 编译（与学习仓库 C 练习同一套严格标准）：
 *   gcc -std=c11 -Wall -Wextra -Wpedantic -Werror \
 *       test_lift_homing.c lift_homing.c -o test_lift_homing.exe
 *
 * 该文件只用于主机验证，不进 Keil 固件工程。
 */

#include "lift_homing.h"

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

static lift_homing_config_t base_config(void)
{
    lift_homing_config_t c;
    c.seek_current = 1000;
    c.backoff_current = -1000;
    c.current_limit = 2000;
    c.stall_speed_rpm = 50;
    c.seek_min_travel_ecd = 8192;
    c.stall_hold_ms = 100U;
    c.backoff_ecd = 4096;
    c.seek_timeout_ms = 3000U;
    c.backoff_timeout_ms = 3000U;
    /*
     * 前置段默认关闭（0），保持"旧行为"作为基准：
     * 这样原有那批用例仍然只考验 SEEK/BACKOFF 本身。
     * 前置段另有专门用例（test_preposition_*）覆盖。
     */
    c.preposition_current = 0;
    c.preposition_ecd = 0;
    c.preposition_timeout_ms = 3000U;
    return c;
}

static void step(
    lift_homing_t *h,
    lift_homing_output_t *out,
    int32_t pos,
    int16_t rpm,
    bool online,
    bool start,
    bool abort,
    uint32_t now)
{
    lift_homing_input_t in;
    in.position_ecd = pos;
    in.speed_rpm = rpm;
    in.online = online;
    in.start_request = start;
    in.abort_request = abort;
    in.now_ms = now;
    lift_homing_update(h, &in, out);
}

/* 走到“已确认死点、进入 BACKOFF”为止的公共前段。返回死点位置。 */
static int32_t drive_to_backoff(lift_homing_t *h, lift_homing_output_t *out)
{
    step(h, out, 0, 2000, true, true, false, 0U);
    step(h, out, 8192, 2000, true, false, false, 200U);
    step(h, out, 8704, 0, true, false, false, 300U);
    step(h, out, 8704, 0, true, false, false, 400U);
    return out->zero_ecd;
}

static void test_normal_homing(void)
{
    lift_homing_t h;
    lift_homing_output_t out;
    lift_homing_config_t c = base_config();

    printf("  [1] 正常找零\n");
    lift_homing_init(&h, &c);

    step(&h, &out, 0, 2000, true, true, false, 0U);
    CHECK(out.state == LIFT_HOMING_STATE_SEEK, "启动后进入 SEEK");
    CHECK(out.commanded_current == 1000, "SEEK 输出找零电流");

    step(&h, &out, 4096, 2000, true, false, false, 100U);
    CHECK(out.state == LIFT_HOMING_STATE_SEEK, "行程不足时保持 SEEK");

    step(&h, &out, 8192, 2000, true, false, false, 200U);
    CHECK(out.state == LIFT_HOMING_STATE_SEEK, "转速未下降不判堵转");

    step(&h, &out, 8704, 0, true, false, false, 300U);
    CHECK(out.state == LIFT_HOMING_STATE_SEEK, "刚静止未达保持时间");
    CHECK(out.stall_ms == 0U, "堵转计时首周期为 0");

    step(&h, &out, 8704, 0, true, false, false, 350U);
    CHECK(out.stall_ms == 50U, "堵转计时累计");

    step(&h, &out, 8704, 0, true, false, false, 400U);
    CHECK(out.state == LIFT_HOMING_STATE_BACKOFF, "保持满 100ms 确认死点");
    CHECK(out.zero_ecd == 8704, "零点记录在死点处");
    CHECK(out.commanded_current == -1000, "BACKOFF 输出回退电流");
    CHECK(out.travel_ecd == 0, "BACKOFF 起始行程为 0");

    step(&h, &out, 6656, -1000, true, false, false, 500U);
    CHECK(out.state == LIFT_HOMING_STATE_BACKOFF, "回退行程不足时保持");

    step(&h, &out, 4608, -1000, true, false, false, 600U);
    CHECK(out.state == LIFT_HOMING_STATE_DONE, "回退到位后完成");
    CHECK(out.commanded_current == 0, "完成后电流归零");
    CHECK(out.zero_ecd == 8704, "完成后零点保留");
    CHECK(out.elapsed_ms == 0U, "完成后耗时归零");
    CHECK(lift_homing_is_homed(&h), "is_homed 为真");
}

static void test_no_false_stall_at_start(void)
{
    lift_homing_t h;
    lift_homing_output_t out;
    lift_homing_config_t c = base_config();

    printf("  [2] 启动瞬间速度为 0 不得误判\n");
    lift_homing_init(&h, &c);

    step(&h, &out, 0, 0, true, true, false, 0U);
    step(&h, &out, 0, 0, true, false, false, 1000U);
    step(&h, &out, 0, 0, true, false, false, 2000U);
    CHECK(out.state == LIFT_HOMING_STATE_SEEK, "行程不够时绝不判堵转");
    CHECK(out.zero_ecd == 0, "未确认死点前零点无效");
}

static void test_seek_timeout(void)
{
    lift_homing_t h;
    lift_homing_output_t out;
    lift_homing_config_t c = base_config();

    printf("  [3] 一直碰不到死点 -> 超时故障\n");
    lift_homing_init(&h, &c);

    step(&h, &out, 0, 2000, true, true, false, 0U);
    step(&h, &out, 4096, 2000, true, false, false, 1000U);
    step(&h, &out, 8192, 2000, true, false, false, 3000U);
    CHECK(out.state == LIFT_HOMING_STATE_FAULT, "超时进入 FAULT");
    CHECK(out.fault == LIFT_HOMING_FAULT_SEEK_TIMEOUT, "故障码为 SEEK_TIMEOUT");
    CHECK(out.commanded_current == 0, "故障时电流归零");
    CHECK(!lift_homing_is_homed(&h), "故障不算已找零");
}

static void test_offline(void)
{
    lift_homing_t h;
    lift_homing_output_t out;
    lift_homing_config_t c = base_config();

    printf("  [4] 反馈丢失\n");
    lift_homing_init(&h, &c);

    step(&h, &out, 0, 0, false, true, false, 0U);
    CHECK(out.state == LIFT_HOMING_STATE_FAULT, "启动即离线 -> FAULT");
    CHECK(out.fault == LIFT_HOMING_FAULT_OFFLINE, "故障码为 OFFLINE");

    lift_homing_init(&h, &c);
    step(&h, &out, 0, 2000, true, true, false, 0U);
    step(&h, &out, 1000, 2000, false, false, false, 100U);
    CHECK(out.state == LIFT_HOMING_STATE_FAULT, "途中离线 -> FAULT");
    CHECK(out.fault == LIFT_HOMING_FAULT_OFFLINE, "故障码为 OFFLINE");
    CHECK(out.commanded_current == 0, "离线时电流归零");
}

static void test_idle_has_no_output(void)
{
    lift_homing_t h;
    lift_homing_output_t out;
    lift_homing_config_t c = base_config();

    printf("  [5] 未启动时不得输出电流\n");
    lift_homing_init(&h, &c);

    step(&h, &out, 0, 0, true, false, false, 0U);
    CHECK(out.state == LIFT_HOMING_STATE_IDLE, "保持 IDLE");
    CHECK(out.commanded_current == 0, "IDLE 电流为 0");
    CHECK(out.elapsed_ms == 0U, "IDLE 耗时为 0");
    CHECK(!lift_homing_is_homed(&h), "IDLE 不算已找零");
}

static void test_abort(void)
{
    lift_homing_t h;
    lift_homing_output_t out;
    lift_homing_config_t c = base_config();

    printf("  [6] 中途中止\n");
    lift_homing_init(&h, &c);

    step(&h, &out, 0, 2000, true, true, false, 0U);
    step(&h, &out, 1000, 2000, true, false, true, 100U);
    CHECK(out.state == LIFT_HOMING_STATE_FAULT, "中止 -> FAULT");
    CHECK(out.fault == LIFT_HOMING_FAULT_ABORTED, "故障码为 ABORTED");
    CHECK(out.commanded_current == 0, "中止后电流归零");

    /* 中止后同周期再给启动沿，应能重新开始。 */
    step(&h, &out, 1000, 2000, true, true, false, 200U);
    CHECK(out.state == LIFT_HOMING_STATE_SEEK, "可重新启动找零");
    CHECK(out.fault == LIFT_HOMING_FAULT_NONE, "重新启动清除故障码");
}

static void test_backoff_timeout(void)
{
    lift_homing_t h;
    lift_homing_output_t out;
    lift_homing_config_t c = base_config();

    printf("  [7] 碰到死点却退不出来 -> 故障\n");
    lift_homing_init(&h, &c);
    (void)drive_to_backoff(&h, &out);
    CHECK(out.state == LIFT_HOMING_STATE_BACKOFF, "前段进入 BACKOFF");

    step(&h, &out, 8704, 0, true, false, false, 3400U);
    CHECK(out.state == LIFT_HOMING_STATE_FAULT, "回退超时 -> FAULT");
    CHECK(out.fault == LIFT_HOMING_FAULT_BACKOFF_TIMEOUT, "故障码为 BACKOFF_TIMEOUT");
    CHECK(out.commanded_current == 0, "故障时电流归零");
}

static void test_repeat_homing(void)
{
    lift_homing_t h;
    lift_homing_output_t out;
    lift_homing_config_t c = base_config();

    printf("  [8] 重复找零并更新零点\n");
    lift_homing_init(&h, &c);

    (void)drive_to_backoff(&h, &out);
    step(&h, &out, 4608, -1000, true, false, false, 600U);
    CHECK(out.state == LIFT_HOMING_STATE_DONE, "第一次找零完成");
    CHECK(out.zero_ecd == 8704, "第一次零点为 8704");

    step(&h, &out, 4608, 1000, true, true, false, 1000U);
    CHECK(out.state == LIFT_HOMING_STATE_SEEK, "第二次找零重新进入 SEEK");
    CHECK(out.zero_ecd == 0, "重新找零时旧零点作废");

    step(&h, &out, 12800, 1000, true, false, false, 1100U);
    step(&h, &out, 13312, 0, true, false, false, 1200U);
    step(&h, &out, 13312, 0, true, false, false, 1300U);
    CHECK(out.state == LIFT_HOMING_STATE_BACKOFF, "第二次确认死点");
    CHECK(out.zero_ecd == 13312, "第二次零点更新为 13312");

    step(&h, &out, 9216, -1000, true, false, false, 1400U);
    CHECK(out.state == LIFT_HOMING_STATE_DONE, "第二次找零完成");
}

static void test_config_clamp(void)
{
    lift_homing_t h;
    lift_homing_output_t out;
    lift_homing_config_t c = base_config();

    printf("  [9] 配置写超量时被硬限幅兜住\n");
    c.seek_current = 5000;      /* 超过 current_limit */
    c.backoff_current = -30000;
    c.stall_speed_rpm = -80;    /* 负值应被取正 */
    lift_homing_init(&h, &c);

    step(&h, &out, 0, 2000, true, true, false, 0U);
    CHECK(out.commanded_current == 2000, "SEEK 电流被限到 current_limit");
    CHECK(h.config.stall_speed_rpm == 80, "负的 stall_speed_rpm 被取正");

    step(&h, &out, 8192, 2000, true, false, false, 200U);
    step(&h, &out, 8704, 0, true, false, false, 300U);
    step(&h, &out, 8704, 0, true, false, false, 400U);
    CHECK(out.state == LIFT_HOMING_STATE_BACKOFF, "仍能正常确认死点");
    CHECK(out.commanded_current == -2000, "BACKOFF 电流被限到 -current_limit");
}

/*
 * 前置段（PREPOSITION）——本机构"停车位置就是死点"的专用修复。
 *
 * 场景：车落地时齿条完全缩回、顶在硬死点上。直接 SEEK 会因为
 * "先走够 seek_min_travel_ecd"这条前置条件永远不成立而超时。
 * 正确行为：先主动往反方向走 preposition_ecd 离开死点，再回头找零。
 */
static void test_preposition_leaves_stop_then_seeks(void)
{
    lift_homing_t h;
    lift_homing_output_t out;
    lift_homing_config_t c = base_config();

    printf("  [11] 前置段：先离开死点再找零\n");

    c.preposition_ecd = 2000;
    c.preposition_current = -1000;   /* 与 seek 反号：先离开死点 */
    c.seek_min_travel_ecd = 8192;
    lift_homing_init(&h, &c);

    /* 上电就贴在死点上（位置 0），请求找零。 */
    step(&h, &out, 0, 0, true, true, false, 0U);
    CHECK(out.state == LIFT_HOMING_STATE_PREPOSITION, "先进入前置段");
    CHECK(out.commanded_current == -1000, "前置段输出离开死点的电流");

    /* 还没走够前置行程，继续。 */
    step(&h, &out, -1500, -800, true, false, false, 200U);
    CHECK(out.state == LIFT_HOMING_STATE_PREPOSITION, "行程不足时保持前置段");
    CHECK(out.travel_ecd == -1500, "前置段行程可观测");

    /* 走够 2000 计数，转入 SEEK，且行程基准重置到当前位置。 */
    step(&h, &out, -2000, -800, true, false, false, 400U);
    CHECK(out.state == LIFT_HOMING_STATE_SEEK, "走够前置行程后转入 SEEK");
    CHECK(out.commanded_current == 1000, "SEEK 换成找零电流");
    CHECK(out.travel_ecd == 0, "SEEK 行程基准重新从当前位置起算");

    /* 往回走：从 -2000 回到 -1000（走了 1000，仍不足 8192）。 */
    step(&h, &out, -1000, 800, true, false, false, 500U);
    CHECK(out.state == LIFT_HOMING_STATE_SEEK, "最小行程未满时保持 SEEK");

    /* 走够最小行程后停在死点：应能正常确认堵转并回退。 */
    step(&h, &out, 7000, 800, true, false, false, 900U);
    CHECK(out.state == LIFT_HOMING_STATE_SEEK, "行程够了但还没停");
    step(&h, &out, 7100, 0, true, false, false, 1000U);
    step(&h, &out, 7100, 0, true, false, false, 1100U);
    CHECK(out.state == LIFT_HOMING_STATE_BACKOFF, "能正常确认死点");
    CHECK(out.zero_ecd == 7100, "零点记录在死点处");
}

static void test_preposition_timeout(void)
{
    lift_homing_t h;
    lift_homing_output_t out;
    lift_homing_config_t c = base_config();

    printf("  [12] 前置段走不动时报专用故障码\n");

    c.preposition_ecd = 2000;
    c.preposition_current = -1000;
    c.preposition_timeout_ms = 500U;
    lift_homing_init(&h, &c);

    step(&h, &out, 0, 0, true, true, false, 0U);
    CHECK(out.state == LIFT_HOMING_STATE_PREPOSITION, "进入前置段");

    /* 一直不动（顶死或电流太小）。 */
    step(&h, &out, 0, 0, true, false, false, 300U);
    CHECK(out.state == LIFT_HOMING_STATE_PREPOSITION, "未超时前保持");

    step(&h, &out, 0, 0, true, false, false, 600U);
    CHECK(out.state == LIFT_HOMING_STATE_FAULT, "超时进入故障");
    CHECK(out.fault == LIFT_HOMING_FAULT_PREPOSITION_TIMEOUT,
          "报的是前置段超时而不是找零超时");
    CHECK(out.commanded_current == 0, "故障后电流归零");
}

/*
 * 复现"车落地就贴在死点上、且没有前置段"的旧工况：
 * 必须报 SEEK_TIMEOUT —— 这正是引入前置段的原因，用例把它钉住。
 */
static void test_at_stop_without_preposition_seek_timeout(void)
{
    lift_homing_t h;
    lift_homing_output_t out;
    lift_homing_config_t c = base_config();

    printf("  [13] 贴在死点上且无前置段 -> SEEK_TIMEOUT（前置段存在的理由）\n");

    c.preposition_ecd = 0;
    c.seek_timeout_ms = 1000U;
    lift_homing_init(&h, &c);

    step(&h, &out, 0, 0, true, true, false, 0U);
    CHECK(out.state == LIFT_HOMING_STATE_SEEK, "无前置段时直接 SEEK");

    /* 一动不动：永远走不够 seek_min_travel_ecd。 */
    step(&h, &out, 0, 0, true, false, false, 500U);
    CHECK(out.state == LIFT_HOMING_STATE_SEEK, "待在死点上也判不出堵转");

    step(&h, &out, 0, 0, true, false, false, 1200U);
    CHECK(out.state == LIFT_HOMING_STATE_FAULT, "只能等超时");
    CHECK(out.fault == LIFT_HOMING_FAULT_SEEK_TIMEOUT,
          "报的是 SEEK_TIMEOUT");
}

static void test_null_safety(void)
{
    lift_homing_t h;
    lift_homing_output_t out;
    lift_homing_input_t in;
    lift_homing_config_t c = base_config();

    printf("  [10] 空指针不崩\n");
    in.position_ecd = 0;
    in.speed_rpm = 0;
    in.online = true;
    in.start_request = false;
    in.abort_request = false;
    in.now_ms = 0U;

    lift_homing_init(NULL, &c);
    lift_homing_init(&h, NULL);
    lift_homing_update(NULL, &in, &out);
    lift_homing_update(&h, NULL, &out);
    lift_homing_update(&h, &in, NULL);
    CHECK(!lift_homing_is_homed(NULL), "is_homed(NULL) 返回 false");
    CHECK(true, "空指针路径未崩溃");
}

int main(void)
{
    printf("lift_homing 单元测试\n");

    test_normal_homing();
    test_no_false_stall_at_start();
    test_seek_timeout();
    test_offline();
    test_idle_has_no_output();
    test_abort();
    test_backoff_timeout();
    test_repeat_homing();
    test_config_clamp();
    test_preposition_leaves_stop_then_seeks();
    test_preposition_timeout();
    test_at_stop_without_preposition_seek_timeout();
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
