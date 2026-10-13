/*
 * lift_kinematics 运动学换算单元测试。
 *
 * 覆盖（对应任务的第九节「运动学测试」）：
 *   · 0 mm 换算
 *   · 正负方向换算
 *   · 200 mm 换算
 *   · 各轴不同零点（在 lift_kin_position_from_raw 上验证）
 *   · 非法轴号
 *   · 超限位置
 *   · 编码器回绕（含整整一圈、半圈边界、值域越界）
 *   · 正向换算与逆向换算的一致性（往返误差 < 1 mm）
 *   · 系数与机械常数的自洽（实测 750.0 计数/mm，见 lift_kinematics.h）
 *   · 四轴极差与偏差
 *   · 标定就绪判据（没量过行程的轴不得接受绝对位置命令）
 *
 * 编译：见 run_tests.ps1。该文件只用于主机验证，不进 Keil 固件工程。
 */

#include "lift_kinematics.h"

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

/* 实车实测的方向：ch0/ch2 反向，ch1/ch3 正向（见 lift_control_task.c）。 */
static const int8_t k_dir[LIFT_CHANNEL_COUNT] = { -1, 1, -1, 1 };

/*
 * 标称配置：8192 计数/圈、13.43:1 减速比、46.693 mm 分度圆。
 * 全部来自 2026-10-11 的用户手测（150000 计数 = 200 mm；齿轮一圈 110000 计数）。
 */

static lift_kin_config_t nominal_config(void)
{
    lift_kin_config_t c;
    uint32_t ch;

    lift_kin_config_init_uncalibrated(&c);
    lift_kin_config_nominal_rack_and_pinion(&c, k_dir);

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        c.position_min_mm[ch] = 0;
        /* 行程按实测的 200 mm 配（软限位 = 硬限位，见 lift_control_task.c）。 */
        c.position_max_mm[ch] = 200;
    }
    return c;
}

/* ---------------------------------------------------------------- */

static void test_counts_per_mm(void)
{
    lift_kin_config_t c = nominal_config();
    /* 定点版是 counts/mm × 1000；浮点版是 counts/mm 本身。 */
    int32_t ppm1000 = lift_kin_counts_per_mm_x1000(&c, 0);
    float ppm = lift_kin_counts_per_mm(&c, 0);
    const int32_t nominal1000 = (int32_t)(LIFT_KIN_NOMINAL_COUNTS_PER_MM * 1000.0F);

    printf("  [1] counts/mm 与机械常数自洽\n");

    /*
     * 2026-10-11 实测定案：
     *   缩回死点 → 伸出硬限位 = 150 000 计数 = 200 mm  →  750.0 计数/mm
     *   齿轮转一圈 = 110 000 计数 = 146.7 mm           →  749.9 计数/mm
     * 两条独立测量互相印证，所以断言按 750.0，容差 ±0.5%。
     */
    CHECK(ppm1000 > (nominal1000 * 995) / 1000,
          "counts/mm 不应显著低于标称 750.0");
    CHECK(ppm1000 < (nominal1000 * 1005) / 1000,
          "counts/mm 不应显著高于标称 750.0");
    CHECK((ppm > 746.0F) && (ppm < 754.0F),
          "浮点值应约等于 750.0 counts/mm");
    /*
     * 两种表示必须来自同一套派生系数。允许 0.001 counts/mm 的差异 ——
     * 定点版是 counts/mm × 1000 的整数截断，理论误差小于 0.001。
     */
    CHECK((ppm - ((float)ppm1000 / 1000.0F)) < 0.001F,
          "定点值 / 1000 必须等于浮点值（两种表示不能各算一套）");
    CHECK((((float)ppm1000 / 1000.0F) - ppm) < 0.001F,
          "定点值 / 1000 必须等于浮点值（两种表示不能各算一套）");

    /* 四轴系数必须相同（同一套机械常数），但各自独立存储。 */
    CHECK(lift_kin_counts_per_mm_x1000(&c, 0) ==
              lift_kin_counts_per_mm_x1000(&c, 3),
          "四轴机械常数相同时系数应相同");

    /*
     * 改一个轴的齿轮直径 → 只有那个轴变。
     *
     * ⚠️ 必须先关掉实测覆盖（counts_per_mm = 0），否则位置换算由实测比例
     *    锚定，改直径**不会**影响它 —— 那是设计行为，不是缺陷。
     *    覆盖本身的行为由 [1b] 单独验。
     */
    {
        const int32_t before0 = lift_kin_counts_per_mm_x1000(&c, 0);
        const float d_base = c.axis[1].pinion_diameter_mm;
        int32_t at_base;
        int32_t at_twice;

        c.axis[1].counts_per_mm = 0.0F;          /* 关掉覆盖，走派生 */

        /* 基准直径下的派生值 */
        c.axis[1].pinion_diameter_mm = d_base;
        at_base = lift_kin_counts_per_mm_x1000(&c, 1);

        /* 直径翻倍 → 系数减半 */
        c.axis[1].pinion_diameter_mm = d_base * 2.0F;
        at_twice = lift_kin_counts_per_mm_x1000(&c, 1);

        CHECK(at_base > 0, "基准直径应给出正的系数");
        /* 严格比较：at_twice ≈ at_base / 2，容差 0.1% */
        CHECK(at_twice < (at_base / 2) + (at_base / 1000),
              "关掉覆盖后，直径翻倍应让系数减半");
        CHECK(at_twice > (at_base / 2) - (at_base / 1000),
              "关掉覆盖后，直径翻倍应让系数减半（不能差太多）");
        CHECK(lift_kin_counts_per_mm_x1000(&c, 0) == before0,
              "改一个轴不得影响其他轴");
    }

    /*
     * [1b] ★ 实测覆盖必须是权威。
     *
     * 机构参数（直径/减速比）是反推出来的中间量，派生的 counts/mm 落不到
     * 整数（46.693 mm → 750.0072，46.694 mm → 749.9919）。标定的权威是
     * 「150000 计数 = 200 mm」这个比例本身，所以覆盖优先。
     */
    {
        lift_kin_config_t o = nominal_config();

        CHECK(lift_kin_counts_per_mm_x1000(&o, 0) == 750000,
              "★ 实测覆盖必须给出精确的 750.000 计数/mm");

        /* 覆盖有效时，改直径不改位置换算 */
        o.axis[0].pinion_diameter_mm = 100.0F;
        CHECK(lift_kin_counts_per_mm_x1000(&o, 0) == 750000,
              "★ 覆盖有效时位置换算锚定实测值，不随直径漂移");

        /* 覆盖关掉后，直径才起作用 */
        o.axis[0].counts_per_mm = 0.0F;
        CHECK(lift_kin_counts_per_mm_x1000(&o, 0) != 750000,
              "关掉覆盖后应由直径派生（用来交叉检查几何参数）");
    }
}

static void test_zero_and_200mm(void)
{
    lift_kin_config_t c = nominal_config();
    int32_t ecd = 0;
    int32_t back = 0;
    int32_t expected;

    printf("  [2] 0 mm / 200 mm / 正负方向\n");

    /* 0 mm → 0 计数（逻辑零点是出口） */
    CHECK(lift_kin_mm_to_ecd(&c, 0, 0, &ecd), "0 mm 换算应成功");
    CHECK(ecd == 0, "0 mm 必须映射到 0 计数");

    CHECK(lift_kin_ecd_to_mm(&c, 0, 0, &back), "0 计数换算应成功");
    CHECK(back == 0, "0 计数必须映射到 0 mm");

    /* 200 mm：750.0 × 200 = 150 000 计数（×4 轴 = 600 000 模态） */
    CHECK(lift_kin_mm_to_ecd(&c, 0, 200, &ecd), "200 mm 换算应成功");
    expected = (int32_t)(((int64_t)lift_kin_counts_per_mm_x1000(&c, 0) * 200) /
                         1000);
    CHECK((ecd > expected - 2) && (ecd < expected + 2),
          "200 mm 应约为 counts_per_mm × 200");
    printf("      200 mm -> %d 计数（四轴合计 %d）\n", ecd, ecd * 4);
    /*
     * ★ 这一条是实测锚点：用户手测「缩回死点 → 伸出硬限位 = 150000 计数
     *   = 200 mm」。换算必须落在这个数附近，不能是别的量级。
     */
    CHECK((ecd >= 149000) && (ecd <= 150000),
          "200 mm 必须约为 150000 计数（实测：150000 计数 = 200 mm）");
    CHECK(ecd != 145143, "200 mm 不应再等于旧图纸推算出的 145143");
    CHECK(ecd != 25000, "200 mm 绝不能被算成旧的占位行程上限");

    /* 正负方向必须严格反对称 */
    {
        int32_t plus = 0;
        int32_t minus = 0;

        CHECK(lift_kin_mm_to_ecd(&c, 1, 150, &plus), "+150 mm 换算应成功");
        CHECK(lift_kin_mm_to_ecd(&c, 1, -150, &minus), "-150 mm 换算应成功");
        CHECK(plus == -minus, "正负方向换算必须严格反对称");
    }

    /* 方向：ch0 的 direction = -1，ch1 的 = +1。
     * 【逻辑】计数必须同号（正 = 抬升），原始域的差异由调用方折掉。 */
    {
        int32_t e0 = 0;
        int32_t e1 = 0;

        (void)lift_kin_mm_to_ecd(&c, 0, 200, &e0);
        (void)lift_kin_mm_to_ecd(&c, 1, 200, &e1);
        CHECK(e0 == e1, "同一 mm 目标在逻辑域必须得到同样的计数");
    }
}

static void test_roundtrip(void)
{
    lift_kin_config_t c = nominal_config();
    int32_t mm;

    printf("  [3] 正反换算一致性（往返误差 < 1 mm）\n");

    for (mm = -200; mm <= 260; mm += 7)
    {
        uint32_t ch;

        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            int32_t ecd = 0;
            int32_t back = 0;

            CHECK(lift_kin_mm_to_ecd(&c, ch, mm, &ecd), "正向换算必须成功");
            CHECK(lift_kin_ecd_to_mm(&c, ch, ecd, &back), "逆向换算必须成功");
            CHECK((back >= mm - 1) && (back <= mm + 1),
                  "往返换算误差必须小于 1 mm");
        }
    }
}

static void test_per_axis_zero(void)
{
    lift_kin_config_t c = nominal_config();
    /*
     * 四轴各自不同的零点（原始编码器域）。这正是"不能假定四轴计数相同"
     * 的那件事：同样的逻辑高度，原始计数完全不同。
     */
    const int32_t raw_zero[LIFT_CHANNEL_COUNT] = { 120000, -80000, 45000, -300000 };
    uint32_t ch;

    printf("  [4] 各轴不同零点\n");

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        c.zero_ecd[ch] = raw_zero[ch];
    }

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        int32_t ecd = 0;

        /* 轴在零点的原始位置 → 逻辑位置 0 */
        CHECK(lift_kin_position_from_raw(&c, ch, raw_zero[ch], &ecd),
              "零点换算应成功");
        CHECK(ecd == 0, "原始位置 == 零点时逻辑位置必须为 0");

        /* 走 200 mm 之后，各轴的【逻辑】计数相同，但【原始】位置不同。 */
        CHECK(lift_kin_mm_to_ecd(&c, ch, 200, &ecd), "200 mm 换算应成功");

        {
            /* direction = -1 的轴，逻辑 +ecd 对应原始 -ecd。 */
            const int32_t logical = ecd;
            const int32_t raw_after = (k_dir[ch] < 0)
                                          ? (raw_zero[ch] - logical)
                                          : (raw_zero[ch] + logical);
            int32_t back = 0;

            CHECK(lift_kin_position_from_raw(&c, ch, raw_after, &back),
                  "抬升后的换算应成功");
            CHECK(back == logical, "不同零点的轴必须都算回同一个逻辑高度");
        }
    }

    /* 四轴零点互不相同 → 原始计数必然互不相同（这正是要防的假设）。 */
    {
        int32_t raw[LIFT_CHANNEL_COUNT];
        uint32_t i;
        bool all_same = true;

        for (i = 0U; i < LIFT_CHANNEL_COUNT; i++)
        {
            int32_t e = 0;
            (void)lift_kin_mm_to_ecd(&c, i, 200, &e);
            raw[i] = (k_dir[i] < 0) ? (raw_zero[i] - e) : (raw_zero[i] + e);
        }
        for (i = 1U; i < LIFT_CHANNEL_COUNT; i++)
        {
            if (raw[i] != raw[0])
            {
                all_same = false;
            }
        }
        CHECK(!all_same,
              "每轴零点不同时，同一个物理高度对应的原始计数【不应】相同");
    }
}

static void test_invalid_inputs(void)
{
    lift_kin_config_t c = nominal_config();
    int32_t out = 12345;
    uint32_t bad_axis = (uint32_t)LIFT_CHANNEL_COUNT;

    printf("  [5] 非法轴号 / 超范围 / 非法配置\n");

    /* 非法轴号：返回 false 且【不修改】输出 */
    CHECK(!lift_kin_mm_to_ecd(&c, bad_axis, 100, &out), "非法轴号必须失败");
    CHECK(out == 12345, "失败时不得修改输出");
    CHECK(!lift_kin_ecd_to_mm(&c, bad_axis, 100, &out), "非法轴号必须失败");
    CHECK(out == 12345, "失败时不得修改输出");
    CHECK(!lift_kin_position_from_raw(&c, bad_axis, 0, &out),
          "非法轴号必须失败");
    CHECK(!lift_kin_axis_is_ready(&c, bad_axis), "非法轴号不得就绪");
    CHECK(lift_kin_counts_per_mm_x1000(&c, bad_axis) == 0,
          "非法轴号的系数必须为 0");

    /* 空指针 */
    CHECK(!lift_kin_mm_to_ecd(NULL, 0, 0, &out), "空配置必须失败");
    CHECK(!lift_kin_mm_to_ecd(&c, 0, 0, NULL), "空输出必须失败");
    CHECK(!lift_kin_mm_to_ecd(&c, 0, 0, NULL), "空输出必须失败");
    CHECK(lift_kin_counts_per_mm(NULL, 0) == 0.0F, "空配置系数为 0");
    CHECK(!lift_kin_config_is_valid(NULL), "空配置非法");
    CHECK(!lift_kin_all_ready(NULL), "空配置不得就绪");
    CHECK(!lift_kin_validate(NULL, NULL, 0.0F), "空配置校验必须失败");

    /* 超范围位置（超过 LIFT_KIN_MAX_TRAVEL_MM） */
    out = 999;
    CHECK(!lift_kin_mm_to_ecd(&c, 0, LIFT_KIN_MAX_TRAVEL_MM + 1, &out),
          "超过最大行程必须失败");
    CHECK(!lift_kin_mm_to_ecd(&c, 0, -(LIFT_KIN_MAX_TRAVEL_MM + 1), &out),
          "超过最大负行程必须失败");
    CHECK(out == 999, "失败时不得修改输出");

    /* 配置非法：计数为 0 / 减速比为 0 / 直径为 0 / 方向为 0 */
    {
        lift_kin_config_t bad = nominal_config();

        bad.axis[2].counts_per_rev = 0;
        CHECK(!lift_kin_axis_config_is_valid(&bad.axis[2]), "计数为 0 非法");
        CHECK(!lift_kin_config_is_valid(&bad), "任一轴非法则整份非法");
        CHECK(!lift_kin_axis_is_ready(&bad, 2), "非法轴不得就绪");
        /* 其他轴不受影响 */
        CHECK(lift_kin_axis_config_is_valid(&bad.axis[0]), "其他轴仍合法");

        bad = nominal_config();
        bad.axis[0].gear_ratio = 0.0F;
        CHECK(!lift_kin_axis_config_is_valid(&bad.axis[0]), "减速比为 0 非法");

        bad = nominal_config();
        bad.axis[0].pinion_diameter_mm = -1.0F;
        CHECK(!lift_kin_axis_config_is_valid(&bad.axis[0]), "负直径非法");

        bad = nominal_config();
        bad.axis[0].direction = 0;
        CHECK(!lift_kin_axis_config_is_valid(&bad.axis[0]), "方向为 0 非法");
        bad.axis[0].direction = 2;
        CHECK(!lift_kin_axis_config_is_valid(&bad.axis[0]), "方向必须只有 ±1");

        bad = nominal_config();
        bad.axis[1].gear_ratio = 1.0e12F;   /* 天文数字 */
        CHECK(!lift_kin_axis_config_is_valid(&bad.axis[1]), "过大减速比非法");
    }
}

static void test_readiness(void)
{
    lift_kin_config_t c;

    printf("  [6] 标定就绪判据（没量过行程不得走绝对位置）\n");

    /* 完全未标定 */
    lift_kin_config_init_uncalibrated(&c);
    CHECK(!lift_kin_all_ready(&c), "未标定不得就绪");
    CHECK(!lift_kin_axis_is_ready(&c, 0), "未标定的轴不得就绪");
    CHECK(!lift_kin_config_is_valid(&c), "未标定的配置非法");

    /* 只填了机械常数、没填行程 → 仍不得就绪 */
    lift_kin_config_init_uncalibrated(&c);
    lift_kin_config_nominal_rack_and_pinion(&c, k_dir);
    CHECK(lift_kin_config_is_valid(&c), "填了机械常数后配置应合法（能换算）");
    CHECK(!lift_kin_axis_is_ready(&c, 0),
          "行程未配置时不得接受绝对位置命令");
    CHECK(!lift_kin_all_ready(&c), "行程未配置时整组不得就绪");

    /* 行程区间倒置 = 未配置 */
    c.position_min_mm[0] = 100;
    c.position_max_mm[0] = 100;
    CHECK(!lift_kin_axis_is_ready(&c, 0), "min >= max 视为未配置");

    /* 超出 LIFT_KIN_MAX_TRAVEL_MM 的行程一定是写错了 */
    c.position_min_mm[0] = 0;
    c.position_max_mm[0] = LIFT_KIN_MAX_TRAVEL_MM + 1;
    CHECK(!lift_kin_axis_is_ready(&c, 0), "行程超过硬上限必须判非法");

    /* 配好四轴行程 → 就绪 */
    {
        uint32_t ch;

        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            c.position_min_mm[ch] = 0;
            c.position_max_mm[ch] = 230;
        }
    }
    CHECK(lift_kin_all_ready(&c), "四轴行程配好后应就绪");
}

static void test_limits(void)
{
    lift_kin_config_t c = nominal_config();
    int32_t mn = -1;
    int32_t mx = -1;
    int32_t v;

    printf("  [7] 软限位与越界判定\n");

    CHECK(lift_kin_limits_mm(&c, 0, &mn, &mx), "已配置的限位应可读出");
    CHECK((mn == 0) && (mx == 200),
          "限位区间应与配置一致（实测行程 0..200 mm）");

    v = 100;
    CHECK(!lift_kin_clamp_mm(&c, 0, &v), "区间内不应被夹");
    CHECK(v == 100, "区间内不得改动");

    v = 300;
    CHECK(lift_kin_clamp_mm(&c, 0, &v), "越上限应被夹");
    CHECK(v == 200, "应夹到上限");

    v = -50;
    CHECK(lift_kin_clamp_mm(&c, 0, &v), "越下限应被夹");
    CHECK(v == 0, "应夹到下限");

    CHECK(!lift_kin_out_of_limits(&c, 0, 0), "下限本身不算越界");
    CHECK(!lift_kin_out_of_limits(&c, 0, 200), "上限本身不算越界");
    CHECK(lift_kin_out_of_limits(&c, 0, 201), "超过上限算越界");
    CHECK(lift_kin_out_of_limits(&c, 0, -1), "低于下限算越界");

    /* 未配置限位：limits 返回 false，clamp 不夹，越界判定为"无法判定"=false */
    {
        lift_kin_config_t u;

        lift_kin_config_init_uncalibrated(&u);
        lift_kin_config_nominal_rack_and_pinion(&u, k_dir);
        CHECK(!lift_kin_limits_mm(&u, 0, &mn, &mx), "未配置限位应返回 false");
        v = 999;
        CHECK(!lift_kin_clamp_mm(&u, 0, &v), "未配置限位时不夹");
        CHECK(v == 999, "未配置限位时不得改动");
        CHECK(!lift_kin_out_of_limits(&u, 0, 999), "未配置限位时无法判越界");
        CHECK(!lift_kin_clamp_mm(&c, (uint32_t)LIFT_CHANNEL_COUNT, &v),
              "非法轴号必须失败");
    }
}

static void test_wrap(void)
{
    int32_t delta = 0;

    printf("  [8] 编码器回绕\n");

    /* 同圈 */
    CHECK(lift_kin_wrap_delta(8192, 100, 150, &delta), "同圈换算应成功");
    CHECK(delta == 50, "同圈正位移");
    CHECK(lift_kin_wrap_delta(8192, 150, 100, &delta), "同圈换算应成功");
    CHECK(delta == -50, "同圈负位移");

    /* 跨越 8191 → 0（正方向跨圈） */
    CHECK(lift_kin_wrap_delta(8192, 8190, 10, &delta), "跨圈换算应成功");
    CHECK(delta == 12, "正跨圈：8190 -> 10 应为 +12");

    /* 跨越 0 → 8191（负方向跨圈） */
    CHECK(lift_kin_wrap_delta(8192, 10, 8190, &delta), "跨圈换算应成功");
    CHECK(delta == -12, "负跨圈：10 -> 8190 应为 -12");

    /* 整整一圈 */
    CHECK(lift_kin_wrap_delta(8192, 4000, 4000, &delta), "整圈换算应成功");
    CHECK(delta == 0, "回到同一计数应为 0");

    /* 半圈边界：±4096 不修正（与 dji_motor.c 的严格 > 判断一致） */
    CHECK(lift_kin_wrap_delta(8192, 0, 4096, &delta), "半圈换算应成功");
    CHECK(delta == 4096, "+4096 不修正");
    CHECK(lift_kin_wrap_delta(8192, 8191, 4096, &delta), "半圈应成功");
    CHECK(delta == -4095, "-4095 不修正");

    /* 刚过半圈：修正 */
    CHECK(lift_kin_wrap_delta(8192, 0, 4097, &delta), "过半圈应成功");
    CHECK(delta == 4097 - 8192, "+4097 应修正为 -4095");

    /* 值域越界 */
    delta = 777;
    CHECK(!lift_kin_wrap_delta(8192, -1, 100, &delta), "last 越界必须失败");
    CHECK(delta == 777, "失败时不得修改输出");
    CHECK(!lift_kin_wrap_delta(8192, 100, 8192, &delta), "new 越界必须失败");
    CHECK(!lift_kin_wrap_delta(0, 0, 0, &delta), "cpr 为 0 必须失败");
    CHECK(!lift_kin_wrap_delta(8192, 0, 0, NULL), "空输出必须失败");
}

static void test_speed(void)
{
    lift_kin_config_t c = nominal_config();
    float mm_s = 0.0F;
    int16_t rpm = 0;

    printf("  [9] 速度换算\n");

    /*
     * 2026-10-11 实测：齿轮一圈 110000 计数 = 146.7 mm 齿条位移，减速比 13.43。
     *   1000 转子 rpm → 1000/13.43 = 74.46 输出 rpm = 1.241 输出圈/s
     *                 → 1.241 × 146.7 = 182.0 mm/s
     * 再乘方向：ch1 = +1 → +182.0；ch0 = -1 → -182.0（逻辑正 = 抬升）。
     */
    CHECK(lift_kin_speed_to_mm_s(&c, 1, 1000, &mm_s), "速度换算应成功");
    CHECK((mm_s > 175.0F) && (mm_s < 190.0F),
          "1000 转子 rpm 应约 182 mm/s");

    CHECK(lift_kin_speed_to_mm_s(&c, 0, 1000, &mm_s), "速度换算应成功");
    CHECK((mm_s < -175.0F) && (mm_s > -190.0F),
          "direction = -1 的轴应给出反号");

    /* 零点转速 */
    CHECK(lift_kin_speed_to_mm_s(&c, 0, 0, &mm_s), "0 rpm 换算应成功");
    CHECK(mm_s == 0.0F, "0 rpm 必须给出 0 mm/s");

    /* 往返：mm/s → rpm → mm/s */
    {
        const float table[5] = { 1.0F, 10.0F, 50.0F, -25.0F, -100.0F };
        int i;

        for (i = 0; i < 5; i++)
        {
            float back = 0.0F;
            uint32_t ch;

            for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
            {
                CHECK(lift_kin_mm_s_to_speed(&c, ch, table[i], &rpm),
                      "反向速度换算应成功");
                CHECK(lift_kin_speed_to_mm_s(&c, ch, rpm, &back),
                      "正向速度换算应成功");
                CHECK((back > table[i] - 1.0F) && (back < table[i] + 1.0F),
                      "速度往返误差应小于 1 mm/s");
            }
        }
    }

    CHECK(!lift_kin_speed_to_mm_s(&c, (uint32_t)LIFT_CHANNEL_COUNT, 0, &mm_s),
          "非法轴号必须失败");
    CHECK(!lift_kin_speed_to_mm_s(NULL, 0, 0, &mm_s), "空配置必须失败");
    CHECK(!lift_kin_speed_to_mm_s(&c, 0, 0, NULL), "空输出必须失败");
}

static void test_spread_and_deviation(void)
{
    lift_kin_config_t c = nominal_config();
    int32_t ecd[LIFT_CHANNEL_COUNT];
    int32_t spread = -1;
    int32_t dev = 0;
    uint32_t ch;

    printf("  [10] 四轴极差与偏差\n");

    /* 四轴相同高度：极差 0，偏差 0 */
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        CHECK(lift_kin_mm_to_ecd(&c, ch, 200, &ecd[ch]), "换算应成功");
    }
    CHECK(lift_kin_position_spread_mm(&c, ecd, &spread), "极差应可算");
    CHECK(spread == 0, "四轴同高时极差必须为 0");
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        CHECK(lift_kin_axis_deviation_mm(&c, ecd, ch, &dev), "偏差应可算");
        CHECK(dev == 0, "四轴同高时偏差必须为 0");
    }

    /*
     * 单轴偏高 10 mm：极差 10。
     *
     * 偏差用【中位参考】= 去掉最高最低之后剩下的平均（四轴时即中间两根的
     * 均值），不是四轴平均。理由：平均会被跑偏的那根拖走，越偏越测不出来；
     * 中位参考不受单轴异常影响，"这一根比其它几根高多少"才是判据要问的问题。
     * 其余三根同高 → 中间两根就是 200 → 偏高那根偏差正好 +10。
     */
    CHECK(lift_kin_mm_to_ecd(&c, 1, 210, &ecd[1]), "换算应成功");
    CHECK(lift_kin_position_spread_mm(&c, ecd, &spread), "极差应可算");
    CHECK(spread == 10, "极差应为 10 mm");
    CHECK(lift_kin_axis_deviation_mm(&c, ecd, 1, &dev), "偏差应可算");
    CHECK(dev >= 9, "偏高 10 mm 的轴相对其余轴应为 +10 mm");
    CHECK(dev <= 11, "偏高 10 mm 的轴相对其余轴应为 +10 mm");
    CHECK(lift_kin_axis_deviation_mm(&c, ecd, 0, &dev), "偏差应可算");
    CHECK((dev >= -1) && (dev <= 0), "其余轴相对参考应为 0 mm");

    /* 单轴偏低 -> 负号 */
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        (void)lift_kin_mm_to_ecd(&c, ch, 200, &ecd[ch]);
    }
    (void)lift_kin_mm_to_ecd(&c, 3, 180, &ecd[3]);
    CHECK(lift_kin_position_spread_mm(&c, ecd, &spread), "极差应可算");
    CHECK(spread == 20, "极差应为 20 mm");
    CHECK(lift_kin_axis_deviation_mm(&c, ecd, 3, &dev), "偏差应可算");
    CHECK((dev <= -19) && (dev >= -21), "偏低 20 mm 的轴应为 -20 mm");

    /*
     * 两根偏高、一根偏低、一根正常：{215, 200, 215, 180}。
     * 去掉一个最高(215)与一个最低(180) → 参考 = (200 + 215) / 2 = 207。
     * 于是偏差是 +8 / −7 / +8 / −27 —— 都是【相对其余轴】的真实差值，
     * 而不是相对"包含自己在内的平均"（那会给出 +7.5 / −7.5 / +7.5 / −27.5，
     * 单轴异常时判据被稀释）。
     */
    (void)lift_kin_mm_to_ecd(&c, 0, 215, &ecd[0]);
    (void)lift_kin_mm_to_ecd(&c, 2, 215, &ecd[2]);
    CHECK(lift_kin_axis_deviation_mm(&c, ecd, 0, &dev), "偏差应可算");
    CHECK(dev == 8, "偏高那根应报 +8 mm（相对其余轴 215 vs 207）");
    CHECK(lift_kin_axis_deviation_mm(&c, ecd, 1, &dev), "偏差应可算");
    CHECK(dev == -7, "正常那根应报 -7 mm");
    CHECK(lift_kin_axis_deviation_mm(&c, ecd, 3, &dev), "偏差应可算");
    CHECK(dev == -27, "偏低那根应报 -27 mm");
    CHECK(lift_kin_position_spread_mm(&c, ecd, &spread), "极差应可算");
    CHECK(spread == 35, "极差应为 35 mm");

    /* 非法输入 */
    CHECK(!lift_kin_position_spread_mm(&c, NULL, &spread), "空位置必须失败");
    CHECK(!lift_kin_position_spread_mm(&c, ecd, NULL), "空输出必须失败");
    CHECK(!lift_kin_axis_deviation_mm(&c, ecd, (uint32_t)LIFT_CHANNEL_COUNT, &dev),
          "非法轴号必须失败");
}

static void test_calibration_report(void)
{
    lift_kin_config_t c = nominal_config();
    lift_kin_calibration_t cal;
    const int32_t nominal100 = (int32_t)(LIFT_KIN_NOMINAL_COUNTS_PER_MM * 100.0F);

    printf("  [11] 标定报告与标称值偏差\n");

    CHECK(lift_kin_validate(&c, &cal, 50.0F), "标称配置应通过校验");
    CHECK(cal.all_valid, "四轴都应合法");
    CHECK(cal.all_ready, "四轴都应就绪");
    CHECK(!cal.deviates_from_nominal[0], "标称配置不应偏离标称值");
    CHECK(cal.counts_per_mm_x100[0] > (nominal100 * 99) / 100,
          "报告里的系数应在 750 量级");
    CHECK(cal.counts_per_mm_x100[0] < (nominal100 * 101) / 100,
          "报告里的系数应在 750 量级");
    CHECK(cal.counts_per_mm_x100[0] == 75000,
          "★ 实测锚点：报告必须给出精确的 750.00 计数/mm");

    /*
     * 把齿轮直径改成一半。
     *
     * ⚠️ 必须同时关掉实测覆盖，否则系数由实测值锚定、不随直径变，
     *    "偏离标称"这条判据就永远不会触发（那是设计行为）。
     *    这里的意图是验**派生路径**的偏离检测。
     */
    c.axis[0].counts_per_mm = 0.0F;
    c.axis[0].pinion_diameter_mm = 34.5F;
    CHECK(lift_kin_validate(&c, &cal, 50.0F), "配置仍然合法");
    CHECK(cal.deviates_from_nominal[0], "直径减半必须报偏离标称");
    CHECK(!cal.deviates_from_nominal[1], "其他轴不应被牵连");

    /* out 为 NULL 也能用（只取返回值） */
    CHECK(lift_kin_validate(&c, NULL, 50.0F), "out 为 NULL 时应只返回结果");
}

int main(void)
{
    printf("lift_kinematics 运动学换算单元测试\n");

    test_counts_per_mm();
    test_zero_and_200mm();
    test_roundtrip();
    test_per_axis_zero();
    test_invalid_inputs();
    test_readiness();
    test_limits();
    test_wrap();
    test_speed();
    test_spread_and_deviation();
    test_calibration_report();

    printf("\n共 %d 项断言，失败 %d 项\n", g_checks, g_failures);
    if (g_failures == 0)
    {
        printf("结果：全部通过\n");
        return 0;
    }
    printf("结果：存在失败\n");
    return 1;
}
