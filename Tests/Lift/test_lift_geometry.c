/*
 * lift_geometry 位置/速度换算单元测试。
 *
 * 编译：见 run_tests.ps1。该文件只用于主机验证，不进 Keil 固件工程。
 */

#include "lift_geometry.h"

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

static int32_t abs_i32(int32_t v)
{
    return v < 0 ? -v : v;
}

static void test_direct_default(void)
{
    lift_geometry_t g;

    printf("  [1] 直连默认配置：一圈 = 一个导程\n");
    lift_geometry_config_3508_direct(&g, 2000);   /* T8 导程 2mm */
    CHECK(lift_geometry_validate(&g), "配置合法");
    CHECK(g.counts_per_rev == 8192, "每圈计数 8192");

    CHECK(lift_geometry_ecd_to_um(&g, 8192) == 2000, "整圈 -> 2000um");
    CHECK(lift_geometry_ecd_to_um(&g, 4096) == 1000, "半圈 -> 1000um");
    CHECK(lift_geometry_ecd_to_um(&g, 0) == 0, "零 -> 零");
    CHECK(lift_geometry_ecd_to_um(&g, -8192) == -2000, "负方向对称");
    CHECK(lift_geometry_ecd_to_um(&g, 1) == 0, "一个计数不足 1um 时截断为 0");
}

static void test_reverse(void)
{
    lift_geometry_t g;

    printf("  [2] 反向换算\n");
    lift_geometry_config_3508_direct(&g, 2000);

    CHECK(lift_geometry_um_to_ecd(&g, 2000) == 8192, "2000um -> 整圈");
    CHECK(lift_geometry_um_to_ecd(&g, 1000) == 4096, "1000um -> 半圈");
    CHECK(lift_geometry_um_to_ecd(&g, 0) == 0, "零 -> 零");
    CHECK(lift_geometry_um_to_ecd(&g, -500) == -2048, "负方向对称");
}

static void test_round_trip(void)
{
    lift_geometry_t g;
    int32_t ecd;

    printf("  [3] 往返一致性（整圈倍数必须精确）\n");
    lift_geometry_config_3508_direct(&g, 2000);

    for (ecd = -40960; ecd <= 40960; ecd += 819)
    {
        const int32_t um = lift_geometry_ecd_to_um(&g, ecd);
        const int32_t back = lift_geometry_um_to_ecd(&g, um);
        CHECK(abs_i32(back - ecd) <= 6, "往返误差不超过 6 个计数");
    }

    for (ecd = -8192 * 4; ecd <= 8192 * 4; ecd += 8192)
    {
        const int32_t um = lift_geometry_ecd_to_um(&g, ecd);
        CHECK(lift_geometry_um_to_ecd(&g, um) == ecd, "整圈倍数精确往返");
    }
}

static void test_gear_reduction(void)
{
    lift_geometry_t g;

    printf("  [4] 电机到丝杆有减速时\n");
    lift_geometry_config_3508_direct(&g, 2000);
    g.drive_num = 2;   /* 电机转 2 圈，丝杆转 1 圈 */
    g.drive_den = 1;

    CHECK(lift_geometry_ecd_to_um(&g, 16384) == 2000, "电机两圈 -> 一个导程");
    CHECK(lift_geometry_ecd_to_um(&g, 8192) == 1000, "电机一圈 -> 半个导程");
    CHECK(lift_geometry_um_to_ecd(&g, 2000) == 16384, "反向同样成立");
}

static void test_speed_rotor_vs_output(void)
{
    lift_geometry_t g;

    printf("  [5] 转速语义开关：转子侧 / 输出侧\n");
    lift_geometry_config_3508_direct(&g, 2000);

    /* 转子侧：1920 rpm 转子 -> 约 100 rpm 输出 -> 约 3332 um/s */
    g.speed_is_rotor = true;
    CHECK(lift_geometry_speed_to_um_per_s(&g, 1920) == 3332,
          "转子侧换算含 3591/187 减速比");

    /* 输出侧：100 rpm 输出 -> 约 3333 um/s */
    g.speed_is_rotor = false;
    CHECK(lift_geometry_speed_to_um_per_s(&g, 100) == 3333,
          "输出侧直接换算");

    /* 两种解释下同一个"100"差约 19 倍，说明这个开关绝不是小事 */
    g.speed_is_rotor = true;
    {
        const int32_t as_rotor = lift_geometry_speed_to_um_per_s(&g, 100);
        g.speed_is_rotor = false;
        {
            const int32_t as_output = lift_geometry_speed_to_um_per_s(&g, 100);
            CHECK((as_output > as_rotor * 15) && (as_output < as_rotor * 25),
                  "同一数值两种解释相差约 19 倍");
        }
    }

    g.speed_is_rotor = false;
    CHECK(lift_geometry_speed_to_um_per_s(&g, 0) == 0, "零转速 -> 零速度");
    CHECK(lift_geometry_speed_to_um_per_s(&g, -100) == -3333, "负转速对称");
}

static void test_invalid_config(void)
{
    lift_geometry_t g;

    printf("  [6] 非法配置一律返回 0\n");
    lift_geometry_config_3508_direct(&g, 0);
    CHECK(!lift_geometry_validate(&g), "导程为 0 判为非法");
    CHECK(lift_geometry_ecd_to_um(&g, 8192) == 0, "非法时换算返回 0");
    CHECK(lift_geometry_um_to_ecd(&g, 1000) == 0, "非法时反向返回 0");
    CHECK(lift_geometry_speed_to_um_per_s(&g, 100) == 0, "非法时速度返回 0");

    lift_geometry_config_3508_direct(&g, 2000);
    g.counts_per_rev = 0;
    CHECK(!lift_geometry_validate(&g), "每圈计数为 0 判为非法");

    lift_geometry_config_3508_direct(&g, 2000);
    g.drive_num = 0;
    CHECK(!lift_geometry_validate(&g), "传动比分子为 0 判为非法");

    lift_geometry_config_3508_direct(&g, 2000);
    g.speed_is_rotor = true;
    g.gear_num = 0;
    CHECK(!lift_geometry_validate(&g), "转子侧但减速比为 0 判为非法");

    lift_geometry_config_3508_direct(&g, 2000);
    g.speed_is_rotor = false;
    g.gear_num = 0;   /* 输出侧不需要减速比 */
    CHECK(lift_geometry_validate(&g), "输出侧不要求减速比");
}

static void test_saturation(void)
{
    lift_geometry_t g;

    printf("  [7] 极端值饱和而不是回绕\n");
    lift_geometry_config_3508_direct(&g, 2000000);   /* 2 米导程，故意夸张 */
    CHECK(lift_geometry_ecd_to_um(&g, 2147483647) == 2147483647,
          "正向溢出饱和到 INT32_MAX");

    lift_geometry_config_3508_direct(&g, 2000);
    CHECK(lift_geometry_speed_to_um_per_s(&g, -32768) < 0,
          "极端负转速仍为负，不回绕");
}

static void test_null_safety(void)
{
    lift_geometry_t g;
    lift_geometry_config_3508_direct(&g, 2000);

    printf("  [8] 空指针不崩\n");
    lift_geometry_config_3508_direct(NULL, 2000);
    CHECK(!lift_geometry_validate(NULL), "validate(NULL) 为 false");
    CHECK(lift_geometry_ecd_to_um(NULL, 8192) == 0, "ecd_to_um(NULL) 返回 0");
    CHECK(lift_geometry_um_to_ecd(NULL, 1000) == 0, "um_to_ecd(NULL) 返回 0");
    CHECK(lift_geometry_speed_to_um_per_s(NULL, 100) == 0, "speed(NULL) 返回 0");
    CHECK(true, "空指针路径未崩溃");
}

int main(void)
{
    printf("lift_geometry 换算单元测试\n");

    test_direct_default();
    test_reverse();
    test_round_trip();
    test_gear_reduction();
    test_speed_rotor_vs_output();
    test_invalid_config();
    test_saturation();
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
