/*
 * lift_sync 主机侧单元测试。
 *
 * 编译（与学习仓库 C 练习同一套严格标准）：
 *   gcc -std=c11 -Wall -Wextra -Wpedantic -Werror \
 *       test_lift_sync.c lift_sync.c -o test_lift_sync.exe
 *
 * 该文件只用于主机验证，不进 Keil 固件工程。
 */

#include "lift_sync.h"

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

/* 四通道布局常量，便于构造测试向量 */
#define FR 0U
#define FL 1U
#define RL 2U
#define RR 3U

static void fill(int32_t out[LIFT_CHANNEL_COUNT], int32_t a, int32_t b, int32_t c, int32_t d)
{
    out[FR] = a;
    out[FL] = b;
    out[RL] = c;
    out[RR] = d;
}

static int32_t mean_of(const int32_t v[LIFT_CHANNEL_COUNT])
{
    int32_t sum = 0;
    uint32_t ch;
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        sum += v[ch];
    }
    return sum / (int32_t)LIFT_CHANNEL_COUNT;
}

static int32_t max_abs_deviation(const int32_t v[LIFT_CHANNEL_COUNT])
{
    const int32_t mean = mean_of(v);
    int32_t worst = 0;
    uint32_t ch;
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        int32_t d = v[ch] - mean;
        if (d < 0)
        {
            d = -d;
        }
        if (d > worst)
        {
            worst = d;
        }
    }
    return worst;
}

static void test_rect_weights_are_a_basis(void)
{
    lift_sync_config_t cfg;

    printf("  [1] 矩形权重必须是正交基\n");
    lift_sync_config_rect(&cfg);
    CHECK(lift_sync_config_is_orthogonal(&cfg), "四行两两正交且范数为 4");

    CHECK(cfg.w_height[FR] == 1 && cfg.w_height[FL] == 1 &&
          cfg.w_height[RL] == 1 && cfg.w_height[RR] == 1, "高度权重全为 +1");
    CHECK(cfg.w_pitch[FR] == 1 && cfg.w_pitch[FL] == 1 &&
          cfg.w_pitch[RL] == -1 && cfg.w_pitch[RR] == -1, "俯仰为前减后");
    CHECK(cfg.w_roll[FR] == -1 && cfg.w_roll[FL] == 1 &&
          cfg.w_roll[RL] == 1 && cfg.w_roll[RR] == -1, "横滚为左减右");
    CHECK(cfg.w_warp[FR] == 1 && cfg.w_warp[FL] == -1 &&
          cfg.w_warp[RL] == 1 && cfg.w_warp[RR] == -1, "翘曲为对角差");
}

static void test_bad_config_is_rejected(void)
{
    lift_sync_config_t cfg;

    printf("  [2] 自检必须能识别坏权重\n");
    lift_sync_config_rect(&cfg);
    cfg.w_warp[FL] = 1;   /* 把翘曲改成与高度同向 -> 不再正交 */
    CHECK(!lift_sync_config_is_orthogonal(&cfg), "非正交权重被判为不合法");

    lift_sync_config_rect(&cfg);
    cfg.w_roll[FR] = 0;   /* 范数不足 */
    CHECK(!lift_sync_config_is_orthogonal(&cfg), "范数不足被判为不合法");
}

static void test_pure_height_command(void)
{
    lift_sync_config_t cfg;
    lift_sync_effort_t u;
    int32_t cur[LIFT_CHANNEL_COUNT];

    printf("  [3] 纯高度指令应四轴等力\n");
    lift_sync_config_rect(&cfg);

    u.height = 1000;
    u.roll = 0;
    u.pitch = 0;
    lift_sync_distribute(&cfg, &u, cur);

    CHECK(cur[FR] == 1000 && cur[FL] == 1000 &&
          cur[RL] == 1000 && cur[RR] == 1000, "四轴电流相同");
    CHECK(max_abs_deviation(cur) == 0, "无差分电流");
}

static void test_pure_pitch_and_roll(void)
{
    lift_sync_config_t cfg;
    lift_sync_effort_t u;
    int32_t cur[LIFT_CHANNEL_COUNT];

    printf("  [4] 纯俯仰 / 纯横滚指令的符号\n");
    lift_sync_config_rect(&cfg);

    u.height = 0;
    u.roll = 0;
    u.pitch = 500;
    lift_sync_distribute(&cfg, &u, cur);
    CHECK(cur[FR] == 500 && cur[FL] == 500, "前两根同向");
    CHECK(cur[RL] == -500 && cur[RR] == -500, "后两根反向等值");

    u.pitch = 0;
    u.roll = 700;
    lift_sync_distribute(&cfg, &u, cur);
    CHECK(cur[FL] == 700 && cur[RL] == 700, "左两根同向");
    CHECK(cur[FR] == -700 && cur[RR] == -700, "右两根反向等值");
}

static void test_warp_component_is_always_zero(void)
{
    lift_sync_config_t cfg;
    lift_sync_effort_t u;
    int32_t cur[LIFT_CHANNEL_COUNT];
    int32_t h;
    int32_t r;
    int32_t p;

    printf("  [5] 翘曲分量必须恒为 0（这是本模块存在的理由）\n");
    lift_sync_config_rect(&cfg);

    for (h = -3000; h <= 3000; h += 997)
    {
        for (r = -500; r <= 500; r += 331)
        {
            for (p = -500; p <= 500; p += 173)
            {
                u.height = h;
                u.roll = r;
                u.pitch = p;
                lift_sync_distribute(&cfg, &u, cur);
                CHECK(lift_sync_warp_component(&cfg, cur) == 0,
                      "任意模态组合下翘曲分量均为 0");
            }
        }
    }

    /* 反证：人为造一个带翘曲的电流，诊断函数必须能看出来。 */
    fill(cur, 100, -100, 100, -100);
    CHECK(lift_sync_warp_component(&cfg, cur) == 400, "诊断函数能识别翘曲分量");
}

static void test_forward_separates_modes(void)
{
    lift_sync_config_t cfg;
    lift_sync_modes_t m;
    int32_t pos[LIFT_CHANNEL_COUNT];

    printf("  [6] 正变换必须把翘曲从其他三个模态里分离干净\n");
    lift_sync_config_rect(&cfg);

    /* 纯高度 */
    fill(pos, 250, 250, 250, 250);
    lift_sync_forward(&cfg, pos, &m);
    CHECK(m.height == 1000, "height = 4 x 250");
    CHECK(m.roll == 0 && m.pitch == 0 && m.warp == 0, "其余模态为 0");

    /* 纯横滚 */
    fill(pos, -150, 150, 150, -150);
    lift_sync_forward(&cfg, pos, &m);
    CHECK(m.roll == 600, "roll = 4 x 150");
    CHECK(m.height == 0 && m.pitch == 0 && m.warp == 0, "其余模态为 0");

    /* 纯俯仰 */
    fill(pos, 80, 80, -80, -80);
    lift_sync_forward(&cfg, pos, &m);
    CHECK(m.pitch == 320, "pitch = 4 x 80");
    CHECK(m.height == 0 && m.roll == 0 && m.warp == 0, "其余模态为 0");

    /* 纯翘曲：一个角的扰动不得污染高度/横滚/俯仰 */
    fill(pos, 100, -100, 100, -100);
    lift_sync_forward(&cfg, pos, &m);
    CHECK(m.warp == 400, "warp = 4 x 100");
    CHECK(m.height == 0, "翘曲不污染高度");
    CHECK(m.roll == 0, "翘曲不污染横滚");
    CHECK(m.pitch == 0, "翘曲不污染俯仰");
}

static void test_round_trip(void)
{
    lift_sync_config_t cfg;
    lift_sync_effort_t u;
    lift_sync_modes_t m;
    int32_t cur[LIFT_CHANNEL_COUNT];

    printf("  [7] 正反变换的往返关系 forward(distribute(U)) = N x U\n");
    lift_sync_config_rect(&cfg);

    u.height = 1234;
    u.roll = -321;
    u.pitch = 456;
    lift_sync_distribute(&cfg, &u, cur);
    lift_sync_forward(&cfg, cur, &m);

    CHECK(m.height == (int32_t)LIFT_CHANNEL_COUNT * u.height, "高度往返一致");
    CHECK(m.roll == (int32_t)LIFT_CHANNEL_COUNT * u.roll, "横滚往返一致");
    CHECK(m.pitch == (int32_t)LIFT_CHANNEL_COUNT * u.pitch, "俯仰往返一致");
    CHECK(m.warp == 0, "翘曲往返仍为 0");
}

static void test_over_constraint_demo(void)
{
    lift_sync_config_t cfg;
    lift_sync_effort_t u;
    int32_t pos[LIFT_CHANNEL_COUNT];
    int32_t sync_cur[LIFT_CHANNEL_COUNT];
    int32_t naive_cur[LIFT_CHANNEL_COUNT];
    const int32_t target = 10000;
    const int32_t kp = 3;
    uint32_t ch;

    printf("  [8] 过约束现场：地面不平角时谁在较劲\n");
    lift_sync_config_rect(&cfg);

    /* 右后角比另外三个角高 100 个计数（地面不平 / 找零零位有偏差）。 */
    fill(pos, 10000, 10000, 10000, 10100);

    /* 模态空间：只给高度指令。四个通道拿到完全相同的电流 -> 零差分。 */
    u.height = 2000;
    u.roll = 0;
    u.pitch = 0;
    lift_sync_distribute(&cfg, &u, sync_cur);
    CHECK(max_abs_deviation(sync_cur) == 0, "模态控制不产生差分电流");
    CHECK(lift_sync_warp_component(&cfg, sync_cur) == 0, "翘曲分量为 0");

    /* 朴素的每轴位置环：每根丝杆各追各的目标，差多少补多少。 */
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        naive_cur[ch] = kp * (target - pos[ch]);
    }
    /* 只有 RR 一根差 100 计数，kp=3 -> 该轴 -300，离均值 -75 偏 225。 */
    CHECK(naive_cur[RR] == -300, "朴素控制给高出的那根 -300");
    CHECK(max_abs_deviation(naive_cur) == 225,
          "每轴位置环被迫产生 225 的差分电流去对抗几何误差");
    CHECK(lift_sync_warp_component(&cfg, naive_cur) == 300,
          "这些差分电流恰好落在翘曲模态上：纯内力，不产生任何有用运动");
}

static void test_null_safety(void)
{
    lift_sync_config_t cfg;
    lift_sync_effort_t u;
    lift_sync_modes_t m;
    int32_t pos[LIFT_CHANNEL_COUNT];
    int32_t cur[LIFT_CHANNEL_COUNT];

    printf("  [9] 空指针不崩\n");
    lift_sync_config_rect(NULL);
    lift_sync_config_rect(&cfg);

    fill(pos, 0, 0, 0, 0);
    fill(cur, 1, 2, 3, 4);
    u.height = 1;
    u.roll = 0;
    u.pitch = 0;

    lift_sync_forward(NULL, pos, &m);
    lift_sync_forward(&cfg, NULL, &m);
    lift_sync_forward(&cfg, pos, NULL);
    lift_sync_distribute(NULL, &u, cur);
    lift_sync_distribute(&cfg, NULL, cur);
    lift_sync_distribute(&cfg, &u, NULL);
    CHECK(lift_sync_warp_component(NULL, cur) == 0, "warp_component(NULL) 返回 0");
    CHECK(lift_sync_warp_component(&cfg, NULL) == 0, "warp_component(pos=NULL) 返回 0");
    CHECK(!lift_sync_config_is_orthogonal(NULL), "is_orthogonal(NULL) 返回 false");
    CHECK(true, "空指针路径未崩溃");
}

int main(void)
{
    printf("lift_sync 单元测试\n");

    test_rect_weights_are_a_basis();
    test_bad_config_is_rejected();
    test_pure_height_command();
    test_pure_pitch_and_roll();
    test_warp_component_is_always_zero();
    test_forward_separates_modes();
    test_round_trip();
    test_over_constraint_demo();
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
