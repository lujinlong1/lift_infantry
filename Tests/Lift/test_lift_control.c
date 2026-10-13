/*
 * lift_control 主机侧单元测试。
 *
 * 编译：见 run_tests.ps1（需链接 pid.c 与 lift_sync.c）。
 * 该文件只用于主机验证，不进 Keil 固件工程。
 */

#include "lift_control.h"
#include "lift_sync.h"

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

/*
 * 两条环的默认测试参数：
 *   外环 kp=0.1、output_limit=3000（模态速度量纲 ecd/s）
 *   内环 kp=1.0、output_limit=3000（电流量纲）
 * 内环 kp 取 1.0 是为了让断言读起来还是"外环输出多少、控制量就是多少"，
 * 把串级当成一个单位增益的传递来测；串级本身的阻尼特性由 lift_properties 覆盖。
 */
static lift_control_config_t base_config(void)
{
    lift_control_config_t c;

    /*
     * ⚠️ 必须先清零：这个结构有十几个字段，以前只填了一部分，剩下的读到的是
     * 栈上垃圾。2026-10-11 加了几个平滑链字段之后，栈布局一变就暴露出
     * "限幅区间是垃圾 → 限幅比例被压成 0 → 所有出力都是 0"。
     * 测试夹具必须自己保证确定初值，不能靠运气。
     */
    memset(&c, 0, sizeof(c));

    c.pid[LIFT_CONTROL_LOOP_POSITION].kp = 0.1F;
    c.pid[LIFT_CONTROL_LOOP_POSITION].ki = 0.0F;
    c.pid[LIFT_CONTROL_LOOP_POSITION].kd = 0.0F;
    c.pid[LIFT_CONTROL_LOOP_POSITION].integral_limit = 0.0F;
    c.pid[LIFT_CONTROL_LOOP_POSITION].output_limit = 3000.0F;
    c.pid[LIFT_CONTROL_LOOP_SPEED].kp = 1.0F;
    c.pid[LIFT_CONTROL_LOOP_SPEED].ki = 0.0F;
    c.pid[LIFT_CONTROL_LOOP_SPEED].kd = 0.0F;
    c.pid[LIFT_CONTROL_LOOP_SPEED].integral_limit = 0.0F;
    c.pid[LIFT_CONTROL_LOOP_SPEED].output_limit = 3000.0F;

    lift_sync_config_rect(&c.sync);
    c.ecd_per_rev = 8192;
    c.gear_ratio = 1.0F;
    c.speed_is_rotor = false;
    c.max_target_step_ecd = 0;   /* 默认不限制斜坡 */
    c.period_s = 0.005F;
    c.settle_tolerance_ecd = 100;
    /*
     * 平滑链默认【全部关闭】，这样老用例测的还是"纯串级"的行为；
     * 平滑链自己的行为由 [7]~[10] 显式打开参数来测。
     */
    c.approach_zone_ecd = 0;
    c.approach_min_speed_ecd_s = 0;
    c.hold_speed_ecd_s = 0;
    c.current_ramp_per_s = 0;
    c.current_limit_start = 0;
    c.current_limit_ramp_per_s = 0;
    c.speed_filter_tau_ms = 0;
    c.gravity_feedforward_current = 0.0F;
    c.feedforward_min_position_ecd = 0;
    c.actuator_gain_ecd_s_per_current = 0.0F;
    return c;
}

static void fill_position(int32_t pos[LIFT_CHANNEL_COUNT], int32_t value)
{
    uint32_t ch;
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        pos[ch] = value;
    }
}

static void fill_speed(int16_t spd[LIFT_CHANNEL_COUNT], int16_t value)
{
    uint32_t ch;
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        spd[ch] = value;
    }
}

static void test_disabled_outputs_zero(void)
{
    lift_control_t ctl;
    lift_control_config_t cfg = base_config();
    lift_control_command_t cmd;
    lift_sync_effort_t effort;
    lift_control_status_t st;
    int32_t pos[LIFT_CHANNEL_COUNT];
    int16_t spd[LIFT_CHANNEL_COUNT];

    printf("  [1] 未使能时输出为零且不复位斜坡\n");
    lift_control_init(&ctl, &cfg);
    fill_position(pos, 1000);
    fill_speed(spd, 0);

    cmd.enabled = false;
    cmd.target_height_ecd = 20000;
    cmd.target_roll_ecd = 0;
    cmd.target_pitch_ecd = 0;

    lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    CHECK(effort.height == 0 && effort.roll == 0 && effort.pitch == 0,
          "未使能输出全零");
    CHECK(st.actual.height == 4000, "仍然汇报实际模态");
    CHECK(!st.at_target, "未使能不报到位");
}

static void test_enable_snaps_ramp_to_current(void)
{
    lift_control_t ctl;
    lift_control_config_t cfg = base_config();
    lift_control_command_t cmd;
    lift_sync_effort_t effort;
    lift_control_status_t st;
    int32_t pos[LIFT_CHANNEL_COUNT];
    int16_t spd[LIFT_CHANNEL_COUNT];

    printf("  [2] 首次使能不得产生冲击\n");
    lift_control_init(&ctl, &cfg);
    fill_position(pos, 1000);   /* 实际高度 = 4000 */
    fill_speed(spd, 0);

    cmd.enabled = true;
    cmd.target_height_ecd = 4000;   /* 与当前位置一致 */
    cmd.target_roll_ecd = 0;
    cmd.target_pitch_ecd = 0;

    lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    CHECK(st.ramped_height_ecd == 4000, "斜坡吸附到当前高度");
    CHECK(effort.height == 0, "无误差即无出力");
    CHECK(st.at_target, "到位标志置位");
}

static void test_target_ramp(void)
{
    lift_control_t ctl;
    lift_control_config_t cfg = base_config();
    lift_control_command_t cmd;
    lift_sync_effort_t effort;
    lift_control_status_t st;
    int32_t pos[LIFT_CHANNEL_COUNT];
    int16_t spd[LIFT_CHANNEL_COUNT];
    int i;

    printf("  [3] 目标斜坡 + 跟踪前馈\n");
    /*
     * 步长取 1 计数/周期 → 速率 = 1/0.005 = 200 模态 ecd/s。
     * 取这么小是为了让"跟踪前馈"和"位置环输出"都落在速度环的线性区里
     * （3000 限幅以内），能直接读出数值。
     */
    cfg.max_target_step_ecd = 1;
    lift_control_init(&ctl, &cfg);
    fill_position(pos, 0);
    fill_speed(spd, 0);

    cmd.enabled = true;
    cmd.target_height_ecd = 5000;
    cmd.target_roll_ecd = 0;
    cmd.target_pitch_ecd = 0;

    lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    CHECK(st.ramped_height_ecd == 1, "第 1 周期只走一步");

    lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    CHECK(st.ramped_height_ecd == 2, "第 2 周期再走一步");
    /*
     * ★ 跟踪前馈（2026-10-11 从台架移植）：目标以 R 在走时，速度指令直接加 R。
     *   位置环那一份是 误差(2) × kp(0.1) = 0.2 → 取整 0，所以这里应当等于 R=200。
     *   没有这个前馈的话，目标走多快轴就落后多少（要靠误差攒出 R）。
     */
    CHECK(st.target_speed_height == 200, "目标速度 = 跟踪前馈给出的斜坡速率");
    CHECK(effort.height == 200, "控制量跟随斜坡速率（速度环单位增益）");

    for (i = 0; i < 6000; i++)
    {
        lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    }
    CHECK(st.ramped_height_ecd == 5000, "足够多周期后到达目标");

    /*
     * 斜坡关掉（步长 0）= 目标阶跃：此时速率算出来是"无穷大"，
     * 跟踪前馈【必须】强制为 0，否则一拍就把内环顶到限幅
     * （台架模块踩过同一个坑：host 实测算出来 2e7 ecd/s）。
     */
    cfg.max_target_step_ecd = 0;
    lift_control_init(&ctl, &cfg);
    lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    CHECK(st.ramped_height_ecd == 5000, "斜坡关掉时目标一步到位");
    CHECK(st.target_speed_height == 500, "斜坡关掉时前馈为 0（只有位置环输出）");
}

static void test_height_command_and_sync(void)
{
    lift_control_t ctl;
    lift_control_config_t cfg = base_config();
    lift_control_command_t cmd;
    lift_sync_effort_t effort;
    lift_control_status_t st;
    int32_t pos[LIFT_CHANNEL_COUNT];
    int16_t spd[LIFT_CHANNEL_COUNT];
    int32_t cur[LIFT_CHANNEL_COUNT];

    printf("  [4] 高度指令经 lift_sync 分配后四轴等力且翘曲为 0\n");
    lift_control_init(&ctl, &cfg);
    fill_position(pos, 0);
    fill_speed(spd, 0);

    cmd.enabled = true;
    cmd.target_height_ecd = 20000;
    cmd.target_roll_ecd = 0;
    cmd.target_pitch_ecd = 0;

    lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    CHECK(effort.height > 0, "低于目标时输出正向高度控制量");
    CHECK(effort.roll == 0 && effort.pitch == 0, "未要求横滚俯仰就不出力");

    lift_sync_distribute(&cfg.sync, &effort, cur);
    CHECK(cur[0] == cur[1] && cur[1] == cur[2] && cur[2] == cur[3],
          "四轴电流相等");
    CHECK(lift_sync_warp_component(&cfg.sync, cur) == 0, "翘曲分量为 0");
    CHECK(!st.at_target, "误差很大时不到位");
}

static void test_roll_command_signs(void)
{
    lift_control_t ctl;
    lift_control_config_t cfg = base_config();
    lift_control_command_t cmd;
    lift_sync_effort_t effort;
    lift_control_status_t st;
    int32_t pos[LIFT_CHANNEL_COUNT];
    int16_t spd[LIFT_CHANNEL_COUNT];
    int32_t cur[LIFT_CHANNEL_COUNT];

    printf("  [5] 横滚指令的符号方向\n");
    lift_control_init(&ctl, &cfg);
    fill_position(pos, 1000);   /* 已水平 */
    fill_speed(spd, 0);

    cmd.enabled = true;
    cmd.target_height_ecd = 4000;
    cmd.target_roll_ecd = 800;
    cmd.target_pitch_ecd = 0;

    lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    CHECK(effort.roll > 0, "要求左高右低");
    lift_sync_distribute(&cfg.sync, &effort, cur);
    CHECK(cur[1] > 0 && cur[2] > 0, "左前左后为正");
    CHECK(cur[0] < 0 && cur[3] < 0, "右前右后为负");
    CHECK(cur[1] == -cur[0], "左右等值反向");
}

/*
 * 速度环专用的新增用例：
 * 四轴都在向下走（速度为负），高度模态速度 = 4 × (-2) 换算后的值。
 * 内环必须把这个速度反馈进去——如果内环被旁路掉（老的单环实现），
 * 输出就只等于外环输出，这条断言会失败。
 */
static void test_speed_loop_is_in_the_path(void)
{
    lift_control_t ctl;
    lift_control_config_t cfg = base_config();
    lift_control_command_t cmd;
    lift_sync_effort_t effort;
    lift_control_status_t st;
    int32_t pos[LIFT_CHANNEL_COUNT];
    int16_t spd[LIFT_CHANNEL_COUNT];
    int32_t expected_modal_speed;

    printf("  [6] 带速度反馈时内环必须参与计算\n");
    lift_control_init(&ctl, &cfg);
    fill_position(pos, 0);
    fill_speed(spd, -2);        /* 输出侧 -2 rpm */
    cmd.enabled = true;
    cmd.target_height_ecd = 0;
    cmd.target_roll_ecd = 0;
    cmd.target_pitch_ecd = 0;

    /*
     * -2 rpm -> -2 × 8192 × 6 / 360 = -273 ecd/s（每轴），
     * 高度模态 = 4 × (-273) = -1092 ecd/s。
     */
    expected_modal_speed = -1092;
    lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    CHECK(st.actual_speed.height == expected_modal_speed,
          "模态速度由 speed_rpm 换算并正变换得到");
    CHECK(st.actual_speed.roll == 0 && st.actual_speed.pitch == 0,
          "全体同向运动不产生横滚俯仰");

    /* 外环无误差 -> 目标速度为 0 -> 内环只能看到"实际在往下走"，故输出反向。 */
    CHECK(st.target_speed_height == 0, "位置无误差时目标速度为零");
    CHECK(effort.height > 0,
          "速度环看到向下运动应输出正向控制量（阻尼方向）");
}

static void test_speed_rotor_switch_scales_by_gear_ratio(void)
{
    lift_control_t ctl;
    lift_control_config_t cfg = base_config();
    lift_control_command_t cmd;
    lift_sync_effort_t effort;
    lift_control_status_t st;
    int32_t pos[LIFT_CHANNEL_COUNT];
    int16_t spd[LIFT_CHANNEL_COUNT];
    int32_t output_side;
    int32_t rotor_side;

    printf("  [7] speed_is_rotor 开关改变速度反馈量纲\n");
    fill_position(pos, 0);
    fill_speed(spd, 100);
    cmd.enabled = true;
    cmd.target_height_ecd = 0;
    cmd.target_roll_ecd = 0;
    cmd.target_pitch_ecd = 0;

    cfg.gear_ratio = 1.0F;
    cfg.speed_is_rotor = false;
    lift_control_init(&ctl, &cfg);
    lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    output_side = st.actual_speed.height;

    cfg.speed_is_rotor = true;
    lift_control_init(&ctl, &cfg);
    lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    rotor_side = st.actual_speed.height;

    CHECK(output_side != 0, "输出侧解释非零");
    CHECK(rotor_side != 0, "转子侧解释非零");

    /*
     * 减速比 19.2 时，同一份 speed_rpm 在两种解释下相差约 19 倍。
     * 这里把齿轮比设成 19.2 直接钉住这个倍数关系。
     */
    cfg.gear_ratio = 19.2F;
    cfg.speed_is_rotor = false;
    lift_control_init(&ctl, &cfg);
    lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    output_side = st.actual_speed.height;

    cfg.speed_is_rotor = true;
    lift_control_init(&ctl, &cfg);
    lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    rotor_side = st.actual_speed.height;

    CHECK(output_side / rotor_side >= 19 && output_side / rotor_side <= 20,
          "两种转速语义相差约一个减速比（约 19 倍）");
}

static void test_bad_sync_config_falls_back(void)
{
    lift_control_t ctl;
    lift_control_config_t cfg = base_config();
    uint32_t ch;

    printf("  [8] 权重不合法时自动回落到矩形布局\n");
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        cfg.sync.w_height[ch] = 0;
        cfg.sync.w_roll[ch] = 0;
        cfg.sync.w_pitch[ch] = 0;
        cfg.sync.w_warp[ch] = 0;
    }

    lift_control_init(&ctl, &cfg);
    CHECK(lift_sync_config_is_orthogonal(&ctl.config.sync),
          "初始化后权重必定合法");
}

static void test_null_safety(void)
{
    lift_control_t ctl;
    lift_control_config_t cfg = base_config();
    lift_control_command_t cmd;
    lift_sync_effort_t effort;
    lift_control_status_t st;
    int32_t pos[LIFT_CHANNEL_COUNT];
    int16_t spd[LIFT_CHANNEL_COUNT];

    printf("  [9] 空指针不崩\n");
    fill_position(pos, 0);
    fill_speed(spd, 0);
    cmd.enabled = true;
    cmd.target_height_ecd = 0;
    cmd.target_roll_ecd = 0;
    cmd.target_pitch_ecd = 0;

    lift_control_init(NULL, &cfg);
    lift_control_init(&ctl, NULL);
    lift_control_reset(NULL);
    lift_sync_config_rect(&cfg.sync);
    lift_control_init(&ctl, &cfg);

    lift_control_update(NULL, pos, spd, &cmd, &effort, &st);
    lift_control_update(&ctl, NULL, spd, &cmd, &effort, &st);
    lift_control_update(&ctl, pos, NULL, &cmd, &effort, &st);
    lift_control_update(&ctl, pos, spd, NULL, &effort, &st);
    lift_control_update(&ctl, pos, spd, &cmd, NULL, &st);
    lift_control_update(&ctl, pos, spd, &cmd, &effort, NULL);
    CHECK(true, "空指针路径未崩溃");
}

/* ==================================================================== */
/*  [7]~[10] 平滑链（2026-10-11 从台架模块移植的五个环节）                */
/* ==================================================================== */

static void test_approach_narrowing(void)
{
    lift_control_t ctl;
    lift_control_config_t cfg = base_config();
    lift_control_command_t cmd;
    lift_sync_effort_t effort;
    lift_control_status_t st;
    int32_t pos[LIFT_CHANNEL_COUNT];
    int16_t spd[LIFT_CHANNEL_COUNT];

    printf("  [7] 接近减速：收窄的是速度指令，且收到非零下限\n");

    cfg.pid[LIFT_CONTROL_LOOP_POSITION].kp = 10.0F;  /* 让位置环输出足够大 */
    cfg.settle_tolerance_ecd = 100;
    cfg.approach_zone_ecd = 2000;
    cfg.approach_min_speed_ecd_s = 400;
    lift_control_init(&ctl, &cfg);
    fill_position(pos, 0);
    fill_speed(spd, 0);

    cmd.enabled = true;
    cmd.target_height_ecd = 5000;
    cmd.target_roll_ecd = 0;
    cmd.target_pitch_ecd = 0;

    /* 误差 5000 远大于减速区 2000 → 不减速，速度指令 = 位置环输出(10000)被限幅 3000 */
    lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    CHECK(st.target_speed_height == 3000, "减速区之外不减速");

    /* 误差 1000：收窄到 400 + (3000-400)×(1000-100)/(2000-100) = 1631 */
    cmd.target_height_ecd = 1000;
    lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    CHECK(st.target_speed_height == 1631, "减速区内按距离线性收窄");

    /* 误差 150（刚出死区）：收窄到 468，但**绝不能到 0**（到 0 会卡在死区外） */
    cmd.target_height_ecd = 150;
    lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    CHECK(st.target_speed_height == 468, "贴近死区时仍有可用的速度下限");
    CHECK(st.target_speed_height >= cfg.approach_min_speed_ecd_s,
          "目标速度不低于 approach_min_speed_ecd_s");
}

static void test_brake_and_hold(void)
{
    lift_control_t ctl;
    lift_control_config_t cfg = base_config();
    lift_control_command_t cmd;
    lift_sync_effort_t effort;
    lift_control_status_t st;
    int32_t pos[LIFT_CHANNEL_COUNT];
    int16_t spd[LIFT_CHANNEL_COUNT];
    int i;

    printf("  [8] 到位后刹停 → HOLD 只留保持前馈\n");

    cfg.settle_tolerance_ecd = 100;
    cfg.hold_speed_ecd_s = 500;
    cfg.gravity_feedforward_current = 3000.0F;   /* 托住车身的保持电流 */
    cfg.feedforward_min_position_ecd = 0;
    /*
     * 速度环输出限幅要留够余量：主链把【PID + 前馈】一起钳在这一层，
     * 所以前馈会吃掉 PID 的余量（这正是电流上限该有的行为）。
     * 想验证"退回 APPROACH 后 PID 重新出力"，就得让上限高于前馈。
     */
    cfg.pid[LIFT_CONTROL_LOOP_SPEED].output_limit = 10000.0F;
    lift_control_init(&ctl, &cfg);
    fill_position(pos, 1000);   /* 实际高度 4000，最低轴 1000 > 门槛 0 */
    fill_speed(spd, 0);

    cmd.enabled = true;
    cmd.target_height_ecd = 4000;   /* 与当前位置一致 → 误差 0 */
    cmd.target_roll_ecd = 0;
    cmd.target_pitch_ecd = 0;

    lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    CHECK((lift_control_hold_state_t)st.hold_state[LIFT_CONTROL_MODE_HEIGHT] ==
              LIFT_CONTROL_HOLD_HOLD, "误差在死区内且停住 → HOLD");
    CHECK(effort.height == 3000, "HOLD 时输出就是保持前馈");
    CHECK(st.target_speed_height == 0, "HOLD 时速度指令为 0");
    CHECK(st.pid_output[LIFT_CONTROL_LOOP_SPEED][LIFT_CONTROL_MODE_HEIGHT] ==
              0.0F, "HOLD 时两条环退出（不再算 PID）");
    CHECK(st.at_target, "HOLD 视为到位");

    /* 目标重新走远 → 退出 HOLD，回到 APPROACH 并清掉两环历史 */
    cmd.target_height_ecd = 6000;
    lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    CHECK((lift_control_hold_state_t)st.hold_state[LIFT_CONTROL_MODE_HEIGHT] ==
              LIFT_CONTROL_HOLD_APPROACH, "误差变大 → 退回 APPROACH");
    CHECK(effort.height > 3000, "退回 APPROACH 后 PID 重新出力（叠加前馈）");

    /* 关掉 hold_speed（=0）时不应进入 HOLD：老行为必须保留 */
    cfg.hold_speed_ecd_s = 0;
    lift_control_init(&ctl, &cfg);
    fill_position(pos, 1000);
    cmd.target_height_ecd = 4000;
    for (i = 0; i < 3; i++)
    {
        lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    }
    CHECK((lift_control_hold_state_t)st.hold_state[LIFT_CONTROL_MODE_HEIGHT] !=
              LIFT_CONTROL_HOLD_HOLD, "hold_speed=0 时永不进 HOLD");
}

static void test_open_limit_and_current_ramp(void)
{
    lift_control_t ctl;
    lift_control_config_t cfg = base_config();
    lift_control_command_t cmd;
    lift_sync_effort_t effort;
    lift_control_status_t st;
    int32_t pos[LIFT_CHANNEL_COUNT];
    int16_t spd[LIFT_CHANNEL_COUNT];
    int i;
    int32_t previous;
    int32_t delta;

    printf("  [9] 逐级开放 + 电流斜坡（di/dt）\n");

    cfg.pid[LIFT_CONTROL_LOOP_POSITION].kp = 10.0F;
    cfg.settle_tolerance_ecd = 0;          /* 不进 BRAKE，保持 APPROACH */
    cfg.current_limit_start = 2000;
    cfg.current_limit_ramp_per_s = 10000;  /* 每 5 ms 放开 50 */
    cfg.current_ramp_per_s = 20000;        /* 每 5 ms 最多变化 100 */
    lift_control_init(&ctl, &cfg);
    fill_position(pos, 0);
    fill_speed(spd, 0);

    cmd.enabled = true;
    cmd.target_height_ecd = 5000;
    cmd.target_roll_ecd = 0;
    cmd.target_pitch_ecd = 0;

    /* 第 1 拍：开放上限 = start = 2000；电流斜坡让它只能走 100 */
    lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    CHECK(st.limit_open_current[LIFT_CONTROL_MODE_HEIGHT] == 2000,
          "第 1 拍上限 = current_limit_start");
    CHECK(effort.height == 100, "第 1 拍电流受斜坡限制（20000×0.005）");

    /* 第 2 拍：上限放开 50，电流再走 100 */
    lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    CHECK(st.limit_open_current[LIFT_CONTROL_MODE_HEIGHT] == 2050,
          "上限按 current_limit_ramp_per_s 放开");
    CHECK(effort.height == 200, "电流继续按斜坡上升");

    /* 长跑：单拍变化量永远不超过斜坡步长，最终爬到"开放上限" */
    previous = effort.height;
    for (i = 0; i < 200; i++)
    {
        lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
        delta = effort.height - previous;
        if (delta < 0)
        {
            delta = -delta;
        }
        CHECK(delta <= 100, "单周期电流变化不超过斜坡步长");
        previous = effort.height;
    }
    CHECK(st.limit_open_current[LIFT_CONTROL_MODE_HEIGHT] >= 3000,
          "上限最终放开到超过速度环输出限幅");
    CHECK(effort.height == 3000, "稳态电流停在速度环输出限幅上");
}

static void test_speed_filter(void)
{
    lift_control_t ctl;
    lift_control_config_t cfg = base_config();
    lift_control_command_t cmd;
    lift_sync_effort_t effort;
    lift_control_status_t st;
    int32_t pos[LIFT_CHANNEL_COUNT];
    int16_t spd[LIFT_CHANNEL_COUNT];

    printf("  [10] 速度反馈低通\n");

    /*
     * 四轴都往上跑 1 rpm → 单轴 1×8192×6/360 = 136（取整）
     * → 模态速度 = 四轴之和 = 544。
     */
    cfg.settle_tolerance_ecd = 100;
    cfg.speed_filter_tau_ms = 20;   /* α = 5/(20+5) = 0.2 */
    lift_control_init(&ctl, &cfg);
    fill_position(pos, 0);
    fill_speed(spd, 1);

    cmd.enabled = true;
    cmd.target_height_ecd = 400;   /* 位置环输出 = 0.1×400 = 40 */
    cmd.target_roll_ecd = 0;
    cmd.target_pitch_ecd = 0;

    lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    CHECK(st.actual_speed.height == 544, "模态速度换算正确");
    /*
     * 首次使能时低通状态吸附到【实测速度 544】，所以速度误差 = 40 − 544 = −504，
     * 内环（kp = 1）输出 −504。
     * 若状态是从 0 起步的，第一拍只有 α×544 = 108，输出会变成 −68 —— 差 7 倍，
     * 也就是"凭空的启动冲击"。这条断言就是钉住这个吸附行为。
     */
    CHECK(effort.height == -504, "低通状态吸附实测速度，首拍不产生假误差");

    /* 关掉低通（tau = 0）：反馈直通，稳态行为一致 */
    cfg.speed_filter_tau_ms = 0;
    lift_control_init(&ctl, &cfg);
    fill_speed(spd, 1);
    lift_control_update(&ctl, pos, spd, &cmd, &effort, &st);
    CHECK(st.actual_speed.height == 544, "低通关掉后反馈仍然是真实速度");
    CHECK(effort.height == -504, "低通关掉后首拍输出一致");
}

int main(void)
{
    printf("lift_control 单元测试\n");

    test_disabled_outputs_zero();
    test_enable_snaps_ramp_to_current();
    test_target_ramp();
    test_height_command_and_sync();
    test_roll_command_signs();
    test_speed_loop_is_in_the_path();
    test_speed_rotor_switch_scales_by_gear_ratio();
    test_bad_sync_config_falls_back();
    test_null_safety();
    test_approach_narrowing();
    test_brake_and_hold();
    test_open_limit_and_current_ramp();
    test_speed_filter();

    printf("\n共 %d 项断言，失败 %d 项\n", g_checks, g_failures);
    if (g_failures == 0)
    {
        printf("结果：全部通过\n");
        return 0;
    }
    printf("结果：存在失败\n");
    return 1;
}
