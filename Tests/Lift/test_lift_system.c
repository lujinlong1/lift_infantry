/*
 * 抬升整链集成测试：homing -> control -> sync -> guard -> 机械模型
 *
 * 这是把四个模块接成一条链后、【端到端】跑一遍的验证：
 *     Phase 0  找零之前，任何运动命令都必须被拦住
 *     Phase 1  四个丝杆同时上电堵转找零，零点必须落在机械死点上
 *     Phase 2  使能抬升，整车高度必须收敛到目标且不产生翘曲内力
 *     Phase 3  上层命令超时，必须【受控缓降】而不是掉下来
 *
 * 编译：见 run_tests.ps1。该文件只用于主机验证，不进 Keil 固件工程。
 */

#include "lift_control.h"
#include "lift_guard.h"
#include "lift_homing.h"
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

#define STEP_MS 5U
#define N LIFT_CHANNEL_COUNT

/* ---------- 模拟机械 ---------- */

/*
 * 台架绝对坐标范围。零点落在 PLANT_STOP_MIN，所以
 *   已找零坐标 = 绝对坐标 - PLANT_STOP_MIN，范围 [0, 30000]
 * 与护栏配置的 position_min_ecd=0 / position_max_ecd=30000 对齐。
 * 若两边不一致（例如绝对范围 -3000..40000 却只给护栏 0..30000），
 * 机构会在护栏限位与机械死点之间来回顶，表现为"抬不上去"。
 */
#define PLANT_START_ECD   2000
#define PLANT_STOP_MIN    0
#define PLANT_STOP_MAX    30000
/*
 * 台架机械模型：一阶速度环节 + 连续位置积分。
 *
 * 关键：模型必须按【控制器自己那套换算】产生速度反馈。
 * 模型内部速度用 ecd/s，反馈前除以 lift_control_speed_to_ecd_s() 折成 rpm。
 * 若像旧模型那样把"每周期步长"直接当 rpm 回报，控制器会以为机械在狂奔，
 * 内环立刻打到限幅——那不是控制器不稳，是台架和控制器对"速度"的理解不一致。
 *
 * 另外两点也是必须的，否则台架自己会制造振荡：
 *   1. 速度必须有一阶滞后（等效转子惯量）。速度瞬间等于指令时，
 *      离散环路每步都过冲。
 *   2. 位置用 double 连续累积，只在读出时取整；按整数计数积分会在
 *      慢速时卡在 1 计数的死区里。
 */
/*
 * 速度环节时间常数：等效转子惯量，也给离散内环留相位裕度。
 * 必须明显大于采样周期(5ms)，否则内环一个周期内来不及响应，
 * 速度误差会瞬间顶到限幅，变成 bang-bang 极限环（表现为两端来回撞）。
 * 20ms 使内环闭环时间常数约 20/(1+speed_kp×G') ≈ 50ms，裕度充足。
 */
#define PLANT_SPEED_TIME_CONSTANT_MS 20U
#define PLANT_FULL_CURRENT 16384.0
/*
 * 机械增益，计数/秒 每 电流单位。取值受两个互相拉扯的约束：
 *   下限：小误差时必须还能推动机械。若稳态修正每周期不足 1 个计数，
 *         位置是整数反馈，机械会停在量化死区里不动（不是控制器的问题）。
 *   上限：要能跟上外环索要的目标速度（见 control_config 的推导），
 *         否则内环长期饱和、系统在两端来回撞。
 * 极性见下面 plant_step 的注释：增益为正。
 */
#define PLANT_GAIN_ECD_S_PER_CURRENT 200.0
/* 每周期最大行程：3000 计数/5ms = 600000 计数/秒 ≈ 4400 rpm，限幅电流下够用。 */
#define PLANT_MAX_COUNTS_PER_CYCLE 3000

typedef struct
{
    int32_t position_ecd[N];
    int16_t speed_rpm[N];
    double  position_accum[N];
    double  speed_ecd_s[N];
    double  ecd_s_per_rpm;
} plant_t;

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
        /*
         * 极性约定（全模块集统一，台架必须服从）：
         *   正电流 -> 绝对计数增大（收回）；负电流 -> 绝对计数减小（伸出/下行）。
         * 所以这里增益为正，护栏 direction=+1，找零用【负】seek_current。
         * 标定真机时这个符号必须单独确认：写反的话位置环会正反馈，
         * 一路顶到行程末端，而现象会伪装成"PID 调不好"。
         */
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
        if (step > (double)PLANT_MAX_COUNTS_PER_CYCLE)
        {
            step = (double)PLANT_MAX_COUNTS_PER_CYCLE;
        }
        if (step < -(double)PLANT_MAX_COUNTS_PER_CYCLE)
        {
            step = -(double)PLANT_MAX_COUNTS_PER_CYCLE;
        }

        p->position_accum[ch] += step;
        p->position_ecd[ch] = (int32_t)p->position_accum[ch];
        /*
         * 反馈速度必须取【实际走出来的】速度，而不是限幅前的指令速度。
         * 若把指令速度直接回传，台架会报出 4395 rpm 这种机械根本达不到的值
         * （等效 600000 计数/秒，而实际只有 300 计数/5ms），
         * 内环会看到虚假的大误差、不停翻符号，表现为"顶到两端来回撞"。
         */
        {
            const double actual_ecd_s = step * (1000.0 / (double)STEP_MS);
            p->speed_rpm[ch] =
                (int16_t)((actual_ecd_s / p->ecd_s_per_rpm) + 0.5);
        }

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

static int16_t to_i16_sat(int32_t value)
{
    if (value > 32767)
    {
        return 32767;
    }
    if (value < -32768)
    {
        return -32768;
    }
    return (int16_t)value;
}

/* ---------- 配置 ---------- */

static lift_homing_config_t homing_config(void)
{
    lift_homing_config_t c = {0};

    /*
     * 找零电流 -400：台架增益 200 计数/秒/电流 -> 稳态 80000 计数/秒，
     * 走完 13000 计数不到 10 个周期；判死后惯性滑行约 e×速度×τ ≈ 320 计数，
     * 在"回退没有冲过头"的 2000 计数余量内。
     * 若把电流加到 -2000，滑行会到 1600 计数，勉强压线——留余量取 -400。
     */
    c.seek_current = -400;          /* 负电流 -> 位置减小 -> 顶向机械下限 */
    c.backoff_current = 400;
    c.current_limit = 2000;
    c.stall_speed_rpm = 50;
    c.seek_min_travel_ecd = 1000;
    c.stall_hold_ms = 100U;
    c.backoff_ecd = 500;
    c.seek_timeout_ms = 30000U;
    c.backoff_timeout_ms = 30000U;
    return c;
}

static lift_guard_config_t guard_config(void)
{
    lift_guard_config_t c = {0};
    uint32_t ch;

    for (ch = 0U; ch < N; ch++)
    {
        c.position_min_ecd[ch] = 0;        /* 已找零坐标：0 = 机械下限 */
        c.position_max_ecd[ch] = 30000;
        c.direction[ch] = 1;               /* 正电流使已找零坐标增大 */
        c.descent_current[ch] = -2500;     /* 缓降方向：向下（坐标减小） */
    }
    c.current_limit = 3000;
    c.descent_stall_speed_rpm = 50;
    c.descent_stall_ms = 100U;
    c.descent_timeout_ms = 5000U;
    return c;
}

/*
 * 两条环的增益由机械常数推导，不是试凑出来的：
 *
 *   机械增益      G   = PLANT_COUNTS_PER_SEC_PER_CURRENT 计数/秒/电流
 *   机械最高转速  Vmax  由 G 与"满电流 16384"共同决定
 *   速度时间常数  T   = PLANT_SPEED_TIME_CONSTANT_MS
 *
 *   内环 kp：让"模态速度误差 1 ecd/s"输出对应的轴电流；
 *            控制器的速度环输入已经统一为 ecd/s，不能再按 rpm 推导。
 *   外环 kp：外环输出本来就是目标模态速度（ecd/s），位置误差同样是 ecd，
 *            带宽取内环的 1/5，避免两级环互相打架
 */
#define PLANT_FULL_CURRENT 16384.0
#define PLANT_MAX_RPM      (PLANT_COUNTS_PER_SEC_PER_CURRENT * 360.0 / \
                            (8192.0 * 6.0))

static lift_control_config_t control_config(void)
{
    lift_control_config_t c = {0};

    /*
     * 量纲链（每一环都要对上，串级最容易在这里出错）：
     *
     *   位置        单轴 ecd
     *   模态位置    4 × 单轴 ecd（正变换不除 4）
     *   speed_rpm   单轴转速，换算成 ecd/s
     *   模态速度    4 × 单轴 ecd/s
     *   台架增益 G  单轴 ecd/s 每 电流   -> 正向通道里每个电流单位只作用在【单轴】上
     *   反变换      模态控制量 -> 四轴电流，每轴 = ±各模态之和（不除 4）
     *
     * 内环误差是【模态】速度误差，而要产生的出力是【轴】电流。
     * 正变换把四个轴速度相加得到模态速度，纯高度指令反变换后每轴
     * 拿到同一个模态控制量，因此一个轴电流对应 4×G 的模态速度。
     * 控制器已经把 speed_rpm 转成 ecd/s，所以：
     *      speed_kp = 1 / (4 × G)
     *
     * 外环：目标速度(ecd/s) = pos_kp × 位置误差(ecd)。这里同样是模态量对模态量，
     * 不需要额外换算。外环输出限幅取机械真能到的模态速度 = 4 × G × 限幅电流。
     */
    const double speed_kp = 1.0 / (4.0 * PLANT_GAIN_ECD_S_PER_CURRENT);
    const double pos_kp = 10.0;
    const double modal_speed_limit =
        4.0 * PLANT_GAIN_ECD_S_PER_CURRENT * 3000.0;

    c.pid[LIFT_CONTROL_LOOP_POSITION].kp = (float)pos_kp;
    c.pid[LIFT_CONTROL_LOOP_POSITION].ki = (float)(pos_kp * 0.02);
    c.pid[LIFT_CONTROL_LOOP_POSITION].kd = 0.0F;
    c.pid[LIFT_CONTROL_LOOP_POSITION].integral_limit = 1.0e7F;
    c.pid[LIFT_CONTROL_LOOP_POSITION].output_limit = (float)modal_speed_limit;
    c.pid[LIFT_CONTROL_LOOP_SPEED].kp = (float)speed_kp;
    c.pid[LIFT_CONTROL_LOOP_SPEED].ki = 0.0F;
    c.pid[LIFT_CONTROL_LOOP_SPEED].kd = 0.0F;
    c.pid[LIFT_CONTROL_LOOP_SPEED].integral_limit = 0.0F;
    c.pid[LIFT_CONTROL_LOOP_SPEED].output_limit = (float)PLANT_FULL_CURRENT;
    lift_sync_config_rect(&c.sync);
    c.ecd_per_rev = 8192;
    c.gear_ratio = 1.0F;
    c.speed_is_rotor = false;
    c.max_target_step_ecd = 0;
    c.period_s = (float)STEP_MS / 1000.0F;
    c.settle_tolerance_ecd = 200;

    return c;
}

int main(void)
{
    plant_t plant;
    lift_homing_t homing[N];
    lift_homing_config_t hcfg = homing_config();
    lift_guard_t guard;
    lift_guard_config_t gcfg = guard_config();
    lift_control_t ctl;
    lift_control_config_t ccfg = control_config();
    lift_control_command_t cmd;
    int32_t zero[N];
    int32_t homed[N];
    uint32_t ch;
    uint32_t t;
    uint32_t steps;
    bool all_homed = false;

    printf("lift 整链集成测试\n");

    plant_init(&plant, &ccfg);
    for (ch = 0U; ch < N; ch++)
    {
        zero[ch] = 0;
        lift_homing_init(&homing[ch], &hcfg);
    }
    lift_guard_init(&guard, &gcfg);
    lift_control_init(&ctl, &ccfg);

    /* ---------- Phase 0：找零之前必须被门禁拦住 ---------- */
    printf("  [P0] 找零之前任何电流都不得下发\n");
    {
        lift_guard_input_t gin;
        lift_guard_output_t gout;
        for (ch = 0U; ch < N; ch++)
        {
            gin.position_ecd[ch] = 0;
            gin.speed_rpm[ch] = 0;
            gin.requested_current[ch] = 3000;   /* 上层硬要给电流 */
            gin.online[ch] = true;
        }
        gin.homed = false;
        gin.command_fresh = true;
        gin.reset_request = false;
        gin.now_ms = 0U;

        lift_guard_update(&guard, &gin, &gout);
        CHECK(gout.state == LIFT_GUARD_STATE_DISARMED, "状态为 DISARMED");
        for (ch = 0U; ch < N; ch++)
        {
            CHECK(gout.output_current[ch] == 0, "门禁拦住电流");
        }
    }

    /* ---------- Phase 1：四轴同时堵转找零 ---------- */
    printf("  [P1] 四轴同时堵转找零\n");
    for (t = 0U; t < 4000U; t += STEP_MS)
    {
        int16_t cmd_current[N];
        all_homed = true;

        for (ch = 0U; ch < N; ch++)
        {
            lift_homing_input_t hin;
            lift_homing_output_t hout;

            hin.position_ecd = plant.position_ecd[ch];
            hin.speed_rpm = plant.speed_rpm[ch];
            hin.online = true;
            hin.start_request = (t == 0U);
            hin.abort_request = false;
            hin.now_ms = t;

            lift_homing_update(&homing[ch], &hin, &hout);
            cmd_current[ch] = hout.commanded_current;

            if (hout.state == LIFT_HOMING_STATE_DONE)
            {
                zero[ch] = hout.zero_ecd;
            }
            else
            {
                all_homed = false;
            }
            CHECK(hout.state != LIFT_HOMING_STATE_FAULT, "找零过程中不得报故障");
        }

        plant_step(&plant, cmd_current);
        if (all_homed)
        {
            break;
        }
    }

    CHECK(all_homed, "四轴都完成找零");
    for (ch = 0U; ch < N; ch++)
    {
        CHECK(zero[ch] == PLANT_STOP_MIN, "零点落在机械死点(-3000)上");
        CHECK(plant.position_ecd[ch] > PLANT_STOP_MIN, "找零后已离开死点");
        CHECK(plant.position_ecd[ch] <= PLANT_STOP_MIN + 2000, "回退没有冲过头");
    }
    CHECK(t <= 100U * STEP_MS, "找零在 500ms 内完成");

    /* ---------- Phase 2：使能抬升 ---------- */
    printf("  [P2] 使能抬升并观察收敛\n");
    cmd.enabled = true;
    cmd.target_height_ecd = 20000;   /* 每轴已找零坐标走到 5000 */
    cmd.target_roll_ecd = 0;
    cmd.target_pitch_ecd = 0;

    steps = 0U;
    for (t = 0U; t < 20000U; t += STEP_MS)
    {
        lift_sync_effort_t effort;
        lift_control_status_t st;
        int32_t cur32[N];
        int16_t req[N];
        lift_guard_input_t gin;
        lift_guard_output_t gout;

        for (ch = 0U; ch < N; ch++)
        {
            homed[ch] = plant.position_ecd[ch] - zero[ch];
        }

        lift_control_update(
            &ctl, homed, plant.speed_rpm, &cmd, &effort, &st);
        lift_sync_distribute(&ctl.config.sync, &effort, cur32);
        for (ch = 0U; ch < N; ch++)
        {
            req[ch] = to_i16_sat(cur32[ch]);
        }        for (ch = 0U; ch < N; ch++)
        {
            gin.position_ecd[ch] = homed[ch];
            gin.speed_rpm[ch] = plant.speed_rpm[ch];
            gin.requested_current[ch] = req[ch];
            gin.online[ch] = true;
        }
        gin.homed = all_homed;
        gin.command_fresh = true;
        gin.reset_request = false;
        gin.now_ms = t;

        lift_guard_update(&guard, &gin, &gout);
        CHECK(gout.state == LIFT_GUARD_STATE_READY, "命令新鲜时处于 READY");
        CHECK(gout.fault == LIFT_GUARD_FAULT_NONE, "抬升过程无故障");

        plant_step(&plant, gout.output_current);
        steps++;

        if (st.at_target)
        {
            break;
        }
    }

    /* 该台架的 20 ms 速度时间常数决定了约 1 s 的收敛预算。 */
    CHECK(steps < 200U, "高度在 1 秒内收敛");

    /* 用停机那一刻的真实位置重新算一遍，避免用到上一周期的旧值。 */
    for (ch = 0U; ch < N; ch++)
    {
        homed[ch] = plant.position_ecd[ch] - zero[ch];
    }
    for (ch = 0U; ch < N; ch++)
    {
        const int32_t dev = homed[ch] - 5000;
        CHECK((dev <= 200) && (dev >= -200), "每轴已找零坐标收敛到 5000 附近");
    }

    {
        lift_sync_modes_t base;
        lift_sync_modes_t corner_modes;
        lift_sync_modes_t warp_modes;
        int32_t perturbed[N];
        lift_sync_effort_t effort;
        lift_control_status_t st;
        int32_t cur32[N];

        lift_sync_forward(&ctl.config.sync, homed, &base);
        CHECK(base.warp == 0, "机械一致时翘曲模态为 0");

        /*
         * 只把右后角抬高 400。按权重表它应当【同时】进入四个模态：
         *   height +400, roll -400, pitch -400, warp -400
         * 这是常见误解的反面教材——"某一个角的偏差"并不等于"纯翘曲"。
         */
        for (ch = 0U; ch < N; ch++)
        {
            perturbed[ch] = homed[ch];
        }
        perturbed[3] += 400;
        lift_sync_forward(&ctl.config.sync, perturbed, &corner_modes);
        CHECK(corner_modes.height == base.height + 400, "单角偏差进入高度模态");
        CHECK(corner_modes.roll == base.roll - 400, "单角偏差进入横滚模态");
        CHECK(corner_modes.pitch == base.pitch - 400, "单角偏差进入俯仰模态");
        CHECK(corner_modes.warp == base.warp - 400, "单角偏差进入翘曲模态");

        /* 真正的纯翘曲扰动是 [+1,-1,+1,-1] 方向，它不污染另外三个模态。 */
        perturbed[0] = homed[0] + 100;
        perturbed[1] = homed[1] - 100;
        perturbed[2] = homed[2] + 100;
        perturbed[3] = homed[3] - 100;
        lift_sync_forward(&ctl.config.sync, perturbed, &warp_modes);
        CHECK(warp_modes.height == base.height, "纯翘曲不污染高度");
        CHECK(warp_modes.roll == base.roll, "纯翘曲不污染横滚");
        CHECK(warp_modes.pitch == base.pitch, "纯翘曲不污染俯仰");
        CHECK(warp_modes.warp == base.warp + 400, "纯翘曲只改翘曲");

        lift_control_update(
            &ctl, homed, plant.speed_rpm, &cmd, &effort, &st);
        lift_sync_distribute(&ctl.config.sync, &effort, cur32);
        CHECK(lift_sync_warp_component(&ctl.config.sync, cur32) == 0,
              "整链条下发的电流翘曲分量严格为 0");
    }

    /* ---------- Phase 3：命令超时 -> 受控缓降 ---------- */
    printf("  [P3] 命令超时后受控缓降到底\n");
    for (t = 0U; t < 20000U; t += STEP_MS)
    {
        lift_guard_input_t gin;
        lift_guard_output_t gout;

        for (ch = 0U; ch < N; ch++)
        {
            homed[ch] = plant.position_ecd[ch] - zero[ch];
            gin.position_ecd[ch] = homed[ch];
            gin.speed_rpm[ch] = plant.speed_rpm[ch];
            gin.requested_current[ch] = 3000;   /* 故意给个大请求，门禁必须压住 */
            gin.online[ch] = true;
        }
        gin.homed = all_homed;
        gin.command_fresh = false;              /* 上层命令超时 */
        gin.reset_request = false;
        gin.now_ms = t;

        lift_guard_update(&guard, &gin, &gout);
        CHECK(gout.fault == LIFT_GUARD_FAULT_NONE, "缓降过程不得报故障");

        if (gout.state != LIFT_GUARD_STATE_FAULT)
        {
            for (ch = 0U; ch < N; ch++)
            {
                /* 高处时必须是缓降电流（负），而不是归零、更不是那个大请求。 */
                if (homed[ch] > 0)
                {
                    CHECK(gout.output_current[ch] < 0, "高处输出缓降电流而不是 0");
                    CHECK(gout.output_current[ch] != 3000, "不得放行上层的大请求");
                }
            }
        }

        plant_step(&plant, gout.output_current);

        if (gout.state == LIFT_GUARD_STATE_PARKED)
        {
            break;
        }
    }

    CHECK(guard.state == LIFT_GUARD_STATE_PARKED, "缓降完成后进入 PARKED");
    for (ch = 0U; ch < N; ch++)
    {
        homed[ch] = plant.position_ecd[ch] - zero[ch];
        CHECK(homed[ch] == 0, "四轴都落回机械下限");
    }

    printf("\n共 %d 项断言，失败 %d 项\n", g_checks, g_failures);
    if (g_failures == 0)
    {
        printf("结果：全部通过\n");
        return 0;
    }
    printf("结果：存在失败\n");
    return 1;
}
