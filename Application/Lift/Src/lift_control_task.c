#include "lift_control_task.h"

#include "bsp_can.h"
#include "cmsis_os2.h"
#include "dji_motor.h"
#include "FreeRTOS.h"
#include "lift_jog_task.h"
#include "robot_motor_commands.h"
#include "robot_motor_config.h"
#include "robot_motors.h"
#include "task.h"

#include <stddef.h>
#include <string.h>

#define LIFT_CONTROL_TASK_STACK_WORDS       768U
#define LIFT_CONTROL_PERIOD_MS              5U

/*
 * 命令刷新超时。必须【大于】遥控任务的 10 ms 周期并留够余量，
 * 又必须让人能及时看到"命令断了"。100 ms 与设备层的抬升超时同量级，
 * 但护栏的受控缓降由 lift_guard 按同一窗口判定（ADR-006）。
 */
#define LIFT_CONTROL_COMMAND_TIMEOUT_MS     100U

/* 抬升组首次上板时的电流上限：必须低于 C620 满量程。
 * 具体数值见下面第 1b 节的 LIFT_BRINGUP_CURRENT_LIMIT_MAX（按实测重设）。 */

/* ==================================================================== */
/*  1. 机械常数：全部是【待实机标定】的占位值，不是调好的参数             */
/*                                                                    */
/*  这里只放"由图纸能算出来的"和"上板必须实测量出来的"两类量。              */
/*  每条都写明来源，标定完直接把数字换掉即可，不需要改控制结构。            */
/* ==================================================================== */

/*
 * 每圈计数。M3508/C620 反馈编码器 8192 计数/圈（**转子侧**）。
 * 电机自带 19.2:1 行星减速箱，所以输出轴一圈 = 转子 19.2 圈
 * ≈ 8192 × 19.2 ≈ 157286 计数。
 * 待实测：手转【输出轴/驱动齿轮】整一圈，看 total_ecd 变化多少
 * （调试手册第 3 节）：8192 = 转子侧，~157286 = 输出侧。
 */
#define LIFT_ECD_PER_REV                    8192

/*
 * ⚠️ 这里的 1.0 是【编码器/速度换算】的传动比，**不是减速箱的减速比**。
 *
 * 已确认（2026-10-09）：电机是官网原装 M3508，**自带 3591/187 ≈ 19.2:1
 * 行星减速箱**，输出轴上装 46 齿 × 模数 1.5 驱动齿轮（分度圆 69 mm）。
 *
 * 为什么这里仍是 1.0：C620 回报的 speed_rpm 与 total_ecd 都是**转子侧**，
 * 而 speed_is_rotor = false 表示"不再折算到负载侧"。
 * 减速箱的效果体现在"机械输出力矩 × 19.2"，已经包含在实测的 K 里。
 * **不要因为"有减速箱"就把它改成 19.2**，那会把速度环增益改错 19 倍。
 */
#define LIFT_GEAR_RATIO                     1.0F

/*
 * C620 反馈的 speed_rpm 是转子侧还是输出侧。已确认是**转子侧**
 * （与 total_ecd 同域），所以这里填 false（不再额外折算）。
 * 两种解释相差约 19.2 倍，翻错会让内环增益错 19 倍。
 */
#define LIFT_SPEED_IS_ROTOR                 false

/*
 * 执行器增益 K = (单轴 ecd/s) 每 电流，也就是"给 1 个电流单位，机构稳态跑多快"。
 *
 * 这个数直接决定内环增益正确与否：K 填大了内环会震荡，填小了抬不起来。
 * 标定方法：单轴给恒定电流，测**巡航段**平均速度（诊断点动 STEP 模式）。
 *   K = 巡航段平均速度 ecd/s ÷ 该段电流
 *
 * 实测（2026-10-09，无载、保留齿轮、axis=3，STEP 4000/6000/8000）：
 *   4000 → 1957 ecd/s → K = 0.49
 *   6000 → 3719 ecd/s → K = 0.62
 *   8000 → 15468 ecd/s → K = 1.93
 *
 * ⚠️ K **不是常数**：4000→8000 之间涨了约 4 倍。
 *    原因：电流越大越容易越过静摩擦，"同样电流能跑多快"自然更高；
 *    低速端还被 C620 的低速抖动吃掉一部分。所以这里取**中低段**的值——
 *    闭环实际工作区在 2000~4000 电流（受 `LIFT_MEASURED_CURRENT_LIMIT`
 *    与重力前馈约束），取 0.5 偏保守的一侧更安全。
 *
 * 上板后核对：`g_lift_control_status.speed_loop_kp` 应 ≈ 30，
 * 离 `speed_loop_kp_max`(≈90) 有 3 倍余量。
 * ⚠️ 2026-10-11：这两个数以前是 120 / 360，因为派生时**漏乘了模态域的 4 倍**
 *    （模态被控对象增益是 4K，不是 K），实际回路增益是目标值的 4 倍。
 *    详见 lift_derive_gains() 里的说明。
 */
#define LIFT_ACTUATOR_GAIN_ECD_S_PER_CURRENT  0.5F

/* 内环回路增益目标 GH = Kp·(4K)·h，稳定条件 GH < 2；取 0.30 留 6 倍余量。 */
#define LIFT_SPEED_LOOP_GAIN_TARGET         0.30F
/* 外环带宽取内环的 1/3：再快就会和速度环抢相位。 */
#define LIFT_POSITION_LOOP_BANDWIDTH_RATIO  0.33F

/*
 * 行程（编码器计数，已找零坐标）。
 *
 * ⚠️ 零点方向已由用户实测确认（2026-10-09）：
 *     齿条向上缩 → 撞死点，此时车在**最低**位置 = 零点
 *     齿条向下伸 → 把车顶起来 = 高度增大
 *   所以零点在最低端，往上都是正的：MIN=0 是安全的，MAX 待实测。
 *   （若以后发现死点在最高端，整条区间要镜像——见调试手册 P4。）
 *
 * 25000 这个上限是【占位】：齿条真实有效行程还没量（LIFT-009）。
 * 量法：找零后手推到另一端，看 homed_position_ecd 的跨度。
 *
 * ⚠️ 2026-10-11 修正下限：原来是 -2000，低于机械死点。但 10.10 的实测结论是
 *   "零点是齿条顶死的最低点 ⇒ position_min = 0 才安全"。配在死点之下等于
 *   允许控制器往死点里推 2000 计数，受控缓降会一路顶死点然后报
 *   DESCENT_STALL。已改成 0。
 */
#define LIFT_TRAVEL_MIN_ECD                 0
#define LIFT_TRAVEL_MAX_ECD                 25000
/*
 * 停放点（缓降停止位置，单轴计数）：零点上方一点点。
 * 缓降是"把车放下来"，放到机械死点前就该停——不能拿 MIN 当目标。
 */
#define LIFT_TRAVEL_PARK_ECD                200
/* 上电/未找零时的安全起点：不往上顶，先给一个确定的小高度。 */
#define LIFT_TRAVEL_START_ECD               2000
#define LIFT_TRAVEL_MARGIN_ECD              800
#define LIFT_LIMIT_ROLL_MAX_ECD             6000
#define LIFT_LIMIT_PITCH_MAX_ECD            6000
/*
 * 重力前馈的施加门槛（单轴计数）：四轴最低的那根高过它才给保持电流。
 * 车压在最低端时重量由地面承担，给电流只会白烧电机。
 */
#define LIFT_FEEDFORWARD_MIN_ECD            400

/*
 * 命令面（模态高度 = 四轴之和）的边界余量与安全起点。
 * 单轴常数换算到命令面要乘通道数——这是本项目反复踩的 4 倍陷阱。
 */
#define LIFT_COMMAND_MARGIN_ECD \
    (LIFT_TRAVEL_MARGIN_ECD * (int32_t)LIFT_CHANNEL_COUNT)
#define LIFT_COMMAND_START_ECD \
    (LIFT_TRAVEL_START_ECD * (int32_t)LIFT_CHANNEL_COUNT)

/* ==================================================================== */
/*  1b. 实测电流与机构关系（2026-10-09，无载、保留齿轮）                 */
/*                                                                    */
/*  ⚠️ 机构关系（用户实测确认，P4 的方向全靠它）：                       */
/*      齿条【完全缩回】→ 顶在硬死点 → 车在最低   = 零点                */
/*      齿条【向下伸出】→ 脚撑地     → 车被顶起来 = 抬升                */
/*    也就是说：**车落地停放时，齿条本来就贴在死点上、车就在零点**。      */
/*    这是本机构最好的一点（零点是"免费"的），但也带来下面这条硬约束。   */
/*                                                                    */
/*  ⚠️ 找零的行程条件：seek_min_travel_ecd = 1000。                      */
/*    堵转判定要求"先走够 1000 计数"才允许判堵转（区分"真到死点"和        */
/*    "卡住不动"）。                                                    */
/*    本机构靠【车下落】贡献行程：给缩回电流后车顺重力往下落，齿条相对      */
/*    车体走过的量就是行程。实测确认"下降时电流托不住车"，所以这条路成立。 */
/*    所以默认【不做】前置段（homing_preposition_ecd = 0）。              */
/*    万一某台车在某个姿态下"落不动"（行程凑不够 1000）：                 */
/*      把 g_lift_control_debug.homing_preposition_ecd 设成 1500~2500，   */
/*      它会先主动伸出再回缩，把行程凑出来，不必改控制逻辑。                */
/* ==================================================================== */

/* 实测启动门槛（刚能动） */
#define LIFT_MEASURED_BREAKAWAY_CURRENT     2000
/*
 * 闭环/护栏的电流上限。
 *
 * ⚠️ 2026-10-09 用户要求【全部限流放到 16000】（= C620 满量程 16384 的 97.7%，
 *    约 19.5 A，是 M3508 的【堵转/峰值】电流，不是连续电流）。
 *    台架调试阶段要的是"先把车撑起来"，所以不再留小余量。
 *    代价：这个上限本身不再提供任何保护，撞死点/卡死时就是满电流硬顶。
 *    连续出力建议不要超过 8200（≈10 A），顶死点别超过 2 秒。
 */
#define LIFT_MEASURED_CURRENT_LIMIT         16000
/*
 * 串级内环（速度环 / 位置环）的输出限幅（电流量纲）。
 * 注意它【只当限幅用】，不参与增益派生（增益只看 K 和周期），
 * 所以改这个值不会改变 speed_kp / position_kp。
 */
#define LIFT_DERIVED_CURRENT_CEILING        16000
/*
 * 位置环输出的限幅：量纲是【速度】(ecd/s)，不是电流，所以它跟上面的
 * 16000 没有关系，保持原值。
 *   4000 ecd/s ÷ 725 计数/mm ≈ 5.5 mm/s（四根齿条一起）
 * 之前它和电流上限共用 4000 这个数，纯属巧合，已拆开（见 lift_derive_gains）。
 * 要提速只改这一个数，别去动电流上限。
 */
#define LIFT_POSITION_LOOP_SPEED_CEILING_ECD_S  4000

/*
 * 找零的参数（机构动作序列见第 1b 节顶部）。
 *
 * SEEK 是"齿条缩回"方向：
 *   齿条缩 → 支撑点变高 → 车往下落 → 一路缩到硬死点停住 → 该处定为 0 点。
 * 下降过程中重力是【帮着走】的，电机只需要松开静摩擦，
 * 不需要出力托车。所以 seek 电流取【偏小】：
 *   - 取大了 → 撞死点更硬（四根同时硬撞，是机构最容易受伤的工况）
 *   - 取小了 → 松不开静摩擦，缩不动 → SEEK_TIMEOUT
 * 实测门槛（无载）是 2000；带载下降有重力帮忙，先给 1500 试，
 * 缩不动再往上加（调试手册的标定表里有这一项）。
 */
#define LIFT_FACTORY_HOMING_SEEK_CURRENT    (-1500)
/* 退出死点：齿条伸出 = 把车抬起来。这个方向没有重力帮忙，必须给足。 */
#define LIFT_FACTORY_HOMING_BACKOFF_CURRENT 2500
/* 找零过程的电流上限：同样按用户要求放到 16000（见 LIFT_MEASURED_CURRENT_LIMIT） */
#define LIFT_FACTORY_HOMING_LIMIT           16000
/* 回退距离：按实测 K，2000 电流下 2 s 约走 1900 计数，回退 800 绰绰有余。 */
#define LIFT_FACTORY_HOMING_BACKOFF_ECD     800


/*
 * 前置段的电流：与 seek 反号的【大】电流——它要把机构从死点上拉开，
 * 而且不能指望重力帮忙（车头两根偏轻，可能缩不回来），必须主动给足力。
 */
#define LIFT_FACTORY_HOMING_PREPOSITION_CURRENT  (+2500)

/*
 * 缓降电流：把车放下来（齿条缩回），方向为【逻辑负】。
 *
 * ⚠️ 必须取够大，但理由要写对（2026-10-11 修正）：
 *     齿条向下伸 → 把车顶起来（抬升）；齿条向上缩 → 车降下来（缓降方向）。
 *   车重压在齿条上、方向是【把齿条压回去（缩回）】，所以重力是【帮】下降的，
 *   不是阻碍——本文件上面 SEEK 那一段写的就是这个，两处曾经自相矛盾。
 *   缓降取小了走不动，原因是【静摩擦】实测门槛约 2000（无载），
 *   不是重力：电流必须先松开静摩擦。取 -1500 连门槛都过不去 → DESCENT_STALL。
 *
 * 落地后必须重标：车重加上去以后，能缩回的最小电流会变。
 */
#define LIFT_FACTORY_DESCENT_CURRENT        (-2500)

/*
 * 参数自检的硬上限：这是"上限的上限"，改它要先想清楚保护还剩多少。
 *
 * ⚠️ 2026-10-09 跟着"全部限流放到 16000"一起放到 16000。
 *    【必须同步改】：如果这里留着 4000，上面把 homing/guard 上限设成 16000
 *    会被 lift_debug_config_is_valid() 判为非法 → 整份配置被拒 →
 *    抬升根本不动作（而且现象很像"没接线"）。这三个常量是一组，不能只改一半。
 */
#define LIFT_BRINGUP_CURRENT_LIMIT_MAX      16000


/*
 * 派生的内环增益及其可行域。上板第一件事就是看 g_lift_control_status
 * 的这三项：position_loop_kp / speed_loop_kp / speed_loop_kp_max。
 * 如果速度环增益已经贴到上限，说明 K 太大或周期太长，必须先解决，
 * 否则"车一通电就冲、然后在目标两侧撞限幅"。
 */
#define LIFT_SPEED_LOOP_KP_MARGIN           0.90F

lift_task_command_t g_lift_task_command;
lift_control_debug_t g_lift_control_debug;
lift_task_status_t g_lift_control_status;

static lift_top_t lift_top;
/* 上一周期的状态快照：安全层要在调用 lift_top_update 之前用到它。 */
static lift_top_status_t lift_last_status;
static bool lift_configuration_applied;
static bool lift_configuration_rejected;
static float lift_speed_loop_kp;
static float lift_position_loop_kp;
static float lift_speed_loop_kp_max;
static bool lift_safety_limited;
static osThreadId_t lift_task_handle;
static StaticTask_t lift_task_control;
static StackType_t lift_task_stack[LIFT_CONTROL_TASK_STACK_WORDS];

/* ==================================================================== */
/*  2. 派生量：从机械常数算出增益，绝不手写"看起来差不多"的增益          */
/* ==================================================================== */

static bool lift_float_valid(float value)
{
    return (value == value) && (value < 1000000000.0F) &&
           (value > -1000000000.0F);
}

static int16_t lift_abs_i16_clamped(int16_t value)
{
    if (value == (-32767 - 1))
    {
        return 32767;
    }
    return value < 0 ? (int16_t)(-value) : value;
}

static int32_t lift_abs_i32_clamped(int32_t value)
{
    if (value == (-2147483647 - 1))
    {
        return 2147483647;
    }
    return value < 0 ? -value : value;
}

/** 单轴 speed_rpm → 单轴 ecd/s。与 lift_control 内部用的是同一个因子。 */
static float lift_ecd_s_per_rpm(const lift_control_debug_t *debug)
{
    float ecd_per_rev = (float)debug->ecd_per_rev;
    float gear = debug->gear_ratio;

    if ((ecd_per_rev <= 0.0F) || (gear <= 0.0F))
    {
        return 1.0F;
    }
    if (debug->speed_is_rotor)
    {
        /* 转子侧转速：先除以减速比回到输出侧。 */
        return (ecd_per_rev / 60.0F) / gear;
    }
    /* 输出侧转速：减速比已经在 speed_rpm 里，不再重复除。 */
    return ecd_per_rev / 60.0F;
}

/**
 * @brief 由机械常数派生串级两条环的增益。
 * @note 只在本任务里算，调试参数里的 kp/ki 字段保持为 0，避免出现
 *       "有两套增益，不知道哪套在生效"的情况。
 */
static void lift_derive_gains(lift_control_debug_t *debug)
{
    const float h = debug->period_s;
    const float gain = debug->actuator_gain_ecd_s_per_current;
    /*
     * ⚠️ 这里有一个量纲陷阱（2026-10-09 顺手拆开）：
     *   速度环输出 = 电流        → 限幅用 LIFT_DERIVED_CURRENT_CEILING
     *   位置环输出 = 速度(ecd/s) → 限幅【不能】跟着电流上限走
     * 两者以前共用同一个常量 4000。把电流上限提到 16000 时如果不管它，
     * 位置环的**速度上限**会顺带从 4000 变成 16000 ecd/s（5.5 → 22 mm/s），
     * 抬升速度凭空快 4 倍 —— 那不是"改限流"，是改运动学，所以拆成两个。
     */
    const float speed_loop_limit = (float)LIFT_DERIVED_CURRENT_CEILING;
    const float position_loop_limit =
        (float)LIFT_POSITION_LOOP_SPEED_CEILING_ECD_S;
    float speed_kp = 0.0F;
    float speed_kp_max = 0.0F;

    if ((h > 0.0F) && (gain > 0.0F))
    {
        /*
         * 速度环：控制量 = Kp × 速度误差，被控对象是"电流 → ecd/s"的增益 K。
         * 回路增益 GH = Kp·K·h（离散），稳定条件 GH < 2，取 0.30 留余量。
         * 极性由 axis_direction 在 lift_top 里统一处理，这里取正。
         *
         * ⚠️⚠️ 2026-10-11 修正：模态量的被控对象增益是 **4K**，不是 K。
         *
         *   链路：模态控制量 effort → distribute 给每轴电流 = effort（高度模态
         *   的权重全是 +1）→ 每轴速度 = K × effort → 模态速度 = Σ(四轴速度)
         *   = **4·K·effort**。
         *
         *   所以模态域的回路增益是 GH = Kp ·(4K)· h。少乘这个 4，实际 GH 就是
         *   目标值的 4 倍：0.30 → 1.20（离稳定边界 2 只剩 40% 余量），
         *   表现为"能收敛但很欠阻尼、目标附近大幅摆动"。
         *   这是本项目第 N 次栽在同一个 4 倍上（模态量 = 四轴之和，不除 4）。
         *
         *   注意速度环的 **限幅量纲不变**：effort 与单轴电流 1:1，
         *   所以 speed_output_limit 仍然是电流。
         */
        const float modal_gain = gain * (float)LIFT_CHANNEL_COUNT;

        speed_kp = LIFT_SPEED_LOOP_GAIN_TARGET / (modal_gain * h);
        speed_kp_max = LIFT_SPEED_LOOP_KP_MARGIN / (modal_gain * h);
    }
    if (speed_kp > speed_kp_max)
    {
        speed_kp = speed_kp_max;
    }

    /* 派生值始终算出并发布到状态里，供 Watch 对比 */
    lift_speed_loop_kp = speed_kp;
    lift_speed_loop_kp_max = speed_kp_max;
    lift_position_loop_kp = speed_kp * LIFT_POSITION_LOOP_BANDWIDTH_RATIO;

    if (debug->pid_manual_override)
    {
        /*
         * 手动模式：不覆盖 kp/ki/kd，用调试者填的值。
         * 只把"由派生值决定的检查量"留在这里（供 status 显示）。
         */
        return;
    }

    debug->speed_kp = speed_kp;
    debug->speed_ki = 0.0F;
    debug->speed_kd = 0.0F;
    /* 内环输出是电流量纲，限幅交给 guard；这里的上限只是为了不让
     * 单个模态的请求远超物理可能。 */
    debug->speed_output_limit = speed_loop_limit;
    debug->speed_integral_limit = 0.0F;

    debug->position_kp = lift_position_loop_kp;
    /* 外环只要一点点积分去补重力前馈的误差；INTEGRAL_LIMIT 按"最大模态
     * 速度 × 1 秒"给，防止积分把限幅顶穿后长时间退不出来。 */
    debug->position_ki = lift_position_loop_kp * 0.05F;
    debug->position_kd = 0.0F;
    debug->position_output_limit = position_loop_limit;
    debug->position_integral_limit = position_loop_limit;
}

/**
 * @brief 数值检查标定参数是否自洽（不检查是否"调好了"）。
 * @note 只拦真正会让机构受伤或让逻辑失效的组合：电流越限、方向非法、
 *       区间倒置、周期非正。增益是派生的，不在这里检查。
 */
static bool lift_debug_config_is_valid(const lift_control_debug_t *debug)
{
    uint32_t ch;

    if (debug == NULL)
    {
        return false;
    }

    if ((debug->ecd_per_rev <= 0) || (debug->gear_ratio <= 0.0F) ||
        (debug->period_s <= 0.0F) ||
        (debug->max_target_step_ecd < 0) ||
        (debug->settle_tolerance_ecd < 0) ||
        (debug->limit_margin_ecd < 0) ||
        !lift_float_valid(debug->gear_ratio) ||
        !lift_float_valid(debug->period_s) ||
        !lift_float_valid(debug->actuator_gain_ecd_s_per_current) ||
        !lift_float_valid(debug->gravity_feedforward_current))
    {
        return false;
    }

    if (debug->actuator_gain_ecd_s_per_current <= 0.0F)
    {
        /* K 未知就不许动：内环增益无从派生，动了必然发散。 */
        return false;
    }

    /* 手动增益模式：必须自己检查增益是否合法（自动模式由派生保证）。 */
    if (debug->pid_manual_override)
    {
        if (!lift_float_valid(debug->position_kp) ||
            !lift_float_valid(debug->position_ki) ||
            !lift_float_valid(debug->position_kd) ||
            !lift_float_valid(debug->speed_kp) ||
            !lift_float_valid(debug->speed_ki) ||
            !lift_float_valid(debug->speed_kd) ||
            (debug->position_kp < 0.0F) || (debug->position_ki < 0.0F) ||
            (debug->position_kd < 0.0F) ||
            (debug->speed_kp < 0.0F) || (debug->speed_ki < 0.0F) ||
            (debug->speed_kd < 0.0F) ||
            (debug->position_output_limit <= 0.0F) ||
            (debug->speed_output_limit <= 0.0F) ||
            (debug->position_integral_limit < 0.0F) ||
            (debug->speed_integral_limit < 0.0F))
        {
            return false;
        }
    }

    if ((debug->homing_current_limit <= 0) ||
        (debug->homing_current_limit > LIFT_BRINGUP_CURRENT_LIMIT_MAX) ||
        (debug->guard_current_limit <= 0) ||
        (debug->guard_current_limit > LIFT_BRINGUP_CURRENT_LIMIT_MAX) ||
        (debug->homing_stall_speed_rpm <= 0) ||
        (debug->guard_stall_speed_rpm <= 0))
    {
        return false;
    }

    if ((debug->homing_seek_timeout_ms == 0U) ||
        (debug->homing_backoff_timeout_ms == 0U) ||
        (debug->homing_stall_hold_ms == 0U) ||
        (debug->guard_stall_ms == 0U) ||
        (debug->guard_timeout_ms == 0U))
    {
        return false;
    }

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        if ((debug->axis_direction[ch] != 1) &&
            (debug->axis_direction[ch] != -1))
        {
            return false;
        }
        if (debug->position_min_ecd[ch] >= debug->position_max_ecd[ch])
        {
            return false;
        }
        /* 上限电流是抬升组共用的；单轴命令不得超过它，且不得为 0。 */
        if ((debug->descent_current[ch] == 0) ||
            (lift_abs_i16_clamped(debug->descent_current[ch]) >
             debug->guard_current_limit))
        {
            return false;
        }
    }

    if (lift_abs_i16_clamped(debug->homing_seek_current) >
            debug->homing_current_limit ||
        lift_abs_i16_clamped(debug->homing_backoff_current) >
            debug->homing_current_limit ||
        lift_abs_i16_clamped(debug->homing_preposition_current) >
            debug->homing_current_limit ||
        (debug->homing_seek_current == 0) ||
        (debug->homing_backoff_current == 0))
    {
        return false;
    }

    if (debug->homing_backoff_ecd == 0)
    {
        return false;
    }

    return true;
}

/** 把标定参数搬进 lift_top 的配置结构。 */
static void lift_build_config(
    lift_top_config_t *config,
    const lift_control_debug_t *debug)
{
    uint32_t ch;

    (void)memset(config, 0, sizeof(*config));
    lift_sync_config_rect(&config->control.sync);

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        config->axis_direction[ch] = debug->axis_direction[ch];

        config->homing.seek_current = debug->homing_seek_current;
        config->homing.backoff_current = debug->homing_backoff_current;
        config->homing.preposition_current =
            debug->homing_preposition_current;
        config->homing.current_limit = debug->homing_current_limit;
        config->homing.stall_speed_rpm = debug->homing_stall_speed_rpm;
        config->homing.preposition_ecd = debug->homing_preposition_ecd;
        config->homing.preposition_timeout_ms =
            debug->homing_preposition_timeout_ms;
        config->homing.seek_min_travel_ecd =
            debug->homing_seek_min_travel_ecd;
        config->homing.stall_hold_ms = debug->homing_stall_hold_ms;
        config->homing.backoff_ecd = debug->homing_backoff_ecd;
        config->homing.seek_timeout_ms = debug->homing_seek_timeout_ms;
        config->homing.backoff_timeout_ms = debug->homing_backoff_timeout_ms;

        config->control.limit_height_min_ecd[ch] = debug->position_min_ecd[ch];
        config->control.limit_height_max_ecd[ch] = debug->position_max_ecd[ch];

        config->guard.position_min_ecd[ch] = debug->position_min_ecd[ch];
        config->guard.position_max_ecd[ch] = debug->position_max_ecd[ch];
        /*
         * direction 描述"正电流让 position_ecd 怎么变"。逻辑坐标已经由
         * axis_direction 归一化成正 = 上升，所以这里恒为 +1；缓降电流
         * 的符号由调试参数给出（默认负 = 往下）。
         */
        config->guard.direction[ch] = 1;
        config->guard.descent_current[ch] = debug->descent_current[ch];
        config->guard.descent_target_ecd[ch] = debug->descent_target_ecd[ch];
    }

    config->guard.current_limit = debug->guard_current_limit;
    config->guard.descent_stall_speed_rpm = debug->guard_stall_speed_rpm;
    config->guard.descent_stall_ms = debug->guard_stall_ms;
    config->guard.descent_timeout_ms = debug->guard_timeout_ms;

    config->control.pid[LIFT_CONTROL_LOOP_POSITION].kp = debug->position_kp;
    config->control.pid[LIFT_CONTROL_LOOP_POSITION].ki = debug->position_ki;
    config->control.pid[LIFT_CONTROL_LOOP_POSITION].kd = debug->position_kd;
    config->control.pid[LIFT_CONTROL_LOOP_POSITION].integral_limit =
        debug->position_integral_limit;
    config->control.pid[LIFT_CONTROL_LOOP_POSITION].output_limit =
        debug->position_output_limit;
    config->control.pid[LIFT_CONTROL_LOOP_SPEED].kp = debug->speed_kp;
    config->control.pid[LIFT_CONTROL_LOOP_SPEED].ki = debug->speed_ki;
    config->control.pid[LIFT_CONTROL_LOOP_SPEED].kd = debug->speed_kd;
    config->control.pid[LIFT_CONTROL_LOOP_SPEED].integral_limit =
        debug->speed_integral_limit;
    config->control.pid[LIFT_CONTROL_LOOP_SPEED].output_limit =
        debug->speed_output_limit;

    config->control.ecd_per_rev = debug->ecd_per_rev;
    config->control.gear_ratio = debug->gear_ratio;
    config->control.speed_is_rotor = debug->speed_is_rotor;
    config->control.max_target_step_ecd = debug->max_target_step_ecd;
    config->control.period_s = debug->period_s;
    config->control.settle_tolerance_ecd = debug->settle_tolerance_ecd;
    config->control.limit_margin_ecd = debug->limit_margin_ecd;
    config->control.gravity_feedforward_current =
        debug->gravity_feedforward_current;
    config->control.feedforward_min_position_ecd =
        debug->feedforward_min_position_ecd;
    config->control.actuator_gain_ecd_s_per_current =
        debug->actuator_gain_ecd_s_per_current;
}

/** 上电标定默认值：全部是占位值 + 保守电流，标定后逐条替换。 */
static void lift_set_default_debug(void)
{
    uint32_t ch;

    (void)memset(&g_lift_control_debug, 0, sizeof(g_lift_control_debug));

    g_lift_control_debug.ecd_per_rev = LIFT_ECD_PER_REV;
    g_lift_control_debug.gear_ratio = LIFT_GEAR_RATIO;
    g_lift_control_debug.speed_is_rotor = LIFT_SPEED_IS_ROTOR;
    g_lift_control_debug.period_s = (float)LIFT_CONTROL_PERIOD_MS / 1000.0F;
    g_lift_control_debug.actuator_gain_ecd_s_per_current =
        LIFT_ACTUATOR_GAIN_ECD_S_PER_CURRENT;
    g_lift_control_debug.gravity_feedforward_current = 0.0F;
    g_lift_control_debug.feedforward_min_position_ecd =
        LIFT_FEEDFORWARD_MIN_ECD;
    g_lift_control_debug.settle_tolerance_ecd = 200;
    g_lift_control_debug.limit_margin_ecd = LIFT_TRAVEL_MARGIN_ECD;
    /* 60 计数/周期 @5 ms = 12000 计数/s ≈ 0.59 mm/周期，防冲击。 */
    g_lift_control_debug.max_target_step_ecd = 60;

    /*
     * 找零：向下顶死点，再反方向退出。
     * 符号约定：逻辑"正" = 上升，所以 seek 是负（往下顶），backoff 是正。
     * 电流按实测重设：给 2000 才动，所以 seek/backoff 都必须在这个量级，
     * 否则必然 SEEK_TIMEOUT / BACKOFF_TIMEOUT。
     * 实测确认方向后，若某根反了，只改 axis_direction，不改这里。
     */
    g_lift_control_debug.homing_seek_current =
        LIFT_FACTORY_HOMING_SEEK_CURRENT;
    g_lift_control_debug.homing_backoff_current =
        LIFT_FACTORY_HOMING_BACKOFF_CURRENT;
    /*
     * 前置段：默认【关闭】（0）。
     *
     * 曾经设成 2500：理由是"车上电就贴在死点上，走不够 seek_min_travel_ecd，
     * 必然 SEEK_TIMEOUT"。但实测的机构行为是——给缩回电流时，车会顺着重力
     * 往下落，**车下落本身就贡献行程**，所以不需要先主动往反方向顶一段。
     *
     * 前置段机构仍然保留（lift_homing 支持），只是默认不走：
     * 万一某台车在某个姿态下真的"落不动"，把它设成 1500~2500 即可，
     * 不用改控制逻辑。
     */
    g_lift_control_debug.homing_preposition_current =
        LIFT_FACTORY_HOMING_PREPOSITION_CURRENT;
    g_lift_control_debug.homing_preposition_ecd = 0;
    g_lift_control_debug.homing_preposition_timeout_ms = 6000U;
    g_lift_control_debug.homing_current_limit = LIFT_FACTORY_HOMING_LIMIT;
    g_lift_control_debug.homing_stall_speed_rpm = 20;
    g_lift_control_debug.homing_seek_min_travel_ecd = 1000;
    /*
     * 堵转确认时间取 300 ms（不是 100）。
     * 这一路下降是"车落到底部撞死点"，撞击会有回弹，速度会抖过阈值；
     * 确认窗口太短会判不出堵转，然后一路等到 SEEK_TIMEOUT。
     */
    g_lift_control_debug.homing_stall_hold_ms = 300U;
    g_lift_control_debug.homing_backoff_ecd = LIFT_FACTORY_HOMING_BACKOFF_ECD;
    g_lift_control_debug.homing_seek_timeout_ms = 8000U;
    g_lift_control_debug.homing_backoff_timeout_ms = 5000U;

    /* 护栏：缓降电流指向行程下限（逻辑负 = 往下）。 */
    g_lift_control_debug.guard_current_limit = LIFT_MEASURED_CURRENT_LIMIT;
    g_lift_control_debug.guard_stall_speed_rpm = 20;
    g_lift_control_debug.guard_stall_ms = 500U;
    g_lift_control_debug.guard_timeout_ms = 15000U;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        /*
         * 方向：把四轴的逻辑坐标统一成 "正 = 抬升 = 齿条伸出"。
         *
         * 实测（2026-10-09，用户手推法）——**伸出**时原始计数往哪走：
         *   抬升1(0) 负    抬升2(1) 正    抬升3(2) 负    抬升4(3) 正
         * 前(0/2)与后(1/3)是镜像安装，所以同一个物理方向原始符号相反。
         * 要让"伸出 = 正"，负的那一对填 -1，正的那一对填 +1：
         *   0/2 → -1 ，1/3 → +1
         *
         * ⚠️ 这条会同时决定三件事，改错一个就是飞车或往错方向顶：
         *   ① 位置读数方向（位置环靠它，错了就是正反馈）
         *   ② 找零/缓降方向（seek = 逻辑负 = 齿条缩回）
         *   ③ 抬升出力方向
         * 所以它只能靠"手推 + 看 delta_ecd"来定，不能靠猜。
         */
        g_lift_control_debug.axis_direction[ch] =
            ((ch == 0U) || (ch == 2U)) ? -1 : 1;
        g_lift_control_debug.position_min_ecd[ch] = LIFT_TRAVEL_MIN_ECD;
        g_lift_control_debug.position_max_ecd[ch] = LIFT_TRAVEL_MAX_ECD;
        g_lift_control_debug.descent_current[ch] =
            LIFT_FACTORY_DESCENT_CURRENT;
        /*
         * 缓降停止位置：零点上方一点点（不是 position_min——那是行程下限，
         * 允许配在死点下方；拿它当目标会让每次超时缓降都顶死点）。
         */
        g_lift_control_debug.descent_target_ecd[ch] = LIFT_TRAVEL_PARK_ECD;
    }

    lift_derive_gains(&g_lift_control_debug);
}

/**
 * @brief 上电参数应用。
 *
 * 在 `lift_control_task_init()` 里先跑一次：标定参数与派生增益必须在第一
 * 个控制周期之前就成立，否则第一拍就会用零增益/零限幅的默认配置去算电流。
 * 后续调试时把 `g_lift_control_debug.apply` 置 1，下一次 apply 会
 * **先撤销使能并使旧零点失效**，再重建整条链——这是刻意的：改了参数就不该
 * 继续拿旧零点去动机构。
 */
static void lift_apply_debug_parameters(bool invalidate_zero)
{
    lift_control_debug_t debug;
    lift_top_config_t config;
    bool apply;

    taskENTER_CRITICAL();
    apply = g_lift_control_debug.apply;
    debug = g_lift_control_debug;
    g_lift_control_debug.apply = false;
    taskEXIT_CRITICAL();

    if (!apply)
    {
        return;
    }

    if (invalidate_zero)
    {
        taskENTER_CRITICAL();
        g_lift_task_command.enabled = false;
        g_lift_task_command.homing_request = false;
        g_lift_task_command.clear_homing_request = false;
        g_lift_task_command.reset_request = false;
        g_lift_task_command.command_update_tick = osKernelGetTickCount();
        taskEXIT_CRITICAL();
    }

    if (!lift_debug_config_is_valid(&debug))
    {
        lift_configuration_applied = false;
        lift_configuration_rejected = true;
        return;
    }

    /* 增益是派生量：参数一改就重算，绝不沿用上一套。 */
    lift_derive_gains(&g_lift_control_debug);

    taskENTER_CRITICAL();
    debug = g_lift_control_debug;
    taskEXIT_CRITICAL();

    lift_build_config(&config, &debug);
    lift_top_init(&lift_top, &config);
    lift_configuration_applied = true;
    lift_configuration_rejected = false;
    lift_safety_limited = false;
    g_lift_control_status.parameter_apply_count++;

    /* 让 Watch 里能直接看到派生增益，不用手算。 */
    g_lift_control_status.speed_loop_kp = lift_speed_loop_kp;
    g_lift_control_status.speed_loop_kp_max = lift_speed_loop_kp_max;
    g_lift_control_status.position_loop_kp = lift_position_loop_kp;
}

/**
 * @brief 处理两个"轻量命令"：只重载增益 / 软零点。
 *
 * 为什么和 apply 分开：`apply` 会作废旧零点（必须重新找零），
 * 而调 PID 时每改一次增益都重新找零是不可接受的。
 * 这两个走轻量路径，保留零点与 homed。
 */
static void lift_handle_light_commands(void)
{
    uint8_t do_gains;
    uint8_t do_zero;
    lift_control_debug_t debug;
    lift_top_config_t config;
    int32_t raw[LIFT_CHANNEL_COUNT];
    robot_motor_state_t motors;
    uint32_t ch;

    taskENTER_CRITICAL();
    do_gains = g_lift_control_debug.apply_gains;
    do_zero = g_lift_control_debug.set_zero_now;
    g_lift_control_debug.apply_gains = 0U;
    g_lift_control_debug.set_zero_now = 0U;
    taskEXIT_CRITICAL();

    if ((do_gains == 0U) && (do_zero == 0U))
    {
        return;
    }

    /* 取一份参数快照（含手动增益）；派生只在自动模式下改 kp/ki/kd */
    lift_derive_gains(&g_lift_control_debug);
    taskENTER_CRITICAL();
    debug = g_lift_control_debug;
    taskEXIT_CRITICAL();

    if (!lift_debug_config_is_valid(&debug))
    {
        lift_configuration_rejected = true;
        return;
    }

    lift_build_config(&config, &debug);

    if (do_zero != 0U)
    {
        robot_motors_get_snapshot(&motors);
        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            raw[ch] = motors.lift.motor[ch].total_ecd;
        }
        lift_top_set_zero_now(&lift_top, raw);
        (void)memset(&lift_last_status, 0, sizeof(lift_last_status));
    }
    else
    {
        lift_top_reload_control(&lift_top, &config);
    }

    lift_configuration_applied = true;
    lift_configuration_rejected = false;
    g_lift_control_status.parameter_apply_count++;
    g_lift_control_status.speed_loop_kp = lift_speed_loop_kp;
    g_lift_control_status.speed_loop_kp_max = lift_speed_loop_kp_max;
    g_lift_control_status.position_loop_kp = lift_position_loop_kp;
}
/* ==================================================================== */
/*  3. 安全层：目标夹取与"自动下降找行程"                                */
/*                                                                      */
/*  为什么需要它：抬升的行程区间是【实测出来】的，上电时并不知道机构现在    */
/*  伸到哪。如果一使能就按目标往上顶，而目标其实在行程之外，位置环会一直     */
/*  顶到机械挡块（或不自锁机构被顶到极限）。这里做两件事：                  */
/*    1. 目标超过上限时，把目标压回"起点 + 余量"（往下走是安全的）；        */
/*    2. 未标定（K 或行程是占位值）时，只允许在起点附近低速试探。          */
/* ==================================================================== */

static int32_t lift_clamp_i32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

/*
 * ---- 命令面高度的单位换算（2026-10-11 修正）--------------------------
 *
 *   target_height_ecd / actual_height 都是【模态高度】= 四轴之和
 *   （lift_sync 的转置伴随约定，不除 4）；
 *   position_min_ecd[ch] / position_max_ecd[ch] 是【单轴】行程。
 *
 * 所以命令面的边界必须是"四轴之和"：
 *      下限 = Σ(单轴下限) + 余量      上限 = Σ(单轴上限) − 余量
 *
 * 以前直接拿单轴区间去夹模态目标，后果是**可达高度只有配置行程的 1/4**，
 * 而且遥控与安全层各用一套边界（推到头目标被"偷走"）。
 * 命令来源（遥控映射）一律用发布出去的 command_height_min/max_ecd。
 */
static int32_t lift_command_travel_sum(bool upper)
{
    int32_t total = 0;
    uint32_t ch;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        total += upper ? g_lift_control_debug.position_max_ecd[ch]
                       : g_lift_control_debug.position_min_ecd[ch];
    }
    return total;
}

/** 命令面允许的高度区间（模态量纲，已含余量）。 */
static void lift_command_height_bounds(int32_t *minimum, int32_t *maximum)
{
    *minimum = lift_command_travel_sum(false) + LIFT_COMMAND_MARGIN_ECD;
    *maximum = lift_command_travel_sum(true) - LIFT_COMMAND_MARGIN_ECD;
}

/**
 * @brief 把上层给的目标压到安全范围内。
 * @return true 表示目标被压过（safety_limited 会置位，供 Watch 看见）。
 *
 * 两条硬规则：
 *   1. 目标 > 行程上限：不是夹到上限（那等于顶死），而是压回起点，让机构
 *      先往下走，去找一个确定的位置；这比"顶在极限等超时"安全得多。
 *   2. 目标 < 行程下限：夹到下限 + 余量，绝不允许命令把机构往死点里推。
 */
static bool lift_apply_safety(const lift_top_status_t *top_status)
{
    int32_t minimum;
    int32_t maximum;
    const int32_t start = LIFT_COMMAND_START_ECD;
    bool limited = false;
    bool any_not_homed = false;
    uint32_t ch;

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        if (top_status->homing_state[ch] != LIFT_HOMING_STATE_DONE)
        {
            any_not_homed = true;
        }
    }

    taskENTER_CRITICAL();
    lift_command_height_bounds(&minimum, &maximum);
    if (g_lift_task_command.target_height_ecd > maximum)
    {
        /*
         * 往上越界：压回起点而不是压到上限。机构此刻可能正顶在极限上，
         * 唯一确定安全的方向是往下。
         */
        g_lift_task_command.target_height_ecd = start;
        limited = true;
    }
    if (g_lift_task_command.target_height_ecd < minimum)
    {
        g_lift_task_command.target_height_ecd = minimum;
        limited = true;
    }
    if (any_not_homed)
    {
        /* 没找零时目标没有意义，直接贴到起点，避免"上一轮的目标"残留。 */
        g_lift_task_command.target_height_ecd = start;
    }
    g_lift_task_command.target_roll_ecd = lift_clamp_i32(
        g_lift_task_command.target_roll_ecd,
        -LIFT_LIMIT_ROLL_MAX_ECD, LIFT_LIMIT_ROLL_MAX_ECD);
    g_lift_task_command.target_pitch_ecd = lift_clamp_i32(
        g_lift_task_command.target_pitch_ecd,
        -LIFT_LIMIT_PITCH_MAX_ECD, LIFT_LIMIT_PITCH_MAX_ECD);
    taskEXIT_CRITICAL();

    return limited;
}

/* ==================================================================== */
/*  4. 每周期：反馈 → 命令 → 输出                                        */
/* ==================================================================== */

static void lift_snapshot_command(lift_task_command_t *command)
{
    if (command == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *command = g_lift_task_command;
    /* request 位是单周期脉冲：本任务消费掉。 */
    g_lift_task_command.homing_request = false;
    g_lift_task_command.clear_homing_request = false;
    g_lift_task_command.reset_request = false;
    taskEXIT_CRITICAL();
}

static void lift_publish_status(
    const lift_top_status_t *top_status,
    const robot_motor_state_t *motors,
    const lift_task_command_t *command,
    const int16_t applied_current[LIFT_CHANNEL_COUNT],
    uint8_t output_gate,
    uint32_t now_ms,
    bool command_fresh)
{
    uint32_t ch;

    g_lift_control_status.running = true;
    g_lift_control_status.configuration_applied = lift_configuration_applied;
    g_lift_control_status.configuration_rejected = lift_configuration_rejected;
    g_lift_control_status.command_fresh = command_fresh;
    g_lift_control_status.command_age_ms =
        (uint32_t)(now_ms - command->command_update_tick);
    g_lift_control_status.homed = top_status->homed;
    g_lift_control_status.at_target = top_status->at_target;
    g_lift_control_status.safety_limited = lift_safety_limited;
    g_lift_control_status.output_gated =
        output_gate != (uint8_t)LIFT_OUTPUT_GATE_NONE;
    g_lift_control_status.output_gate = output_gate;
    g_lift_control_status.state = (uint8_t)top_status->state;
    g_lift_control_status.fault = (uint8_t)top_status->fault;
    g_lift_control_status.guard_state = (uint8_t)top_status->guard_state;
    g_lift_control_status.guard_fault = (uint8_t)top_status->guard_fault;
    g_lift_control_status.actual_height = top_status->actual_modes.height;
    g_lift_control_status.actual_roll = top_status->actual_modes.roll;
    g_lift_control_status.actual_pitch = top_status->actual_modes.pitch;
    g_lift_control_status.actual_warp = top_status->actual_modes.warp;
    g_lift_control_status.actual_speed_height =
        top_status->actual_modes_speed.height;
    g_lift_control_status.actual_speed_roll =
        top_status->actual_modes_speed.roll;
    g_lift_control_status.actual_speed_pitch =
        top_status->actual_modes_speed.pitch;
    g_lift_control_status.actual_speed_warp =
        top_status->actual_modes_speed.warp;
    g_lift_control_status.target_height_ecd = command->target_height_ecd;
    g_lift_control_status.target_roll_ecd = command->target_roll_ecd;
    g_lift_control_status.target_pitch_ecd = command->target_pitch_ecd;

    /*
     * 控制链内部量（都是模态量纲）。以前这些只在 lift_top_status 里算完就
     * 扔掉，Watch 里看不到——"限位到底有没有在压我的输出"只能靠猜。
     */
    g_lift_control_status.limit_scale = top_status->control.limit_scale;
    g_lift_control_status.limit_active = top_status->control.limit_active;
    g_lift_control_status.ramped_height_ecd =
        top_status->control.ramped_height_ecd;
    g_lift_control_status.target_speed_height_ecd_s =
        top_status->control.target_speed_height;
    g_lift_control_status.control_effort_height =
        top_status->control_effort_height;
    g_lift_control_status.error_height_ecd =
        top_status->control.error_height_ecd;

    /*
     * 命令面允许的高度区间（模态量纲，含余量）。命令来源按它夹取，
     * 就与 lift_apply_safety 用的是同一套边界，不会再出现"推到某个值
     * 被安全层改写回起点"的怪现象。
     */
    {
        int32_t minimum;
        int32_t maximum;

        lift_command_height_bounds(&minimum, &maximum);
        g_lift_control_status.command_height_min_ecd = minimum;
        g_lift_control_status.command_height_max_ecd = maximum;
    }

    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        g_lift_control_status.homing_state[ch] =
            (uint8_t)top_status->homing_state[ch];
        g_lift_control_status.homing_fault[ch] =
            (uint8_t)top_status->homing_fault[ch];
        g_lift_control_status.online[ch] =
            motors->lift.motor[ch].initialized &&
            dji_motor_feedback_is_online(
                &motors->lift.motor[ch], now_ms,
                pdMS_TO_TICKS(LIFT_CONTROL_COMMAND_TIMEOUT_MS));
        g_lift_control_status.limit_blocked[ch] =
            top_status->limit_blocked[ch];
        g_lift_control_status.raw_position_ecd[ch] =
            motors->lift.motor[ch].total_ecd;
        g_lift_control_status.zero_ecd[ch] = top_status->zero_ecd[ch];
        g_lift_control_status.homed_position_ecd[ch] =
            top_status->homed_position_ecd[ch];
        g_lift_control_status.speed_rpm[ch] =
            motors->lift.motor[ch].speed_rpm;
        g_lift_control_status.temperature[ch] =
            motors->lift.motor[ch].temperature;
        g_lift_control_status.output_current[ch] =
            top_status->output_current[ch];
        g_lift_control_status.applied_current[ch] = applied_current[ch];
        /*
         * 把行程软限位的实际区间发布出去：命令来源必须按同一套边界夹取，
         * 否则"遥控以为能到 30000、控制器只让到 25000"，操作手会觉得
         * 推到底却不动，而去怀疑增益。
         */
        g_lift_control_status.limit_height_min_ecd[ch] =
            lift_top.config.control.limit_height_min_ecd[ch];
        g_lift_control_status.limit_height_max_ecd[ch] =
            lift_top.config.control.limit_height_max_ecd[ch];
    }
}

static void lift_control_task(void *argument)
{
    robot_motor_state_t motors;
    lift_task_command_t command;
    lift_top_input_t input;
    lift_control_command_t control_command;
    lift_top_status_t top_status;
    TickType_t last_wake_tick;

    (void)argument;
    last_wake_tick = xTaskGetTickCount();

    for (;;)
    {
        const uint32_t now = osKernelGetTickCount();
        const uint32_t command_age =
            (uint32_t)(now - g_lift_task_command.command_update_tick);
        const bool command_fresh =
            g_lift_task_command.enabled &&
            (command_age <= pdMS_TO_TICKS(LIFT_CONTROL_COMMAND_TIMEOUT_MS));
        /*
         * 这一组电流是否被别的模块占用（台架点动 / CAN 测试）。
         * 占用期间主链完全不下发，也不能把这些周期当成"命令超时"。
         */
        const bool bench_owns =
            g_dji_can_test_status.running ||
            g_dji_can_test_status.zero_holding ||
            g_lift_jog_status.running;
        bool all_online = true;
        uint32_t ch;

        /* 调试器改了参数：先撤销使能、让旧零点失效，再重建整条链。 */
        lift_apply_debug_parameters(true);
        /* 轻量命令：只重载增益 / 软零点（都不作废旧零点）。 */
        lift_handle_light_commands();

        (void)memset(&input, 0, sizeof(input));
        (void)memset(&control_command, 0, sizeof(control_command));
        /* 安全层用的是上一周期的找零状态：命令必须在 UPDATE 之前就夹好。 */
        lift_safety_limited = lift_apply_safety(&lift_last_status);
        if (lift_safety_limited)
        {
            /* 以前这个计数器只声明、没人写，Watch 里永远显示 0。 */
            g_lift_control_status.safety_refresh_count++;
        }
        lift_snapshot_command(&command);
        robot_motors_get_snapshot(&motors);

        input.now_ms = now;
        input.command_fresh = command_fresh;
        input.homing_request = command.homing_request;
        input.clear_homing_request = command.clear_homing_request;
        input.reset_request = command.reset_request;
        control_command.enabled = command.enabled;
        control_command.target_height_ecd = command.target_height_ecd;
        control_command.target_roll_ecd = command.target_roll_ecd;
        control_command.target_pitch_ecd = command.target_pitch_ecd;

        /*
         * 台架点动 / CAN 测试占用这一组电流时（bench_owns）：
         * 主链本来就不出力（下面的门禁挡着），但**不能**把"没有新命令"
         * 理解成遥控失联——否则占用期间主链会一路走受控缓降状态，
         * 占用一结束就把车往下放。
         * 处理：命令当作新鲜的，并把目标吸附到当前姿态（lift_last_status
         * 是上一周期的实际模态）。于是台架放手后主链的动作是"保持"。
         */
        if (bench_owns)
        {
            input.command_fresh = true;
            control_command.target_height_ecd =
                lift_last_status.actual_modes.height;
            control_command.target_roll_ecd =
                lift_last_status.actual_modes.roll;
            control_command.target_pitch_ecd =
                lift_last_status.actual_modes.pitch;
        }

        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            input.position_ecd[ch] = motors.lift.motor[ch].total_ecd;
            input.speed_rpm[ch] = motors.lift.motor[ch].speed_rpm;
            input.online[ch] = dji_motor_feedback_is_online(
                &motors.lift.motor[ch], now,
                pdMS_TO_TICKS(LIFT_CONTROL_COMMAND_TIMEOUT_MS));
            all_online = all_online && input.online[ch];
        }

        if (!lift_configuration_applied)
        {
            /* 参数没生效：既不找零也不运动。 */
            input.homing_request = false;
            control_command.enabled = false;
            input.command_fresh = false;
        }

        lift_top_update(&lift_top, &input, &control_command, &top_status);
        lift_last_status = top_status;

        /*
         * 门禁：算出"该发什么"之后，这里决定"实际发什么"。
         * 真实下发的四路电流单独记在 applied_current，并把是哪道闸拦下的
         * 记在 output_gate —— 否则 Watch 里"有电流"和"实际没出力"分不清。
         */
        {
            int16_t applied[LIFT_CHANNEL_COUNT] = {0, 0, 0, 0};
            uint8_t gate = (uint8_t)LIFT_OUTPUT_GATE_NONE;

            if (g_dji_can_test_status.running ||
                g_dji_can_test_status.zero_holding)
            {
                gate = (uint8_t)LIFT_OUTPUT_GATE_CAN_TEST;
            }
            else if (g_lift_jog_status.running)
            {
                gate = (uint8_t)LIFT_OUTPUT_GATE_JOG;
            }
            else if (!lift_configuration_applied)
            {
                gate = (uint8_t)LIFT_OUTPUT_GATE_NO_CONFIG;
            }
            else if (!all_online)
            {
                gate = (uint8_t)LIFT_OUTPUT_GATE_OFFLINE;
            }
            else
            {
                for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
                {
                    applied[ch] = top_status.output_current[ch];
                }
            }

            /*
             * ⚠️ 被别的模块占用时【不写】这一组电流，而不是"写 0"。
             *
             * 2026-10-11 修正：原来门禁是写 0，而占用方（台架点动）每 5 ms
             * 也在写这一组——两个同周期同优先级的任务抢同一个命令缓存，
             * 发送任务（2 ms 一帧）取到的是"谁最后写"，于是点动电流被主链的
             * 0 斩波：实际送进 C620 的电流带一个未知的占空比，
             * K、启动门槛、保持电流这些标定数据全部失真。
             *
             * 现在：占用期间主链完全不碰这一组，由占用方独占；
             * 但"参数没生效 / 掉线"两种门禁仍然主动写 0 —— 那是保护动作，
             * 必须真的把电流压掉。
             */
            if (gate != (uint8_t)LIFT_OUTPUT_GATE_JOG &&
                gate != (uint8_t)LIFT_OUTPUT_GATE_CAN_TEST)
            {
                robot_lift_set_currents(
                    applied[0], applied[1], applied[2], applied[3]);
            }

            lift_publish_status(
                &top_status, &motors, &command, applied, gate, now,
                lift_configuration_applied && all_online &&
                (command_fresh || bench_owns));
        }
        g_lift_control_status.update_count++;

        vTaskDelayUntil(
            &last_wake_tick,
            pdMS_TO_TICKS(LIFT_CONTROL_PERIOD_MS));
    }
}

/* ==================================================================== */
/*  5. 对外接口                                                          */
/* ==================================================================== */

void lift_control_task_init(void)
{
    (void)memset(&g_lift_task_command, 0, sizeof(g_lift_task_command));
    (void)memset(&g_lift_control_status, 0, sizeof(g_lift_control_status));

    lift_set_default_debug();
    g_lift_control_debug.apply_gains = 0U;
    g_lift_control_debug.set_zero_now = 0U;
    g_lift_control_debug.pid_manual_override = false;

    (void)memset(&lift_last_status, 0, sizeof(lift_last_status));
    lift_configuration_applied = false;
    lift_configuration_rejected = false;
    lift_safety_limited = false;

    /*
     * 上电就把标定参数与派生增益装进控制链：第一个控制周期之前必须
     * 已经有合法的配置，否则第一拍会用零配置去算电流、算出来的东西
     * 谁也不知道是什么。使能仍然是关的——没有遥控命令不会有电流。
     */
    g_lift_control_debug.apply = true;
    lift_apply_debug_parameters(false);

    if (!lift_configuration_applied)
    {
        /* 默认参数自检没过：保持不可运动，并把原因留在状态里。 */
        lift_configuration_rejected = true;
    }
}

bool lift_control_task_create(void)
{
    const osThreadAttr_t attributes = {
        .name = "lift_ctrl",
        .cb_mem = &lift_task_control,
        .cb_size = sizeof(lift_task_control),
        .stack_mem = lift_task_stack,
        .stack_size = sizeof(lift_task_stack),
        .priority = osPriorityNormal
    };

    if (lift_task_handle != NULL)
    {
        return true;
    }

    lift_task_handle = osThreadNew(lift_control_task, NULL, &attributes);
    return lift_task_handle != NULL;
}

void lift_control_enable(bool enable)
{
    taskENTER_CRITICAL();
    g_lift_task_command.enabled = enable;
    g_lift_task_command.command_update_tick = osKernelGetTickCount();
    taskEXIT_CRITICAL();
}

void lift_control_set_target(
    int32_t height_ecd,
    int32_t roll_ecd,
    int32_t pitch_ecd)
{
    taskENTER_CRITICAL();
    g_lift_task_command.target_height_ecd = height_ecd;
    g_lift_task_command.target_roll_ecd = roll_ecd;
    g_lift_task_command.target_pitch_ecd = pitch_ecd;
    g_lift_task_command.command_update_tick = osKernelGetTickCount();
    taskEXIT_CRITICAL();
}

void lift_control_request_homing(void)
{
    taskENTER_CRITICAL();
    g_lift_task_command.homing_request = true;
    g_lift_task_command.command_update_tick = osKernelGetTickCount();
    taskEXIT_CRITICAL();
}

void lift_control_request_clear_homing(void)
{
    taskENTER_CRITICAL();
    g_lift_task_command.clear_homing_request = true;
    g_lift_task_command.command_update_tick = osKernelGetTickCount();
    taskEXIT_CRITICAL();
}

void lift_control_request_reset(void)
{
    taskENTER_CRITICAL();
    g_lift_task_command.reset_request = true;
    g_lift_task_command.command_update_tick = osKernelGetTickCount();
    taskEXIT_CRITICAL();
}
