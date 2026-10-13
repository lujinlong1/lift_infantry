#include "lift_control_task.h"

#include "bsp_can.h"
#include "chassis_control.h"
#include "cmsis_os2.h"
#include "dji_motor.h"
#include "FreeRTOS.h"
#include "lift_jog_task.h"
#include "lift_kinematics.h"
#include "lift_safety.h"
#include "lift_seq.h"
#include "robot_motor_commands.h"
#include "robot_motor_config.h"
#include "robot_motors.h"
#include "task.h"

#include <stddef.h>
#include <string.h>

#define LIFT_CONTROL_TASK_STACK_WORDS       1024U
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
 * 已确认（2026-10-09）：电机是官网原装 M3508，**自带行星减速箱**，
 * 输出轴装驱动齿轮（齿条 200 mm 行程对应 150000 计数，见 LIFT_TRAVEL_MAX_ECD）。
 * ⚠️ 2026-10-11 实测后，**减速比按实测取 13.43**，不再用图纸的 19.2；
 *    换算的权威值是 750.0 计数/mm（见 lift_kinematics.h）。
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

/* Legacy static actuator K, retained for the old modal API only.
 * It does not derive gains or predict travel in the active axis path. */
#define LIFT_ACTUATOR_GAIN_ECD_S_PER_CURRENT 0.5F

/*
 * 行程（编码器计数，已找零坐标）。
 *
 * ⚠️ 零点方向已由用户实测确认（2026-10-09）：
 *     齿条向上缩 → 撞死点，此时车在**最低**位置 = 零点
 *     齿条向下伸 → 把车顶起来 = 高度增大
 *   所以零点在最低端，往上都是正的：MIN=0 是安全的。
 *   （若以后发现死点在最高端，整条区间要镜像——见调试手册 P4。）
 *
 * ⚠️ 2026-10-11 修正下限：原来是 -2000，低于机械死点。但 10.10 的实测结论是
 *   "零点是齿条顶死的最低点 ⇒ position_min = 0 才安全"。配在死点之下等于
 *   允许控制器往死点里推 2000 计数，受控缓降会一路顶死点然后报
 *   DESCENT_STALL。已改成 0。
 *
 * ★★ 2026-10-11 实测定案（用户手测，取代之前所有图纸推算）★★
 *
 *   缩回死点 → 伸出【硬限位】全程 = 150 000 计数 = 200 mm
 *   ⇒ 750.0 计数/mm（转子侧，单轴）
 *   ⇒ 齿轮转一圈 = 110 000 计数 = 146.7 mm 齿条位移
 *
 *   两条测量互相印证：110000 ÷ 150000 × 200 = 146.7 mm/圈，与单独测的
 *   "齿轮一圈 110000 计数"一致。所以这一组数采用，**不再参考图纸的 69 mm
 *   分度圆 / 19.2:1**（那套推出来是 725.7 计数/mm，偏小 3.2%，且与
 *   "一圈 146.7 mm" 直接冲突）。
 *
 *   由 146.7 mm/圈 + 13.43:1 反推，实际分度圆直径 ≈ 46.7 mm（而不是 69 mm）。
 *   这个数只用于让 lift_kinematics 的公式结构保持不变；**换算的权威是实测的
 *   750.0 计数/mm**，不是这个直径。
 */
#define LIFT_TRAVEL_MIN_ECD                 0
/*
 * ★ 上界 = 150000 = 伸出【硬限位】本身（用户 2026-10-11 指定）。
 *
 * 用户确认：零点是缩回死点，伸出端是硬限位，两者之间正好 150000 计数
 * = 200 mm。所以 0..150000 就是完整跨度，软限位取满，200 mm 成为
 * **可命令**的位置。
 *
 * ⚠️ 代价必须知道（不是我反对，是要你知道会看到什么）：
 *   软限位 = 硬限位 ⇒ 一旦目标给到 150000，位置环会**一直顶在铁挡块上**：
 *     · 位置误差永远差最后一点，积分会缓慢累积
 *     · 该轴电流会长时间停在「托住 + 顶住」的水平，温升持续
 *     · 不会自动报故障（堵转看门狗只判"该动却不动"，满目标时它不判"该动"）
 *   所以：**150000 只适合"顶一下建立基准点然后转保持电流"**，
 *   不适合让位置环长期停在 150000。
 *   日常"抬到 200 mm"的目标应留一点余量（见 LIFT_TRAVEL_MARGIN_ECD）。
 */
#define LIFT_TRAVEL_MAX_ECD                 150000
/*
 * 停放点（缓降停止位置，单轴计数）：零点上方一点点。
 * 缓降是"把车放下来"，放到机械死点前就该停——不能拿 MIN 当目标。
 */
#define LIFT_TRAVEL_PARK_ECD                200
/* 上电/未找零时的安全起点：不往上顶，先给一个确定的小高度。 */
#define LIFT_TRAVEL_START_ECD               2000
/*
 * 命令面余量（单轴计数）。
 *
 * 2026-10-11：800 → 1500。行程从 34 mm 变成 200 mm 之后，同样的绝对余量
 * 对应的相对余量小了 6 倍。伸出端是铁碰铁，命令面（遥控/流程）给目标时
 * 留 1500 计数（2 mm）的余量，避免"操作手推到顶"就等于"长期顶限位"。
 *
 * 注意：这只夹【命令面】。位置环本身的软限位是 LIFT_TRAVEL_MAX_ECD，
 * 也就是 150000 —— 需要"顶到硬限位建立基准"时，绕开命令面直接给力即可
 * （走 FORCE 模式或堵转找零），不靠位置环。
 */
#define LIFT_TRAVEL_MARGIN_ECD              1500
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
/*  1c. 平滑链参数（2026-10-11 从台架模块移植，见 lift_control.h）        */
/*                                                                    */
/*  量纲约定：控制器的模态量 = 四轴之和 = 单轴 × 4（转置伴随，不除 4）。  */
/*  操作手想的是"每根齿条多少 mm/s"，所以下面按 mm/s 给，换算集中在这里。  */
/*                                                                    */
/*  ★ 机械换算（2026-10-11 实测定案，取代图纸推算）：                     */
/*      750.0 计数/mm（转子侧，单轴）                                   */
/*      = 150000 计数 / 200 mm（缩回死点 → 伸出硬限位）                  */
/*      齿轮一圈 = 110000 计数 = 146.7 mm                                */
/*    详见 LIFT_TRAVEL_MAX_ECD 上方那段。                               */
/* ==================================================================== */

#define LIFT_COUNTS_PER_MM_AXIS             750

/*
 * ---- 命令通道所有权（2026-10-11 修一个已证实的缺陷）------------------
 *
 * CAN1 `0x200` 的 0..3 槽是抬升四轴的电流缓存。写它的有三个模块：
 *   - 本任务（主链，5 ms / prio 24）
 *   - `lift_jog_task`（台架诊断，5 ms / prio 24）
 *   - 发送层的 DJI CAN 测试（整帧替换）
 *
 * 前两个同周期同优先级，谁后写谁生效。原来的约定是靠
 * `g_lift_jog_status.running` 互相谦让，但台架在空闲分支里每 5 ms 写一次
 * `robot_lift_set_currents(0,0,0,0)`，而 `running` 恰好在写 0 的同一行被置
 * false —— 于是"台架空闲"时主链算出来的电流被每 5 ms 砍成 0，
 * 真实进 C620 的值是一个取决于调度相位的占空比。
 *
 * 现在改成显式令牌（见 lift_safety.h 的所有权一节）：
 * 主链是本组的【默认所有者】，认领 → 写 → 立即归还；
 * 台架只在自己真正跑起来时占用。
 */
#define LIFT_OWNER_TIMEOUT_MS               20U

/* 单轴 → 模态（×4）。下面所有 MODAL 常量都走它，避免再漏乘。 */
#define LIFT_PER_AXIS_TO_MODAL(x) \
    ((x) * (int32_t)LIFT_CHANNEL_COUNT)

/* mm/s（单轴）→ 模态 ecd/s */
#define LIFT_MM_S_TO_MODAL_SPEED(mm_s) \
    LIFT_PER_AXIS_TO_MODAL((mm_s) * (int32_t)LIFT_COUNTS_PER_MM_AXIS)

/* mm/s（单轴）→ 模态"每控制周期步长"（目标斜坡用） */
#define LIFT_MM_S_TO_MODAL_STEP(mm_s) \
    LIFT_PER_AXIS_TO_MODAL( \
        ((mm_s) * (int32_t)LIFT_COUNTS_PER_MM_AXIS * \
         (int32_t)LIFT_CONTROL_PERIOD_MS) / 1000)

/*
 * 目标斜坡速率（单轴 mm/s）。这是"抬升有多快"的总开关：
 * 位置环的速度上限、接近减速都跟着它量级走。
 *   0 = 关掉斜坡（目标阶跃）——只有调 PID 时才这么干。
 */
#define LIFT_TARGET_RATE_MM_S               8
/* 位置环速度上限（单轴 mm/s）：只需要略大于斜坡速率，用来兜住大误差时的冲量 */
#define LIFT_MAX_SPEED_MM_S                 15
/* 接近减速区起点（单轴 mm）：离目标还有这么多就开始减速 */
#define LIFT_APPROACH_ZONE_MM               5
/* 接近段最低目标速度（单轴 mm/s）：**不能填 0**（填 0 会卡在死区外进不去） */
#define LIFT_APPROACH_MIN_SPEED_MM_S        1
/* 判定"停住了"的转速（转子 rpm）：20 rpm ≈ 2730 单轴计数/s */
#define LIFT_HOLD_SPEED_RPM                 20
#define LIFT_HOLD_SPEED_ECD_S \
    LIFT_PER_AXIS_TO_MODAL( \
        ((int32_t)LIFT_HOLD_SPEED_RPM * LIFT_ECD_PER_REV) / 60)
/* 电流 di/dt 上限（模态控制量/秒）：60000 → 5 ms 每周期 300 → 0→16000 约 0.27 s */
#define LIFT_CURRENT_RAMP_PER_S             60000
/* 逐级开放的起点与放开速度（用户实测"空载大幅摆动"的解药之一） */
#define LIFT_CURRENT_LIMIT_START            2000
#define LIFT_CURRENT_LIMIT_RAMP_PER_S       10000
/* 速度反馈一阶低通（ms）：压 C620 整数 rpm 的量化毛刺；0 = 关 */
#define LIFT_SPEED_FILTER_TAU_MS            20

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
 * 位置环输出的限幅：量纲是【速度】(模态 ecd/s)，不是电流，所以它跟上面的
 * 16000 没有关系。
 *
 * ⚠️ 2026-10-11 重定：以前是 4000 模态计数/s，换算成单轴只有
 *    4000/4 = 1000 计数/s ÷ 750 计数/mm ≈ **1.33 mm/s** —— 爬 20 cm 要 150 秒，
 *    实际上"抬不起来"。现在改成按 mm/s 给（15 mm/s 单轴），换算过程写在下面的
 *    宏里，要提速/降速只改 LIFT_MAX_SPEED_MM_S 一个数。
 */
#define LIFT_POSITION_LOOP_SPEED_CEILING_ECD_S \
    LIFT_MM_S_TO_MODAL_SPEED(LIFT_MAX_SPEED_MM_S)

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


lift_task_command_t g_lift_task_command;
lift_control_debug_t g_lift_control_debug;
lift_task_status_t g_lift_control_status;
lift_seq_t g_lift_sequence;
lift_seq_pub_t g_lift_seq_status;
lift_fault_snapshot_t g_lift_fault_snapshot;

static lift_top_t lift_top;
/*
 * 分阶段上台阶流程的运行实例。放在本任务的作用域里（不是 lift_top 里）：
 * 它要读 lift_top 的完整状态、要写命令、还要告诉操作手下一步做什么，
 * 属于"编排"层而不是"执行"层。
 */
static lift_seq_config_t lift_seq_cfg;/* 上一周期的状态快照：安全层要在调用 lift_top_update 之前用到它。 */
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
/*  2. 参数预设与数值检查（初始增益必须经过实机标定）          */
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

/*
 * 这里曾经有 `lift_abs_i32_clamped()` 与 `lift_ecd_s_per_rpm()` 两个函数。
 * 前者在参数自检改成直接用 int16 硬上限之后就没有调用者了；
 * 后者是"speed_rpm → ecd/s"的私有换算，现在这件事统一归
 * `lift_kinematics` 管（`lift_kin_speed_to_mm_s`），而控制链内部那份
 * 仍然在 `lift_control_speed_to_ecd_s`。两份私有副本正是本项目
 * 反复出错的根源，所以两个都没留。
 */

/* Explicit starting preset, not a stability prediction from a static K.
 * Position output is single-axis ecd/s, speed output is C620 current units. */
static void lift_select_pid_preset(lift_control_debug_t *debug)
{
    if (!debug->pid_manual_override)
    {
        debug->position_kp = 5.0F;
        debug->position_ki = 0.0F;
        debug->position_kd = 0.0F;
        debug->position_integral_limit = 0.0F;
        debug->position_output_limit = 11250.0F;
        debug->speed_kp = 2.0F;
        debug->speed_ki = 0.5F;
        debug->speed_kd = 0.0F;
        debug->speed_integral_limit = 16000.0F;
        debug->speed_output_limit = 16000.0F;
    }
    lift_speed_loop_kp = debug->speed_kp;
    lift_speed_loop_kp_max = 0.0F; /* no fabricated stability bound */
    lift_position_loop_kp = debug->position_kp;
}

/**
 * @brief 数值检查标定参数是否自洽（不检查是否"调好了"）。
 * @note 只拦真正会让机构受伤或让逻辑失效的组合：电流越限、方向非法、
 *       区间倒置、周期非正、非有限增益。数值合法不等于实机稳定。
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
        (debug->approach_zone_ecd < 0) ||
        (debug->approach_min_speed_ecd_s < 0) ||
        (debug->hold_speed_ecd_s < 0) ||
        (debug->current_ramp_per_s < 0) ||
        (debug->current_limit_start < 0) ||
        (debug->current_limit_ramp_per_s < 0) ||
        (debug->speed_filter_tau_ms < 0) ||
        !lift_float_valid(debug->gear_ratio) ||
        !lift_float_valid(debug->period_s) ||
        !lift_float_valid(debug->actuator_gain_ecd_s_per_current) ||
        !lift_float_valid(debug->gravity_feedforward_current))
    {
        return false;
    }

    /* Validate both preset and manual gains. */
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

    if (!lift_float_valid(debug->position_output_limit) ||
        !lift_float_valid(debug->speed_output_limit) ||
        !lift_float_valid(debug->position_integral_limit) ||
        !lift_float_valid(debug->speed_integral_limit) ||
        debug->position_ki != 0.0F || debug->position_kd != 0.0F ||
        !lift_float_valid(debug->target_rate_mm_s) || debug->target_rate_mm_s <= 0.0F ||
        !lift_float_valid(debug->target_accel_mm_s2) || debug->target_accel_mm_s2 <= 0.0F ||
        !lift_float_valid(debug->sync_slow_mm) || debug->sync_slow_mm < 0.0F ||
        !lift_float_valid(debug->sync_stop_mm) || debug->sync_stop_mm <= debug->sync_slow_mm)
        return false;
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        pid_config_t pos = debug->axis_position_pid[ch];
        pid_config_t spd = debug->axis_speed_pid[ch];
        float ff = debug->axis_feedforward_override ? debug->axis_load_current[ch] :
            debug->gravity_feedforward_current;
        if (!lift_float_valid(debug->axis_counts_per_mm[ch]) ||
            debug->axis_counts_per_mm[ch] <= 0.0F ||
            !lift_float_valid(ff) || ff < 0.0F || ff > debug->guard_current_limit ||
            (int64_t)debug->position_max_ecd[ch] - debug->position_min_ecd[ch] <=
                debug->limit_margin_ecd)
            return false;
        if (debug->axis_pid_override &&
            (!lift_float_valid(pos.kp) || pos.kp < 0.0F || pos.ki != 0.0F || pos.kd != 0.0F ||
             !lift_float_valid(pos.output_limit) || pos.output_limit <= 0.0F ||
             !lift_float_valid(spd.kp) || spd.kp < 0.0F ||
             !lift_float_valid(spd.ki) || spd.ki < 0.0F ||
             !lift_float_valid(spd.kd) || spd.kd < 0.0F ||
             !lift_float_valid(spd.integral_limit) || spd.integral_limit < 0.0F ||
             !lift_float_valid(spd.output_limit) || spd.output_limit <= 0.0F))
            return false;
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
        (debug->homing_contact_current <= 0) ||
        (debug->homing_contact_current > debug->homing_current_limit) ||
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
{    uint32_t ch;

    (void)memset(config, 0, sizeof(*config));
    lift_sync_config_rect(&config->control.sync);
    config->homing_known_zero_mask = ROBOT_LIFT_REAR_MASK;
    config->homing_contact_mask = ROBOT_LIFT_FRONT_MASK;
    config->homing_contact_current = debug->homing_contact_current;
    config->homing_contact_confirm_timeout_ms = 60000U;

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

    config->control.axis_control_enabled = true;
    config->control.axis_current_limit = debug->guard_current_limit;
    config->control.target_rate_mm_s = debug->target_rate_mm_s;
    config->control.target_accel_mm_s2 = debug->target_accel_mm_s2;
    config->control.sync_slow_mm = debug->sync_slow_mm;
    config->control.sync_stop_mm = debug->sync_stop_mm;
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        config->control.counts_per_mm[ch] = debug->axis_counts_per_mm[ch];
        config->control.axis_load_current[ch] = debug->axis_feedforward_override ?
            debug->axis_load_current[ch] : debug->gravity_feedforward_current;
        config->control.axis_pid[LIFT_CONTROL_LOOP_POSITION][ch] = debug->axis_pid_override ?
            debug->axis_position_pid[ch] : config->control.pid[LIFT_CONTROL_LOOP_POSITION];
        config->control.axis_pid[LIFT_CONTROL_LOOP_SPEED][ch] = debug->axis_pid_override ?
            debug->axis_speed_pid[ch] : config->control.pid[LIFT_CONTROL_LOOP_SPEED];
    }

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
    /* 平滑链（2026-10-11 从台架移植） */
    config->control.approach_zone_ecd = debug->approach_zone_ecd;
    config->control.approach_min_speed_ecd_s = debug->approach_min_speed_ecd_s;
    config->control.hold_speed_ecd_s = debug->hold_speed_ecd_s;
    config->control.current_ramp_per_s = debug->current_ramp_per_s;
    config->control.current_limit_start = debug->current_limit_start;
    config->control.current_limit_ramp_per_s =
        debug->current_limit_ramp_per_s;
    config->control.speed_filter_tau_ms = debug->speed_filter_tau_ms;
}

/* ==================================================================== */
/*  运动学换算层（2026-10-11 新增，见 lift_kinematics.h）                */
/*                                                                    */
/*  在这之前全项目没有任何一处把编码器计数换成毫米：                    */
/*    - `lift_geometry.c` 是【丝杆导程】模型（机构其实是齿轮齿条），     */
/*      而且零调用；                                                  */
/*    - `LIFT_COUNTS_PER_MM_AXIS` 只用来把 mm/s 折成模态速度参数，       */
/*      从来没有用来换算位置。                                          */
/*  结果是"抬升 200 mm"这句话在固件里无法表达，只能手填一串计数。        */
/* ==================================================================== */

/* 本任务持有的运动学配置（由 g_lift_control_debug 每次应用时重建）。 */
static lift_kin_config_t lift_kin;
static bool lift_kin_ok;               /* 四轴系数与行程都配好了 */
static int32_t lift_kin_travel_max_ecd[LIFT_CHANNEL_COUNT];  /* 正方向行程 */

/**
 * @brief 由标定参数构建运动学配置。
 *
 * 只填【能由图纸算出来的】和【操作手明确给的】两类量：
 *   系数：8192 计数/圈、19.2:1、69 mm 分度圆（由 lift_kinematics 自己算）
 *   方向：debug->axis_direction（P2 实测）
 *   行程：debug->position_min/max_ecd（找零后手推到两端量出来的【计数】）
 *
 * ⚠️ 行程仍然以【计数】为权威：它是实测出来的，而且与 lift_control /
 *    lift_guard 里的软限位是同一套边界。运动学层只负责"毫米 ↔ 计数"，
 *    不参与限位的决定权。
 */
static void lift_build_kinematics(const lift_control_debug_t *debug)
{
    int8_t direction[LIFT_CHANNEL_COUNT];
    int32_t travel_span[LIFT_CHANNEL_COUNT];
    uint32_t ch;
    uint32_t axis_count = (uint32_t)LIFT_CHANNEL_COUNT;

    lift_kin_config_init_uncalibrated(&lift_kin);

    for (ch = 0U; ch < axis_count; ch++)
    {
        direction[ch] = debug->axis_direction[ch];
    }
    lift_kin_config_nominal_rack_and_pinion(&lift_kin, direction);
    for (ch = 0U; ch < axis_count; ch++)
        lift_kin.axis[ch].counts_per_mm = debug->axis_counts_per_mm[ch];

    /*
     * 行程区间：用【计入方向的逻辑跨度】换算出毫米。
     *
     * 逻辑区间是 [0, span]，其中 span = |position_max − position_min|。
     * 这样无论某轴的零点设在哪，毫米值都是"从行程下限往上量"，
     * 与 lift_top 已经归一化过的逻辑坐标一致。
     */
    lift_kin_ok = true;
    for (ch = 0U; ch < axis_count; ch++)
    {
        int32_t span = debug->position_max_ecd[ch] -
                       debug->position_min_ecd[ch];
        int32_t span_mm = 0;

        if (span < 0)
        {
            span = -span;
        }
        travel_span[ch] = span;

        if (!lift_kin_axis_config_is_valid(&lift_kin.axis[ch]) ||
            !lift_kin_ecd_to_mm(&lift_kin, ch, span, &span_mm) ||
            (span_mm <= 0))
        {
            /* 系数非法或行程为 0：这一轴不能用毫米表达。 */
            lift_kin.position_min_mm[ch] = 0;
            lift_kin.position_max_mm[ch] = 0;
            lift_kin_ok = false;
            continue;
        }

        if (span_mm > LIFT_KIN_MAX_TRAVEL_MM)
        {
            /*
             * 按当前系数算出来超过 1000 mm —— 一定是行程（或系数）写错了。
             * 不裁剪，直接判这一轴不可用：宁可拒绝动作，也不要拿一个
             * 明显不对的数去驱动机构。
             */
            lift_kin.position_min_mm[ch] = 0;
            lift_kin.position_max_mm[ch] = 0;
            lift_kin_ok = false;
            continue;
        }

        lift_kin.position_min_mm[ch] = 0;
        lift_kin.position_max_mm[ch] = span_mm;
    }

    for (ch = 0U; ch < axis_count; ch++)
    {
        lift_kin_travel_max_ecd[ch] = travel_span[ch];
    }
}

/* Convert every sequence target; do not average away front/rear or warp.
 * Reject the complete command if any axis cannot be converted. */
static bool lift_command_from_axis_mm(const int32_t target_mm[LIFT_CHANNEL_COUNT],
    lift_control_command_t *command)
{
    int32_t ecd[LIFT_CHANNEL_COUNT];
    lift_sync_modes_t modes;
    uint32_t ch;
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        if (!lift_kin_mm_to_ecd(&lift_kin, ch, target_mm[ch], &ecd[ch]))
            return false;
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        command->axis_target_ecd[ch] = ecd[ch];
    command->axis_targets_valid = true;
    lift_sync_forward(&lift_top.config.control.sync, ecd, &modes);
    command->target_height_ecd = modes.height;
    command->target_roll_ecd = modes.roll;
    command->target_pitch_ecd = modes.pitch;
    return true;
}

/* PRECHECK owns the command without inventing a zero-position target. */
static bool lift_route_sequence(const lift_seq_status_t *seq,
    lift_top_input_t *input, lift_control_command_t *command)
{
    uint32_t ch;
    if (seq->command_valid && seq->command_enabled && lift_kin_ok)
    {
        if (seq->command_ecd_valid)
        {
            command->axis_targets_valid = true;
            for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
                command->axis_target_ecd[ch] = seq->axis_target_ecd[ch];
        }
        else if (!lift_command_from_axis_mm(seq->axis_target_mm, command))
        {
            command->enabled = false;
            input->command_fresh = false;
            return false;
        }
    }
    else if (seq->state == LIFT_SEQ_STATE_PRECHECK)
    {
        command->axis_targets_valid = true;
        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
            command->axis_target_ecd[ch] = lift_top.config.axis_direction[ch] *
                (input->position_ecd[ch] - lift_top.zero_ecd[ch]);
    }
    else
    {
        command->enabled = false;
        input->command_fresh = false;
        return false;
    }
    command->enabled = true;
    input->command_fresh = true;
    if (seq->command_zero_output) command->enabled = false;
    return true;
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
    g_lift_control_debug.settle_tolerance_ecd = 150; /* single-axis 0.2 mm */
    g_lift_control_debug.target_rate_mm_s = 8.0F;
    g_lift_control_debug.target_accel_mm_s2 = 20.0F;
    g_lift_control_debug.sync_slow_mm = 3.0F;
    g_lift_control_debug.sync_stop_mm = 10.0F;
    g_lift_control_debug.limit_margin_ecd = LIFT_TRAVEL_MARGIN_ECD;
    /*
     * 目标斜坡（模态计数/周期）：8 mm/s 单轴 → 29 单轴计数/周期 → 116 模态。
     * 以前是 60（≈4.1 mm/s），现在按 mm/s 给，改 LIFT_TARGET_RATE_MM_S 即可。
     */
    g_lift_control_debug.max_target_step_ecd =
        LIFT_MM_S_TO_MODAL_STEP(LIFT_TARGET_RATE_MM_S);

    /* ---- 平滑链默认值（见 lift_control.h 的五件套说明）---- */
    g_lift_control_debug.approach_zone_ecd =
        LIFT_PER_AXIS_TO_MODAL(LIFT_APPROACH_ZONE_MM *
                               (int32_t)LIFT_COUNTS_PER_MM_AXIS);
    g_lift_control_debug.approach_min_speed_ecd_s =
        LIFT_MM_S_TO_MODAL_SPEED(LIFT_APPROACH_MIN_SPEED_MM_S);
    g_lift_control_debug.hold_speed_ecd_s = 375; /* single-axis 0.5 mm/s */
    g_lift_control_debug.current_ramp_per_s = LIFT_CURRENT_RAMP_PER_S;
    g_lift_control_debug.current_limit_start = LIFT_CURRENT_LIMIT_START;
    g_lift_control_debug.current_limit_ramp_per_s =
        LIFT_CURRENT_LIMIT_RAMP_PER_S;
    g_lift_control_debug.speed_filter_tau_ms = LIFT_SPEED_FILTER_TAU_MS;

    /*
     * 找零：向下顶死点，再反方向退出。
     * 符号约定：逻辑"正" = 上升，所以 seek 是负（往下顶），backoff 是正。
     * 电流按实测重设：给 2000 才动，所以 seek/backoff 都必须在这个量级，
     * 否则必然 SEEK_TIMEOUT / BACKOFF_TIMEOUT。
     * 实测确认方向后，若某根反了，只改 axis_direction，不改这里。
     */
    g_lift_control_debug.homing_seek_current =
        LIFT_FACTORY_HOMING_SEEK_CURRENT;
    /* Preserve the old low seek-current magnitude; direction is extension. */
    g_lift_control_debug.homing_contact_current =
        -LIFT_FACTORY_HOMING_SEEK_CURRENT;
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
         * ★ 2026-10-11 用户手推复核（第二次独立确认，结论与 10-09 一致）：
         *     推"伸出"（车被顶起）时原始计数的变化方向：
         *       ch0 −1    ch1 +1    ch2 −1    ch3 +1
         *   即 0/2 → -1，1/3 → +1。两侧镜像安装，同一个物理方向
         *   原始符号相反，所以负的那一对填 -1、正的那一对填 +1。
         *
         * ⚠️ 这条会同时决定三件事，改错一个就是飞车或往错方向顶：
         *   ① 位置读数方向（位置环靠它，错了就是正反馈）
         *   ② 找零/缓降方向（seek = 逻辑负 = 齿条缩回）
         *   ③ 抬升出力方向
         * 所以它只能靠"手推 + 看 delta_ecd"来定，不能靠猜。
         * 台架（lift_jog_task）用的也是这一张表（见 lift_jog_task.c 的
         * lift_test_logical_to_raw），所以台架标定出来的方向对主链同样有效。
         */
        g_lift_control_debug.axis_direction[ch] =
            ((ch == 0U) || (ch == 2U)) ? -1 : 1;
        g_lift_control_debug.axis_counts_per_mm[ch] = LIFT_COUNTS_PER_MM_AXIS;
        g_lift_control_debug.axis_position_pid[ch] = (pid_config_t){5.0F, 0.0F, 0.0F, 0.0F, 11250.0F};
        g_lift_control_debug.axis_speed_pid[ch] = (pid_config_t){2.0F, 0.5F, 0.0F, 16000.0F, 16000.0F};
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

    lift_select_pid_preset(&g_lift_control_debug);
}

/**
 * @brief 上电参数应用。
 *
 * 在 `lift_control_task_init()` 里先跑一次：标定参数与所选增益必须在第一
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
        g_lift_task_command.rear_at_minimum_confirmed = false;
        g_lift_task_command.front_contact_confirmed_mask = 0U;
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

    /* 选择参数预设；手动模式保留用户增益。 */
    lift_select_pid_preset(&g_lift_control_debug);

    taskENTER_CRITICAL();
    debug = g_lift_control_debug;
    taskEXIT_CRITICAL();

    lift_build_config(&config, &debug);
    lift_top_init(&lift_top, &config);
    lift_build_kinematics(&debug);
    lift_configuration_applied = true;
    lift_configuration_rejected = false;
    lift_safety_limited = false;
    g_lift_control_status.parameter_apply_count++;

    /* 让 Watch 里能直接看到所选增益，不用手算。 */
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

    /* 取一份参数快照（含手动增益）；预设只在非手动模式下改 kp/ki/kd */
    lift_select_pid_preset(&g_lift_control_debug);
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
    lift_build_kinematics(&debug);

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

    /*
     * ★ 运动学标定完成时，命令面的夹取规则不同（2026-10-11）。
     *
     * 标定完成后，每轴目标是由 lift_seq 用【每轴独立的标定系数】算出来的，
     * 再正变换成模态量。此时"模态高度"就是四条齿条各自计数的和，
     * 它【本来就不该】与"四轴之和的行程上限"比较 —— 两个上限的来路不同：
     * 模态上限是"四轴各自限位之和"，而每轴目标已经各自被自己的限位夹过了。
     * 拿模态上限再去夹一次，会把合法的每轴目标整体压回起点，
     * 现象是"标定做完了反而一点都不动"。
     *
     * 每轴安全性在换算那一步已经保证：目标毫米只能落在 [0, travel_max_mm]
     * 之间（lift_kin 的行程区间就是从 lift_control/lift_guard 的那一套
     * 单轴限位换算来的），所以这里只需要管姿态。
     */
    if (!lift_kin_ok)
    {
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
    }
    if (any_not_homed)
    {
        /* 没找零时目标没有意义，直接贴到起点，避免"上一轮的目标"残留。 */
        g_lift_task_command.target_height_ecd = start;
    }
    if (!lift_kin_ok)
    {
        g_lift_task_command.target_roll_ecd = lift_clamp_i32(
            g_lift_task_command.target_roll_ecd,
            -LIFT_LIMIT_ROLL_MAX_ECD, LIFT_LIMIT_ROLL_MAX_ECD);
        g_lift_task_command.target_pitch_ecd = lift_clamp_i32(
            g_lift_task_command.target_pitch_ecd,
            -LIFT_LIMIT_PITCH_MAX_ECD, LIFT_LIMIT_PITCH_MAX_ECD);
    }
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
    g_lift_task_command.front_contact_confirmed_mask = 0U;
    if (command->homing_request)
    {
        g_lift_task_command.rear_at_minimum_confirmed = false;
    }
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
     * 故障停机动作（悬空不清零 / 落地才切断）与"能不能托住"的可见性。
     * stop_capable 为 false 说明保持电流还没标定 —— **这是能力缺口**，
     * 一旦出故障就只能断电掉车，操作手必须知道。
     */
    g_lift_control_status.stop_action = (uint8_t)top_status->stop_action;
    g_lift_control_status.stop_active = top_status->stop_active;
    g_lift_control_status.stop_capable = top_status->stop_capable;
    g_lift_control_status.kinematics_ready = lift_kin_ok;
    for (ch = 0U; ch < (uint32_t)LIFT_CHANNEL_COUNT; ch++)
    {
        g_lift_control_status.counts_per_mm_x1000[ch] =
            lift_kin_counts_per_mm_x1000(&lift_kin, ch);
        g_lift_control_status.travel_max_mm[ch] = lift_kin.position_max_mm[ch];
    }
    g_lift_control_status.axis_control_active = lift_top.config.control.axis_control_enabled &&
        lift_top.control.axis_initialized && output_gate == LIFT_OUTPUT_GATE_NONE;
    g_lift_control_status.axis_sync_scale = top_status->control.sync_scale;
    g_lift_control_status.axis_sync_spread_mm = top_status->control.sync_spread_mm;
    for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
    {
        g_lift_control_status.axis_target_ecd[ch] = top_status->control.axis_target_ecd[ch];
        g_lift_control_status.axis_reference_ecd[ch] = top_status->control.axis_reference_ecd[ch];
        g_lift_control_status.axis_position_error[ch] = top_status->control.axis_position_error[ch];
        g_lift_control_status.axis_target_speed_ecd_s[ch] = top_status->control.axis_target_speed_ecd_s[ch];
        g_lift_control_status.axis_speed_ecd_s[ch] = top_status->control.axis_speed_ecd_s[ch];
        g_lift_control_status.axis_filtered_speed_ecd_s[ch] = top_status->control.axis_filtered_speed_ecd_s[ch];
        g_lift_control_status.axis_speed_error[ch] = top_status->control.axis_speed_error[ch];
        g_lift_control_status.axis_position_pid_output[ch] = top_status->control.axis_position_output[ch];
        g_lift_control_status.axis_speed_pid_output[ch] = top_status->control.axis_speed_output[ch];
        g_lift_control_status.axis_feedforward_current[ch] = top_status->control.axis_feedforward_current[ch];
        g_lift_control_status.axis_speed_integral[ch] = top_status->control.axis_speed_integral[ch];
        g_lift_control_status.axis_limit_now[ch] = top_status->control.axis_limit_now[ch];
        g_lift_control_status.axis_current_lower[ch] = top_status->control.axis_current_lower[ch];
        g_lift_control_status.axis_current_upper[ch] = top_status->control.axis_current_upper[ch];
        g_lift_control_status.axis_current_demand[ch] = top_status->control.axis_current_demand[ch];
        g_lift_control_status.axis_sync_error_mm[ch] = top_status->control.axis_sync_error_mm[ch];
        g_lift_control_status.axis_hold_state[ch] = top_status->control.axis_hold_state[ch];
        g_lift_control_status.axis_final_current[ch] =
            (float)(lift_top.config.axis_direction[ch] * applied_current[ch]);
        g_lift_control_status.axis_feedback_current[ch] = (int16_t)
            (lift_top.config.axis_direction[ch] * motors->lift.motor[ch].given_current);
    }

    /*
     * 四轴一致性（毫米）。必须在【每轴独立换算之后】算：
     * 拿计数直接比会让系数大的那根显得"更偏"，判据跟着标定系数走，
     * 没有物理意义。
     */
    {
        int32_t spread = 0;
        int32_t dev_max = 0;

        if (lift_kin_position_spread_mm(&lift_kin,
                                        top_status->homed_position_ecd,
                                        &spread))
        {
            g_lift_control_status.sync_error_mm = spread;
        }
        else
        {
            g_lift_control_status.sync_error_mm = 0;
        }

        for (ch = 0U; ch < (uint32_t)LIFT_CHANNEL_COUNT; ch++)
        {
            int32_t dev = 0;

            if (lift_kin_axis_deviation_mm(&lift_kin,
                                           top_status->homed_position_ecd,
                                           ch, &dev))
            {
                if (dev < 0)
                {
                    dev = -dev;
                }
                if (dev > dev_max)
                {
                    dev_max = dev;
                }
            }
        }
        g_lift_control_status.deviation_max_mm = dev_max;
    }
    /* 平滑链的实时状态（按模态）：到位了为什么还有电流，先看这两个 */
    for (ch = 0U; ch < (uint32_t)LIFT_CONTROL_MODE_COUNT; ch++)
    {
        g_lift_control_status.mode_hold_state[ch] =
            top_status->control.hold_state[ch];
        g_lift_control_status.mode_limit_open_current[ch] =
            top_status->control.limit_open_current[ch];
    }

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
         * 否则"遥控以为能到 X、控制器只让到 Y"，操作手会觉得
         * 推到底却不动，而去怀疑增益。
         */
        g_lift_control_status.limit_height_min_ecd[ch] =
            lift_top.config.control.limit_height_min_ecd[ch];
        g_lift_control_status.limit_height_max_ecd[ch] =
            lift_top.config.control.limit_height_max_ecd[ch];
    }
}

static bool lift_test_start_pending;
static bool lift_stair_start_pending;
static bool lift_stair_owned;
static int32_t stair_distance[3], stair_speed[3];
static int32_t stair_previous_ecd[4];
static int8_t stair_direction[4];
static float stair_counts_per_mm;
static float stair_progress, stair_last_progress, stair_velocity;
static int8_t stair_leg = -1;
static uint32_t stair_motion_tick, stair_settle_tick;
static bool stair_settling;

/* Called before the sequence update. Completion belongs only to the current
 * leg; pauses retain wheel progress, including coasting while paused. */
static bool lift_stair_drive(const robot_motor_state_t *motors, uint32_t now)
{
    const lift_seq_state_t stage = g_lift_sequence.state;
    const int8_t leg = stage == LIFT_SEQ_STATE_APPROACH_STEP ? 0 :
        stage == LIFT_SEQ_STATE_ADVANCE_MIDDLE ? 1 :
        stage == LIFT_SEQ_STATE_FINAL_ADVANCE ? 2 : -1;
    bool stationary = true;
    float increment = 0.0F;
    uint32_t ch;
    if (!lift_stair_owned) return false;
    chassis_control_stair_velocity(0.0F);
    if (g_dji_can_test_status.running || g_dji_can_test_status.zero_holding ||
        g_lift_jog_status.running || !lift_configuration_applied ||
        lift_last_status.fault != LIFT_TOP_FAULT_NONE)
    {
        g_lift_control_debug.test_pause = true;
        stair_velocity = 0.0F;
        return false;
    }
    if (g_lift_control_debug.stair_drive_release)
    {
        g_lift_control_debug.stair_drive_release = false;
        if (stage == LIFT_SEQ_STATE_IDLE || stage == LIFT_SEQ_STATE_COMPLETE)
        {
            chassis_control_stair_release();
            lift_stair_owned = false;
            return false;
        }
    }
    if (leg >= 0 && leg != stair_leg)
    {
        stair_leg = leg;
        stair_progress = stair_last_progress = stair_velocity = 0.0F;
        stair_motion_tick = now;
        stair_settling = false;
        for (ch = 0U; ch < 4U; ch++)
            stair_previous_ecd[ch] = motors->chassis.wheel[ch].total_ecd;
    }
    for (ch = 0U; ch < 4U; ch++)
    {
        const dji_motor_feedback_t *m = &motors->chassis.wheel[ch];
        if (!dji_motor_feedback_is_online(m, now,
                pdMS_TO_TICKS(LIFT_CONTROL_COMMAND_TIMEOUT_MS)))
        {
            g_lift_control_debug.stair_drive_stop_reason = 1U;
            g_lift_control_debug.test_pause = true;
            stair_velocity = 0.0F;
            return false;
        }
        if (m->speed_rpm > 2 || m->speed_rpm < -2) stationary = false;
        if (stair_leg >= 0)
        {
            /* Unsigned subtraction avoids UB across encoder counter wrap. */
            const int32_t delta = (int32_t)((uint32_t)m->total_ecd -
                (uint32_t)stair_previous_ecd[ch]);
            increment += (float)delta * (float)stair_direction[ch];
            stair_previous_ecd[ch] = m->total_ecd;
        }
    }
    if (stair_leg >= 0) stair_progress += increment / (4.0F * stair_counts_per_mm);
    g_lift_control_debug.stair_drive_progress_mm = (int32_t)stair_progress;
    g_lift_control_debug.stair_drive_leg = leg;
    if (leg < 0 || g_lift_control_debug.test_pause ||
        g_lift_control_debug.sequence_abort)
    {
        stair_velocity = 0.0F;
        stair_motion_tick = now;
        stair_last_progress = stair_progress;
        stair_settling = false;
        return false;
    }
    if (stair_progress < -10.0F)
    {
        g_lift_control_debug.stair_drive_stop_reason = 3U;
        g_lift_control_debug.test_pause = true;
        stair_velocity = 0.0F;
        return false;
    }
    if (stair_progress >= (float)stair_distance[leg])
    {
        stair_velocity = 0.0F;
        if (!stationary) stair_settling = false;
        else if (!stair_settling) { stair_settling = true; stair_settle_tick = now; }
        return stair_settling && (uint32_t)(now - stair_settle_tick) >= 100U;
    }
    if (stair_progress - stair_last_progress >= 2.0F)
    {
        stair_last_progress = stair_progress;
        stair_motion_tick = now;
    }
    else if ((uint32_t)(now - stair_motion_tick) >= 2000U)
    {
        g_lift_control_debug.stair_drive_stop_reason = 2U;
        g_lift_control_debug.test_pause = true;
        stair_velocity = 0.0F;
        return false;
    }
    {
        float desired = ((float)stair_distance[leg] - stair_progress) * 2.0F;
        if (desired < 10.0F) desired = 10.0F;
        if (desired > (float)stair_speed[leg]) desired = (float)stair_speed[leg];
        if (stair_velocity < desired) stair_velocity += 0.5F; /* 100 mm/s2 @5ms */
        if (stair_velocity > desired) stair_velocity = desired;
        chassis_control_stair_velocity(stair_velocity * 0.001F);
    }
    return false;
}

static void lift_handle_test_start(bool bench_owns)
{
    bool requested;
    bool stair_requested;
    int32_t height_mm;
    const bool was_pending = lift_test_start_pending;
    taskENTER_CRITICAL();
    requested = g_lift_control_debug.test_start;
    stair_requested = g_lift_control_debug.stair_start;
    height_mm = g_lift_control_debug.test_height_mm;
    g_lift_control_debug.test_start = false;
    g_lift_control_debug.stair_start = false;
    taskEXIT_CRITICAL();
    if (!requested && !stair_requested && !lift_test_start_pending) return;
    if (requested && stair_requested)
    {
        g_lift_control_debug.test_start_result = 3U;
        return;
    }
    if (was_pending) stair_requested = lift_stair_start_pending;
    lift_test_start_pending = false;
    lift_stair_start_pending = false;
    if (stair_requested)
    {
        robot_motor_state_t start_motors;
        uint32_t ch;
        const float rpm_per_mps = g_chassis_control_debug.rpm_per_mps;
        if (!(rpm_per_mps > 0.0F && rpm_per_mps < 100000.0F) ||
            g_lift_control_debug.stair_retracted_mm < 0 ||
            g_lift_control_debug.stair_retracted_mm >= height_mm ||
            g_lift_control_debug.stair_contact_mm < g_lift_control_debug.stair_retracted_mm ||
            g_lift_control_debug.stair_contact_mm > height_mm)
        {
            g_lift_control_debug.test_start_result = 8U;
            return;
        }
        for (ch = 0U; ch < 3U; ch++)
            if (g_lift_control_debug.stair_distance_mm[ch] < 1 ||
                g_lift_control_debug.stair_distance_mm[ch] > 2000 ||
                g_lift_control_debug.stair_speed_mm_s[ch] < 10 ||
                g_lift_control_debug.stair_speed_mm_s[ch] > 200)
            {
                g_lift_control_debug.test_start_result = 8U;
                return;
            }
        for (ch = 0U; ch < 4U; ch++)
            if (g_chassis_control_debug.motor_direction[ch] != 1 &&
                g_chassis_control_debug.motor_direction[ch] != -1)
            {
                g_lift_control_debug.test_start_result = 8U;
                return;
            }
        robot_motors_get_snapshot(&start_motors);
        for (ch = 0U; ch < 4U; ch++)
            if (!dji_motor_feedback_is_online(&start_motors.chassis.wheel[ch],
                osKernelGetTickCount(), pdMS_TO_TICKS(LIFT_CONTROL_COMMAND_TIMEOUT_MS)) ||
                start_motors.chassis.wheel[ch].speed_rpm > 2 ||
                start_motors.chassis.wheel[ch].speed_rpm < -2)
            {
                g_lift_control_debug.test_start_result = 9U;
                return;
            }
    }
    if (height_mm < 9 || height_mm > 200)
    {
        g_lift_control_debug.test_start_result = 6U;
        return;
    }
    if (g_lift_control_debug.sequence_abort)
    {
        g_lift_control_debug.test_start_result = 3U;
        return;
    }
    if (bench_owns)
    {
        g_lift_control_debug.test_start_result = 5U;
        return;
    }
    if (!lift_last_status.homed)
    {
        robot_motor_state_t zero_motors;
        int32_t zero_raw[LIFT_CHANNEL_COUNT];
        uint32_t ch;
        if (!g_lift_control_debug.test_auto_zero)
        {
            g_lift_control_debug.test_start_result = 2U;
            return;
        }
        /* Software datum only: never clear a fault or interrupt another owner.
         * Current pose is NOT evidence of a physical lower endpoint. */
        if (lift_top.state != LIFT_TOP_STATE_IDLE ||
            lift_top.fault != LIFT_TOP_FAULT_NONE ||
            g_lift_sequence.state != LIFT_SEQ_STATE_IDLE)
        {
            g_lift_control_debug.test_start_result = 4U;
            return;
        }
        robot_motors_get_snapshot(&zero_motors);
        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            const dji_motor_feedback_t *motor = &zero_motors.lift.motor[ch];
            if (!dji_motor_feedback_is_online(motor, osKernelGetTickCount(),
                    pdMS_TO_TICKS(LIFT_CONTROL_COMMAND_TIMEOUT_MS)) ||
                motor->speed_rpm > 2 || motor->speed_rpm < -2)
            {
                g_lift_control_debug.test_start_result = 7U;
                return;
            }
            zero_raw[ch] = motor->total_ecd;
        }
        lift_top_set_zero_now(&lift_top, zero_raw);
        (void)memset(&lift_last_status, 0, sizeof(lift_last_status));
        lift_last_status.homed = true;
        lift_last_status.state = LIFT_TOP_STATE_READY;
    }
    if (lift_seq_is_active(&g_lift_sequence))
    {
        g_lift_control_debug.test_start_result = 3U;
        return;
    }
    /* A rejected PRECHECK has not issued a lifting command. Never reset a
     * motion/guard fault, or restart an active sequence, with this shortcut. */
    if (!was_pending && g_lift_sequence.state == LIFT_SEQ_STATE_FAULT &&
        g_lift_sequence.fault == LIFT_SAFETY_FAULT_LIMIT &&
        g_lift_sequence.fault_detail != NULL &&
        strcmp(g_lift_sequence.fault_detail,
               "axis already outside soft limits") == 0 &&
        lift_last_status.guard_state != LIFT_GUARD_STATE_FAULT)
    {
        g_lift_control_debug.sequence_reset = true;
        g_lift_task_command.reset_request = true;
        lift_test_start_pending = true;
        lift_stair_start_pending = stair_requested;
        g_lift_control_debug.test_start_result = 1U;
        return;
    }
    if (g_lift_sequence.state != LIFT_SEQ_STATE_IDLE ||
        lift_last_status.state != LIFT_TOP_STATE_READY ||
        lift_last_status.fault != LIFT_TOP_FAULT_NONE ||
        lift_last_status.guard_state == LIFT_GUARD_STATE_FAULT)
    {
        g_lift_control_debug.test_start_result = 4U;
        return;
    }
    g_lift_sequence.config.clearance_mm = height_mm;
    if (height_mm == 200 && g_lift_control_debug.test_extend_travel)
    {
        lift_control_debug_t travel_debug;
        lift_top_config_t travel_config;
        uint32_t ch;
        /* Provisional 200mm compensation, plus the latest ~10mm shortfall:
         * prior front/rear 825/900 counts/mm scaled by 1.05. Absolute values
         * prevent repeated expansion; custom calibration remains untouched. */
        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
            if (g_lift_control_debug.axis_counts_per_mm[ch] == 750.0F &&
                g_lift_control_debug.position_max_ecd[ch] == 150000)
            {
                const bool front = (ROBOT_LIFT_FRONT_MASK & (1U << ch)) != 0U;
                g_lift_control_debug.axis_counts_per_mm[ch] = front ? 866.25F : 945.0F;
                g_lift_control_debug.position_max_ecd[ch] = front ? 173250 : 189000;
            }
        travel_debug = g_lift_control_debug;
        lift_build_config(&travel_config, &travel_debug);
        lift_top_reload_control(&lift_top, &travel_config);
        lift_build_kinematics(&travel_debug);
    }
    g_lift_sequence.config.lift_test_only = !stair_requested;
    g_lift_sequence.config.stair_debug_run = stair_requested;
    g_lift_sequence.config.contact_mm = stair_requested ?
        g_lift_control_debug.stair_contact_mm : 8;
    g_lift_sequence.config.retracted_mm = stair_requested ?
        g_lift_control_debug.stair_retracted_mm : 2;
    if (stair_requested)
    {
        uint32_t ch;
        for (ch = 0U; ch < 3U; ch++)
        {
            stair_distance[ch] = g_lift_control_debug.stair_distance_mm[ch];
            stair_speed[ch] = g_lift_control_debug.stair_speed_mm_s[ch];
        }
        for (ch = 0U; ch < 4U; ch++)
            stair_direction[ch] = g_chassis_control_debug.motor_direction[ch];
        stair_counts_per_mm = 8192.0F * g_chassis_control_debug.rpm_per_mps / 60000.0F;
        stair_leg = -1;
        stair_progress = 0.0F;
        g_lift_control_debug.stair_drive_stop_reason = 0U;
        g_lift_control_debug.stair_retracted_mask = 0U;
        lift_stair_owned = true;
        chassis_control_stair_velocity(0.0F);
    }
    /* This machine's front=ch0/ch1, rear=ch2/ch3 is physically confirmed. */
    g_lift_control_debug.sequence_layout_confirmed = true;
    g_lift_control_debug.sequence_drive_done = false;
    g_lift_control_debug.sequence_confirm = false;
    g_lift_control_debug.sequence_release = false;
    g_lift_control_debug.sequence_reset = false;
    g_lift_control_debug.sequence_abort = false;
    g_lift_control_debug.sequence_start = true;
    g_lift_control_debug.test_pause = false;
    g_lift_control_debug.test_resume = false;
    g_lift_control_debug.test_lower = false;
    g_lift_control_debug.test_start_result = 1U;
}

static void lift_control_task(void *argument)
{
    robot_motor_state_t motors;
    lift_task_command_t command;
    lift_top_input_t input;
    lift_control_command_t control_command;
    lift_top_status_t top_status;
    TickType_t last_wake_tick;
    uint32_t previous_tick = 0U;

    (void)argument;
    last_wake_tick = xTaskGetTickCount();

    /*
     * 命令通道看门狗先跑一次，保证上电后主链一定拿得到所有权
     * （否则一个"上一轮没归还"的认领会把通道锁住到超时）。
     */
    lift_owner_set_timeout_ms(LIFT_OWNER_TIMEOUT_MS);

    for (;;)
    {
        const uint32_t now = osKernelGetTickCount();
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

        lift_owner_tick(now);

        /* 调试器改了参数：先撤销使能、让旧零点失效，再重建整条链。 */
        lift_apply_debug_parameters(true);
        /* 轻量命令：只重载增益 / 软零点（都不作废旧零点）。 */
        lift_handle_light_commands();
        lift_handle_test_start(bench_owns);

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
        input.control_dt_s = previous_tick == 0U ? lift_top.config.control.period_s :
            (float)(uint32_t)(now - previous_tick) * (float)portTICK_PERIOD_MS * 0.001F;
        previous_tick = now;
        g_lift_control_status.cycle_dt_s = input.control_dt_s;
        input.command_fresh = command.enabled &&
            (uint32_t)(now - command.command_update_tick) <=
                pdMS_TO_TICKS(LIFT_CONTROL_COMMAND_TIMEOUT_MS);
        input.homing_request = command.homing_request;
        input.homing_known_zero_confirmed_mask =
            command.rear_at_minimum_confirmed ? ROBOT_LIFT_REAR_MASK : 0U;
        input.homing_contact_confirmed_mask =
            (uint8_t)(command.front_contact_confirmed_mask & ROBOT_LIFT_FRONT_MASK);
        input.clear_homing_request = command.clear_homing_request;
        input.reset_request = command.reset_request;
        control_command.enabled = command.enabled;
        control_command.target_height_ecd = command.target_height_ecd;
        control_command.target_roll_ecd = command.target_roll_ecd;
        control_command.target_pitch_ecd = command.target_pitch_ecd;

        for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
        {
            input.position_ecd[ch] = motors.lift.motor[ch].total_ecd;
            input.speed_rpm[ch] = motors.lift.motor[ch].speed_rpm;
            input.online[ch] = dji_motor_feedback_is_online(
                &motors.lift.motor[ch], now,
                pdMS_TO_TICKS(LIFT_CONTROL_COMMAND_TIMEOUT_MS));
            all_online = all_online && input.online[ch];
        }

        /*
         * ---- 分阶段上台阶流程（lift_seq）----
         *
         * 顺序很关键：流程在【安全层之后、lift_top 之前】跑。
         *   · 在安全层之后：这样流程给出的是"已经过基础夹取的姿态"派生值；
         *   · 在 lift_top 之前：这样它这一拍产出的目标【当拍生效】，
         *     而不是慢一拍（调试时"按了确认要等一拍"最容易被当成卡住）。
         *
         * 流程只在【它自己 active】时接管命令；不 active 时手动命令照旧。
         */
        {
            lift_seq_input_t sin;
            lift_seq_status_t sst;
            const lift_seq_state_t stage_before = g_lift_sequence.state;
            const uint32_t stage_start_before = g_lift_sequence.step_start_ms;
            int32_t targets_before[LIFT_CHANNEL_COUNT];

            for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
                targets_before[ch] = g_lift_sequence.axis_target_mm[ch];

            (void)memset(&sin, 0, sizeof(sin));
            sin.now_ms = now;
            sin.homed = lift_last_status.homed;
            sin.calibrated = lift_kin_ok;
            sin.layout_confirmed = g_lift_control_debug.sequence_layout_confirmed;
            sin.guard_fault = (lift_last_status.guard_state ==
                               LIFT_GUARD_STATE_FAULT);
            sin.drive_done = lift_stair_drive(&motors, now) ||
                g_lift_control_debug.sequence_drive_done;
            g_lift_control_debug.sequence_drive_done = false;
            sin.contact_front = g_lift_control_debug.sequence_contact_front;
            sin.contact_rear = g_lift_control_debug.sequence_contact_rear;
            sin.sync_error_mm = g_lift_control_status.sync_error_mm;
            sin.deviation_max_mm = g_lift_control_status.deviation_max_mm;
            sin.tracking_deviation_valid = lift_top.config.control.axis_control_enabled;
            for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
            {
                float deviation = lift_last_status.control.axis_sync_error_mm[ch];
                if (deviation < 0.0F) deviation = -deviation;
                if (deviation > (float)sin.tracking_deviation_mm)
                    sin.tracking_deviation_mm = (int32_t)(deviation + 0.999F);
            }

            for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
            {
                float mm_s = 0.0F;

                sin.axis_online[ch] = input.online[ch];
                /*
                 * 限位区间取【当前生效的控制器配置】，不是 debug 结构 ——
                 * 参数被拒时两者会不一致，而真正在限制输出的是前者
                 * （见 lift_publish_status 里同一处的说明）。
                 */
                sin.limit_min_ecd[ch] =
                    lift_top.config.control.limit_height_min_ecd[ch];
                sin.limit_max_ecd[ch] =
                    lift_top.config.control.limit_height_max_ecd[ch];
                /* Permit startup from -1000 counts at a zero lower limit
                 * on all four axes. Normal output limits remain unchanged:
                 * below zero, only upward recovery is allowed. */
                if (sin.limit_min_ecd[ch] == 0)
                {
                    sin.contact_zero_recovery_ecd[ch] = 1000;
                }
                sin.limit_blocked[ch] = lift_last_status.limit_blocked[ch];
                sin.position_ecd[ch] = lift_top.config.axis_direction[ch] *
                    (input.position_ecd[ch] - lift_top.zero_ecd[ch]);
                sin.current[ch] = (int16_t)(lift_top.config.axis_direction[ch] *
                    motors.lift.motor[ch].given_current);
                if (!lift_kin_ecd_to_mm(&lift_kin, ch,
                                        sin.position_ecd[ch],
                                        &sin.position_mm[ch]))
                {
                    sin.position_mm[ch] = 0;
                }
                if (lift_kin_speed_to_mm_s(&lift_kin, ch,
                                           input.speed_rpm[ch], &mm_s))
                {
                    sin.speed_mm_s[ch] = (int32_t)mm_s;
                }
                else
                {
                    sin.speed_mm_s[ch] = 0;
                }
            }

            /*
             * 单周期脉冲：process 由流程的请求 API 置位，本任务消费掉。
             */
            taskENTER_CRITICAL();
            sin.start_request = g_lift_control_debug.sequence_start;
            sin.abort_request = g_lift_control_debug.sequence_abort;
            sin.confirm_request = g_lift_control_debug.sequence_confirm;
            sin.release_request = g_lift_control_debug.sequence_release;
            sin.reset_request = g_lift_control_debug.sequence_reset;
            sin.test_pause_request = g_lift_control_debug.test_pause;
            sin.test_resume_request = g_lift_control_debug.test_resume;
            sin.test_lower_request = g_lift_control_debug.test_lower;
            g_lift_control_debug.test_pause = false;
            g_lift_control_debug.test_resume = false;
            g_lift_control_debug.test_lower = false;
            g_lift_control_debug.sequence_start = false;
            g_lift_control_debug.sequence_abort = false;
            g_lift_control_debug.sequence_confirm = false;
            g_lift_control_debug.sequence_release = false;
            g_lift_control_debug.sequence_reset = false;
            taskEXIT_CRITICAL();

            lift_seq_update(&g_lift_sequence, &sin, &sst);
            if (lift_stair_owned && (sst.state != stage_before || bench_owns ||
                lift_last_status.fault != LIFT_TOP_FAULT_NONE))
                chassis_control_stair_velocity(0.0F);
            if (stage_before != LIFT_SEQ_STATE_TEST_PAUSED &&
                sst.state == LIFT_SEQ_STATE_TEST_PAUSED)
            {
                /* Remove the old advancing trajectory immediately, while
                 * retaining the speed PI's support integral/current limit. */
                lift_top.control.planner_speed_mm_s = 0.0F;
                for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
                {
                    lift_top.control.axis_reference[ch] = (float)sin.position_ecd[ch];
                    lift_top.control.axis_reference_speed[ch] = 0.0F;
                }
            }
            g_lift_seq_status.test_stall_axis = g_lift_sequence.test_stall_axis;

            if (stage_before != LIFT_SEQ_STATE_FAULT &&
                sst.state == LIFT_SEQ_STATE_FAULT)
            {
                taskENTER_CRITICAL();
                g_lift_fault_snapshot.valid = false;
                g_lift_fault_snapshot.count++;
                g_lift_fault_snapshot.tick_ms = now;
                g_lift_fault_snapshot.stage_elapsed_ms =
                    (uint32_t)(now - stage_start_before);
                g_lift_fault_snapshot.stage = (uint8_t)stage_before;
                g_lift_fault_snapshot.stage_name =
                    lift_seq_state_name(stage_before);
                g_lift_fault_snapshot.fault = (uint8_t)sst.fault;
                g_lift_fault_snapshot.detail = sst.fault_detail;
                g_lift_fault_snapshot.fault_axis = sst.fault_axis;
                g_lift_fault_snapshot.guard_state =
                    (uint8_t)lift_last_status.guard_state;
                g_lift_fault_snapshot.guard_fault =
                    (uint8_t)lift_last_status.guard_fault;
                g_lift_fault_snapshot.top_fault = (uint8_t)lift_last_status.fault;
                g_lift_fault_snapshot.command_fresh =
                    g_lift_control_status.command_fresh;
                g_lift_fault_snapshot.sync_error_mm = sin.sync_error_mm;
                g_lift_fault_snapshot.deviation_max_mm = sin.deviation_max_mm;
                for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
                {
                    g_lift_fault_snapshot.position_ecd[ch] = sin.position_ecd[ch];
                    g_lift_fault_snapshot.target_ecd[ch] =
                        g_lift_control_status.axis_target_ecd[ch];
                    g_lift_fault_snapshot.target_mm[ch] = targets_before[ch];
                    g_lift_fault_snapshot.applied_current[ch] =
                        g_lift_control_status.axis_final_current[ch];
                    g_lift_fault_snapshot.feedback_current[ch] = sin.current[ch];
                    g_lift_fault_snapshot.speed_rpm[ch] = input.speed_rpm[ch];
                    g_lift_fault_snapshot.online[ch] = input.online[ch];
                }
                g_lift_fault_snapshot.valid = true;
                taskEXIT_CRITICAL();
            }

            g_lift_seq_status.seq_state = (uint8_t)sst.state;
            g_lift_seq_status.seq_fault = (uint8_t)sst.fault;
            g_lift_seq_status.seq_stop_action = (uint8_t)sst.stop_action;
            g_lift_seq_status.seq_support = (uint8_t)sst.support;
            g_lift_seq_status.seq_active = lift_seq_is_active(&g_lift_sequence);
            g_lift_seq_status.seq_waiting_confirm = sst.waiting_confirm;
            g_lift_seq_status.seq_drive_request = sst.drive_request;
            g_lift_seq_status.seq_released = sst.released;
            g_lift_seq_status.seq_confirm_count = sst.confirm_count;
            g_lift_seq_status.seq_elapsed_ms = sst.elapsed_ms;
            g_lift_seq_status.seq_hold_mm = sst.hold_mm;
            g_lift_seq_status.seq_operator_action =
                (sst.operator_action != NULL) ? sst.operator_action : "";
            g_lift_seq_status.seq_fault_detail =
                (sst.fault_detail != NULL) ? sst.fault_detail : "";
            g_lift_seq_status.seq_fault_axis = sst.fault_axis;
            for (ch = 0U; ch < LIFT_CHANNEL_COUNT; ch++)
            {
                g_lift_seq_status.seq_axis_target_mm[ch] =
                    sst.axis_target_mm[ch];
            }

            /* Active valid commands refresh their own watchdog. PRECHECK
             * holds the measured pose; FAULT reaches the top stop executor.
             * IDLE/COMPLETE return to the existing manual watchdog policy. */
            if (lift_seq_is_active(&g_lift_sequence) &&
                lift_configuration_applied && all_online)
            {
                (void)lift_route_sequence(&sst, &input, &control_command);
            }

            /*
             * 流程报故障时的停机动作。CUT 是"没有策略信息"时的默认，
             * 与修之前的硬编码行为一致，所以不会被这一条悄悄改掉。
             */
            input.fault_stop_action =
                (sst.state == LIFT_SEQ_STATE_FAULT)
                    ? sst.stop_action
                    : LIFT_STOP_CUT;
            input.external_fault_request = (sst.state == LIFT_SEQ_STATE_FAULT);
            input.stop_target_ecd[0] = g_lift_control_debug.descent_target_ecd[0];
            input.stop_target_ecd[1] = g_lift_control_debug.descent_target_ecd[1];
            input.stop_target_ecd[2] = g_lift_control_debug.descent_target_ecd[2];
            input.stop_target_ecd[3] = g_lift_control_debug.descent_target_ecd[3];
        }

        if (bench_owns)
        {
            /* No unused controller is allowed to integrate behind an output gate. */
            input.command_fresh = true;
            input.homing_request = false;
            input.external_fault_request = false;
            control_command.enabled = false;
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
                    if (lift_stair_owned && g_lift_sequence.config.stair_debug_run)
                    {
                        const lift_seq_state_t stage = g_lift_sequence.state;
                        const bool retracting_front = stage >= LIFT_SEQ_STATE_RETRACT_FRONT &&
                            stage <= LIFT_SEQ_STATE_COMPLETE &&
                            (ROBOT_LIFT_FRONT_MASK & (1U << ch)) != 0U;
                        const bool retracting_rear = stage >= LIFT_SEQ_STATE_RETRACT_REAR &&
                            stage <= LIFT_SEQ_STATE_COMPLETE &&
                            (ROBOT_LIFT_REAR_MASK & (1U << ch)) != 0U;
                        int32_t mm;
                        const int32_t pose = lift_top.config.axis_direction[ch] *
                            (input.position_ecd[ch] - lift_top.zero_ecd[ch]);
                        if ((retracting_front || retracting_rear) &&
                            lift_kin_ecd_to_mm(&lift_kin, ch, pose, &mm) &&
                            mm >= -1 && mm <= g_lift_sequence.config.retracted_mm + 1)
                            g_lift_control_debug.stair_retracted_mask |= (uint8_t)(1U << ch);
                        if ((g_lift_control_debug.stair_retracted_mask & (1U << ch)) != 0U)
                            applied[ch] = 0;
                    }
                }
            }

            /*
             * ★ 命令通道所有权（2026-10-11）。
             *
             * 主链是本组的【默认所有者】。写之前先认领、写完立刻归还 ——
             * "认领 → 写 → 归还"这个三拍是必须的：
             *   · 不认领就写 → 台架正在点动时会被主链覆盖
             *   · 认领后不归还 → 台架永远抢不到通道（本用例在主机测试里
             *     就是这么失败的，正好证明令牌真的在起作用）
             *
             * 被别的模块占用（台架点动 / CAN 测试）时【不写】，而不是"写 0"：
             * 原来门禁是写 0，而占用方每 5 ms 也在写这一组 —— 两个同周期
             * 同优先级的任务抢同一个命令缓存，发送任务（2 ms 一帧）取到的
             * 是"谁最后写"，于是点动电流被主链的 0 斩波：实际送进 C620 的
             * 电流带一个未知占空比，K、启动门槛、保持电流全部失真。
             *
             * 但"参数没生效 / 掉线"两种门禁仍然主动写 0 —— 那是保护动作，
             * 必须真的把电流压掉。
             */
            if (gate != (uint8_t)LIFT_OUTPUT_GATE_JOG &&
                gate != (uint8_t)LIFT_OUTPUT_GATE_CAN_TEST)
            {
                if (lift_owner_claim(LIFT_OWNER_CONTROL, now))
                {
                    robot_lift_set_currents(
                        applied[0], applied[1], applied[2], applied[3]);
                    lift_owner_release(LIFT_OWNER_CONTROL);
                }
                else
                {
                    /*
                     * 通道被别人占着：本周期不下发。
                     * 这不是故障（占用来得及归还），但要能看见 ——
                     * 否则"算出来有电流、实际没出力"又变成要猜的事。
                     */
                    gate = (uint8_t)LIFT_OUTPUT_GATE_JOG;
                    (void)memset(applied, 0, sizeof(applied));
                    lift_control_reset(&lift_top.control);
                }
            }

            command.target_height_ecd = control_command.target_height_ecd;
            command.target_roll_ecd = control_command.target_roll_ecd;
            command.target_pitch_ecd = control_command.target_pitch_ecd;
            lift_publish_status(
                &top_status, &motors, &command, applied, gate, now,
                lift_configuration_applied && all_online &&
                input.command_fresh);
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
    (void)memset(&g_lift_seq_status, 0, sizeof(g_lift_seq_status));
    (void)memset(&g_lift_fault_snapshot, 0, sizeof(g_lift_fault_snapshot));

    lift_set_default_debug();
    lift_test_start_pending = false;
    lift_stair_start_pending = false;
    lift_stair_owned = false;
    stair_leg = -1;
    g_lift_control_debug.stair_drive_leg = -1;
    g_lift_control_debug.stair_contact_mm = 8;
    g_lift_control_debug.stair_retracted_mm = 2;
    for (uint32_t ch = 0U; ch < 3U; ch++)
        g_lift_control_debug.stair_speed_mm_s[ch] = 50;
    g_lift_control_debug.test_height_mm = 50;
    g_lift_control_debug.test_auto_zero = true;
    g_lift_control_debug.test_extend_travel = true;
    g_lift_control_debug.apply_gains = 0U;
    g_lift_control_debug.set_zero_now = 0U;
    g_lift_control_debug.pid_manual_override = false;

    /*
     * 上台阶流程的默认配置。
     *
     * ⚠️ `sequence_layout_confirmed` 出厂是 false —— 这是**有意的**：
     *    前后轮组到底是哪两个通道还没有人工确认（资料互相矛盾），
     *    收错一组就是把承重支撑收掉。流程会在 PRECHECK 拒绝启动，
     *    并把原因写进 `g_lift_seq_status.seq_fault_detail`。
     */
    lift_seq_config_default(&lift_seq_cfg);
    lift_seq_init(&g_lift_sequence, &lift_seq_cfg);
    (void)memset(&g_lift_seq_status, 0, sizeof(g_lift_seq_status));
    g_lift_seq_status.seq_operator_action = "";
    g_lift_seq_status.seq_fault_detail = "";
    g_lift_seq_status.seq_fault_axis = -1;

    /* 命令通道所有权：上电即由主链接管。 */
    lift_owner_reset();
    lift_owner_set_timeout_ms(LIFT_OWNER_TIMEOUT_MS);
    (void)lift_owner_claim(LIFT_OWNER_CONTROL, osKernelGetTickCount());
    lift_owner_release(LIFT_OWNER_CONTROL);

    (void)memset(&lift_last_status, 0, sizeof(lift_last_status));
    lift_configuration_applied = false;
    lift_configuration_rejected = false;
    lift_safety_limited = false;

    /*
     * 上电就把标定参数与所选增益装进控制链：第一个控制周期之前必须
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
