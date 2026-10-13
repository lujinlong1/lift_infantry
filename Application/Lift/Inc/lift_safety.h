#ifndef LIFT_SAFETY_H
#define LIFT_SAFETY_H

/*
 * 抬升安全策略与命令通道所有权（对应议题 LIFT-006 / LIFT-010）。
 *
 * 这个模块单独存在，是因为有两件事【必须只有一个地方说了算】：
 *
 * ── 一、抬升电流通道的所有权 ──────────────────────────────────────────
 *
 * CAN1 `0x200` 的 0..3 槽就是抬升四轴的电流。往这个缓存里写的模块有三个：
 *   - `lift_control_task`（主链，5 ms / prio 24）
 *   - `lift_jog_task`（台架诊断，5 ms / prio 24）
 *   - `robot_motor_commands.c` 里的 DJI CAN 测试覆写（发送层整帧替换）
 *
 * 前两个【同周期、同优先级】，谁后写谁生效。原来的约定是靠
 * `g_lift_jog_status.running` 互相谦让，但台架模块在空闲分支里每 5 ms
 * 写一次 `robot_lift_set_currents(0,0,0,0)`，而 `running` 恰好在写 0 的
 * 同一行被置 false —— 于是"台架空闲"时主链算出来的电流被每 5 ms 砍成 0，
 * 真实进 C620 的值是一个取决于调度相位的占空比。
 *
 * 这条不能靠"两边各自判断"来修（那正是出错的原因），必须有一个显式的
 * 所有权令牌：**同一时刻只有一个写入者**。
 *
 * 默认所有者在【主链】：抬升本来就是它的职责；诊断只在真正跑起来时抢。
 *
 * ── 二、故障时的停机策略 ──────────────────────────────────────────────
 *
 * 本机构【不自锁】（ADR-006）。"出故障就把四个电流清零"在底盘【有机械
 * 支撑】时是对的（立刻停），但在底盘【悬空】时等于把车摔下来。
 * 所以停机动作必须分三种，而且必须是显式选择，不能靠默认：
 *
 *   HOLD  保持：继续给保持前馈把车托在原地，同时锁存故障、拒绝新命令
 *   LOWER 受控下降：主动往停放点走，走到才停
 *   CUT   断电：直接清零 —— **只允许在"已经确认有机械支撑"时使用**
 *
 * `lift_safety_policy_for()` 是唯一的决策点。它有意做成纯函数：
 * 给它一组明确的前提，它给出停机动作。前提里没有任何一个是"猜"出来的
 * （见 lift_safety_context_t 的注释）。
 *
 * 设计约束：纯逻辑，无 RTOS/HAL 依赖，无动态分配，可在主机上直接单测。
 */

#include <stdbool.h>
#include <stdint.h>

#ifndef LIFT_CHANNEL_COUNT
#define LIFT_CHANNEL_COUNT 4U
#endif

/* ------------------------------------------------------------------ */
/*  命令通道所有权                                                      */
/* ------------------------------------------------------------------ */

typedef enum
{
    LIFT_OWNER_NONE = 0,   /* 无人认领（上电瞬间；主链会在第一个周期认领） */
    LIFT_OWNER_CONTROL,    /* 主控制链 */
    LIFT_OWNER_JOG,        /* 台架诊断点动 */
    LIFT_OWNER_CAN_TEST    /* DJI CAN 测试（发送层整帧替换，见 robot_motor_commands.c） */
} lift_owner_t;

/*
 * 认领通道。返回 false 表示【已经被别人占用】，调用方必须放弃本次下发。
 * 同一个所有者重复认领是允许的（返回 true，并刷新认领时刻）。
 */
bool lift_owner_claim(lift_owner_t owner, uint32_t now_ms);

/*
 * 归还通道。只有当前所有者本人才还得了 —— 别人调用是空操作，
 * 避免"误释放把别人正在驱动的通道抢走"。
 */
void lift_owner_release(lift_owner_t owner);

/* 当前所有者。 */
lift_owner_t lift_owner_current(void);

/*
 * 某人是否【有权写】这一组电流。
 * @param want 调用方自己的身份
 * @return true 表示"现在可以写"（自己是所有者，或通道无人认领）
 */
bool lift_owner_may_write(lift_owner_t want);

/* 认领后多久没刷新就视为失效（看门狗）。0 表示不设失效。 */
void lift_owner_set_timeout_ms(uint32_t timeout_ms);
uint32_t lift_owner_timeout_ms(void);

/*
 * 推进看门狗：超过 timeout 没被刷新的认领自动失效，避免某个任务死掉之后
 * 通道被永久占住（那会让另外两个写入者都不敢写，等于抬升永久失效）。
 */
void lift_owner_tick(uint32_t now_ms);

/* 全部清零（仅供初始化与主机测试使用）。 */
void lift_owner_reset(void);

/* ------------------------------------------------------------------ */
/*  故障停机策略                                                        */
/* ------------------------------------------------------------------ */

typedef enum
{
    LIFT_STOP_HOLD = 0,   /* 保持：继续给保持前馈，故障锁存但不掉车 */
    LIFT_STOP_LOWER,      /* 受控下降到停放点 */
    LIFT_STOP_CUT         /* 直接断电：仅在已确认有机械支撑时使用 */
} lift_stop_action_t;

typedef enum
{
    LIFT_SUPPORT_UNKNOWN = 0,   /* 不知道车有没有被托住 —— 最保守地按"悬空"处理 */
    LIFT_SUPPORT_GROUND,        /* 车落在地面/最低端（齿条贴在死点上） */
    LIFT_SUPPORT_MECHANICAL,    /* 人装了机械支撑 / 垫块，已人工确认 */
    LIFT_SUPPORT_AIRBORNE       /* 明确知道机构把整车顶在空中 */
} lift_support_t;

typedef enum
{
    LIFT_SAFETY_FAULT_NONE = 0,
    LIFT_SAFETY_FAULT_OFFLINE,          /* 反馈丢失 */
    LIFT_SAFETY_FAULT_LIMIT,            /* 超软限位 / 触硬限位 */
    LIFT_SAFETY_FAULT_SYNC,             /* 四轴高度差或姿态误差超限 */
    LIFT_SAFETY_FAULT_OVERCURRENT,      /* 电流异常 / 长时间堵转 */
    LIFT_SAFETY_FAULT_TIMEOUT,          /* 动作超时 */
    LIFT_SAFETY_FAULT_UNSUPPORTED,      /* 该动作的前置支撑条件没确认 */
    LIFT_SAFETY_FAULT_ABORTED,          /* 收到中止命令 */
    LIFT_SAFETY_FAULT_NOT_CALIBRATED,   /* 没标定就要求走绝对位置 */
    LIFT_SAFETY_FAULT_COUNT
} lift_safety_fault_t;

typedef struct
{
    lift_safety_fault_t fault;
    /* 四个前提，缺一不可，全都要由调用方【明确】给出 */
    lift_support_t support;      /* 车现在靠什么托着 */
    bool homed;                  /* 四轴都有可信零点 */
    bool calibrated;             /* 运动学标定完成（没有标定不得走绝对位置） */
    bool moving;                 /* 是否正在做抬升动作（决定 HOLD 还是 LOWER 更合适） */
} lift_safety_context_t;

/*
 * @brief 唯一的停机策略决策点。
 *
 * 规则（顺序即优先级）：
 *   1. 已确认有机械支撑 或 车已落地  → CUT（没有任何跌落风险，立刻停）
 *   2. 悬空 / 未知，但没找零        → CUT（没有位置基准，LOWER 也无从谈起；
 *                                          此时唯一能做的是不再出力）
 *   3. 悬空 / 未知，且已找零        → 正在动作过 → LOWER；否则 → HOLD
 *      理由：正在动的时候，受控下降到停放点比"原地保持一个已经出故障的
 *            系统"更可预测；没在动的时候保持最省事也最安全。
 *
 * ⚠️ UNKNOWN 按"悬空"处理：宁可多花电流托着，也不要赌一把把车摔了。
 */
lift_stop_action_t lift_safety_policy_for(
    const lift_safety_context_t *context);

/* ------------------------------------------------------------------ */
/*  停机动作的实际执行                                                  */
/* ------------------------------------------------------------------ */

typedef struct
{
    /*
     * HOLD / LOWER 的出力（单轴电流，逻辑量纲，正 = 抬升 = 齿条伸出）。
     * 填 0 时 HOLD 与 LOWER 会【退化成 CUT】—— 这是刻意的：
     * 标定不出保持电流时，唯一已知安全的动作就是断电，
     * 而不是拿一个猜出来的数去托车。
     */
    int32_t hold_current;
    /*
     * 上限（C620 量纲）。执行时会夹到 ±min(hold_current, current_limit)。
     */
    int32_t current_limit;
    /* 逻辑是否容许多轴在停机动作中保持不同电流（当前恒为"是"）。 */
    bool    per_axis_allowed;
} lift_stop_config_t;

typedef struct
{
    /* 已找零逻辑位置（计数）。 */
    int32_t position_ecd[LIFT_CHANNEL_COUNT];
    /* 停机动作的目标位置（已找零坐标，计数）：LOWER 往这里走。 */
    int32_t park_ecd[LIFT_CHANNEL_COUNT];
    /*
     * HOLD 的保持目标（已找零坐标，计数）：一般给"当前实际位置"，
     * 用来判断"托住了没有"。
     */
    int32_t hold_ecd[LIFT_CHANNEL_COUNT];
} lift_stop_state_t;

/*
 * @brief 执行停机动作。
 *
 * @param action  lift_safety_policy_for() 的结果
 * @param config  出力配置（见 lift_stop_config_t）
 * @param state   位置信息
 * @param out_current  输出：四轴最终电流（覆盖上游算出来的值）
 * @param out_active   输出：本周期是否仍在执行停机动作
 *                     （HOLD 永远 active；LOWER 到位后 active=false；
 *                       CUT 恒为 false，因为断电是即时完成的）
 *
 * ★ 这个函数的存在理由：在本模块出现之前，"故障 = 四轴电流清零"是硬编码
 *   在 lift_top.c 里的。本机构【不自锁】，清零等于把悬在空中的车摔下来。
 *   现在策略由 lift_safety_policy_for() 决定，执行由这里决定，
 *   两边都是纯函数、都能在主机上单测 —— 安全逻辑不再依赖"某处记得写对"。
 *
 * ⚠️ HOLD / LOWER 在 hold_current <= 0 时【退化成 CUT】并置 out_active=false。
 *    调用方必须把这件事报给操作手（"保持电流未标定，故障即断电"）。
 */
void lift_stop_apply(
    lift_stop_action_t action,
    const lift_stop_config_t *config,
    const lift_stop_state_t *state,
    int32_t out_current[LIFT_CHANNEL_COUNT],
    bool *out_active);

/*
 * @brief 停机动作是否真的能托住车（= HOLD/LOWER 不会退化成 CUT）。
 *
 * 这是"能力缺口"的显式表达：返回 false 说明标定还没做，
 * 当前状态下任何故障都会导致断电掉车。Watch / 自检脚本应该报出它。
 */
bool lift_stop_can_hold(const lift_stop_config_t *config);

/* 故障码的短名（Watch / 日志用，静态字符串，不分配）。 */
const char *lift_safety_fault_name(lift_safety_fault_t fault);
const char *lift_stop_action_name(lift_stop_action_t action);
const char *lift_support_name(lift_support_t support);

#endif
