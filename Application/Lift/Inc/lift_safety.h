#ifndef LIFT_SAFETY_H
#define LIFT_SAFETY_H

/* Lift ownership and legacy electrical fault-stop requests.
 * HOLD = positive constant logical current; LOWER = negative constant current
 * until all configured park targets are reached; CUT = zero current command.
 * None proves support, controlled speed, actual standstill, or fall prevention.
 * Non-self-locking loads require independent mechanical support/brake/catch.
 * The policy's support input may be an operator declaration or stage estimate;
 * it must never be interpreted as measured physical support.
 * Pure logic, no allocation. Target ownership transactions use short locks.
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
/* Explicit atomic handover, never release another identity: JOG may replace
 * CONTROL; CAN_TEST may replace CONTROL/JOG. CONTROL may not preempt.
 * previous returns the displaced/current identity, including on rejection. */
bool lift_owner_claim_override(lift_owner_t owner, uint32_t now_ms,
                                lift_owner_t *previous);

/*
 * 归还通道。只有当前所有者本人才还得了 —— 别人调用是空操作，
 * 避免"误释放把别人正在驱动的通道抢走"。
 */
void lift_owner_release(lift_owner_t owner);

/* Atomic observation, not authorization for a later cache write. */
lift_owner_t lift_owner_current(void);

/*
 * 某人是否【有权写】这一组电流。
 * @param want 调用方自己的身份
 * @return true 表示"现在可以写"（自己是所有者，或通道无人认领）
 */
bool lift_owner_may_write(lift_owner_t want);

/* Lease timeout in ms, 0 disables expiry. Caller times/intervals must be less
 * than 2^31ms apart; older sampled times cannot expire a freshly renewed lease. */
void lift_owner_set_timeout_ms(uint32_t timeout_ms);
uint32_t lift_owner_timeout_ms(void);

/*
 * 推进看门狗：超过 timeout 没被刷新的认领自动失效，避免某个任务死掉之后
 * 通道被永久占住（那会让另外两个写入者都不敢写，等于抬升永久失效）。
 */
void lift_owner_tick(uint32_t now_ms);

/* Fault boundary for bench owners. The control task is the sole setter:
 * latch on fault; clear only after explicit supported, disabled recovery.
 * Setting true revokes a bench lease atomically; it never changes CONTROL. */
void lift_owner_inhibit_tests(bool inhibited);
bool lift_owner_tests_inhibited(void);

/* 全部清零（仅供初始化与主机测试使用）。 */
void lift_owner_reset(void);

/* ------------------------------------------------------------------ */
/*  故障停机策略                                                        */
/* ------------------------------------------------------------------ */

/* Recovery requirements are operator/software preconditions, NOT proof of
 * physical support. Confirmation must follow installation of mechanical support. */
typedef enum {
    LIFT_RECOVERY_SUPPORT = 1U, LIFT_RECOVERY_DISABLE = 2U,
    LIFT_RECOVERY_OFFLINE = 4U, LIFT_RECOVERY_MOVING = 8U,
    LIFT_RECOVERY_SEQUENCE = 16U
} lift_recovery_block_t;

typedef enum
{
    LIFT_STOP_HOLD = 0,   /* positive constant logical current request */
    LIFT_STOP_LOWER,      /* negative constant logical current until park predicate */
    LIFT_STOP_CUT         /* zero current command; no standstill guarantee */
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

/* Legacy request selection, retained without changing motion policy:
 * declared ground/mechanical support -> CUT; no datum -> CUT;
 * otherwise moving -> LOWER, stationary -> HOLD. These are electrical requests,
 * not validated safety measures. The executor may reject them and output zero. */
lift_stop_action_t lift_safety_policy_for(
    const lift_safety_context_t *context);

/* ------------------------------------------------------------------ */
/*  停机动作的实际执行                                                  */
/* ------------------------------------------------------------------ */

typedef struct
{
    /* Legacy logical current magnitude. <=0 or current_limit<=0 produces zero.
     * A positive value is configuration only, never proof of holding capacity. */
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
     * 保留接口；当前执行器不闭环使用该字段。
     */
    int32_t hold_ecd[LIFT_CHANNEL_COUNT];
} lift_stop_state_t;

/* Overwrite all four logical current requests. Invalid action/config/state
 * produces zero when an output buffer exists. out_active means nonzero requests
 * remain; it says nothing about CAN delivery, motion, or physical support.
 * LOWER has no velocity feedback here; its name does not guarantee slow descent. */
void lift_stop_apply(
    lift_stop_action_t action,
    const lift_stop_config_t *config,
    const lift_stop_state_t *state,
    int32_t out_current[LIFT_CHANNEL_COUNT],
    bool *out_active);

/* Compatibility API: true only means a positive magnitude/ceiling is configured.
 * It does NOT verify force, support or a safe stop. */
bool lift_stop_can_hold(const lift_stop_config_t *config);

/* 故障码的短名（Watch / 日志用，静态字符串，不分配）。 */
const char *lift_safety_fault_name(lift_safety_fault_t fault);
const char *lift_stop_action_name(lift_stop_action_t action);
const char *lift_support_name(lift_support_t support);

#endif
