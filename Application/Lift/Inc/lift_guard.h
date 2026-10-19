#ifndef LIFT_GUARD_H
#define LIFT_GUARD_H

/*
 * 抬升丝杆安全闸（对应议题 LIFT-006 受控缓降、LIFT-009 软件限位）。
 *
 * 架构位置：控制层提交「想要多少电流」，本模块是下发到 robot_lift_set_currents()
 * 之前的最后一道闸，只负责把意图变成安全的电流。禁止控制层绕过本模块直接下发。
 *
 * 本模块同时承担四件事，顺序即优先级：
 *   1. 离线保护   —— 任一轴反馈丢失，全组立即归零并锁存故障
 *   2. 找零门禁   —— 未找零时任何运动命令都不得通过
 *   3. 受控缓降   —— 上层命令超时时【主动往下走】，绝不简单归零（ADR-006）
 *   4. 行程软限位 —— 已到界且仍向界外推的电流被拦为 0（**兜底**，见下）
 *
 * ⚠️ 关于第 4 条的分工：行程限位的【正常工作】在 lift_control 的模态限幅里，
 *    不在本模块。四根丝杆顶的是同一块刚性车身，任一轴电流都含"帮别的轴
 *    调平"的分量；逐轴清零若当常规手段，会把触界的那根从协同里踢出去，
 *    车身再也拉不平。本模块的逐轴清零只用于止损（上游限幅失效时）。
 *    因此正常运行时 limit_blocked 应始终为 false。
 *
 * 设计约束：纯逻辑，不调用时间函数（now_ms 注入），不依赖 FreeRTOS/HAL，
 *           不做动态分配，因此可在主机上直接单测。
 */

#include <stdbool.h>
#include <stdint.h>

#ifndef LIFT_CHANNEL_COUNT
#define LIFT_CHANNEL_COUNT 4U
#endif

typedef enum
{
    LIFT_GUARD_STATE_DISARMED = 0,   /* 未找零，只允许找零动作 */
    LIFT_GUARD_STATE_READY,          /* 已找零且命令新鲜，接受运动命令 */
    LIFT_GUARD_STATE_DESCENDING,     /* 命令超时，正在受控缓降 */
    LIFT_GUARD_STATE_PARKED,         /* 命令超时且已落到底，无输出 */
    LIFT_GUARD_STATE_FAULT           /* 故障锁存，输出全零 */
} lift_guard_state_t;

typedef enum
{
    LIFT_GUARD_FAULT_NONE = 0,
    LIFT_GUARD_FAULT_OFFLINE,            /* 反馈丢失 */
    LIFT_GUARD_FAULT_DESCENT_STALL,      /* 该下降却不动：卡死或机械抱死 */
    LIFT_GUARD_FAULT_DESCENT_TIMEOUT     /* 缓降超时仍未到底 */
} lift_guard_fault_t;

typedef struct
{
    /* 软限位（已找零坐标系，编码器计数） */
    int32_t position_min_ecd[LIFT_CHANNEL_COUNT];
    int32_t position_max_ecd[LIFT_CHANNEL_COUNT];
    /* 方向：+1 表示正电流使 position_ecd 增大，-1 相反 */
    int8_t  direction[LIFT_CHANNEL_COUNT];
    /* 缓降电流（带符号）。方向由 direction 与符号共同决定，应指向 position_min。 */
    int16_t descent_current[LIFT_CHANNEL_COUNT];
    /*
     * 缓降的停止位置（已找零坐标，单轴计数）。
     *
     * ⚠️ 必须【单独】给，不能拿 position_min_ecd 当目标（2026-10-11 修正）：
     *    position_min_ecd 是"行程下限"，它可以（也曾经）配在机械死点之下，
     *    于是每次命令超时的受控缓降都会一路顶进死点 → 500 ms 后
     *    DESCENT_STALL 锁存 → 输出全零 → 不自锁机构掉车。
     *    本机构的零点是"齿条顶死的机械最低点"，缓降目标是零点上方一点点
     *    （LIFT_TRAVEL_PARK_ECD），不是行程下限。
     */
    int32_t descent_target_ecd[LIFT_CHANNEL_COUNT];
    /* 本模块硬限幅，兜住上层任何越界请求 */
    int16_t current_limit;
    /* 缓降判定：|speed| <= 该值且持续 descent_stall_ms 视为降不动 */
    int16_t  descent_stall_speed_rpm;
    uint32_t descent_stall_ms;
    /* 缓降总超时 */
    uint32_t descent_timeout_ms;
} lift_guard_config_t;

typedef struct
{
    int32_t position_ecd[LIFT_CHANNEL_COUNT];   /* 已找零坐标 */
    int16_t speed_rpm[LIFT_CHANNEL_COUNT];
    int16_t requested_current[LIFT_CHANNEL_COUNT];
    bool    online[LIFT_CHANNEL_COUNT];
    bool    homed;                /* 全部通道已找零 */
    bool    command_fresh;        /* 上层运动命令是否在超时窗口内 */
    bool    reset_request;        /* 上升沿清除锁存故障 */
    uint32_t now_ms;
} lift_guard_input_t;

typedef struct
{
    lift_guard_state_t state;
    lift_guard_fault_t fault;
    int16_t output_current[LIFT_CHANNEL_COUNT];
    bool    limit_blocked[LIFT_CHANNEL_COUNT];  /* 该轴因软限位被拦 */
    bool    at_target[LIFT_CHANNEL_COUNT];      /* 缓降目标已到达 */
    int8_t fault_axis; /* diagnostic only: -1 means group-wide */
    uint32_t descent_elapsed_ms;
} lift_guard_output_t;

typedef struct
{
    lift_guard_config_t config;
    lift_guard_state_t  state;
    lift_guard_fault_t  fault;
    int8_t fault_axis; /* diagnostic latch */
    uint32_t descent_start_ms;
    uint32_t stall_start_ms[LIFT_CHANNEL_COUNT];
    bool     stall_active[LIFT_CHANNEL_COUNT];
    bool     reset_last;
} lift_guard_t;

/* 初始化并按硬限幅修正配置。config 为 NULL 时不修改实例。 */
void lift_guard_init(lift_guard_t *self, const lift_guard_config_t *config);

/* 推进一个控制周期。in/out 任一为 NULL 时直接返回。 */
void lift_guard_update(
    lift_guard_t *self,
    const lift_guard_input_t *in,
    lift_guard_output_t *out);

/* 故障是否已锁存（须显式 reset 才清除）。 */
bool lift_guard_has_fault(const lift_guard_t *self);

#endif
