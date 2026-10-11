#ifndef LIFT_HOMING_H
#define LIFT_HOMING_H

/*
 * 抬升丝杆上电堵转找零状态机（对应议题 LIFT-005、决策 ADR-007）。
 *
 * 设计约束：
 *   1. 纯逻辑模块：不调用任何时间函数，now_ms 由调用方注入；
 *      不依赖 FreeRTOS、不依赖 HAL、不做动态分配。因此可在主机上直接单测。
 *   2. 单通道：一个实例只管一根丝杆。四根丝杆同时找零由上层调度器组合。
 *   3. 失败即停：任何异常都进入 FAULT 且输出电流为 0，绝不继续硬顶。
 *
 * 流程：
 *   IDLE --start--> [PREPOSITION --走够前置行程-->] SEEK --确认堵转--> BACKOFF
 *                       |                             |                  |
 *                       +-------- 异常 ---------------+------------------+
 *                                                                        v
 *                                                                     DONE
 *   任一异常 --> FAULT（电流归零，须 reset）
 *
 * PREPOSITION 是可选的（preposition_ecd = 0 时跳过），存在的理由是：
 * 本机构的停车位置正好是死点，不先离开死点就永远满足不了堵转判定的
 * 最小行程条件。详见 lift_homing_config_t 里 preposition_ecd 的注释。
 *
 * 判定“到达机械死点”的三个条件缺一不可：
 *   ① 已走出 seek_min_travel_ecd 以上行程（防止刚启动就误判）
 *   ② |speed_rpm| <= stall_speed_rpm
 *   ③ 上述状态连续保持 stall_hold_ms
 */

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    LIFT_HOMING_STATE_IDLE = 0,
    LIFT_HOMING_STATE_PREPOSITION,   /* 先离开死点，见下面 preposition_ecd 的说明 */
    LIFT_HOMING_STATE_SEEK,
    LIFT_HOMING_STATE_BACKOFF,
    LIFT_HOMING_STATE_DONE,
    LIFT_HOMING_STATE_FAULT
} lift_homing_state_t;

typedef enum
{
    LIFT_HOMING_FAULT_NONE = 0,
    LIFT_HOMING_FAULT_OFFLINE,           /* 找零途中反馈丢失 */
    LIFT_HOMING_FAULT_PREPOSITION_TIMEOUT, /* 前置段走不到位（顶住/卡住/电流太小） */
    LIFT_HOMING_FAULT_SEEK_TIMEOUT,      /* 超时仍未碰到死点（行程不够/被卡住） */
    LIFT_HOMING_FAULT_BACKOFF_TIMEOUT,   /* 碰到死点却退不出来（顶死/卡死） */
    LIFT_HOMING_FAULT_ABORTED            /* 被上层主动中止 */
} lift_homing_fault_t;

typedef struct
{
    /* 顶向死点的小电流，符号决定找零方向。必须是小值。 */
    int16_t  seek_current;
    /* 离开死点的小电流，通常与 seek_current 反号。 */
    int16_t  backoff_current;
    /*
     * 前置离开死点的电流。**必须与 seek_current 反号**，否则会把机构
     * 直接顶在死点上。仅当 preposition_ecd > 0 时使用。
     */
    int16_t  preposition_current;
    /* 本模块自身的硬限幅，防止配置写错导致大电流顶死。 */
    int16_t  current_limit;
    /* |speed_rpm| <= 该值 视为“不动”。 */
    int16_t  stall_speed_rpm;
    /*
     * ---- 为什么需要"先离开死点"这一段 ----------------------------------
     *
     * 堵转判定有个前置条件：必须先走够 seek_min_travel_ecd 计数，才允许
     * 开始判"堵转"。这是为了防止一上电就误判。
     *
     * 但本机构的停车位置**正好就是死点**（车落地时齿条完全缩回、顶在硬
     * 死点上）。此时找零：机构已经贴在死点上推不动 → 永远走不够最小行程
     * → 只能等超时，报 SEEK_TIMEOUT。
     *
     * 而且不能指望"重力把机构拉离死点"：车头那两根偏轻，可能根本缩不回来。
     *
     * 所以找零必须【自己保证行程】：先主动往反方向走 preposition_ecd，
     * 再回头向死点走。这样最小行程这条前置条件在任何起始位置都成立，
     * 也不再依赖重力。
     *
     * 取 0 = 不做前置段（旧行为，供主机测试与"已知离死点很远"的场景）。
     */
    int32_t  preposition_ecd;
    /* 判定堵转前必须先走够的行程（编码器计数），防止启动瞬间误判。 */
    int32_t  seek_min_travel_ecd;
    /* “不动”需连续保持多久才确认为到达死点。 */
    uint32_t stall_hold_ms;
    /* 退出死点的回退行程（取绝对值比较）。 */
    int32_t  backoff_ecd;
    /* 前置段的超时：走不到 preposition_ecd 就报 PREPOSITION_TIMEOUT。 */
    uint32_t preposition_timeout_ms;
    uint32_t seek_timeout_ms;
    uint32_t backoff_timeout_ms;
} lift_homing_config_t;

typedef struct
{
    int32_t  position_ecd;    /* 上电起累计的编码器位置 */
    int16_t  speed_rpm;       /* 反馈转速 */
    bool     online;          /* 反馈是否在线 */
    bool     start_request;   /* 上升沿启动一次找零 */
    bool     abort_request;   /* 上升沿中止当前找零 */
    uint32_t now_ms;          /* 由调用方注入的时间戳 */
} lift_homing_input_t;

typedef struct
{
    lift_homing_state_t  state;
    lift_homing_fault_t  fault;
    int16_t  commanded_current;   /* 本模块希望下发的电流；非工作状态恒为 0 */
    int32_t  zero_ecd;            /* 死点处的 position_ecd；仅 DONE 时可信 */
    int32_t  travel_ecd;          /* 当前阶段已走行程（带符号） */
    uint32_t stall_ms;            /* 当前连续“不动”时长 */
    uint32_t elapsed_ms;          /* 当前阶段已耗时；IDLE/DONE/FAULT 为 0 */
} lift_homing_output_t;

typedef struct
{
    lift_homing_config_t config;
    lift_homing_state_t  state;
    lift_homing_fault_t  fault;
    int32_t  start_position_ecd;
    int32_t  zero_ecd;
    int32_t  backoff_start_ecd;
    uint32_t state_start_ms;
    uint32_t stall_start_ms;
    bool     stall_active;
    bool     start_last;
    bool     abort_last;
} lift_homing_t;

/* 初始化实例并校验配置；config 为 NULL 时不做任何修改。 */
void lift_homing_init(lift_homing_t *self, const lift_homing_config_t *config);

/* 推进一个控制周期。in/out 任一为 NULL 时直接返回，不修改状态。 */
void lift_homing_update(
    lift_homing_t *self,
    const lift_homing_input_t *in,
    lift_homing_output_t *out);

/*
 * 强制把某轴标记为"已找零"，并把给定位置当作零点（**软零点**）。
 *
 * 用途：台架上调 PID 时不想先跑真实找零 —— 直接"把现在的位置当零点"，
 * 闭环就能跑起来。调用方负责把 raw/logical 折算好。
 *
 * ⚠️ 这只是让闭环能跑，**它没有机械基准**：
 *    零点随机构当前位置漂，所以行程限位、堵转判定的意义都打了折扣。
 *    只允许在【架空、有人看着】的台架调试阶段用。
 */
void lift_homing_force_done(lift_homing_t *self, int32_t logical_position_ecd);

/* 只有 DONE 才返回 true，表示 zero_ecd 可信。 */
bool lift_homing_is_homed(const lift_homing_t *self);

#endif
