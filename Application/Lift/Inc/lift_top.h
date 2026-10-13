#ifndef LIFT_TOP_H
#define LIFT_TOP_H

/*
 * 抬升总编排（编排 lift_homing / lift_control / lift_sync / lift_guard）。
 *
 * 为什么需要它：下面四个模块各自都验证过了，但**没有东西持有状态**——
 * "四轴都找零了吗""每轴的零点偏移是多少""这一周期该走找零支路还是控制支路"。
 * 本模块就是那个持有者，也是固件任务唯一需要调用的入口。
 *
 * 两个互斥支路：
 *
 *   未找零：  前轴伸出寻接触（逐轴确认）+ 已确认最低点的后轴静止置零
 *            --> 直接下发（不经 guard，因为还没有完整位置基准）
 *   已找零：  lift_control -> lift_sync -> lift_guard --> 下发
 *
 * 关键安全约定：
 *   1. 四个通道必须【全部】建立零点才算 homed。后轴仅在当次请求明确
 *      确认机械最低点、在线且静止时置零；前轴须确认接触承重点并停稳。
 *   2. 重新找零必须是两步：先 clear_homing 使 homed 失效，再 homing_request。
 *      禁止在已找零状态下直接重新找零——那会让机构在不受限位保护的情况下下行。
 *   3. 顶层故障锁存，只有 reset_request 上升沿能清。
 *
 * 设计约束：纯逻辑，时间由 now_ms 注入，不依赖 FreeRTOS/HAL，不做动态分配。
 */

#include "lift_control.h"
#include "lift_guard.h"
#include "lift_homing.h"
#include "lift_safety.h"
#include "lift_sync.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    LIFT_TOP_STATE_IDLE = 0,   /* 未找零，也没有正在找零 */
    LIFT_TOP_STATE_HOMING,     /* 正在找零 */
    LIFT_TOP_STATE_READY,      /* 已找零，接受运动命令 */
    LIFT_TOP_STATE_FAULT       /* 故障锁存，输出全零 */
} lift_top_state_t;

typedef enum
{
    LIFT_TOP_FAULT_NONE = 0,
    LIFT_TOP_FAULT_HOMING_FAILED,   /* 至少一个通道找零失败 */
    LIFT_TOP_FAULT_GUARD,           /* 安全闸报故（离线 / 缓降异常） */
    LIFT_TOP_FAULT_SEQUENCE,
    LIFT_TOP_FAULT_ZERO_UNCONFIRMED, /* 已知最低点轴尚未确认，拒绝找零 */
    LIFT_TOP_FAULT_ZERO_MOVING       /* 已知最低点轴仍在运动，拒绝置零 */
} lift_top_fault_t;

typedef struct
{
    lift_homing_config_t homing;
    lift_guard_config_t guard;
    lift_control_config_t control;
    /*
     * 原始编码器/电流到逻辑抬升坐标的方向。
     * +1 = 原始正方向就是逻辑上升方向；-1 = 该轴的编码器和电流都反向。
     * 前后齿条电机方向不一致时只改这里，不改模态权重和控制算法。
     */
    int8_t axis_direction[LIFT_CHANNEL_COUNT];
    /* 故障停机动作的出力配置（见 lift_stop_config_t）。 */
    lift_stop_config_t stop;
    /* These axes establish zero without motion, only with an explicit
     * same-request mechanical-datum confirmation. Zero preserves legacy homing. */
    uint8_t homing_known_zero_mask;
    /* Contact datum axes seek positive extension, never retract/back off.
     * Motor stall only pauses seeking; physical contact requires confirmation. */
    uint8_t homing_contact_mask;
    int16_t homing_contact_current; /* positive logical extension current */
    uint32_t homing_contact_confirm_timeout_ms;
} lift_top_config_t;

typedef struct
{
    int32_t position_ecd[LIFT_CHANNEL_COUNT];   /* 原始 total_ecd */
    int16_t speed_rpm[LIFT_CHANNEL_COUNT];
    bool    online[LIFT_CHANNEL_COUNT];
    bool    homing_request;          /* 上升沿：开始一次找零（仅未 homed 时有效） */
    bool    clear_homing_request;    /* 上升沿：使已找零结果失效 */
    bool    reset_request;           /* 上升沿：清顶层与安全闸的锁存故障 */
    bool    command_fresh;           /* 上层运动命令是否在超时窗口内 */
    /*
     * ★ 故障停机策略。本机构【不自锁】，所以"出故障"不等于"断电"：
     *   底盘悬空时断电 = 掉车。调用方（lift_control_task）按
     *   `lift_safety_policy_for()` 的结果填这里。
     *
     * 约定：FAULT 之外的状态忽略本字段。填 CUT 是最保守的选择
     * （也是"没有策略信息时"的默认行为）。
     */
    lift_stop_action_t fault_stop_action;
    /*
     * HOLD / LOWER 的逐轴目标（已找零逻辑计数）。
     *   HOLD  → 保持目标（一般给当前实际位置）
     *   LOWER → 停放点（往这里走）
     * 调用方填；本模块只做透传与"到位"判断。
     */
    int32_t stop_target_ecd[LIFT_CHANNEL_COUNT];
    uint32_t now_ms;
    float control_dt_s; /* measured task interval; zero uses configured period */
    bool external_fault_request; /* sequence fault must reach the stop executor */
    uint8_t homing_known_zero_confirmed_mask;
    uint8_t homing_contact_confirmed_mask;
} lift_top_input_t;

typedef struct
{
    lift_top_state_t state;
    lift_top_fault_t fault;
    bool    homed;
    int32_t zero_ecd[LIFT_CHANNEL_COUNT];           /* 原始 total_ecd 死点位置 */
    int32_t homed_position_ecd[LIFT_CHANNEL_COUNT]; /* 已找零坐标 */
    int16_t output_current[LIFT_CHANNEL_COUNT];     /* 最终下发电流 */
    lift_sync_modes_t actual_modes;                 /* 实际模态位置（ecd） */
    lift_sync_modes_t actual_modes_speed;           /* 实际模态速度（ecd/s） */
    lift_sync_modes_t ramped_target_ecd;            /* 斜坡后的目标模态位置 */
    lift_sync_modes_t target_speed_ecd_s;           /* 外环输出：目标模态速度 */
    float position_loop_height;                     /* 外环原始输出（float，供标定） */
    float position_loop_roll;
    float position_loop_pitch;
    float speed_loop_height;                        /* 内环原始输出（float，供标定） */
    float speed_loop_roll;
    float speed_loop_pitch;
    lift_homing_state_t homing_state[LIFT_CHANNEL_COUNT];
    lift_homing_fault_t homing_fault[LIFT_CHANNEL_COUNT];
    lift_guard_state_t  guard_state;
    lift_guard_fault_t  guard_fault;
    /*
     * ★ 故障态下【真实下发】的停机动作，以及它是否还在起作用。
     *
     * 这两个字段是"故障 = 断电掉车"这个隐患的可见化：
     *   stop_action == CUT   → 四轴电流为 0（唯一安全的情形是车有支撑）
     *   stop_action == HOLD  → 四轴在给保持电流托住车
     *   stop_action == LOWER → 四轴在往停放点走
     *   stop_active          → 动作还没完成
     *   stop_capable == false → 保持电流没标定，HOLD/LOWER 会退化成 CUT
     *                          （**这是能力缺口，必须让操作手看到**）
     */
    lift_stop_action_t stop_action;
    bool    stop_active;
    bool    stop_capable;
    /*
     * 限位可见性：limit_blocked[] 正常应【始终为 false】。
     * 它为 true 说明上游模态限幅没兜住、guard 在用逐轴清零止损，
     * 应该去查 lift_control 的 limit_* 配置，而不是去调 guard。
     */
    bool    limit_blocked[LIFT_CHANNEL_COUNT];
    /* 控制层完整状态，供 Watch 与标定使用（含 limit_scale / pid 各分量）。 */
    lift_control_status_t control;
    int32_t control_effort_height;   /* lift_control 交给 lift_sync 的模态控制量 */
    bool    at_target;
} lift_top_status_t;

typedef struct
{
    lift_top_config_t config;
    lift_homing_t homing[LIFT_CHANNEL_COUNT];
    lift_guard_t guard;
    lift_control_t control;
    int32_t zero_ecd[LIFT_CHANNEL_COUNT];
    bool    homed;
    lift_top_state_t state;
    lift_top_fault_t fault;
    bool    homing_last;
    bool    clear_last;
    bool    reset_last;
    bool    motion_command_seen; /* watchdog arms after a fresh enabled command */
} lift_top_t;

/* 初始化全部子模块。config 为 NULL 时不修改实例。 */
void lift_top_init(lift_top_t *self, const lift_top_config_t *config);

/*
 * 推进一个控制周期。这是固件任务唯一需要调用的抬升入口。
 * 任一指针为 NULL 时直接返回，不修改状态。
 */
void lift_top_update(
    lift_top_t *self,
    const lift_top_input_t *in,
    const lift_control_command_t *command,
    lift_top_status_t *status);

/*
 * 【软零点】把四轴当前位置当作零点，直接进入 READY。
 *
 * 用途：台架调 PID 时跳过真实找零（省时、也避免找零时车要落地这一环）。
 *
 * ⚠️ 它不是真实找零：零点没有机械基准，机构一挪零点就"错"了。
 *    所以行程限位与堵转判定的意义都打折。
 *    **只在架空、有人看着的台架调试阶段用**；正式跑之前必须重新真实找零。
 *
 * @param raw_position 四轴原始编码器位置（与 in->position_ecd 同域）
 */
void lift_top_set_zero_now(
    lift_top_t *self,
    const int32_t raw_position[LIFT_CHANNEL_COUNT]);

/*
 * 【只重载控制参数】把新的 control 配置装进运行中的链，**不动零点、不动 homed**。
 *
 * 用途：调 PID 时改增益不能把零点作废（否则每调一次都要重新找零，没法调）。
 * 它只重建控制子模块（清 PID 状态与斜坡）和安全闸，保留 self->zero_ecd / homed。
 */
void lift_top_reload_control(
    lift_top_t *self,
    const lift_top_config_t *config);

/* 是否已可用（已找零且无锁存故障）。 */
bool lift_top_is_operational(const lift_top_t *self);

#endif
