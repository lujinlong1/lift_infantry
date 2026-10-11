#ifndef LIFT_CONTROL_TASK_H
#define LIFT_CONTROL_TASK_H

#include "lift_top.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * 抬升控制任务：固件里唯一驱动抬升模块链的地方。
 *
 * 分层（见 讨论池/抬升模块架构.md）：
 *
 *   遥控（remote_lift） / 将来 lift_seq
 *        │  写 g_lift_task_command
 *        ▼
 *   本任务（5 ms）
 *        │  快照反馈 → lift_top_update() → robot_lift_set_currents()
 *        ▼
 *   lift_top ─► lift_control ─► lift_sync ─► lift_guard ─► CAN1 0x200
 *
 * 对外只暴露三样东西：
 *   1. g_lift_task_command：命令面（其它任务/调试器写，本任务读）
 *   2. g_lift_control_debug：标定参数（改完置 apply=1 才生效）
 *   3. g_lift_control_status：只读状态，给 Keil Watch 与遥测用
 */

/*
 * 运行时命令面。所有 request 位都是【单周期脉冲】：本任务消费后清零，
 * 调用方不需要（也不应该）自己清。
 *
 * 注意 enabled 的语义：它必须被【周期刷新】。设备层对抬升组也有
 * "命令超时归零"（100 ms），但那是兜底；抬升的正确超时行为是
 * lift_guard 的**受控缓降**（ADR-006），不是断力——本机构不自锁。
 */
typedef struct
{
    volatile bool enabled;
    volatile int32_t target_height_ecd;
    volatile int32_t target_roll_ecd;
    volatile int32_t target_pitch_ecd;
    volatile bool homing_request;
    volatile bool clear_homing_request;
    volatile bool reset_request;
    /* 每次命令 API 调用都会刷新；用于受控超时缓降判定。 */
    volatile uint32_t command_update_tick;
} lift_task_command_t;

/*
 * 标定/调试参数。只有 apply 被置 1 时才拷贝进纯逻辑控制器。
 * 位置与电流都是【逻辑坐标】：单轴的原始极性由 axis_direction 处理。
 *
 * 这些值全部来自"待实机标定项"（LIFT-005/007/009/010），默认值是
 * 占位值 + 保守电流，不是调好的参数。
 */
typedef struct
{
    /* 完整生效：重建整条链，并【作废旧零点】（必须重新找零）。 */
    volatile bool apply;
    /*
     * 只重载增益：把 position_kp 与 speed_kp 这一组装进运行中的链，
     * 不动零点、不动 homed —— 调 PID 专用，改完立刻生效、不用重新找零。
     */
    volatile uint8_t apply_gains;
    /*
     * 软零点：把四轴当前位置当作零点、直接进入 READY。
     * 台架调 PID 时跳过真实找零用。⚠️ 无机械基准，限位/堵转判定意义打折，
     * 只能架空调试用；正式跑之前必须重新真实找零。
     */
    volatile uint8_t set_zero_now;
    /*
     * 手动增益开关：
     *   0 = 增益由机械常数自动派生（推荐，防"两套增益"混乱）
     *   1 = 用手填的 position_kp/ki/kd 与 speed_kp/ki/kd（调试用）
     * 置 1 后 lift_derive_gains() 不再覆盖这些字段。
     */
    volatile bool pid_manual_override;
    volatile int8_t axis_direction[4];

    volatile int16_t homing_seek_current;
    volatile int16_t homing_backoff_current;
    volatile int16_t homing_preposition_current;
    volatile int16_t homing_current_limit;
    volatile int16_t homing_stall_speed_rpm;
    volatile int32_t homing_preposition_ecd;
    volatile uint32_t homing_preposition_timeout_ms;
    volatile int32_t homing_seek_min_travel_ecd;
    volatile uint32_t homing_stall_hold_ms;
    volatile int32_t homing_backoff_ecd;
    volatile uint32_t homing_seek_timeout_ms;
    volatile uint32_t homing_backoff_timeout_ms;

    volatile int32_t position_min_ecd[4];
    volatile int32_t position_max_ecd[4];
    volatile int16_t descent_current[4];
    volatile int16_t guard_current_limit;
    volatile int16_t guard_stall_speed_rpm;
    volatile uint32_t guard_stall_ms;
    volatile uint32_t guard_timeout_ms;

    volatile float position_kp;
    volatile float position_ki;
    volatile float position_kd;
    volatile float position_integral_limit;
    volatile float position_output_limit;
    volatile float speed_kp;
    volatile float speed_ki;
    volatile float speed_kd;
    volatile float speed_integral_limit;
    volatile float speed_output_limit;
    volatile int32_t ecd_per_rev;
    volatile float gear_ratio;
    volatile bool speed_is_rotor;
    volatile int32_t max_target_step_ecd;
    volatile float period_s;
    volatile int32_t settle_tolerance_ecd;
    volatile int32_t limit_margin_ecd;
    volatile float gravity_feedforward_current;
    volatile float actuator_gain_ecd_s_per_current;
} lift_control_debug_t;

/* 平铺的 volatile 状态，专供 Keil Watch 与调试记录使用。 */

/*
 * 本周期是谁把真实下发压成 0 的。顺序即优先级（与 lift_control_task.c 一致）。
 *
 * 为什么需要它：output_current 是"模块链算出来的电流"，
 * applied_current 才是"真的进了 CAN1 0x200 的电流"。两者不一致时，
 * 没有这个字段就只能靠猜——而"显示有电流、实际没出力"在调试期最容易误导人。
 */
typedef enum
{
    LIFT_OUTPUT_GATE_NONE = 0,   /* 没有门禁，applied = output */
    LIFT_OUTPUT_GATE_NO_CONFIG,  /* 标定参数没生效 */
    LIFT_OUTPUT_GATE_OFFLINE,    /* 有抬升轴掉线（四轴必须全在线） */
    LIFT_OUTPUT_GATE_JOG,        /* 诊断点动模块在占用 */
    LIFT_OUTPUT_GATE_CAN_TEST    /* CAN 点动测试在占用 */
} lift_output_gate_t;

typedef struct
{
    volatile bool running;
    volatile bool configuration_applied;
    volatile bool homed;
    volatile bool at_target;
    volatile bool safety_limited;         /* 目标被压回安全起点（未找零） */
    volatile bool output_gated;           /* 本周期真实下发被门禁压成 0 */
    /*
     * output_gate 取值（见 lift_output_gate_t）：
     *   0 无门禁 / 1 参数未生效 / 2 抬升轴掉线 / 3 诊断点动占用 / 4 CAN 点动测试占用
     * 有它才能一眼分清"控制器算出来是 0"和"算出来有值但被闸掉了"。
     */
    volatile uint8_t output_gate;
    volatile uint8_t state;
    volatile uint8_t fault;
    volatile uint8_t homing_state[4];
    volatile uint8_t homing_fault[4];
    volatile uint8_t guard_state;
    volatile uint8_t guard_fault;
    volatile bool online[4];
    volatile bool command_fresh;
    volatile bool configuration_rejected;
    volatile bool limit_blocked[4];
    volatile int32_t raw_position_ecd[4];
    volatile int32_t zero_ecd[4];
    volatile int32_t homed_position_ecd[4];
    volatile int16_t speed_rpm[4];
    volatile uint8_t temperature[4];
    /*
     * 模块链算出的电流（命令层）。想确认"到底发出去多少"要看
     * applied_current[]，两者只有在 output_gated = 0 时才相同。
     */
    volatile int16_t output_current[4];
    /* 真实交给设备层、最终进 CAN1 0x200 的电流，带符号。 */
    volatile int16_t applied_current[4];
    volatile int32_t actual_height;
    volatile int32_t actual_roll;
    volatile int32_t actual_pitch;
    volatile int32_t actual_warp;
    volatile int32_t actual_speed_height;
    volatile int32_t actual_speed_roll;
    volatile int32_t actual_speed_pitch;
    volatile int32_t actual_speed_warp;
    volatile int32_t target_height_ecd;
    volatile int32_t target_roll_ecd;
    volatile int32_t target_pitch_ecd;
    /*
     * 行程软限位的实际区间（已找零坐标，含余量）。发布出来是给命令来源
     * （遥控映射）用的：来源必须知道"能推到哪"，而不是自己再猜一套数字。
     */
    volatile int32_t limit_height_min_ecd[4];
    volatile int32_t limit_height_max_ecd[4];
    volatile uint32_t command_age_ms;
    volatile uint32_t update_count;
    volatile uint32_t parameter_apply_count;
    volatile uint32_t safety_refresh_count;   /* 目标被强制压回的周期数 */
    /*
     * 由机械常数派生出来的增益。**它们不是手填的**，改 K 或周期就会整体变。
     * 上板第一件事就是看这三项：speed_loop_kp 若已经贴到 speed_loop_kp_max，
     * 说明 K 太大或周期太长，必须先解决，否则"一通电就冲、然后在目标两侧撞限幅"。
     * 用 float 存是为了在 Watch 里直接看到数值。
     */
    volatile float speed_loop_kp;
    volatile float speed_loop_kp_max;
    volatile float position_loop_kp;
} lift_task_status_t;

extern lift_task_command_t g_lift_task_command;
extern lift_control_debug_t g_lift_control_debug;
extern lift_task_status_t g_lift_control_status;

/* 初始化命令面与标定参数的默认值（不产生任何电流）。 */
void lift_control_task_init(void);

/* 用静态内存创建 5 ms 抬升控制任务。 */
bool lift_control_task_create(void);

/* ---- 命令 API：任何来源（遥控/时序/调试器）都走这几个入口 ---- */

void lift_control_enable(bool enable);
void lift_control_set_target(
    int32_t height_ecd,
    int32_t roll_ecd,
    int32_t pitch_ecd);
void lift_control_request_homing(void);
void lift_control_request_clear_homing(void);
void lift_control_request_reset(void);

#endif
