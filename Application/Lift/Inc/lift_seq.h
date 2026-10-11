#ifndef LIFT_SEQ_H
#define LIFT_SEQ_H

/*
 * 抬升动作时序执行器（对应议题 LIFT-011）。
 *
 * 为什么只做执行器、不做具体台阶动作：
 *   四个丝杆怎么配合上台阶，取决于行程、轮径、台阶高度、重心位置——
 *   这些机械参数目前还没有。把猜测写死成代码，比不写更糟。
 *   所以本模块只负责【执行】一张数据表：
 *     什么时候该等、等多久算超时、到位后要稳多久、哪一步要人工确认。
 *   真正的台阶动作表是一组 lift_seq_step_t 常量，等机械确认后填写。
 *
 * 支持的步骤类型：
 *   MOVE  移动到指定模态目标，等到位并稳定 settle_ms
 *   WAIT  进入时锁定当前姿态并保持，只等外部确认（操作手按一下再走）
 *   STOP  结束序列，把控制权交还手动
 *
 * 三种"不往下走"的原因，分别报不同的错，便于现场判断：
 *   NOT_HOMED    还没找零就请求跑序列
 *   STEP_TIMEOUT 某一步超时（机构没到位 / 卡住 / 操作手忘了确认）
 *   ABORTED      被主动中止
 *
 * ⚠️ 集成契约（很重要）：
 *   本模块只在 RUNNING / WAIT_CONFIRM 期间把 command_valid 置真。
 *   序列结束（DONE）后 command_valid 变为假，调用方必须【立刻】提供新鲜的手动命令。
 *   否则安全闸会在超时后走受控缓降（ADR-006），整车会自己降下来。
 *   手动控制的正确形态是"增量式绝对高度目标 + 每周期刷新"，而不是松开就不发。
 *
 * 设计约束：纯逻辑，时间由 now_ms 注入，不依赖 FreeRTOS/HAL，不做动态分配。
 * 步骤表由调用方持有（建议 static const），本模块只保存指针。
 */

#include "lift_control.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    LIFT_SEQ_STEP_MOVE = 0,   /* 移动到指定模态目标 */
    LIFT_SEQ_STEP_WAIT,       /* 锁定当前姿态，等确认 */
    LIFT_SEQ_STEP_STOP        /* 结束序列 */
} lift_seq_step_kind_t;

typedef struct
{
    lift_seq_step_kind_t kind;
    int32_t  height_ecd;       /* 仅 MOVE 使用 */
    int32_t  roll_ecd;         /* 仅 MOVE 使用 */
    int32_t  pitch_ecd;        /* 仅 MOVE 使用 */
    uint32_t timeout_ms;       /* 0 = 不限时 */
    uint32_t settle_ms;        /* 仅 MOVE：到位后需保持多久才算完成 */
    bool     require_confirm;  /* 仅 MOVE：到位后是否还需人工确认才推进 */
} lift_seq_step_t;

typedef struct
{
    const lift_seq_step_t *steps;
    uint32_t count;
} lift_seq_program_t;

typedef enum
{
    LIFT_SEQ_STATE_IDLE = 0,
    LIFT_SEQ_STATE_RUNNING,
    LIFT_SEQ_STATE_WAIT_CONFIRM,
    LIFT_SEQ_STATE_DONE,
    LIFT_SEQ_STATE_FAULT
} lift_seq_state_t;

typedef enum
{
    LIFT_SEQ_FAULT_NONE = 0,
    LIFT_SEQ_FAULT_NOT_HOMED,
    LIFT_SEQ_FAULT_STEP_TIMEOUT,
    LIFT_SEQ_FAULT_ABORTED
} lift_seq_fault_t;

typedef struct
{
    bool    start_request;        /* 上升沿启动 */
    bool    abort_request;        /* 上升沿中止 */
    bool    confirm_request;      /* 上升沿确认推进 */
    bool    homed;                /* 来自 lift_top */
    bool    at_target;            /* 来自 lift_top / lift_control 的到位标志 */
    int32_t actual_height_ecd;    /* WAIT 步骤用来锁定当前姿态 */
    int32_t actual_roll_ecd;
    int32_t actual_pitch_ecd;
    uint32_t now_ms;
} lift_seq_input_t;

typedef struct
{
    lift_seq_state_t state;
    lift_seq_fault_t fault;
    uint32_t step_index;
    uint32_t step_count;
    uint32_t step_elapsed_ms;
    lift_control_command_t command;   /* 直接交给 lift_top */
    bool     command_valid;           /* 真 = 调用方须让 lift_top 认为命令新鲜 */
} lift_seq_status_t;

typedef struct
{
    lift_seq_program_t program;
    lift_seq_state_t state;
    lift_seq_fault_t fault;
    uint32_t step_index;
    uint32_t step_start_ms;
    uint32_t settle_start_ms;
    bool     settle_active;
    bool     start_last;
    bool     abort_last;
    bool     confirm_last;
    int32_t  hold_height_ecd;
    int32_t  hold_roll_ecd;
    int32_t  hold_pitch_ecd;
} lift_seq_t;

/* 初始化。program->steps 必须由调用方长期持有；program 为 NULL 时不修改实例。 */
void lift_seq_init(lift_seq_t *self, const lift_seq_program_t *program);

/* 推进一个控制周期。in/status 任一为 NULL 时直接返回。 */
void lift_seq_update(
    lift_seq_t *self,
    const lift_seq_input_t *in,
    lift_seq_status_t *status);

/* 序列是否正在控制抬升（调用方据此决定命令来源）。 */
bool lift_seq_is_active(const lift_seq_t *self);

/* 清故障并回到 IDLE（由调用方在解除顶层故障后调用）。 */
void lift_seq_reset(lift_seq_t *self);

#endif
