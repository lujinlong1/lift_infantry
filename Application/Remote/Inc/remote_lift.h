#ifndef REMOTE_LIFT_H
#define REMOTE_LIFT_H

/*
 * 遥控 → 抬升命令链（只做"把操作手意图翻译成抬升命令"，不做控制）。
 *
 * 架构位置（见 讨论池/抬升模块架构.md 第 2 节）：
 *
 *   DBUS 摇杆 ─► remote_lift_map() ─► lift_control_set_target()/enable()
 *                                        │
 *                                        ▼  g_lift_task_command
 *                                  lift_control_task（5 ms）
 *                                        │
 *                                        ▼
 *                                  lift_top ─► … ─► robot_lift_set_currents()
 *
 * 为什么不把这段逻辑写进 lift_control_task：
 *   抬升模块是纯逻辑（时间注入、无 RTOS 依赖），命令来源可以是遥控、也可以是
 *   将来的 lift_seq 动作表。把"操作手怎么给"放进控制任务，会让两者绑死。
 *
 * ── 命令形态：增量式绝对高度目标 + 每周期刷新（这是硬要求）──────────
 *
 * 摇杆给的是【增量】，每周期在内部累积成一个【绝对目标】，然后每周期
 * 调用一次 lift_control_set_target() 刷新时间戳。
 *
 * 绝不能做成"松杆就不发命令"：抬升安全闸把命令超时解释为
 * **受控缓降**（ADR-006），一松杆整车就会自己降下来。
 *
 * ── 安全约定 ────────────────────────────────────────────────────────
 *
 *   1. 未找零时本模块【不发任何运动命令】，只报 need_homing。
 *      没有位置基准就没有安全边界（ADR-007）。
 *   2. 首次使能（或重新使能）时目标吸附到【当前位置】，不是从 0 或旧目标
 *      跳过去，避免上电/接管瞬间冲击机构。
 *   3. 目标被夹在 lift_control_status 报出的行程区间内（已含"下降之后自动
 *      回正"的姿态余量），摇杆推到底也不会把目标写到行程之外。
 *   4. 车身歪超过 level_tolerance_ecd（例如落地时一个轮子先着地）时，
 *      本模块让高度目标【下降】，优先把车放平，而不是继续往上顶。
 *   5. 连接超时（丢遥控）时立即撤销使能；随后由 lift_guard 走受控缓降。
 *
 * ── 操作手动作（左拨杆三段，右拨杆管"抬升是否可用"）──────────────
 *
 *   右拨杆上/中档 + 左摇杆上下 = 抬升 / 下降（增量，松杆保持当前高度）
 *   左拨杆下档（保持）        = 抬升作废 + 清故障 + "清零点"（两步的第一步）
 *   左拨杆中档（保持）        = 请求找零
 *   左拨杆上档                = 云台/发射（原有功能），抬升保持高度
 *
 * 找零会让机构一路下行到底部堵转，必须由人明确要求，且必须在"清零点"之后
 * 才允许——这就是 ADR-007 的两步重新找零。因此**每次上电后（以及每次想让
 * 已找零结果失效时）都必须先在下档保持够久，再切到中档保持够久**，本模块
 * 才发出找零请求。上电时"已清零点"标志是假的，所以不可能一步就启动找零。
 * 之所以用"保持"而不是"点一下"，是为了让一次误碰不可能启动找零。
 */

#include <stdbool.h>
#include <stdint.h>

/* 左拨杆三段（与 dbus_remote.h 的 DBUS_SWITCH_* 数值一致，这里给语义名）。 */
typedef enum
{
    REMOTE_LIFT_LEFT_DOWN = 0,   /* 抬升作废 + 清零点 / 清故障 */
    REMOTE_LIFT_LEFT_MID,        /* 请求找零（需先清零点） */
    REMOTE_LIFT_LEFT_UP          /* 正常操作（云台/发射），抬升保持高度 */
} remote_lift_left_t;

typedef struct
{
    volatile bool enabled;

    /* 摇杆映射 */
    volatile int16_t stick_channel;     /* DBUS channel[0..4]，默认 1（左摇杆上下） */
    volatile int16_t stick_deadband;
    volatile float   stick_scale;       /* (计数/周期) 每 摇杆单位 */
    volatile float   stick_filter_tau_s;
    volatile int8_t  stick_direction;   /* +1 / -1 */

    /* 每周期目标最大变化（计数），防止机械冲击；<=0 表示不额外限制 */
    volatile int32_t max_target_step_ecd;

    /* 姿态保持上限：|横滚| 或 |俯仰| 超过它就让抬升往下降（落地歪车时回正） */
    volatile int32_t level_tolerance_ecd;

    /*
     * 命令刷新超时（ms）。必须【小于】抬升护栏的命令超时（100 ms），
     * 否则护栏会先判定超时而缓降，本模块再刷也来不及。默认 40 ms。
     */
    volatile uint32_t command_timeout_ms;

    /* 找零时序：在下档保持 homing_clear_hold_ms，再在中档保持 homing_arm_ms */
    volatile uint32_t homing_clear_hold_ms;
    volatile uint32_t homing_arm_ms;
} remote_lift_debug_t;

typedef struct
{
    volatile bool     enabled;            /* 本模块是否在给抬升发命令 */
    volatile bool     need_homing;        /* 已使能但还没找零 */
    volatile bool     homing_armed;       /* 本次上电已在下档清过零点 */
    volatile int32_t  target_height_ecd;  /* 当前绝对高度目标（已找零坐标） */
    volatile int32_t  actual_height_ecd;  /* 最近一次读到的实际高度 */
    volatile float    stick_filtered;     /* 滤波后的摇杆量，供 Watch 看抖动 */
    volatile uint32_t command_age_ms;
    volatile uint32_t enable_count;       /* 使能次数：每次接管都会吸附当前位置 */
    volatile uint32_t refresh_count;
    volatile uint32_t block_count;        /* 未找零 / 姿态 / 超时被拦的次数 */
    volatile uint32_t no_homing_count;    /* 未找零就推摇杆的次数（提示操作手） */
    volatile uint32_t level_block_count;  /* 因车身不水平而降高度的次数 */
} remote_lift_status_t;

extern remote_lift_debug_t g_remote_lift_debug;
extern remote_lift_status_t g_remote_lift_status;

/* 填入默认映射与初始状态。不产生任何输出。 */
void remote_lift_init(void);

/*
 * 推进一个遥控周期（由 remote_task 每 10 ms 调用一次）。
 *
 * channel       5 个通道的原始值
 * dbus_online   遥控是否在线（失联立即撤销使能）
 * lift_enabled  操作手是否把抬升拨到"可用"（右拨杆上/中档）
 * left          左拨杆位置（remote_lift_left_t）
 * now_ms        当前系统毫秒
 */
void remote_lift_map(
    const int16_t channel[5],
    bool dbus_online,
    bool lift_enabled,
    remote_lift_left_t left,
    uint32_t now_ms);

#endif
