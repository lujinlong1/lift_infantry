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
 * ⚠️ 2026-10-11：**本模块默认关闭**（`g_remote_lift_debug.enabled = false`）。
 *    抬升目前只在 debug 里调（Watch 直接写 g_lift_task_command，
 *    或用台架模块 lift_jog_task）。要在整车联调前处理的两件事：
 *      ① 摇杆通道还没有着落：stick_channel 默认 1，而 channel[1] 是【右摇杆
 *         竖直 = Pitch 速度输入】（见 remote_control_task.c 的 remote_set_gimbal），
 *         channel[2]/[3] 被底盘占用、channel[0] 是 Yaw、channel[4] 是发射拨盘
 *         —— 现在没有空闲的模拟通道，必须先定"用哪个操作件抬升"。
 *      ② 上面的三段拨杆时序（找零/清零点）从来没在实机上跑通过，
 *         打开之前必须先在台架上把找零两步走一遍。
 *
 * 找零会让机构一路下行到底部堵转，必须由人明确要求，且必须在"清零点"之后
 * 才允许——这就是 ADR-007 的两步重新找零。因此**每次上电后（以及每次想让
 * 已找零结果失效时）都必须先在下档保持够久，再切到中档保持够久**，本模块
 * 才发出找零请求。上电时"已清零点"标志是假的，所以不可能一步就启动找零。
 * 之所以用"保持"而不是"点一下"，是为了让一次误碰不可能启动找零。
 */

#include "dbus_remote.h"

#include <stdbool.h>
#include <stdint.h>

/* 左拨杆三段。
 *
 * ⚠️ 2026-10-11 修正：这里以前写成 DOWN=0 / MID=1 / UP=2，而 dbus_remote.h 的
 *    真实原始值是 UP=1 / DOWN=2 / MID=3（0 表示帧没对齐，dbus_remote.c 直接
 *    丢弃该帧）。调用点是 (remote_lift_left_t)remote.switch_left 直接强转，
 *    于是：物理上档(1) 被当成"中档"分支 → 撤销使能；下档(2)/中档(3) 落进
 *    "上档"分支 → 反而正常操作；而唯一会置 homing_armed 的分支要求 left==0，
 *    **永远进不去** ⇒ 清零点、找零、清故障三个功能全部不可达，homed 恒 false。
 *
 *    现在直接取 DBUS_SWITCH_* 的值，类型与原始值不再可能对不上。
 */
typedef enum
{
    REMOTE_LIFT_LEFT_UP = DBUS_SWITCH_UP,       /* 1 正常操作（云台/发射） */
    REMOTE_LIFT_LEFT_DOWN = DBUS_SWITCH_DOWN,   /* 2 抬升作废 + 清零点 / 清故障 */
    REMOTE_LIFT_LEFT_MID = DBUS_SWITCH_MID      /* 3 请求找零（需先清零点） */
} remote_lift_left_t;

typedef struct
{
    volatile bool enabled;

    /* 摇杆映射 */
    /*
     * ⚠️ channel[0..4] 的占用情况（2026-10-11 核对，别照着旧注释填）：
     *   0 = 右摇杆水平 → Yaw 速度
     *   1 = 右摇杆竖直 → 【Pitch 速度】   ← 本模块默认也指向它，会互相打架
     *   2 = 左摇杆水平 → 底盘横移
     *   3 = 左摇杆竖直 → 底盘前后
     *   4 = 发射拨盘
     * 所以"用哪个操作件抬升"是个**还没定**的设计问题，见文件头说明。
     */
    volatile int16_t stick_channel;     /* DBUS channel[0..4] */
    volatile int16_t stick_deadband;
    /* 摇杆量 → 每周期目标增量（【模态高度】计数；满杆 = 660 × 本值）。 */
    volatile float   stick_scale;
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
