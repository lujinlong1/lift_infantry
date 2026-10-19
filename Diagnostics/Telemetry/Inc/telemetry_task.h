#ifndef TELEMETRY_TASK_H
#define TELEMETRY_TASK_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 波形源：VOFA+ 一次只收一种帧（帧率才不会被摊薄）。
 * 置 g_telemetry_status.source 即可切换，Pitch 调试随时能切回来。
 */
typedef enum
{
    TELEMETRY_SOURCE_PITCH = 0,   /* 8 通道：Pitch 遥控/控制量（原行为） */
    TELEMETRY_SOURCE_LIFT  = 1,   /* 28 通道：旧台架控制器 */
    TELEMETRY_SOURCE_LIFT_CONTROL = 2, /* unchanged 28-channel layout */
    TELEMETRY_SOURCE_LIFT_DIAGNOSTICS = 3 /* selected axis full chain; P0 report */
} telemetry_source_t;

/*
 * 抬升帧的 28 个通道（VOFA+ JustFloat，顺序 = 通道号）。
 *
 * ★ 2026-10-11 改成【每根轴一块】：28 = 4 块 × 7，第 ch 根的块从 ch*7 开始
 *   （通道号按 1 起算就是第 7ch+1 ~ 7ch+7 条）：
 *      +0 实际位置 axis_offset_ecd      （相对使能时刻，正 = 伸出）
 *      +1 目标位置 axis_target_offset
 *      +2 下发电流 axis_output_current  （我们发出去的，含方向符号）
 *      +3 实际速度 axis_speed_ecd_s     （速度环的反馈）
 *      +4 目标速度 axis_target_speed_ecd_s（位置环输出 = 速度环给定）
 *      +5 本周期电流上限 axis_limit_now  （逐级开放后的实时限幅）
 *      +6 电调回显电流 axis_given_current（C620 测得电流，并非命令确认）
 *
 *   因此通道 0~6 是 ch0，7~13 是 ch1，14~20 是 ch2，21~27 是 ch3。
 *   （以前是按"量"分组，四根交织，看一根要在四个地方找。）
 *
 * ★ 怎么用这 7 条曲线：
 *   - 框 [+3] 和 [+4] 放一张图：一眼看出"内环还是外环"——
 *       实际速度绕着目标速度振 → **内环**问题 → 调 speed_pid.kp
 *       实际速度干净、位置却在振 → **外环**问题 → 调 position_pid.kp
 *   - [+5] 看"逐级开放/减速区"有没有按预期工作：
 *       一使能应该是 current_limit_start（2000），然后爬升到满值。
 *   - [+6] vs [+2] 分辨"电流发不出去"和"环路调不好"：
 *       两者都满 → 电调测得大电流，结合发送状态与反馈年龄排查；
 *       [+6] ≈ 0 而 [+2] 满 → 查 CAN 状态、反馈年龄和负载，不能仅凭电流确认丢帧。
 *       （2026-10-09 出现过"电流满 16000、轴却以 384535 ecd/s 跑飞"，
 *         必须先排除"电流其实没送到"这一种可能。）
 *
 * source=2 uses the same channel blocks for the main lift controller:
 * +0 homed position, +1 final target, +2 logical applied current,
 * +3 actual speed, +4 target speed, +5 current limit, +6 logical feedback
 * current. Positions are encoder counts from the confirmed physical datum;
 * positive current and speed mean extension on every axis.
 */
#define TELEMETRY_LIFT_CHANNELS   28U
/* 每根轴占几个通道（改通道内容时这个数要跟着改，打包函数按它算块起点）。 */
#define TELEMETRY_LIFT_CHANNELS_PER_AXIS  7U

typedef struct
{
    volatile bool     enabled;
    volatile uint8_t  source;      /* telemetry_source_t：发哪一路波形 */
    volatile uint8_t diagnostic_axis; /* source=3: 0..3 */
    volatile uint32_t snapshot_drop_count;
    volatile uint32_t sent_count;
    volatile uint32_t busy_count;
    volatile uint32_t error_count;
} telemetry_status_t;

extern telemetry_status_t g_telemetry_status;

/** 初始化无线串口波形发送状态。 */
void telemetry_init(void);

/** 创建固定频率发送 JustFloat 数据帧的任务。 */
bool telemetry_task_create(void);

#endif
