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
    TELEMETRY_SOURCE_LIFT  = 1    /* 16 通道：抬升四轴串级调试 */
} telemetry_source_t;

/*
 * 抬升帧的 28 个通道（VOFA+ JustFloat，顺序 = 通道号）：
 *   0~3   四轴实际位置 axis_offset_ecd[0..3]        （相对使能时刻，正=伸出）
 *   4~7   四轴目标位置 axis_target_offset[0..3]
 *   8~11  四轴输出电流 axis_output_current[0..3]     （我们下发的，含方向符号）
 *   12~15 四轴实际速度 axis_speed_ecd_s[0..3]        （速度环的反馈）
 *   16~19 ★四轴目标速度 axis_target_speed_ecd_s[0..3]（位置环的输出 = 速度环的给定）
 *   20~23 ★四轴电流上限 axis_limit_now[0..3]         （逐级开放后的实时限幅）
 *   24~27 ★四轴回显电流 axis_given_current[0..3]     （C620 报回来的，确认命令到了）
 *
 * ★ 16~19 是调串级最关键的两个量的另一半：
 *   拿 12~15（实际速度）和 16~19（目标速度）放同一张图，一眼就能看出
 *   "是内环在振还是外环在振"：
 *     - 实际速度绕着目标速度振  → **内环**问题 → 调 speed_pid.kp
 *     - 实际速度跟得很干净、位置却在振 → **外环**问题 → 调 position_pid.kp
 *
 * ★ 20~23 用来看"逐级开放"和"减速区"有没有按预期工作：
 *   一使能应该是 current_limit_start（2000），然后爬升到 16000。
 *
 * ★ 24~27 用来分辨"电流发不出去"和"环路调不好"：
 *   - 24~27 ≈ 8~11（都是满值）→ 命令送到了，问题在环路/被控对象
 *   - 24~27 ≈ 0 而 8~11 是满值 → **命令没送到电调**，查 CAN 线/电调，跟 PID 无关
 *   （2026-10-09 加：出现过"电流满 16000、轴却以 384535 ecd/s 跑飞"，
 *     必须先排除"电流其实没送到"这一种可能。）
 */
#define TELEMETRY_LIFT_CHANNELS   28U

typedef struct
{
    volatile bool     enabled;
    volatile uint8_t  source;      /* telemetry_source_t：发哪一路波形 */
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
