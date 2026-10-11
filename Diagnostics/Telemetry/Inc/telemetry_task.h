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
    TELEMETRY_SOURCE_LIFT  = 1    /* 28 通道：抬升四轴串级调试（每轴一块） */
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
 *      +6 电调回显电流 axis_given_current（C620 报回来的，确认命令到了）
 *
 *   于是 y1~y4 = 第 0 根的全部量、y5~y8 = 第 1 根……看一根就框一块。
 *   （以前是按"量"分组，四根交织，看一根要在四个地方找。）
 *
 * ★ 怎么用这 7 条曲线：
 *   - 框 [+3] 和 [+4] 放一张图：一眼看出"内环还是外环"——
 *       实际速度绕着目标速度振 → **内环**问题 → 调 speed_pid.kp
 *       实际速度干净、位置却在振 → **外环**问题 → 调 position_pid.kp
 *   - [+5] 看"逐级开放/减速区"有没有按预期工作：
 *       一使能应该是 current_limit_start（2000），然后爬升到满值。
 *   - [+6] vs [+2] 分辨"电流发不出去"和"环路调不好"：
 *       两者都满 → 命令送到了，问题在环路/被控对象；
 *       [+6] ≈ 0 而 [+2] 满 → **命令没送到电调**，查 CAN 线/电调，跟 PID 无关。
 *       （2026-10-09 出现过"电流满 16000、轴却以 384535 ecd/s 跑飞"，
 *         必须先排除"电流其实没送到"这一种可能。）
 */
#define TELEMETRY_LIFT_CHANNELS   28U
/* 每根轴占几个通道（改通道内容时这个数要跟着改，打包函数按它算块起点）。 */
#define TELEMETRY_LIFT_CHANNELS_PER_AXIS  7U

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
