#include "telemetry_task.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "lift_jog_task.h"
#include "pitch_control.h"
#include "remote_control_task.h"
#include "task.h"
#include "usart.h"

#include <string.h>

/*
 * 无线串口波形（VOFA+ JustFloat）。
 *
 * ── 两种波形源，用 g_telemetry_status.source 切 ────────────────────
 *   0 = PITCH（8 通道，原来的行为）
 *   1 = LIFT （16 通道，抬升四轴串级调试）
 *
 * 为什么不做成"两帧交替发"：USART1 是 115200，抬升帧 68 字节 ≈ 5.9 ms，
 * 20 ms 周期本来就只能用掉三成带宽。交替发只会让两边都变慢、还难对齐时间轴；
 * 一次只发一种最简单，切换成本就是改一个变量。
 */
#define TELEMETRY_TASK_STACK_WORDS 512U
#define TELEMETRY_PERIOD_MS        20U
#define TELEMETRY_PITCH_CHANNELS   8U
#define TELEMETRY_MAX_CHANNELS     TELEMETRY_LIFT_CHANNELS
#define TELEMETRY_FRAME_SIZE       \
    (TELEMETRY_MAX_CHANNELS * sizeof(float) + 4U)

telemetry_status_t g_telemetry_status;

static osThreadId_t telemetry_task_handle;
static StaticTask_t telemetry_task_control;
static StackType_t  telemetry_task_stack[TELEMETRY_TASK_STACK_WORDS];

/* USART1 TX DMA 直接访问该缓冲区，因此必须由 scatter 文件放在 SRAM。 */
static uint8_t telemetry_frame[TELEMETRY_FRAME_SIZE]
    __attribute__((section(".bss.telemetry_dma_buffer")));

/* JustFloat 帧尾：小端 float +inf。 */
static const uint8_t telemetry_tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};

/**
 * @brief 将当前 Pitch 控制量打包成 VOFA+ JustFloat 帧（8 通道）。
 * @note STM32F4 使用小端浮点，JustFloat 的4字节帧尾为 00 00 80 7F。
 */
static uint32_t telemetry_build_pitch_frame(void)
{
    const float values[TELEMETRY_PITCH_CHANNELS] = {
        g_remote_mapping_status.pitch_raw_rate_deg_s,
        g_remote_mapping_status.pitch_filtered_rate_deg_s,
        g_remote_mapping_status.pitch_ramped_rate_deg_s,
        g_pitch_control_command.target_angle_deg,
        g_pitch_control_status.commanded_angle_deg,
        g_pitch_control_status.measured_angle_deg,
        g_pitch_control_status.commanded_velocity_deg_s,
        g_pitch_control_status.measured_velocity_deg_s};

    (void)memcpy(telemetry_frame, values, sizeof(values));
    return sizeof(values);
}

/**
 * @brief 将抬升台架四轴串级的关键量打包成 VOFA+ JustFloat 帧（16 通道）。
 * @note 直接读 g_lift_jog_status，所以**不管三个模式有没有使能都在刷新**，
 *       没使能时是一组"静止"的曲线，正好当基线看。
 */
static uint32_t telemetry_build_lift_frame(void)
{
    float values[TELEMETRY_LIFT_CHANNELS];
    uint32_t ch;

    for (ch = 0U; ch < 4U; ch++)
    {
        /* 0~3 实际位置，4~7 目标位置，8~11 输出电流，12~15 实际速度，
         * 16~19 目标速度，20~23 生效的电流上限，24~27 电调回显电流 */
        values[ch] = (float)g_lift_jog_status.axis_offset_ecd[ch];
        values[4U + ch] = (float)g_lift_jog_status.axis_target_offset[ch];
        values[8U + ch] = (float)g_lift_jog_status.axis_output_current[ch];
        values[12U + ch] = (float)g_lift_jog_status.axis_speed_ecd_s[ch];
        values[16U + ch] = (float)g_lift_jog_status.axis_target_speed_ecd_s[ch];
        values[20U + ch] = (float)g_lift_jog_status.axis_limit_now[ch];
        values[24U + ch] = (float)g_lift_jog_status.axis_given_current[ch];
    }

    (void)memcpy(telemetry_frame, values, sizeof(values));
    return sizeof(values);
}

/** 周期性发送一路波形帧；DMA 忙时丢弃本周期，不阻塞控制任务。 */
static void telemetry_task(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();

    (void)argument;
    for (;;)
    {
        if (g_telemetry_status.enabled)
        {
            uint32_t payload;

            if (g_telemetry_status.source ==
                (uint8_t)TELEMETRY_SOURCE_LIFT)
            {
                payload = telemetry_build_lift_frame();
            }
            else
            {
                payload = telemetry_build_pitch_frame();
            }

            (void)memcpy(
                &telemetry_frame[payload], telemetry_tail,
                sizeof(telemetry_tail));

            if (HAL_UART_Transmit_DMA(
                    &huart1, telemetry_frame,
                    (uint16_t)(payload + sizeof(telemetry_tail))) == HAL_OK)
            {
                g_telemetry_status.sent_count++;
            }
            else
            {
                g_telemetry_status.busy_count++;
            }
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}

/** 清空波形发送统计并默认开启无线串口发送（默认发 Pitch，和以前一致）。 */
void telemetry_init(void)
{
    (void)memset(&g_telemetry_status, 0, sizeof(g_telemetry_status));
    g_telemetry_status.enabled = true;
    g_telemetry_status.source = (uint8_t)TELEMETRY_SOURCE_PITCH;
}

/** 创建静态内存形式的无线波形发送任务。 */
bool telemetry_task_create(void)
{
    const osThreadAttr_t attributes = {
        .name = "telemetry",
        .cb_mem = &telemetry_task_control,
        .cb_size = sizeof(telemetry_task_control),
        .stack_mem = telemetry_task_stack,
        .stack_size = sizeof(telemetry_task_stack),
        .priority = osPriorityBelowNormal};

    if (telemetry_task_handle != NULL)
    {
        return true;
    }
    telemetry_task_handle = osThreadNew(telemetry_task, NULL, &attributes);
    return telemetry_task_handle != NULL;
}
