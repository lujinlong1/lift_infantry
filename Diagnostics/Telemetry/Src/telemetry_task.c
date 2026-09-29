#include "telemetry_task.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "pitch_control.h"
#include "remote_control_task.h"
#include "task.h"
#include "usart.h"

#include <string.h>

#define TELEMETRY_TASK_STACK_WORDS 512U
#define TELEMETRY_PERIOD_MS        20U
#define TELEMETRY_CHANNEL_COUNT    8U
#define TELEMETRY_FRAME_SIZE       \
    (TELEMETRY_CHANNEL_COUNT * sizeof(float) + 4U)

telemetry_status_t g_telemetry_status;

static osThreadId_t telemetry_task_handle;
static StaticTask_t telemetry_task_control;
static StackType_t telemetry_task_stack[TELEMETRY_TASK_STACK_WORDS];

/* USART1 TX DMA 直接访问该缓冲区，因此必须由 scatter 文件放在 SRAM。 */
static uint8_t telemetry_frame[TELEMETRY_FRAME_SIZE]
    __attribute__((section(".bss.telemetry_dma_buffer")));

/**
 * @brief 将当前 Pitch 控制量打包成 VOFA+ JustFloat 帧。
 * @note STM32F4 使用小端浮点，JustFloat 的4字节帧尾为 00 00 80 7F。
 */
static void telemetry_build_pitch_frame(void)
{
    const float values[TELEMETRY_CHANNEL_COUNT] = {
        g_remote_mapping_status.pitch_raw_rate_deg_s,
        g_remote_mapping_status.pitch_filtered_rate_deg_s,
        g_remote_mapping_status.pitch_ramped_rate_deg_s,
        g_pitch_control_command.target_angle_deg,
        g_pitch_control_status.commanded_angle_deg,
        g_pitch_control_status.measured_angle_deg,
        g_pitch_control_status.commanded_velocity_deg_s,
        g_pitch_control_status.measured_velocity_deg_s};
    const uint8_t frame_tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};

    (void)memcpy(telemetry_frame, values, sizeof(values));
    (void)memcpy(
        &telemetry_frame[sizeof(values)], frame_tail, sizeof(frame_tail));
}

/** 周期性发送 Pitch 波形帧；DMA 忙时丢弃本周期，不阻塞控制任务。 */
static void telemetry_task(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();

    (void)argument;
    for (;;)
    {
        if (g_telemetry_status.enabled)
        {
            telemetry_build_pitch_frame();
            if (HAL_UART_Transmit_DMA(
                    &huart1, telemetry_frame, TELEMETRY_FRAME_SIZE) == HAL_OK)
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

/** 清空波形发送统计并默认开启无线串口发送。 */
void telemetry_init(void)
{
    (void)memset(&g_telemetry_status, 0, sizeof(g_telemetry_status));
    g_telemetry_status.enabled = true;
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
