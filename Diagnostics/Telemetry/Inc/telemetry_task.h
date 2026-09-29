#ifndef TELEMETRY_TASK_H
#define TELEMETRY_TASK_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    volatile bool enabled;
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
