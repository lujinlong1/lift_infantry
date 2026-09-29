#ifndef RUNTIME_MONITOR_H
#define RUNTIME_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

#define RUNTIME_MONITOR_MAX_TASKS 24U
#define RUNTIME_MONITOR_TASK_NAME_LENGTH 16U

typedef enum
{
    RUNTIME_TASK_RUNNING = 0,
    RUNTIME_TASK_READY,
    RUNTIME_TASK_BLOCKED,
    RUNTIME_TASK_SUSPENDED,
    RUNTIME_TASK_DELETED,
    RUNTIME_TASK_INVALID
} runtime_task_state_t;

typedef struct
{
    char name[RUNTIME_MONITOR_TASK_NAME_LENGTH];
    uint32_t task_number;
    uint32_t priority;
    uint32_t stack_min_free_bytes;
    runtime_task_state_t state;
} runtime_task_snapshot_t;

typedef struct
{
    volatile bool valid;
    volatile uint32_t sample_count;
    volatile uint32_t uptime_ms;
    volatile uint32_t task_count;
    volatile uint32_t task_capacity;
    volatile uint32_t truncated_count;
    volatile uint32_t heap_free_bytes;
    volatile uint32_t heap_min_free_bytes;
    runtime_task_snapshot_t tasks[RUNTIME_MONITOR_MAX_TASKS];
} runtime_monitor_status_t;

extern runtime_monitor_status_t g_runtime_monitor_status;

void runtime_monitor_init(void);
bool runtime_monitor_task_create(void);

#endif
