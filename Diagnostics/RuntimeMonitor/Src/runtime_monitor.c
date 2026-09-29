#include "runtime_monitor.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

#define RUNTIME_MONITOR_STACK_WORDS 384U
#define RUNTIME_MONITOR_PERIOD_MS   500U

runtime_monitor_status_t g_runtime_monitor_status;

static osThreadId_t runtime_monitor_task_handle;
static StaticTask_t runtime_monitor_task_control;
static StackType_t runtime_monitor_task_stack[RUNTIME_MONITOR_STACK_WORDS];
static TaskStatus_t task_status_buffer[RUNTIME_MONITOR_MAX_TASKS];

static runtime_task_state_t runtime_monitor_convert_state(eTaskState state)
{
    switch (state)
    {
        case eRunning:
            return RUNTIME_TASK_RUNNING;
        case eReady:
            return RUNTIME_TASK_READY;
        case eBlocked:
            return RUNTIME_TASK_BLOCKED;
        case eSuspended:
            return RUNTIME_TASK_SUSPENDED;
        case eDeleted:
            return RUNTIME_TASK_DELETED;
        default:
            return RUNTIME_TASK_INVALID;
    }
}

static void runtime_monitor_sample(void)
{
    UBaseType_t total_task_count;
    UBaseType_t captured_task_count;
    UBaseType_t index;

    total_task_count = uxTaskGetNumberOfTasks();
    captured_task_count = uxTaskGetSystemState(
        task_status_buffer, RUNTIME_MONITOR_MAX_TASKS, NULL);

    g_runtime_monitor_status.valid = false;
    g_runtime_monitor_status.uptime_ms =
        (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    g_runtime_monitor_status.task_count = (uint32_t)captured_task_count;
    g_runtime_monitor_status.task_capacity = RUNTIME_MONITOR_MAX_TASKS;
    g_runtime_monitor_status.heap_free_bytes =
        (uint32_t)xPortGetFreeHeapSize();
    g_runtime_monitor_status.heap_min_free_bytes =
        (uint32_t)xPortGetMinimumEverFreeHeapSize();

    if (total_task_count > RUNTIME_MONITOR_MAX_TASKS)
    {
        g_runtime_monitor_status.truncated_count++;
    }

    for (index = 0U; index < captured_task_count; index++)
    {
        runtime_task_snapshot_t *snapshot =
            &g_runtime_monitor_status.tasks[index];
        const TaskStatus_t *source = &task_status_buffer[index];

        (void)memset(snapshot, 0, sizeof(*snapshot));
        (void)strncpy(
            snapshot->name, source->pcTaskName,
            RUNTIME_MONITOR_TASK_NAME_LENGTH - 1U);
        snapshot->task_number = (uint32_t)source->xTaskNumber;
        snapshot->priority = (uint32_t)source->uxCurrentPriority;
        snapshot->stack_min_free_bytes =
            (uint32_t)source->usStackHighWaterMark * sizeof(StackType_t);
        snapshot->state = runtime_monitor_convert_state(source->eCurrentState);
    }

    for (; index < RUNTIME_MONITOR_MAX_TASKS; index++)
    {
        (void)memset(
            &g_runtime_monitor_status.tasks[index], 0,
            sizeof(g_runtime_monitor_status.tasks[index]));
    }

    g_runtime_monitor_status.sample_count++;
    g_runtime_monitor_status.valid = true;
}

static void runtime_monitor_task(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();

    (void)argument;
    for (;;)
    {
        runtime_monitor_sample();
        vTaskDelayUntil(
            &last_wake, pdMS_TO_TICKS(RUNTIME_MONITOR_PERIOD_MS));
    }
}

void runtime_monitor_init(void)
{
    (void)memset(
        &g_runtime_monitor_status, 0, sizeof(g_runtime_monitor_status));
    g_runtime_monitor_status.task_capacity = RUNTIME_MONITOR_MAX_TASKS;
}

bool runtime_monitor_task_create(void)
{
    const osThreadAttr_t attributes = {
        .name = "runtime_mon",
        .cb_mem = &runtime_monitor_task_control,
        .cb_size = sizeof(runtime_monitor_task_control),
        .stack_mem = runtime_monitor_task_stack,
        .stack_size = sizeof(runtime_monitor_task_stack),
        .priority = osPriorityLow};

    if (runtime_monitor_task_handle != NULL)
    {
        return true;
    }

    runtime_monitor_task_handle =
        osThreadNew(runtime_monitor_task, NULL, &attributes);
    return runtime_monitor_task_handle != NULL;
}
