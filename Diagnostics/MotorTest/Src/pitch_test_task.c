#include "pitch_test_task.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "robot_motor_commands.h"
#include "robot_motors.h"
#include "task.h"

#define PITCH_TEST_TASK_STACK_WORDS    512U
#define PITCH_TEST_TASK_PERIOD_MS      10U
#define PITCH_TEST_DEFAULT_DURATION_MS 2000U
#define PITCH_TEST_MAX_DURATION_MS     5000U
#define PITCH_TEST_ONLINE_TIMEOUT_MS   100U
/* 点动电压上限：GM6020 满量程 ±30000，诊断只允许用其中一小部分。 */
#define PITCH_TEST_MAX_CURRENT         3000

pitch_test_command_t g_pitch_test_command = {
    .enabled = false,
    .test_current = 500,
    .duration_ms = PITCH_TEST_DEFAULT_DURATION_MS
};

pitch_test_status_t g_pitch_test_status;

static osThreadId_t pitch_test_task_handle;
static StaticTask_t pitch_test_task_control;
static StackType_t pitch_test_task_stack[PITCH_TEST_TASK_STACK_WORDS];

/** 修正 Pitch 测试持续时间，防止电机长期保持通电。 */
static uint32_t pitch_test_limit_duration(uint32_t duration_ms)
{
    if (duration_ms == 0U)
    {
        return PITCH_TEST_DEFAULT_DURATION_MS;
    }
    if (duration_ms > PITCH_TEST_MAX_DURATION_MS)
    {
        return PITCH_TEST_MAX_DURATION_MS;
    }
    return duration_ms;
}

/** 把诊断电压限制在安全范围内。 */
static int16_t pitch_test_limit_current(int16_t test_current)
{
    if (test_current > PITCH_TEST_MAX_CURRENT)
    {
        return PITCH_TEST_MAX_CURRENT;
    }
    if (test_current < -PITCH_TEST_MAX_CURRENT)
    {
        return -PITCH_TEST_MAX_CURRENT;
    }
    return test_current;
}

/** 汇总 Pitch 的 DJI 解码反馈和在线状态。 */
static void pitch_test_update_status(void)
{
    robot_motor_state_t motors;
    uint32_t current_tick;

    robot_motors_get_snapshot(&motors);
    current_tick = osKernelGetTickCount();

    g_pitch_test_status.feedback_update_count =
        motors.gimbal.pitch.update_count;
    g_pitch_test_status.ecd = motors.gimbal.pitch.ecd;
    g_pitch_test_status.total_ecd = motors.gimbal.pitch.total_ecd;
    g_pitch_test_status.speed_rpm = motors.gimbal.pitch.speed_rpm;
    g_pitch_test_status.given_current = motors.gimbal.pitch.given_current;
    g_pitch_test_status.temperature = motors.gimbal.pitch.temperature;
    g_pitch_test_status.online = dji_motor_feedback_is_online(
        &motors.gimbal.pitch,
        current_tick,
        pdMS_TO_TICKS(PITCH_TEST_ONLINE_TIMEOUT_MS));
}

/** 在限定时间内按恒定电压点动 Pitch，结束后立刻归零。 */
static void pitch_test_task(void *argument)
{
    TickType_t last_wake_tick;
    uint32_t start_tick = 0U;
    uint32_t duration_ticks = 0U;
    uint32_t elapsed_ticks;
    bool was_enabled = false;

    (void)argument;
    last_wake_tick = xTaskGetTickCount();

    for (;;)
    {
        if (g_pitch_test_command.enabled && !was_enabled)
        {
            start_tick = osKernelGetTickCount();
            duration_ticks = pdMS_TO_TICKS(pitch_test_limit_duration(
                g_pitch_test_command.duration_ms));
            g_pitch_test_status.running = true;
            was_enabled = true;
        }

        if (was_enabled)
        {
            elapsed_ticks = (uint32_t)(osKernelGetTickCount() - start_tick);
            g_pitch_test_status.elapsed_ms =
                elapsed_ticks * portTICK_PERIOD_MS;
            robot_gimbal_set_pitch_current(
                pitch_test_limit_current(g_pitch_test_command.test_current));

            if (!g_pitch_test_command.enabled ||
                (elapsed_ticks >= duration_ticks))
            {
                robot_gimbal_set_pitch_current(0);
                g_pitch_test_command.enabled = false;
                g_pitch_test_status.running = false;
                was_enabled = false;
            }
        }

        pitch_test_update_status();

        vTaskDelayUntil(
            &last_wake_tick,
            pdMS_TO_TICKS(PITCH_TEST_TASK_PERIOD_MS));
    }
}

/** 创建静态内存形式的 Pitch 诊断任务。 */
bool pitch_test_task_create(void)
{
    const osThreadAttr_t attributes = {
        .name = "pitch_test",
        .cb_mem = &pitch_test_task_control,
        .cb_size = sizeof(pitch_test_task_control),
        .stack_mem = pitch_test_task_stack,
        .stack_size = sizeof(pitch_test_task_stack),
        .priority = osPriorityNormal
    };

    if (pitch_test_task_handle != NULL)
    {
        return true;
    }

    pitch_test_task_handle = osThreadNew(
        pitch_test_task, NULL, &attributes);
    return pitch_test_task_handle != NULL;
}
