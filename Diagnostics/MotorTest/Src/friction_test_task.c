#include "friction_test_task.h"

#include "can_rx_service.h"
#include "can_tx_service.h"
#include "cmsis_os2.h"
#include "dji_motor.h"
#include "FreeRTOS.h"
#include "robot_motor_commands.h"
#include "robot_motors.h"
#include "task.h"

#include <stddef.h>

#define FRICTION_TEST_TASK_STACK_WORDS    512U
#define FRICTION_TEST_TASK_PERIOD_MS      10U
#define FRICTION_TEST_DEFAULT_DURATION_MS 2000U
#define FRICTION_TEST_MAX_DURATION_MS     5000U
#define FRICTION_TEST_CURRENT_LIMIT       1000
#define FRICTION_TEST_ONLINE_TIMEOUT_MS   100U

friction_test_command_t g_friction_test_command = {
    .enabled = false,
    .motor_index = 0U,
    .test_current = 500,
    .duration_ms = FRICTION_TEST_DEFAULT_DURATION_MS
};

friction_test_status_t g_friction_test_status;

static osThreadId_t friction_test_task_handle;
static StaticTask_t friction_test_task_control;
static StackType_t friction_test_task_stack[FRICTION_TEST_TASK_STACK_WORDS];

/** 将点动测试电流限制在低风险范围内。 */
static int16_t friction_test_limit_current(int16_t current)
{
    if (current > FRICTION_TEST_CURRENT_LIMIT)
    {
        return FRICTION_TEST_CURRENT_LIMIT;
    }
    if (current < -FRICTION_TEST_CURRENT_LIMIT)
    {
        return -FRICTION_TEST_CURRENT_LIMIT;
    }
    return current;
}

/** 修正点动测试持续时间，防止忘记关闭时长期输出。 */
static uint32_t friction_test_limit_duration(uint32_t duration_ms)
{
    if (duration_ms == 0U)
    {
        return FRICTION_TEST_DEFAULT_DURATION_MS;
    }
    if (duration_ms > FRICTION_TEST_MAX_DURATION_MS)
    {
        return FRICTION_TEST_MAX_DURATION_MS;
    }
    return duration_ms;
}

/** 向整个发射机构写入零电流并更新测试显示值。 */
static void friction_test_stop_output(void)
{
    robot_shooter_set_command(0, 0, 0, 0, 0, 0);
    g_friction_test_status.applied_current = 0;
}

/** 仅对选中的一个摩擦轮施加测试电流，其余通道保持为零。 */
static void friction_test_apply_output(uint8_t motor_index, int16_t current)
{
    int16_t friction[4] = {0, 0, 0, 0};

    if (motor_index < 4U)
    {
        friction[motor_index] = current;
    }

    robot_shooter_set_command(
        friction[0], friction[1], friction[2], friction[3], 0, 0);
}

/** 汇总四个摩擦轮反馈以及 CAN 收发统计，供调试器观察。 */
static void friction_test_update_status(void)
{
    robot_motor_state_t motors;
    can_tx_service_stats_t tx_stats;
    uint32_t index;
    const uint32_t current_tick = osKernelGetTickCount();

    robot_motors_get_snapshot(&motors);
    can_tx_service_get_stats(&tx_stats);

    for (index = 0U; index < 4U; index++)
    {
        const dji_motor_feedback_t *feedback =
            &motors.shooter.friction[index];

        g_friction_test_status.ecd[index] = feedback->ecd;
        g_friction_test_status.speed_rpm[index] = feedback->speed_rpm;
        g_friction_test_status.feedback_current[index] =
            feedback->given_current;
        g_friction_test_status.temperature[index] = feedback->temperature;
        g_friction_test_status.update_count[index] = feedback->update_count;
        g_friction_test_status.online[index] =
            dji_motor_feedback_is_online(
                feedback,
                current_tick,
                pdMS_TO_TICKS(FRICTION_TEST_ONLINE_TIMEOUT_MS));
    }

    g_friction_test_status.can_rx_count =
        can_rx_service_get_received_count();
    g_friction_test_status.can_rx_dropped =
        can_rx_service_get_dropped_count();
    g_friction_test_status.can_tx_count = tx_stats.sent_count;
    g_friction_test_status.can_tx_busy = tx_stats.busy_count;
    g_friction_test_status.can_tx_errors =
        tx_stats.invalid_count + tx_stats.error_count;
}

/** 执行有电流与时长双重限制的摩擦轮点动测试。 */
static void friction_test_task(void *argument)
{
    TickType_t last_wake_tick;
    uint32_t start_tick = 0U;
    uint32_t duration_ticks = 0U;
    uint32_t elapsed_ticks;
    int16_t current;
    uint8_t motor_index;
    bool was_enabled = false;

    (void)argument;
    last_wake_tick = xTaskGetTickCount();

    for (;;)
    {
        motor_index = g_friction_test_command.motor_index;
        current = friction_test_limit_current(
            g_friction_test_command.test_current);

        if (g_friction_test_command.enabled && !was_enabled)
        {
            if (motor_index < 4U)
            {
                start_tick = osKernelGetTickCount();
                duration_ticks = pdMS_TO_TICKS(friction_test_limit_duration(
                    g_friction_test_command.duration_ms));
                g_friction_test_status.running = true;
                g_friction_test_status.active_motor_index = motor_index;
                was_enabled = true;
            }
            else
            {
                g_friction_test_command.enabled = false;
            }
        }

        if (was_enabled)
        {
            elapsed_ticks = (uint32_t)(osKernelGetTickCount() - start_tick);
            g_friction_test_status.elapsed_ms =
                elapsed_ticks * portTICK_PERIOD_MS;

            if (!g_friction_test_command.enabled ||
                (elapsed_ticks >= duration_ticks))
            {
                friction_test_stop_output();
                g_friction_test_command.enabled = false;
                g_friction_test_status.running = false;
                was_enabled = false;
            }
            else
            {
                friction_test_apply_output(motor_index, current);
                g_friction_test_status.applied_current = current;
            }
        }
        else
        {
            friction_test_stop_output();
        }

        friction_test_update_status();

        vTaskDelayUntil(
            &last_wake_tick,
            pdMS_TO_TICKS(FRICTION_TEST_TASK_PERIOD_MS));
    }
}

/** 创建静态内存形式的摩擦轮诊断任务。 */
bool friction_test_task_create(void)
{
    const osThreadAttr_t attributes = {
        .name = "friction_test",
        .cb_mem = &friction_test_task_control,
        .cb_size = sizeof(friction_test_task_control),
        .stack_mem = friction_test_task_stack,
        .stack_size = sizeof(friction_test_task_stack),
        .priority = osPriorityNormal
    };

    if (friction_test_task_handle != NULL)
    {
        return true;
    }

    friction_test_task_handle = osThreadNew(
        friction_test_task, NULL, &attributes);
    return friction_test_task_handle != NULL;
}
