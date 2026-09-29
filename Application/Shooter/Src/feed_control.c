#include "feed_control.h"

#include "cmsis_os2.h"
#include "dji_motor.h"
#include "FreeRTOS.h"
#include "pid.h"
#include "robot_motor_commands.h"
#include "robot_motors.h"
#include "task.h"

#include <string.h>

#define FEED_TASK_STACK_WORDS 512U
#define FEED_PERIOD_MS 5U
#define FEED_TIMEOUT_MS 100U

feed_control_command_t g_feed_control_command;
feed_control_status_t g_feed_control_status;
feed_control_debug_t g_feed_control_debug;

static pid_controller_t feed_pid[FEED_MOTOR_COUNT];
static osThreadId_t feed_task_handle;
static StaticTask_t feed_task_control;
static StackType_t feed_task_stack[FEED_TASK_STACK_WORDS];

/** 按两种拨弹电机各自的允许范围限制目标转速。 */
static int16_t feed_limit_rpm(uint32_t index, int16_t rpm)
{
    const int16_t limit = index == FEED_MOTOR_PROJECTILE ? 10000 : 12000;
    if (rpm > limit)
        return limit;
    if (rpm < -limit)
        return (int16_t)-limit;
    return rpm;
}

/** 清空两个拨弹电机速度 PID 的历史状态。 */
static void feed_reset(void)
{
    uint32_t i;
    for (i = 0U; i < FEED_MOTOR_COUNT; i++)
        pid_reset(&feed_pid[i]);
}

/** 校验在线调试参数并更新两个拨弹电机的 PID 配置。 */
static void feed_apply_debug(void)
{
    uint32_t i;
    for (i = 0U; i < FEED_MOTOR_COUNT; i++)
    {
        float protocol_limit = i == FEED_MOTOR_PROJECTILE ? 10000.0F : 16384.0F;
        if (g_feed_control_debug.integral_limit[i] < 0.0F)
            g_feed_control_debug.integral_limit[i] = 0.0F;
        if (g_feed_control_debug.output_limit[i] < 0.0F)
            g_feed_control_debug.output_limit[i] = 0.0F;
        if (g_feed_control_debug.output_limit[i] > protocol_limit)
            g_feed_control_debug.output_limit[i] = protocol_limit;
        feed_pid[i].config.kp = g_feed_control_debug.kp[i];
        feed_pid[i].config.ki = g_feed_control_debug.ki[i];
        feed_pid[i].config.kd = g_feed_control_debug.kd[i];
        feed_pid[i].config.integral_limit = g_feed_control_debug.integral_limit[i];
        feed_pid[i].config.output_limit = g_feed_control_debug.output_limit[i];
    }
    feed_reset();
    g_feed_control_debug.apply = false;
}

/**
 * @brief 拨弹机构速度闭环任务。
 * @note 两个电机分别判断使能和在线状态，任一电机停机不会掩盖另一个电机的状态。
 */
static void feed_task(void *argument)
{
    robot_motor_state_t motors;
    const dji_motor_feedback_t *feedback[FEED_MOTOR_COUNT];
    int16_t output[FEED_MOTOR_COUNT];
    uint32_t i;
    const float dt = (float)FEED_PERIOD_MS / 1000.0F;
    TickType_t last_wake = xTaskGetTickCount();
    (void)argument;

    for (;;)
    {
        if (g_feed_control_debug.apply)
            feed_apply_debug();
        if (g_feed_control_debug.reset)
        {
            feed_reset();
            g_feed_control_debug.reset = false;
        }

        robot_motors_get_snapshot(&motors);
        feedback[FEED_MOTOR_PROJECTILE] = &motors.shooter.projectile_feeder;
        feedback[FEED_MOTOR_TRIGGER] = &motors.shooter.trigger;

        for (i = 0U; i < FEED_MOTOR_COUNT; i++)
        {
            g_feed_control_status.target_rpm[i] =
                feed_limit_rpm(i, g_feed_control_command.target_rpm[i]);
            g_feed_control_status.measured_rpm[i] = feedback[i]->speed_rpm;
            g_feed_control_status.online[i] = dji_motor_feedback_is_online(
                feedback[i], osKernelGetTickCount(),
                pdMS_TO_TICKS(FEED_TIMEOUT_MS));

            if (!g_feed_control_command.enabled[i] ||
                !g_feed_control_status.online[i])
            {
                pid_reset(&feed_pid[i]);
                output[i] = 0;
                g_feed_control_status.running[i] = false;
            }
            else
            {
                output[i] = (int16_t)pid_calculate(
                    &feed_pid[i],
                    (float)g_feed_control_status.target_rpm[i],
                    (float)feedback[i]->speed_rpm, dt);
                g_feed_control_status.running[i] = true;
            }
            g_feed_control_status.output_current[i] = output[i];
            g_feed_control_status.error[i] = feed_pid[i].error;
            g_feed_control_status.p_output[i] = feed_pid[i].p_output;
            g_feed_control_status.i_output[i] = feed_pid[i].i_output;
            g_feed_control_status.d_output[i] = feed_pid[i].d_output;
        }

        robot_shooter_set_feed_currents(output[0], output[1]);
        g_feed_control_status.update_count++;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FEED_PERIOD_MS));
    }
}

/** 初始化拨弹命令、状态以及两个速度环的默认参数。 */
void feed_control_init(void)
{
    const pid_config_t configs[FEED_MOTOR_COUNT] = {
        {4.0F, 0.5F, 0.0F, 8000.0F, 2000.0F},
        {4.0F, 0.8F, 0.0F, 12000.0F, 3000.0F}};
    uint32_t i;
    (void)memset(&g_feed_control_command, 0, sizeof(g_feed_control_command));
    (void)memset(&g_feed_control_status, 0, sizeof(g_feed_control_status));
    (void)memset(&g_feed_control_debug, 0, sizeof(g_feed_control_debug));
    for (i = 0U; i < FEED_MOTOR_COUNT; i++)
    {
        pid_init(&feed_pid[i], &configs[i]);
        g_feed_control_debug.kp[i] = configs[i].kp;
        g_feed_control_debug.ki[i] = configs[i].ki;
        g_feed_control_debug.kd[i] = configs[i].kd;
        g_feed_control_debug.integral_limit[i] = configs[i].integral_limit;
        g_feed_control_debug.output_limit[i] = configs[i].output_limit;
    }
}

/** 创建静态内存形式的拨弹控制任务。 */
bool feed_control_task_create(void)
{
    const osThreadAttr_t attr = {
        .name = "feed_ctrl", .cb_mem = &feed_task_control,
        .cb_size = sizeof(feed_task_control), .stack_mem = feed_task_stack,
        .stack_size = sizeof(feed_task_stack), .priority = osPriorityNormal};
    if (feed_task_handle != NULL)
        return true;
    feed_task_handle = osThreadNew(feed_task, NULL, &attr);
    return feed_task_handle != NULL;
}
