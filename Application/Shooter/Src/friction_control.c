#include "friction_control.h"

#include "cmsis_os2.h"
#include "dji_motor.h"
#include "FreeRTOS.h"
#include "pid.h"
#include "robot_motor_commands.h"
#include "robot_motors.h"
#include "task.h"

#include <string.h>

#define FRICTION_TASK_STACK_WORDS 512U
#define FRICTION_PERIOD_MS 5U
#define FRICTION_TIMEOUT_MS 100U
#define FRICTION_RPM_LIMIT 12000
#define FRICTION_OUTPUT_LIMIT 3000.0F
#define FRICTION_INTEGRAL_LIMIT 12000.0F

friction_control_command_t g_friction_control_command;
friction_control_status_t g_friction_control_status;
friction_control_debug_t g_friction_control_debug;

static pid_controller_t friction_pid[ROBOT_FRICTION_MOTOR_COUNT];
static osThreadId_t friction_control_task_handle;
static StaticTask_t friction_control_task_control;
static StackType_t friction_control_task_stack[FRICTION_TASK_STACK_WORDS];

/** 清空全部摩擦轮速度 PID 的历史状态。 */
static void friction_reset_pid(void)
{
    uint32_t i;
    for (i = 0U; i < ROBOT_FRICTION_MOTOR_COUNT; i++)
    {
        pid_reset(&friction_pid[i]);
    }
}

/** 将摩擦轮目标转速限制在机械和协议允许范围内。 */
static int16_t friction_limit_rpm(int16_t rpm)
{
    if (rpm > FRICTION_RPM_LIMIT)
        return FRICTION_RPM_LIMIT;
    if (rpm < -FRICTION_RPM_LIMIT)
        return -FRICTION_RPM_LIMIT;
    return rpm;
}

/** 校验在线调试参数，并同步应用到全部摩擦轮 PID。 */
static void friction_apply_debug(void)
{
    pid_config_t config;
    uint32_t i;

    if (g_friction_control_debug.kp < 0.0F)
        g_friction_control_debug.kp = 0.0F;
    if (g_friction_control_debug.ki < 0.0F)
        g_friction_control_debug.ki = 0.0F;
    if (g_friction_control_debug.kd < 0.0F)
        g_friction_control_debug.kd = 0.0F;
    if (g_friction_control_debug.integral_limit < 0.0F)
        g_friction_control_debug.integral_limit = 0.0F;
    if (g_friction_control_debug.output_limit < 0.0F)
        g_friction_control_debug.output_limit = 0.0F;
    if (g_friction_control_debug.output_limit > (float)ROBOT_2006_CURRENT_LIMIT)
        g_friction_control_debug.output_limit = (float)ROBOT_2006_CURRENT_LIMIT;

    config.kp = g_friction_control_debug.kp;
    config.ki = g_friction_control_debug.ki;
    config.kd = g_friction_control_debug.kd;
    config.integral_limit = g_friction_control_debug.integral_limit;
    config.output_limit = g_friction_control_debug.output_limit;
    for (i = 0U; i < ROBOT_FRICTION_MOTOR_COUNT; i++)
        friction_pid[i].config = config;
    friction_reset_pid();
    g_friction_control_debug.apply = false;
}

/** 立即发送零电流并复位 PID，用于停机或反馈离线保护。 */
static void friction_stop(void)
{
    friction_reset_pid();
    robot_shooter_set_friction_currents(0, 0);
    g_friction_control_status.running = false;
    for (uint32_t i = 0U; i < ROBOT_FRICTION_MOTOR_COUNT; i++)
        g_friction_control_status.output_current[i] = 0;
}

/**
 * @brief 摩擦轮同步速度闭环任务。
 * @note 全部反馈必须在线才允许输出，避免缺轮时继续发射。
 */
static void friction_task(void *argument)
{
    robot_motor_state_t motors;
    int16_t output[ROBOT_FRICTION_MOTOR_COUNT];
    uint32_t i;
    bool all_online;
    const float dt = (float)FRICTION_PERIOD_MS / 1000.0F;
    TickType_t last_wake = xTaskGetTickCount();
    (void)argument;

    for (;;)
    {
        if (g_friction_control_debug.apply)
            friction_apply_debug();
        if (g_friction_control_debug.reset)
        {
            friction_reset_pid();
            g_friction_control_debug.reset = false;
        }

        robot_motors_get_snapshot(&motors);
        all_online = true;
        for (i = 0U; i < ROBOT_FRICTION_MOTOR_COUNT; i++)
        {
            g_friction_control_status.target_rpm[i] =
                friction_limit_rpm(g_friction_control_command.target_rpm[i]);
            g_friction_control_status.measured_rpm[i] =
                motors.shooter.friction[i].speed_rpm;
            g_friction_control_status.online[i] =
                dji_motor_feedback_is_online(
                    &motors.shooter.friction[i],
                    osKernelGetTickCount(),
                    pdMS_TO_TICKS(FRICTION_TIMEOUT_MS));
            all_online = all_online && g_friction_control_status.online[i];
        }

        if (!g_friction_control_command.enabled || !all_online)
        {
            friction_stop();
        }
        else
        {
            for (i = 0U; i < ROBOT_FRICTION_MOTOR_COUNT; i++)
            {
                output[i] = (int16_t)pid_calculate(
                    &friction_pid[i],
                    (float)g_friction_control_status.target_rpm[i],
                    (float)motors.shooter.friction[i].speed_rpm,
                    dt);
                g_friction_control_status.output_current[i] = output[i];
                g_friction_control_status.error[i] = friction_pid[i].error;
                g_friction_control_status.p_output[i] = friction_pid[i].p_output;
                g_friction_control_status.i_output[i] = friction_pid[i].i_output;
                g_friction_control_status.d_output[i] = friction_pid[i].d_output;
            }
            robot_shooter_set_friction_currents(output[0], output[1]);
            g_friction_control_status.running = true;
        }
        g_friction_control_status.update_count++;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FRICTION_PERIOD_MS));
    }
}

/** 初始化摩擦轮控制命令、状态和全部相同的速度 PID。 */
void friction_control_init(void)
{
    const pid_config_t config = {4.0F, 0.8F, 0.0F,
                                 FRICTION_INTEGRAL_LIMIT,
                                 FRICTION_OUTPUT_LIMIT};
    uint32_t i;
    (void)memset(&g_friction_control_command, 0, sizeof(g_friction_control_command));
    (void)memset(&g_friction_control_status, 0, sizeof(g_friction_control_status));
    (void)memset(&g_friction_control_debug, 0, sizeof(g_friction_control_debug));
    g_friction_control_debug.kp = config.kp;
    g_friction_control_debug.ki = config.ki;
    g_friction_control_debug.integral_limit = config.integral_limit;
    g_friction_control_debug.output_limit = config.output_limit;
    for (i = 0U; i < ROBOT_FRICTION_MOTOR_COUNT; i++)
        pid_init(&friction_pid[i], &config);
}

/** 创建静态内存形式的摩擦轮控制任务。 */
bool friction_control_task_create(void)
{
    const osThreadAttr_t attr = {
        .name = "friction_ctrl", .cb_mem = &friction_control_task_control,
        .cb_size = sizeof(friction_control_task_control),
        .stack_mem = friction_control_task_stack,
        .stack_size = sizeof(friction_control_task_stack),
        .priority = osPriorityNormal};
    if (friction_control_task_handle != NULL)
        return true;
    friction_control_task_handle = osThreadNew(friction_task, NULL, &attr);
    return friction_control_task_handle != NULL;
}
