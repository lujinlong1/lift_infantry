#include "shooter_control.h"

#include "cmsis_os2.h"
#include "feed_control.h"
#include "friction_control.h"
#include "FreeRTOS.h"
#include "robot_motors.h"
#include "task.h"

#include <string.h>

#define SHOOTER_TASK_STACK_WORDS 512U
#define SHOOTER_PERIOD_MS 10U
#define SHOOTER_FAULT_SPINUP_TIMEOUT 1U
#define SHOOTER_FAULT_JAM_RETRY 2U
#define SHOOTER_FAULT_FEED_OFFLINE 3U

shooter_control_command_t g_shooter_control_command;
shooter_control_debug_t g_shooter_control_debug;
shooter_control_status_t g_shooter_control_status;

static osThreadId_t shooter_task_handle;
static StaticTask_t shooter_task_control;
static StackType_t shooter_task_stack[SHOOTER_TASK_STACK_WORDS];

/** 返回 32 位有符号数的绝对值，用于转速误差和堵转判断。 */
static int32_t shooter_abs_i32(int32_t value)
{
    return value < 0 ? -value : value;
}

/** 修正发射调试参数中的负值和非法方向，保证状态机输入有效。 */
static void shooter_validate_debug(void)
{
    uint32_t index;

    for (index = 0U; index < ROBOT_FRICTION_MOTOR_COUNT; index++)
    {
        if (g_shooter_control_debug.friction_rpm[index] < 0)
        {
            g_shooter_control_debug.friction_rpm[index] =
                (int16_t)-g_shooter_control_debug.friction_rpm[index];
        }
    }
    if (g_shooter_control_debug.trigger_rpm < 0)
    {
        g_shooter_control_debug.trigger_rpm =
            (int16_t)-g_shooter_control_debug.trigger_rpm;
    }
    if (g_shooter_control_debug.trigger_ecd_per_shot < 0)
    {
        g_shooter_control_debug.trigger_ecd_per_shot =
            -g_shooter_control_debug.trigger_ecd_per_shot;
    }
    if (g_shooter_control_debug.friction_ready_tolerance_rpm < 0)
    {
        g_shooter_control_debug.friction_ready_tolerance_rpm = 0;
    }
    if (g_shooter_control_debug.jam_speed_threshold_rpm < 0)
    {
        g_shooter_control_debug.jam_speed_threshold_rpm = 0;
    }
    if ((g_shooter_control_debug.feed_direction != 1) &&
        (g_shooter_control_debug.feed_direction != -1))
    {
        g_shooter_control_debug.feed_direction = 1;
    }
}

/** 切换发射状态并清零对外显示的状态持续时间。 */
static void shooter_set_state(shooter_state_t state, uint32_t tick)
{
    g_shooter_control_status.state = (uint8_t)state;
    g_shooter_control_status.state_elapsed_ms = 0U;
    (void)tick;
}

/** 设置全部摩擦轮的目标转速和总使能。 */
static void shooter_set_friction(bool enabled)
{
    uint32_t i;
    for (i = 0U; i < ROBOT_FRICTION_MOTOR_COUNT; i++)
        g_friction_control_command.target_rpm[i] =
            g_shooter_control_debug.friction_rpm[i];
    g_friction_control_command.enabled = enabled;
}

/** 按指定方向设置拨弹盘目标转速及使能。 */
static void shooter_set_feed(bool enabled, int8_t direction)
{
    g_feed_control_command.target_rpm[FEED_MOTOR_TRIGGER] =
        (int16_t)(direction * g_shooter_control_debug.trigger_rpm);
    g_feed_control_command.enabled[FEED_MOTOR_TRIGGER] = enabled;
}

/** 关闭摩擦轮与拨弹机构，并清除“可发射”标志。 */
static void shooter_stop_all(void)
{
    shooter_set_feed(false, 1);
    shooter_set_friction(false);
    g_shooter_control_status.ready = false;
}

/** 判断全部摩擦轮是否在线且速度误差进入允许范围。 */
static bool shooter_friction_is_stable(void)
{
    uint32_t i;
    for (i = 0U; i < ROBOT_FRICTION_MOTOR_COUNT; i++)
    {
        if (!g_friction_control_status.online[i] ||
            (shooter_abs_i32(
                 (int32_t)g_friction_control_status.target_rpm[i] -
                 g_friction_control_status.measured_rpm[i]) >
             g_shooter_control_debug.friction_ready_tolerance_rpm))
            return false;
    }
    return true;
}

/** 根据拨弹方向判断单发目标累计编码器值是否已经到达。 */
static bool shooter_single_target_reached(int32_t current)
{
    if (g_shooter_control_debug.feed_direction >= 0)
        return current >= g_shooter_control_status.trigger_target_ecd;
    return current <= g_shooter_control_status.trigger_target_ecd;
}

/**
 * @brief 发射机构上层状态机任务。
 * @note 状态顺序为停机、升速、就绪、发射、反转排堵和故障；底层电流闭环
 *       由 friction_control 与 feed_control 分别完成。
 */
static void shooter_task(void *argument)
{
    robot_motor_state_t motors;
    shooter_state_t state = SHOOTER_STATE_STOPPED;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t state_start = osKernelGetTickCount();
    uint32_t stable_start = 0U;
    uint32_t jam_start = 0U;
    bool fire_last = false;
    bool retry_single = false;
    (void)argument;

    for (;;)
    {
        uint32_t now = osKernelGetTickCount();
        bool fire_rising = g_shooter_control_command.fire && !fire_last;
        bool stable;
        bool feed_online;
        shooter_validate_debug();
        robot_motors_get_snapshot(&motors);
        feed_online = dji_motor_feedback_is_online(
            &motors.shooter.trigger,
            now,
            pdMS_TO_TICKS(100U));
        g_shooter_control_status.trigger_current_ecd =
            motors.shooter.trigger.total_ecd;
        g_shooter_control_status.state_elapsed_ms =
            (now - state_start) * portTICK_PERIOD_MS;

        if (!g_shooter_control_command.armed)
        {
            shooter_stop_all();
            state = SHOOTER_STATE_STOPPED;
            g_shooter_control_status.state = (uint8_t)state;
            g_shooter_control_status.fault_code = 0U;
            g_shooter_control_status.jam_retries = 0U;
            stable_start = 0U;
            jam_start = 0U;
            state_start = now;
            g_shooter_control_status.friction_stable = false;
        }
        else
        {
            switch (state)
            {
                case SHOOTER_STATE_STOPPED:
                    shooter_set_friction(true);
                    state = SHOOTER_STATE_SPINUP;
                    state_start = now;
                    shooter_set_state(state, now);
                    break;

                case SHOOTER_STATE_SPINUP:
                    shooter_set_friction(true);
                    shooter_set_feed(false, 1);
                    stable = shooter_friction_is_stable();
                    g_shooter_control_status.friction_stable = stable;
                    if (stable)
                    {
                        /* 必须连续稳定一段时间，避免转速瞬间经过目标值就判定就绪。 */
                        if (stable_start == 0U)
                            stable_start = now;
                        if ((now - stable_start) >=
                            pdMS_TO_TICKS(g_shooter_control_debug.ready_stable_ms))
                        {
                            state = SHOOTER_STATE_READY;
                            state_start = now;
                            g_shooter_control_status.ready = true;
                            shooter_set_state(state, now);
                        }
                    }
                    else
                        stable_start = 0U;
                    if ((now - state_start) >=
                        pdMS_TO_TICKS(g_shooter_control_debug.spinup_timeout_ms))
                    {
                        state = SHOOTER_STATE_FAULT;
                        g_shooter_control_status.fault_code =
                            SHOOTER_FAULT_SPINUP_TIMEOUT;
                    }
                    break;

                case SHOOTER_STATE_READY:
                    shooter_set_friction(true);
                    shooter_set_feed(false, 1);
                    if ((g_shooter_control_command.fire_mode ==
                         SHOOTER_FIRE_SINGLE) && fire_rising)
                    {
                        /* 单发只响应按键上升沿，并用累计编码器确定一发的行程。 */
                        int8_t direction =
                            g_shooter_control_debug.feed_direction >= 0 ? 1 : -1;
                        g_shooter_control_status.trigger_target_ecd =
                            motors.shooter.trigger.total_ecd +
                            direction *
                            g_shooter_control_debug.trigger_ecd_per_shot;
                        state = SHOOTER_STATE_FIRING_SINGLE;
                        g_shooter_control_status.ready = false;
                        retry_single = true;
                        state_start = now;
                        jam_start = 0U;
                    }
                    else if ((g_shooter_control_command.fire_mode ==
                              SHOOTER_FIRE_CONTINUOUS) &&
                             g_shooter_control_command.fire)
                    {
                        state = SHOOTER_STATE_FIRING_CONTINUOUS;
                        g_shooter_control_status.ready = false;
                        retry_single = false;
                        state_start = now;
                        jam_start = 0U;
                    }
                    break;

                case SHOOTER_STATE_FIRING_SINGLE:
                case SHOOTER_STATE_FIRING_CONTINUOUS:
                {
                    int8_t direction =
                        g_shooter_control_debug.feed_direction >= 0 ? 1 : -1;
                    shooter_set_friction(true);
                    if (!feed_online)
                    {
                        shooter_set_feed(false, direction);
                        state = SHOOTER_STATE_FAULT;
                        g_shooter_control_status.ready = false;
                        g_shooter_control_status.fault_code =
                            SHOOTER_FAULT_FEED_OFFLINE;
                        break;
                    }
                    else
                    {
                        shooter_set_feed(true, direction);
                    }
                    if ((state == SHOOTER_STATE_FIRING_SINGLE) &&
                        shooter_single_target_reached(
                            motors.shooter.trigger.total_ecd))
                    {
                        shooter_set_feed(false, direction);
                        g_shooter_control_status.shot_count++;
                        g_shooter_control_status.jam_retries = 0U;
                        state = SHOOTER_STATE_READY;
                        g_shooter_control_status.ready = true;
                        state_start = now;
                    }
                    else if ((state == SHOOTER_STATE_FIRING_CONTINUOUS) &&
                             !g_shooter_control_command.fire)
                    {
                        shooter_set_feed(false, direction);
                        g_shooter_control_status.jam_retries = 0U;
                        state = SHOOTER_STATE_READY;
                        g_shooter_control_status.ready = true;
                        state_start = now;
                    }
                    else if (shooter_abs_i32(
                                 motors.shooter.trigger.speed_rpm) <=
                             g_shooter_control_debug.jam_speed_threshold_rpm)
                    {
                        /* 有供弹指令但拨盘持续低速，认为可能卡弹并进入反转排堵。 */
                        if (jam_start == 0U)
                            jam_start = now;
                        if ((now - jam_start) >=
                            pdMS_TO_TICKS(g_shooter_control_debug.jam_timeout_ms))
                        {
                            g_shooter_control_status.jam_count++;
                            g_shooter_control_status.jam_retries++;
                            state = SHOOTER_STATE_REVERSING;
                            g_shooter_control_status.ready = false;
                            state_start = now;
                        }
                    }
                    else
                        jam_start = 0U;
                    break;
                }

                case SHOOTER_STATE_REVERSING:
                    /* 短时反转释放卡住的弹丸，超过重试次数后锁定为故障。 */
                    shooter_set_friction(true);
                    shooter_set_feed(
                        true,
                        g_shooter_control_debug.feed_direction >= 0 ? -1 : 1);
                    if ((now - state_start) >=
                        pdMS_TO_TICKS(g_shooter_control_debug.reverse_ms))
                    {
                        if (g_shooter_control_status.jam_retries >
                            g_shooter_control_debug.max_jam_retries)
                        {
                            state = SHOOTER_STATE_FAULT;
                            g_shooter_control_status.ready = false;
                            g_shooter_control_status.fault_code =
                                SHOOTER_FAULT_JAM_RETRY;
                        }
                        else
                        {
                            state = retry_single ?
                                SHOOTER_STATE_FIRING_SINGLE :
                                SHOOTER_STATE_FIRING_CONTINUOUS;
                            state_start = now;
                            jam_start = 0U;
                        }
                    }
                    break;

                case SHOOTER_STATE_FAULT:
                default:
                    shooter_stop_all();
                    break;
            }
        }

        g_shooter_control_status.state = (uint8_t)state;
        g_shooter_control_status.update_count++;
        fire_last = g_shooter_control_command.fire;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SHOOTER_PERIOD_MS));
    }
}

/** 初始化发射状态机及默认转速、单发行程和排堵参数。 */
void shooter_control_init(void)
{
    uint32_t i;
    (void)memset(&g_shooter_control_command, 0,
                 sizeof(g_shooter_control_command));
    (void)memset(&g_shooter_control_status, 0,
                 sizeof(g_shooter_control_status));
    (void)memset(&g_shooter_control_debug, 0,
                 sizeof(g_shooter_control_debug));
    for (i = 0U; i < ROBOT_FRICTION_MOTOR_COUNT; i++)
        g_shooter_control_debug.friction_rpm[i] = 3000;
    g_shooter_control_debug.trigger_rpm = 1000;
    g_shooter_control_debug.feed_direction = 1;
    g_shooter_control_debug.trigger_ecd_per_shot = 19661;
    g_shooter_control_debug.friction_ready_tolerance_rpm = 150;
    g_shooter_control_debug.jam_speed_threshold_rpm = 100;
    g_shooter_control_debug.ready_stable_ms = 300U;
    g_shooter_control_debug.spinup_timeout_ms = 5000U;
    g_shooter_control_debug.jam_timeout_ms = 300U;
    g_shooter_control_debug.reverse_ms = 200U;
    g_shooter_control_debug.max_jam_retries = 3U;
}

/** 创建静态内存形式的发射机构状态机任务。 */
bool shooter_control_task_create(void)
{
    const osThreadAttr_t attr = {
        .name = "shooter", .cb_mem = &shooter_task_control,
        .cb_size = sizeof(shooter_task_control),
        .stack_mem = shooter_task_stack,
        .stack_size = sizeof(shooter_task_stack),
        .priority = osPriorityNormal};
    if (shooter_task_handle != NULL)
        return true;
    shooter_task_handle = osThreadNew(shooter_task, NULL, &attr);
    return shooter_task_handle != NULL;
}
