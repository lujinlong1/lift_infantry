#include "pitch_control.h"

#include "cmsis_os2.h"
#include "dm_motor.h"
#include "FreeRTOS.h"
#include "robot_motor_commands.h"
#include "robot_motors.h"
#include "task.h"

#include <string.h>

#define PITCH_STACK_WORDS 512U
#define PITCH_PERIOD_MS   5U
#define PITCH_TIMEOUT_MS  100U
#define DEG_TO_RAD        0.01745329252F
#define RAD_TO_DEG        57.29577951F
#define PITCH_MAX_KP      500.0F
#define PITCH_MAX_KD      5.0F
#define PITCH_MAX_TORQUE  18.0F
#define PITCH_MAX_RATE    720.0F
#define PITCH_DEFAULT_ABSOLUTE_MIN_RAD (-3.0F)
#define PITCH_DEFAULT_ABSOLUTE_MAX_RAD (-2.55F)

pitch_command_t g_pitch_control_command;
pitch_debug_t g_pitch_control_debug;
pitch_status_t g_pitch_control_status;

static osThreadId_t pitch_task_handle;
static StaticTask_t pitch_task_control;
static StackType_t pitch_task_stack[PITCH_STACK_WORDS];

/** @brief 返回 Pitch 参数的浮点绝对值。 */
static float pitch_abs(float value)
{
    return value < 0.0F ? -value : value;
}

/** @brief 将 Pitch 目标或参数限制在指定范围内。 */
static float pitch_clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

/**
 * @brief 将电机绝对位置限位换算为本次零点下的逻辑角度限位。
 * @note 最终范围取相对软限位与绝对位置限位的交集。
 */
static void pitch_get_effective_angle_limits(
    float *minimum_deg,
    float *maximum_deg)
{
    float angle_a;
    float angle_b;
    float absolute_minimum_deg;
    float absolute_maximum_deg;

    *minimum_deg = g_pitch_control_debug.min_angle_deg;
    *maximum_deg = g_pitch_control_debug.max_angle_deg;
    if (!g_pitch_control_debug.absolute_limit_enabled ||
        !g_pitch_control_status.zero_valid)
    {
        return;
    }

    angle_a =
        (g_pitch_control_debug.absolute_min_position_rad -
         g_pitch_control_status.zero_position_rad) *
        RAD_TO_DEG * (float)g_pitch_control_debug.direction;
    angle_b =
        (g_pitch_control_debug.absolute_max_position_rad -
         g_pitch_control_status.zero_position_rad) *
        RAD_TO_DEG * (float)g_pitch_control_debug.direction;
    absolute_minimum_deg = angle_a < angle_b ? angle_a : angle_b;
    absolute_maximum_deg = angle_a > angle_b ? angle_a : angle_b;

    if (*minimum_deg < absolute_minimum_deg)
    {
        *minimum_deg = absolute_minimum_deg;
    }
    if (*maximum_deg > absolute_maximum_deg)
    {
        *maximum_deg = absolute_maximum_deg;
    }
}

/**
 * @brief 校验 MIT 增益、力矩、速度、软限位和绝对硬限位参数。
 */
static void pitch_validate_debug(void)
{
    float temporary;

    g_pitch_control_debug.kp = pitch_clamp(
        g_pitch_control_debug.kp, 0.0F, PITCH_MAX_KP);
    g_pitch_control_debug.kd = pitch_clamp(
        g_pitch_control_debug.kd, 0.0F, PITCH_MAX_KD);
    g_pitch_control_debug.torque = pitch_clamp(
        g_pitch_control_debug.torque, -PITCH_MAX_TORQUE, PITCH_MAX_TORQUE);
    g_pitch_control_debug.gravity_torque = pitch_clamp(
        g_pitch_control_debug.gravity_torque,
        -PITCH_MAX_TORQUE,
        PITCH_MAX_TORQUE);
    g_pitch_control_debug.max_target_rate_deg_s = pitch_clamp(
        pitch_abs(g_pitch_control_debug.max_target_rate_deg_s),
        0.0F,
        PITCH_MAX_RATE);

    if (g_pitch_control_debug.min_angle_deg >
        g_pitch_control_debug.max_angle_deg)
    {
        temporary = g_pitch_control_debug.min_angle_deg;
        g_pitch_control_debug.min_angle_deg =
            g_pitch_control_debug.max_angle_deg;
        g_pitch_control_debug.max_angle_deg = temporary;
    }
    if (g_pitch_control_debug.absolute_min_position_rad >
        g_pitch_control_debug.absolute_max_position_rad)
    {
        temporary = g_pitch_control_debug.absolute_min_position_rad;
        g_pitch_control_debug.absolute_min_position_rad =
            g_pitch_control_debug.absolute_max_position_rad;
        g_pitch_control_debug.absolute_max_position_rad = temporary;
    }
    if ((g_pitch_control_debug.direction != 1) &&
        (g_pitch_control_debug.direction != -1))
    {
        g_pitch_control_debug.direction = 1;
    }
}

/**
 * @brief Pitch 轴 5 ms MIT 上层控制任务。
 * @details 完成零点标定、相对角度换算、目标斜坡、双层限位和无力零增益命令。
 */
static void pitch_task(void *argument)
{
    robot_motor_state_t motors;
    TickType_t last_wake = xTaskGetTickCount();

    (void)argument;

    for (;;)
    {
        const uint32_t now = osKernelGetTickCount();
        float logical_angle;
        float motor_target;
        float velocity = 0.0F;
        float kp = 0.0F;
        float kd = 0.0F;
        float torque = 0.0F;
        float effective_minimum_deg;
        float effective_maximum_deg;
        uint8_t mode = g_pitch_control_command.mode;

        pitch_validate_debug();
        robot_motors_get_snapshot(&motors);
        g_pitch_control_status.online = dm_motor_feedback_is_online(
            &motors.gimbal.pitch,
            now,
            pdMS_TO_TICKS(PITCH_TIMEOUT_MS));

        /* 首次收到有效反馈时把当前位置作为临时零点。 */
        if (g_pitch_control_status.online &&
            (!g_pitch_control_status.zero_valid ||
             g_pitch_control_debug.set_zero))
        {
            g_pitch_control_status.zero_position_rad =
                motors.gimbal.pitch.position;
            g_pitch_control_status.zero_valid = true;
            g_pitch_control_debug.set_zero = false;
            g_pitch_control_status.commanded_angle_deg = 0.0F;
        }

        logical_angle =
            (motors.gimbal.pitch.position -
             g_pitch_control_status.zero_position_rad) *
            RAD_TO_DEG * (float)g_pitch_control_debug.direction;
        g_pitch_control_status.measured_angle_deg = logical_angle;
        g_pitch_control_status.measured_position_rad =
            motors.gimbal.pitch.position;
        g_pitch_control_status.measured_velocity_deg_s =
            motors.gimbal.pitch.velocity * RAD_TO_DEG *
            (float)g_pitch_control_debug.direction;
        pitch_get_effective_angle_limits(
            &effective_minimum_deg, &effective_maximum_deg);
        if (effective_minimum_deg > effective_maximum_deg)
        {
            /* 参数交集为空时冻结在当前位置，禁止生成不可达目标。 */
            effective_minimum_deg = logical_angle;
            effective_maximum_deg = logical_angle;
        }
        g_pitch_control_status.effective_min_angle_deg =
            effective_minimum_deg;
        g_pitch_control_status.effective_max_angle_deg =
            effective_maximum_deg;

        if (!g_pitch_control_status.online ||
            !g_pitch_control_status.zero_valid ||
            (mode > PITCH_MODE_SPEED))
        {
            mode = PITCH_MODE_DISABLED;
        }

        if (mode == PITCH_MODE_POSITION)
        {
            const float target = pitch_clamp(
                g_pitch_control_command.target_angle_deg,
                effective_minimum_deg,
                effective_maximum_deg);
            const float step =
                g_pitch_control_debug.max_target_rate_deg_s *
                ((float)PITCH_PERIOD_MS / 1000.0F);
            float logical_velocity = pitch_clamp(
                g_pitch_control_command.target_speed_deg_s,
                -g_pitch_control_debug.max_target_rate_deg_s,
                g_pitch_control_debug.max_target_rate_deg_s);

            g_pitch_control_status.commanded_angle_deg += pitch_clamp(
                target - g_pitch_control_status.commanded_angle_deg,
                -step,
                step);
            g_pitch_control_status.commanded_angle_deg = pitch_clamp(
                g_pitch_control_status.commanded_angle_deg,
                effective_minimum_deg,
                effective_maximum_deg);
            if (((target <= effective_minimum_deg) &&
                 (logical_velocity < 0.0F)) ||
                ((target >= effective_maximum_deg) &&
                 (logical_velocity > 0.0F)))
            {
                logical_velocity = 0.0F;
            }
            velocity = logical_velocity * DEG_TO_RAD *
                       (float)g_pitch_control_debug.direction;
            kp = g_pitch_control_debug.kp;
            kd = g_pitch_control_debug.kd;
            torque = pitch_clamp(
                g_pitch_control_debug.torque +
                    g_pitch_control_debug.gravity_torque,
                -PITCH_MAX_TORQUE,
                PITCH_MAX_TORQUE);
        }
        else if (mode == PITCH_MODE_SPEED)
        {
            float logical_velocity = pitch_clamp(
                g_pitch_control_command.target_speed_deg_s,
                -g_pitch_control_debug.max_target_rate_deg_s,
                g_pitch_control_debug.max_target_rate_deg_s);

            /* 速度模式到软限位后只禁止向外运动，仍允许反向退出。 */
            if (((logical_angle <= g_pitch_control_debug.min_angle_deg) &&
                 (logical_velocity < 0.0F)) ||
                ((logical_angle >= g_pitch_control_debug.max_angle_deg) &&
                 (logical_velocity > 0.0F)))
            {
                logical_velocity = 0.0F;
            }

            velocity = logical_velocity * DEG_TO_RAD *
                       (float)g_pitch_control_debug.direction;

            /* 绝对位置是独立的第二道保护，不依赖上电零点是否准确。 */
            if (g_pitch_control_debug.absolute_limit_enabled &&
                (((motors.gimbal.pitch.position <=
                   g_pitch_control_debug.absolute_min_position_rad) &&
                  (velocity < 0.0F)) ||
                 ((motors.gimbal.pitch.position >=
                   g_pitch_control_debug.absolute_max_position_rad) &&
                  (velocity > 0.0F))))
            {
                velocity = 0.0F;
            }
            kd = g_pitch_control_debug.kd;
            torque = g_pitch_control_debug.torque;
            g_pitch_control_status.commanded_angle_deg = logical_angle;
        }
        else
        {
            /* 无力时保持位置字段连续，但 kp/kd/力矩全为 0，不产生主动保持力。 */
            g_pitch_control_status.commanded_angle_deg = logical_angle;
        }

        motor_target = g_pitch_control_status.zero_position_rad +
                       g_pitch_control_status.commanded_angle_deg *
                           DEG_TO_RAD *
                           (float)g_pitch_control_debug.direction;

        g_pitch_control_status.at_absolute_minimum =
            motors.gimbal.pitch.position <=
            g_pitch_control_debug.absolute_min_position_rad;
        g_pitch_control_status.at_absolute_maximum =
            motors.gimbal.pitch.position >=
            g_pitch_control_debug.absolute_max_position_rad;
        g_pitch_control_status.absolute_limit_active =
            g_pitch_control_debug.absolute_limit_enabled &&
            ((motor_target <
              g_pitch_control_debug.absolute_min_position_rad) ||
             (motor_target >
              g_pitch_control_debug.absolute_max_position_rad));
        if (g_pitch_control_debug.absolute_limit_enabled &&
            (mode == PITCH_MODE_POSITION))
        {
            motor_target = pitch_clamp(
                motor_target,
                g_pitch_control_debug.absolute_min_position_rad,
                g_pitch_control_debug.absolute_max_position_rad);
            /* 位置目标虽已限幅，速度前馈也必须禁止继续冲向绝对限位外。 */
            if (((motors.gimbal.pitch.position <=
                  g_pitch_control_debug.absolute_min_position_rad) &&
                 (velocity < 0.0F)) ||
                ((motors.gimbal.pitch.position >=
                  g_pitch_control_debug.absolute_max_position_rad) &&
                 (velocity > 0.0F)))
            {
                velocity = 0.0F;
            }
        }

        /* DM 只在 Enable 状态下返回反馈，因此业务 Disabled 不等于发送电机 Disable 特殊帧。 */
        robot_gimbal_request_pitch_enable(true);
        robot_gimbal_set_pitch_mit(
            motor_target, velocity, kp, kd, torque);

        g_pitch_control_status.active_mode = mode;
        g_pitch_control_status.target_angle_deg =
            g_pitch_control_command.target_angle_deg;
        g_pitch_control_status.commanded_velocity_deg_s =
            velocity * RAD_TO_DEG *
            (float)g_pitch_control_debug.direction;
        g_pitch_control_status.commanded_torque = torque;
        g_pitch_control_status.update_count++;

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PITCH_PERIOD_MS));
    }
}

/** @brief 初始化 Pitch 命令、MIT 参数、相对软限位和绝对硬限位。 */
void pitch_control_init(void)
{
    (void)memset(
        &g_pitch_control_command, 0, sizeof(g_pitch_control_command));
    (void)memset(&g_pitch_control_debug, 0, sizeof(g_pitch_control_debug));
    (void)memset(&g_pitch_control_status, 0, sizeof(g_pitch_control_status));

    g_pitch_control_debug.kp = 35.0F;
    g_pitch_control_debug.kd = 1.2F;
    g_pitch_control_debug.torque = 0.0F;
    g_pitch_control_debug.gravity_torque = -0.41F;
    g_pitch_control_debug.max_target_rate_deg_s = 30.0F;
    g_pitch_control_debug.min_angle_deg = -30.0F;
    g_pitch_control_debug.max_angle_deg = 30.0F;
    g_pitch_control_debug.absolute_limit_enabled = true;
    g_pitch_control_debug.absolute_min_position_rad =
        PITCH_DEFAULT_ABSOLUTE_MIN_RAD;
    g_pitch_control_debug.absolute_max_position_rad =
        PITCH_DEFAULT_ABSOLUTE_MAX_RAD;
    /* 枪管向上时电机 position 减小，因此逻辑正方向与电机方向相反。 */
    g_pitch_control_debug.direction = -1;
}

/** @brief 使用静态内存创建 Pitch 控制任务。 */
bool pitch_control_task_create(void)
{
    const osThreadAttr_t attributes = {
        .name = "pitch_ctrl",
        .cb_mem = &pitch_task_control,
        .cb_size = sizeof(pitch_task_control),
        .stack_mem = pitch_task_stack,
        .stack_size = sizeof(pitch_task_stack),
        .priority = osPriorityNormal
    };

    if (pitch_task_handle != NULL)
    {
        return true;
    }

    pitch_task_handle = osThreadNew(pitch_task, NULL, &attributes);
    return pitch_task_handle != NULL;
}
