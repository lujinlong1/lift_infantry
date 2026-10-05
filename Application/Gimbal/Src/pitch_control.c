#include "pitch_control.h"

#include "cmsis_os2.h"
#include "dji_motor.h"
#include "FreeRTOS.h"
#include "pid.h"
#include "robot_motor_commands.h"
#include "robot_motors.h"
#include "task.h"

#include <string.h>

#define PITCH_STACK_WORDS 512U
#define PITCH_PERIOD_MS   5U
#define PITCH_TIMEOUT_MS  100U
#define PITCH_ECD_RANGE   8192.0F
#define PITCH_ECD_HALF    4096
#define DEG_TO_RAD        0.01745329252F
#define RAD_TO_DEG        57.29577951F
/* GM6020 反馈 speed_rpm 为输出侧转速：rpm * 360 / 60 = deg/s。 */
#define RPM_TO_DEG_S      6.0F

#define PITCH_MAX_ANGLE_ACCEL_LIMIT 100000.0F
#define PITCH_MAX_SPEED_DEG_S       3000.0F
#define PITCH_MAX_CURRENT           30000.0F
#define PITCH_MAX_RATE              720.0F

pitch_command_t g_pitch_control_command;
pitch_debug_t g_pitch_control_debug;
pitch_status_t g_pitch_control_status;

static pid_controller_t pitch_angle_pid;
static pid_controller_t pitch_speed_pid;
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

/** @brief 清空角度环和速度环的历史状态，防止模式切换时积分残留。 */
static void pitch_reset_pid(void)
{
    pid_reset(&pitch_angle_pid);
    pid_reset(&pitch_speed_pid);
}

/**
 * @brief 把累计编码器换算为相对上电零点的逻辑角度。
 * @note 先乘后除：先算 (total_ecd - zero) * 360 再除以 8192*减速比，避免整除丢精度。
 */
static float pitch_get_relative_angle(int32_t total_ecd)
{
    return ((float)g_pitch_control_debug.direction *
            (float)(total_ecd - g_pitch_control_status.zero_total_ecd) *
            360.0F) /
           (PITCH_ECD_RANGE * g_pitch_control_debug.gear_ratio);
}

/**
 * @brief 用单圈绝对编码器值计算同一坐标下的角度，用于绝对硬限位。
 * @note 与 yaw_get_front_angle 同法：超过半圈时反向补一圈，避免 0/8191 跳变。
 */
static float pitch_get_ecd_angle(uint16_t ecd)
{
    int32_t delta = (int32_t)ecd -
                    (int32_t)g_pitch_control_status.zero_total_ecd;

    if (delta > PITCH_ECD_HALF)
    {
        delta -= (int32_t)PITCH_ECD_RANGE;
    }
    else if (delta < -PITCH_ECD_HALF)
    {
        delta += (int32_t)PITCH_ECD_RANGE;
    }

    return (float)g_pitch_control_debug.direction * (float)delta * 360.0F /
           (PITCH_ECD_RANGE * g_pitch_control_debug.gear_ratio);
}

/** @brief 校验调试参数并一次性更新串级 PID 配置。 */
static void pitch_apply_debug(void)
{
    if (g_pitch_control_debug.angle_integral_limit < 0.0F)
        g_pitch_control_debug.angle_integral_limit = 0.0F;
    if (g_pitch_control_debug.speed_integral_limit < 0.0F)
        g_pitch_control_debug.speed_integral_limit = 0.0F;
    if (g_pitch_control_debug.max_speed_deg_s < 0.0F)
        g_pitch_control_debug.max_speed_deg_s = 0.0F;
    if (g_pitch_control_debug.max_speed_deg_s > PITCH_MAX_SPEED_DEG_S)
        g_pitch_control_debug.max_speed_deg_s = PITCH_MAX_SPEED_DEG_S;
    if (g_pitch_control_debug.max_current < 0.0F)
        g_pitch_control_debug.max_current = 0.0F;
    if (g_pitch_control_debug.max_current > PITCH_MAX_CURRENT)
        g_pitch_control_debug.max_current = PITCH_MAX_CURRENT;
    if (g_pitch_control_debug.gravity_torque > PITCH_MAX_CURRENT)
        g_pitch_control_debug.gravity_torque = PITCH_MAX_CURRENT;
    if (g_pitch_control_debug.gravity_torque < -PITCH_MAX_CURRENT)
        g_pitch_control_debug.gravity_torque = -PITCH_MAX_CURRENT;
    if (g_pitch_control_debug.gear_ratio <= 0.0F)
        g_pitch_control_debug.gear_ratio = 1.0F;
    g_pitch_control_debug.max_target_rate_deg_s = pitch_clamp(
        pitch_abs(g_pitch_control_debug.max_target_rate_deg_s),
        0.0F,
        PITCH_MAX_RATE);
    if ((g_pitch_control_debug.direction != 1) &&
        (g_pitch_control_debug.direction != -1))
    {
        g_pitch_control_debug.direction = 1;
    }

    pitch_angle_pid.config.kp = g_pitch_control_debug.angle_kp;
    pitch_angle_pid.config.ki = g_pitch_control_debug.angle_ki;
    pitch_angle_pid.config.kd = g_pitch_control_debug.angle_kd;
    pitch_angle_pid.config.integral_limit =
        g_pitch_control_debug.angle_integral_limit;
    /* 外环的输出就是内环的目标，所以外环上限必须是转速而不是电压。 */
    pitch_angle_pid.config.output_limit =
        g_pitch_control_debug.max_speed_deg_s;
    pitch_speed_pid.config.kp = g_pitch_control_debug.speed_kp;
    pitch_speed_pid.config.ki = g_pitch_control_debug.speed_ki;
    pitch_speed_pid.config.kd = g_pitch_control_debug.speed_kd;
    pitch_speed_pid.config.integral_limit =
        g_pitch_control_debug.speed_integral_limit;
    pitch_speed_pid.config.output_limit = g_pitch_control_debug.max_current;

    pitch_reset_pid();
    g_pitch_control_debug.apply = false;
    g_pitch_control_status.parameter_apply_count++;
}

/**
 * @brief 把相对软限位与绝对位置硬限位取交集，得到本周期真正生效的限位。
 * @note 两道保护都不合法时冻结在当前位置，绝不生成不可达目标。
 */
static void pitch_get_effective_angle_limits(
    float logical_angle,
    float *minimum_deg,
    float *maximum_deg)
{
    float angle_a;
    float angle_b;

    *minimum_deg = g_pitch_control_debug.min_angle_deg;
    *maximum_deg = g_pitch_control_debug.max_angle_deg;

    /* 软限位被写成空区间时，先退化成"冻结当前位置"。 */
    if (*minimum_deg > *maximum_deg)
    {
        *minimum_deg = logical_angle;
        *maximum_deg = logical_angle;
        return;
    }

    if (!g_pitch_control_debug.absolute_limit_enabled ||
        !g_pitch_control_status.zero_valid)
    {
        return;
    }

    angle_a = pitch_get_ecd_angle(g_pitch_control_debug.absolute_min_ecd);
    angle_b = pitch_get_ecd_angle(g_pitch_control_debug.absolute_max_ecd);
    if (angle_a > angle_b)
    {
        const float temporary = angle_a;
        angle_a = angle_b;
        angle_b = temporary;
    }

    if (*minimum_deg < angle_a)
    {
        *minimum_deg = angle_a;
    }
    if (*maximum_deg > angle_b)
    {
        *maximum_deg = angle_b;
    }
    if (*minimum_deg > *maximum_deg)
    {
        /* 交集为空：软限位与硬限位互相矛盾，冻结当前位置。 */
        *minimum_deg = logical_angle;
        *maximum_deg = logical_angle;
    }
}

/**
 * @brief Pitch 轴 5 ms 串级控制任务。
 * @details 角度来自累计编码器，速度来自 GM6020 的 speed_rpm；
 *          外环出目标转速、内环出电压量，再叠加重力前馈。
 */
static void pitch_task(void *argument)
{
    robot_motor_state_t motors;
    uint8_t previous_mode = PITCH_MODE_DISABLED;
    TickType_t last_wake = xTaskGetTickCount();

    (void)argument;

    for (;;)
    {
        const uint32_t now = osKernelGetTickCount();
        const float dt = (float)PITCH_PERIOD_MS / 1000.0F;
        uint8_t mode = g_pitch_control_command.mode;
        float logical_angle;
        float target_limit_angle;
        float effective_minimum_deg;
        float effective_maximum_deg;
        float target_speed = 0.0F;
        float current = 0.0F;

        if (g_pitch_control_debug.apply)
        {
            pitch_apply_debug();
        }
        if (g_pitch_control_debug.reset)
        {
            pitch_reset_pid();
            g_pitch_control_debug.reset = false;
        }

        robot_motors_get_snapshot(&motors);
        g_pitch_control_status.online = dji_motor_feedback_is_online(
            &motors.gimbal.pitch, now, pdMS_TO_TICKS(PITCH_TIMEOUT_MS));

        /* 首次收到有效反馈时把当前位置作为临时零点；也支持调试器手动重置。 */
        if (g_pitch_control_status.online &&
            (!g_pitch_control_status.zero_valid ||
             g_pitch_control_debug.set_zero))
        {
            g_pitch_control_status.zero_total_ecd =
                motors.gimbal.pitch.total_ecd;
            g_pitch_control_status.zero_valid = true;
            g_pitch_control_debug.set_zero = false;
            g_pitch_control_status.commanded_angle_deg = 0.0F;
            pitch_reset_pid();
        }

        logical_angle = pitch_get_relative_angle(motors.gimbal.pitch.total_ecd);
        g_pitch_control_status.raw_ecd = motors.gimbal.pitch.ecd;
        g_pitch_control_status.measured_angle_deg = logical_angle;
        g_pitch_control_status.measured_velocity_deg_s =
            (float)g_pitch_control_debug.direction *
            (float)motors.gimbal.pitch.speed_rpm * RPM_TO_DEG_S /
            g_pitch_control_debug.gear_ratio;
        pitch_get_effective_angle_limits(
            logical_angle,
            &effective_minimum_deg,
            &effective_maximum_deg);
        g_pitch_control_status.effective_min_angle_deg = effective_minimum_deg;
        g_pitch_control_status.effective_max_angle_deg = effective_maximum_deg;

        if (!g_pitch_control_status.online ||
            !g_pitch_control_status.zero_valid ||
            (mode > PITCH_MODE_SPEED))
        {
            mode = PITCH_MODE_DISABLED;
        }

        if (mode != previous_mode)
        {
            pitch_reset_pid();
            previous_mode = mode;
        }

        if (mode == PITCH_MODE_POSITION)
        {
            const float step =
                g_pitch_control_debug.max_target_rate_deg_s * dt;
            float logical_velocity = pitch_clamp(
                g_pitch_control_command.target_speed_deg_s,
                -g_pitch_control_debug.max_target_rate_deg_s,
                g_pitch_control_debug.max_target_rate_deg_s);

            /* 目标先夹到限位内，斜坡再朝它走——斜坡不能把目标带出限位。 */
            target_limit_angle = pitch_clamp(
                g_pitch_control_command.target_angle_deg,
                effective_minimum_deg,
                effective_maximum_deg);
            g_pitch_control_status.target_angle_deg = target_limit_angle;
            g_pitch_control_status.commanded_angle_deg += pitch_clamp(
                target_limit_angle - g_pitch_control_status.commanded_angle_deg,
                -step,
                step);
            g_pitch_control_status.commanded_angle_deg = pitch_clamp(
                g_pitch_control_status.commanded_angle_deg,
                effective_minimum_deg,
                effective_maximum_deg);

            /*
             * 位置模式的软限位语义是"夹目标"：到达限位后，任何继续向界外的
             * 速度前馈都必须清零，否则积分会把机构压在限位上。
             */
            if (((target_limit_angle <= effective_minimum_deg) &&
                 (logical_velocity < 0.0F)) ||
                ((target_limit_angle >= effective_maximum_deg) &&
                 (logical_velocity > 0.0F)))
            {
                logical_velocity = 0.0F;
            }

            /* 外环：角度误差 -> 目标转速。 */
            target_speed = pid_calculate(
                &pitch_angle_pid,
                g_pitch_control_status.commanded_angle_deg,
                logical_angle,
                dt);
            target_speed += logical_velocity;
        }
        else if (mode == PITCH_MODE_SPEED)
        {
            const float logical_velocity = pitch_clamp(
                g_pitch_control_command.target_speed_deg_s,
                -g_pitch_control_debug.max_target_rate_deg_s,
                g_pitch_control_debug.max_target_rate_deg_s);

            target_speed = logical_velocity;
            g_pitch_control_status.commanded_angle_deg = logical_angle;
            g_pitch_control_status.target_angle_deg =
                g_pitch_control_command.target_angle_deg;
            pid_reset(&pitch_angle_pid);
        }
        else
        {
            pitch_reset_pid();
            g_pitch_control_status.commanded_angle_deg = logical_angle;
            g_pitch_control_status.target_angle_deg =
                g_pitch_control_command.target_angle_deg;
        }

        /*
         * 速度模式下的软限位语义与位置模式不同：到界只禁止继续向外，
         * 仍然允许反向退出来。绝对硬限位再叠加同一判断。
         */
        if (mode == PITCH_MODE_SPEED)
        {
            if (((logical_angle <= effective_minimum_deg) &&
                 (target_speed < 0.0F)) ||
                ((logical_angle >= effective_maximum_deg) &&
                 (target_speed > 0.0F)))
            {
                target_speed = 0.0F;
            }
        }

        if (mode != PITCH_MODE_DISABLED)
        {
            /* 内环：目标转速 -> 电压量。 */
            current = pid_calculate(
                &pitch_speed_pid,
                target_speed,
                g_pitch_control_status.measured_velocity_deg_s,
                dt);
            current += g_pitch_control_debug.gravity_torque;
            current = pitch_clamp(
                current,
                -g_pitch_control_debug.max_current,
                g_pitch_control_debug.max_current);
        }
        else
        {
            /* 无力：电流为零，不主动保持。 */
            pid_reset(&pitch_speed_pid);
        }

        g_pitch_control_status.active_mode = mode;
        g_pitch_control_status.angle_error_deg = pitch_angle_pid.error;
        g_pitch_control_status.speed_error_deg_s = pitch_speed_pid.error;
        g_pitch_control_status.angle_p_output = pitch_angle_pid.p_output;
        g_pitch_control_status.angle_i_output = pitch_angle_pid.i_output;
        g_pitch_control_status.angle_d_output = pitch_angle_pid.d_output;
        g_pitch_control_status.speed_p_output = pitch_speed_pid.p_output;
        g_pitch_control_status.speed_i_output = pitch_speed_pid.i_output;
        g_pitch_control_status.speed_d_output = pitch_speed_pid.d_output;
        g_pitch_control_status.commanded_velocity_deg_s = target_speed;
        g_pitch_control_status.commanded_torque = current;
        g_pitch_control_status.output_current = (int16_t)current;

        /*
         * 绝对硬限位与位置目标无关的第二道保护：即使目标算错，
         * 只要单圈编码器已到机械死点，就禁止继续向界外走。
         */
        g_pitch_control_status.at_absolute_minimum = false;
        g_pitch_control_status.at_absolute_maximum = false;
        g_pitch_control_status.absolute_limit_active =
            g_pitch_control_debug.absolute_limit_enabled;
        if (g_pitch_control_debug.absolute_limit_enabled)
        {
            float absolute_minimum_deg =
                pitch_get_ecd_angle(g_pitch_control_debug.absolute_min_ecd);
            float absolute_maximum_deg =
                pitch_get_ecd_angle(g_pitch_control_debug.absolute_max_ecd);

            if (absolute_minimum_deg > absolute_maximum_deg)
            {
                const float temporary = absolute_minimum_deg;
                absolute_minimum_deg = absolute_maximum_deg;
                absolute_maximum_deg = temporary;
            }

            g_pitch_control_status.at_absolute_minimum =
                logical_angle <= absolute_minimum_deg;
            g_pitch_control_status.at_absolute_maximum =
                logical_angle >= absolute_maximum_deg;
            if (((logical_angle <= absolute_minimum_deg) &&
                 (current < 0.0F)) ||
                ((logical_angle >= absolute_maximum_deg) &&
                 (current > 0.0F)))
            {
                current = 0.0F;
                g_pitch_control_status.output_current = 0;
                g_pitch_control_status.commanded_torque = 0.0F;
            }
        }

        robot_gimbal_set_pitch_current((int16_t)current);

        g_pitch_control_status.update_count++;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PITCH_PERIOD_MS));
    }
}

/** @brief 初始化 Pitch 命令、串级 PID 参数、相对软限位与绝对硬限位。 */
void pitch_control_init(void)
{
    /*
     * 角度环：输出是目标转速，上限取 max_speed_deg_s。
     * 速度环：输出是电压量，上限取 max_current。
     * 两个上限的量纲不同，串级最容易搞反的就是这一处。
     */
    const pid_config_t angle_config = {
        80.0F, 0.0F, 0.0F, 0.0F, 1000.0F};
    const pid_config_t speed_config = {
        180.0F, 0.0F, 0.0F, 0.0F, 16000.0F};

    (void)memset(
        &g_pitch_control_command, 0, sizeof(g_pitch_control_command));
    (void)memset(&g_pitch_control_debug, 0, sizeof(g_pitch_control_debug));
    (void)memset(&g_pitch_control_status, 0, sizeof(g_pitch_control_status));

    pid_init(&pitch_angle_pid, &angle_config);
    pid_init(&pitch_speed_pid, &speed_config);

    g_pitch_control_debug.angle_kp = angle_config.kp;
    g_pitch_control_debug.angle_ki = angle_config.ki;
    g_pitch_control_debug.angle_kd = angle_config.kd;
    g_pitch_control_debug.angle_integral_limit = angle_config.integral_limit;
    g_pitch_control_debug.max_speed_deg_s = angle_config.output_limit;
    g_pitch_control_debug.speed_kp = speed_config.kp;
    g_pitch_control_debug.speed_ki = speed_config.ki;
    g_pitch_control_debug.speed_kd = speed_config.kd;
    g_pitch_control_debug.speed_integral_limit = speed_config.integral_limit;
    g_pitch_control_debug.max_current = speed_config.output_limit;

    /* 重力前馈待实机标定：先给 0，方向确认后再填。 */
    g_pitch_control_debug.gravity_torque = 0.0F;
    g_pitch_control_debug.max_target_rate_deg_s = 30.0F;
    g_pitch_control_debug.min_angle_deg = -30.0F;
    g_pitch_control_debug.max_angle_deg = 30.0F;

    /*
     * 绝对硬限位默认关闭：GM6020 的单圈 ecd 限位必须先标定机械死点，
     * 标定前填入任何数值都是猜测。标定步骤见调试手册。
     */
    g_pitch_control_debug.absolute_limit_enabled = false;
    g_pitch_control_debug.absolute_min_ecd = 0U;
    g_pitch_control_debug.absolute_max_ecd = 0U;

    g_pitch_control_debug.gear_ratio = 1.0F;
    g_pitch_control_debug.direction = 1;
    g_pitch_control_debug.set_zero = false;
    g_pitch_control_debug.apply = false;
    g_pitch_control_debug.reset = false;
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
