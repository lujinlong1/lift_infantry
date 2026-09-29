#include "yaw_control.h"

#include "cmsis_os2.h"
#include "dji_motor.h"
#include "FreeRTOS.h"
#include "pid.h"
#include "robot_motor_commands.h"
#include "robot_motors.h"
#include "task.h"

#include <string.h>

#define YAW_TASK_STACK_WORDS 512U
#define YAW_PERIOD_MS 5U
#define YAW_TIMEOUT_MS 100U
#define YAW_ECD_RANGE 8192.0F

yaw_control_command_t g_yaw_control_command;
yaw_control_debug_t g_yaw_control_debug;
yaw_control_status_t g_yaw_control_status;

static pid_controller_t yaw_angle_pid;
static pid_controller_t yaw_speed_pid;
static osThreadId_t yaw_task_handle;
static StaticTask_t yaw_task_control;
static StackType_t yaw_task_stack[YAW_TASK_STACK_WORDS];

/** 将数值限制在给定的闭区间内。 */
static float yaw_clamp(float value, float minimum, float maximum)
{
    if (value > maximum)
        return maximum;
    if (value < minimum)
        return minimum;
    return value;
}

/** 清空角度环和速度环的历史状态，防止切换模式时积分残留。 */
static void yaw_reset_pid(void)
{
    pid_reset(&yaw_angle_pid);
    pid_reset(&yaw_speed_pid);
}

/** 校验调试参数并一次性更新串级 PID 配置。 */
static void yaw_apply_debug(void)
{
    if (g_yaw_control_debug.angle_integral_limit < 0.0F)
        g_yaw_control_debug.angle_integral_limit = 0.0F;
    if (g_yaw_control_debug.speed_integral_limit < 0.0F)
        g_yaw_control_debug.speed_integral_limit = 0.0F;
    if (g_yaw_control_debug.max_speed_rpm < 0.0F)
        g_yaw_control_debug.max_speed_rpm = 0.0F;
    if (g_yaw_control_debug.max_current < 0.0F)
        g_yaw_control_debug.max_current = 0.0F;
    if (g_yaw_control_debug.max_current > 30000.0F)
        g_yaw_control_debug.max_current = 30000.0F;
    if (g_yaw_control_debug.gear_ratio <= 0.0F)
        g_yaw_control_debug.gear_ratio = 1.0F;
    if ((g_yaw_control_debug.motor_direction != 1) &&
        (g_yaw_control_debug.motor_direction != -1))
        g_yaw_control_debug.motor_direction = 1;

    yaw_angle_pid.config.kp = g_yaw_control_debug.angle_kp;
    yaw_angle_pid.config.ki = g_yaw_control_debug.angle_ki;
    yaw_angle_pid.config.kd = g_yaw_control_debug.angle_kd;
    yaw_angle_pid.config.integral_limit =
        g_yaw_control_debug.angle_integral_limit;
    yaw_angle_pid.config.output_limit =
        g_yaw_control_debug.max_speed_rpm;
    yaw_speed_pid.config.kp = g_yaw_control_debug.speed_kp;
    yaw_speed_pid.config.ki = g_yaw_control_debug.speed_ki;
    yaw_speed_pid.config.kd = g_yaw_control_debug.speed_kd;
    yaw_speed_pid.config.integral_limit =
        g_yaw_control_debug.speed_integral_limit;
    yaw_speed_pid.config.output_limit = g_yaw_control_debug.max_current;
    yaw_reset_pid();
    g_yaw_control_debug.apply = false;
    g_yaw_control_status.parameter_apply_count++;
}

/** 根据累计编码器值计算相对上电零点的云台角度。 */
static float yaw_get_relative_angle(int32_t total_ecd)
{
    return ((float)g_yaw_control_debug.motor_direction *
            (float)(total_ecd - g_yaw_control_status.zero_total_ecd) *
            360.0F) /
           (YAW_ECD_RANGE * g_yaw_control_debug.gear_ratio);
}

/**
 * @brief 使用单圈绝对编码器计算枪管相对车头的角度，不受重新上电影响。
 * @param ecd 当前 GM6020 单圈编码器值。
 * @return 以标定车头为 0 度、范围为 -180~180 度的逻辑角度。
 */
static float yaw_get_front_angle(uint16_t ecd)
{
    int32_t delta = (int32_t)ecd -
                    (int32_t)g_yaw_control_debug.front_ecd;

    if (delta > (int32_t)(YAW_ECD_RANGE / 2.0F))
    {
        delta -= (int32_t)YAW_ECD_RANGE;
    }
    else if (delta < -(int32_t)(YAW_ECD_RANGE / 2.0F))
    {
        delta += (int32_t)YAW_ECD_RANGE;
    }

    return (float)g_yaw_control_debug.motor_direction *
           (float)delta * 360.0F /
           (YAW_ECD_RANGE * g_yaw_control_debug.gear_ratio);
}

/**
 * @brief Yaw 轴控制任务。
 * @note 角度模式采用“角度环输出目标转速、速度环输出电流”的串级控制；
 *       速度模式只使用速度环。电机离线或零点无效时强制无力。
 */
static void yaw_task(void *argument)
{
    const float dt = (float)YAW_PERIOD_MS / 1000.0F;
    robot_motor_state_t motors;
    uint8_t previous_mode = YAW_MODE_DISABLED;
    TickType_t last_wake = xTaskGetTickCount();
    (void)argument;

    for (;;)
    {
        uint32_t now = osKernelGetTickCount();
        uint8_t mode = g_yaw_control_command.mode;
        float target_speed = 0.0F;
        float current = 0.0F;

        if (g_yaw_control_debug.apply)
            yaw_apply_debug();
        if (g_yaw_control_debug.reset)
        {
            yaw_reset_pid();
            g_yaw_control_debug.reset = false;
        }

        robot_motors_get_snapshot(&motors);
        g_yaw_control_status.online = dji_motor_feedback_is_online(
            &motors.gimbal.yaw, now, pdMS_TO_TICKS(YAW_TIMEOUT_MS));

        if (g_yaw_control_status.online &&
            (!g_yaw_control_status.zero_valid ||
             g_yaw_control_debug.set_zero))
        {
            /* 第一次收到有效反馈时以当前位置为零点，也支持调试器手动重置零点。 */
            g_yaw_control_status.zero_total_ecd =
                motors.gimbal.yaw.total_ecd;
            g_yaw_control_status.zero_valid = true;
            g_yaw_control_debug.set_zero = false;
            yaw_reset_pid();
        }

        g_yaw_control_status.measured_angle_deg = yaw_get_relative_angle(
            motors.gimbal.yaw.total_ecd);
        g_yaw_control_status.raw_ecd = motors.gimbal.yaw.ecd;
        if (g_yaw_control_status.online &&
            g_yaw_control_debug.capture_front_ecd)
        {
            g_yaw_control_debug.front_ecd = motors.gimbal.yaw.ecd;
            g_yaw_control_debug.front_ecd_valid = true;
            g_yaw_control_debug.capture_front_ecd = false;
        }
        g_yaw_control_status.front_angle_deg =
            g_yaw_control_debug.front_ecd_valid ?
                yaw_get_front_angle(motors.gimbal.yaw.ecd) : 0.0F;
        g_yaw_control_status.measured_speed_rpm =
            (float)g_yaw_control_debug.motor_direction *
            (float)motors.gimbal.yaw.speed_rpm;

        if ((mode > YAW_MODE_ANGLE) ||
            !g_yaw_control_status.online ||
            !g_yaw_control_status.zero_valid)
            mode = YAW_MODE_DISABLED;

        if (mode != previous_mode)
        {
            yaw_reset_pid();
            previous_mode = mode;
        }

        if (mode == YAW_MODE_ANGLE)
        {
            /* 外环把角度误差换算为内环所需的目标转速。 */
            float target_angle =
                g_yaw_control_command.target_angle_deg;
            g_yaw_control_status.target_angle_deg = target_angle;
            target_speed = pid_calculate(
                &yaw_angle_pid,
                target_angle,
                g_yaw_control_status.measured_angle_deg,
                dt);
        }
        else if (mode == YAW_MODE_SPEED)
        {
            target_speed = yaw_clamp(
                g_yaw_control_command.target_speed_rpm,
                -g_yaw_control_debug.max_speed_rpm,
                g_yaw_control_debug.max_speed_rpm);

            pid_reset(&yaw_angle_pid);
        }
        else
        {
            yaw_reset_pid();
        }

        if (mode != YAW_MODE_DISABLED)
        {
            current = pid_calculate(
                &yaw_speed_pid,
                target_speed,
                g_yaw_control_status.measured_speed_rpm,
                dt);
            current *= (float)g_yaw_control_debug.motor_direction;
        }

        g_yaw_control_status.active_mode = mode;
        g_yaw_control_status.target_speed_rpm = target_speed;
        g_yaw_control_status.angle_error_deg = yaw_angle_pid.error;
        g_yaw_control_status.speed_error_rpm = yaw_speed_pid.error;
        g_yaw_control_status.angle_p_output = yaw_angle_pid.p_output;
        g_yaw_control_status.angle_i_output = yaw_angle_pid.i_output;
        g_yaw_control_status.angle_d_output = yaw_angle_pid.d_output;
        g_yaw_control_status.speed_p_output = yaw_speed_pid.p_output;
        g_yaw_control_status.speed_i_output = yaw_speed_pid.i_output;
        g_yaw_control_status.speed_d_output = yaw_speed_pid.d_output;
        g_yaw_control_status.output_current = (int16_t)current;
        robot_gimbal_set_yaw_current((int16_t)current);
        g_yaw_control_status.update_count++;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(YAW_PERIOD_MS));
    }
}

/** 初始化 Yaw 轴命令、状态、默认限位和串级 PID 参数。 */
void yaw_control_init(void)
{
    const pid_config_t angle_config = {
        80.0F, 0.0F, 0.0F, 100.0F, 1000.0F};
    const pid_config_t speed_config = {
        200.0F, 0.5F, 0.0F, 10000.0F, 10000.0F};
    (void)memset(&g_yaw_control_command, 0,
                 sizeof(g_yaw_control_command));
    (void)memset(&g_yaw_control_debug, 0,
                 sizeof(g_yaw_control_debug));
    (void)memset(&g_yaw_control_status, 0,
                 sizeof(g_yaw_control_status));
    pid_init(&yaw_angle_pid, &angle_config);
    pid_init(&yaw_speed_pid, &speed_config);
    g_yaw_control_debug.angle_kp = angle_config.kp;
    g_yaw_control_debug.angle_integral_limit = angle_config.integral_limit;
    g_yaw_control_debug.max_speed_rpm = angle_config.output_limit;
    g_yaw_control_debug.speed_kp = speed_config.kp;
    g_yaw_control_debug.speed_ki = speed_config.ki;
    g_yaw_control_debug.speed_integral_limit = speed_config.integral_limit;
    g_yaw_control_debug.max_current = speed_config.output_limit;
    g_yaw_control_debug.gear_ratio = 1.0F;
    g_yaw_control_debug.motor_direction = 1;
    /* 实车标定：枪管正对车头时 GM6020 单圈绝对编码器值为 7080。 */
    g_yaw_control_debug.front_ecd = 4728U;
    g_yaw_control_debug.front_ecd_valid = true;
    g_yaw_control_debug.capture_front_ecd = false;
}

/** 创建静态内存形式的 Yaw 轴周期控制任务。 */
bool yaw_control_task_create(void)
{
    const osThreadAttr_t attr = {
        .name = "yaw_ctrl", .cb_mem = &yaw_task_control,
        .cb_size = sizeof(yaw_task_control), .stack_mem = yaw_task_stack,
        .stack_size = sizeof(yaw_task_stack), .priority = osPriorityNormal};
    if (yaw_task_handle != NULL)
        return true;
    yaw_task_handle = osThreadNew(yaw_task, NULL, &attr);
    return yaw_task_handle != NULL;
}
