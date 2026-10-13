#include "chassis_control.h"
#include "chassis_spin.h"

#include "cmsis_os2.h"
#include "dji_motor.h"
#include "FreeRTOS.h"
#include "pid.h"
#include "robot_motor_commands.h"
#include "robot_motors.h"
#include "task.h"

#include <stddef.h>
#include <string.h>

#define CHASSIS_CONTROL_TASK_STACK_WORDS  512U
#define CHASSIS_CONTROL_PERIOD_MS         5U
#define CHASSIS_CONTROL_TIMEOUT_MS        100U
#define CHASSIS_TARGET_RPM_LIMIT          9000
#define CHASSIS_PID_OUTPUT_LIMIT          3000.0F
#define CHASSIS_PID_INTEGRAL_LIMIT        12000.0F
#define CHASSIS_PID_KP                    4.0F
#define CHASSIS_PID_KI                    0.8F
#define CHASSIS_PID_KD                    0.0F
#define CHASSIS_RPM_PER_MPS               2404.946F
#define CHASSIS_ROTATION_RADIUS_M         0.2F

chassis_control_command_t g_chassis_control_command;
chassis_control_status_t g_chassis_control_status;
chassis_control_debug_t g_chassis_control_debug;

static pid_controller_t wheel_speed_pid[4];
static volatile bool stair_owned;
static volatile float stair_vx;
static volatile uint32_t stair_tick;
static osThreadId_t chassis_control_task_handle;
static StaticTask_t chassis_control_task_control;
static StackType_t chassis_control_task_stack[
    CHASSIS_CONTROL_TASK_STACK_WORDS];

/**
 * @brief 将单个底盘电机的目标转速限制在安全范围内。
 * @param rpm 希望转速，单位 RPM。
 * @return 限幅后的转速。
 */
static int16_t chassis_limit_rpm(int16_t rpm)
{
    if (rpm > CHASSIS_TARGET_RPM_LIMIT)
    {
        return CHASSIS_TARGET_RPM_LIMIT;
    }
    if (rpm < -CHASSIS_TARGET_RPM_LIMIT)
    {
        return -CHASSIS_TARGET_RPM_LIMIT;
    }
    return rpm;
}

/**
 * @brief 将 PID 浮点输出限幅并转换为 CAN 电流整数。
 * @param current PID 计算出的电流指令。
 * @return 符合 Debug 输出上限的电流值。
 */
static int16_t chassis_float_to_current(float current)
{
    const float limit = g_chassis_control_debug.output_limit;

    if (current > limit)
    {
        current = limit;
    }
    else if (current < -limit)
    {
        current = -limit;
    }
    else
    {
    }

    return (int16_t)current;
}

/**
 * @brief 清空四个车轮速度 PID 的积分和历史误差。
 * @note 模式切换、停机或重新应用参数时调用，防止旧积分造成电流突变。
 */
static void chassis_reset_all_pid(void)
{
    uint32_t index;

    for (index = 0U; index < 4U; index++)
    {
        pid_reset(&wheel_speed_pid[index]);
    }
}

/**
 * @brief 校验 Debug 中的底盘参数，并同步到四个 PID 控制器。
 * @note 处理完成后会自动清除 apply 标志，同时重置 PID。
 */
static void chassis_apply_debug_parameters(void)
{
    uint32_t index;

    if (g_chassis_control_debug.output_limit < 0.0F)
    {
        g_chassis_control_debug.output_limit = 0.0F;
    }
    if (g_chassis_control_debug.output_limit > 16384.0F)
    {
        g_chassis_control_debug.output_limit = 16384.0F;
    }
    if (g_chassis_control_debug.integral_limit < 0.0F)
    {
        g_chassis_control_debug.integral_limit = 0.0F;
    }
    if (g_chassis_control_debug.rpm_per_mps <= 0.0F)
    {
        g_chassis_control_debug.rpm_per_mps = CHASSIS_RPM_PER_MPS;
    }
    if (g_chassis_control_debug.rotation_radius_m <= 0.0F)
    {
        g_chassis_control_debug.rotation_radius_m = CHASSIS_ROTATION_RADIUS_M;
    }

    for (index = 0U; index < 4U; index++)
    {
        if ((g_chassis_control_debug.motor_direction[index] != 1) &&
            (g_chassis_control_debug.motor_direction[index] != -1))
        {
            g_chassis_control_debug.motor_direction[index] = 1;
        }
        wheel_speed_pid[index].config.kp = g_chassis_control_debug.kp;
        wheel_speed_pid[index].config.ki = g_chassis_control_debug.ki;
        wheel_speed_pid[index].config.kd = g_chassis_control_debug.kd;
        wheel_speed_pid[index].config.integral_limit =
            g_chassis_control_debug.integral_limit;
        wheel_speed_pid[index].config.output_limit =
            g_chassis_control_debug.output_limit;
    }

    chassis_reset_all_pid();
    g_chassis_control_debug.apply = false;
    g_chassis_control_status.parameter_apply_count++;
}

/**
 * @brief 根据直接 RPM 或麦克纳姆运动学计算四轮目标转速。
 * @param target 输出的四轮 RPM 数组，顺序为 0x201~0x204。
 * @param spin_wz_radps 小陀螺模块生成的角速度。
 */
static void chassis_calculate_targets(int16_t target[4], float spin_wz_radps)
{
    float wheel_mps[4];
    float wheel_rpm[4];
    float maximum = 0.0F;
    float scale = 1.0F;
    float value;
    uint32_t index;

    if (stair_owned || g_chassis_control_command.mode == CHASSIS_CONTROL_MODE_MECANUM)
    {
        const float vx = stair_owned ? stair_vx : g_chassis_control_command.vx_mps;
        const float vy = stair_owned ? 0.0F : g_chassis_control_command.vy_mps;
        const float commanded_wz = stair_owned ? 0.0F : g_chassis_spin_command.enabled ?
            spin_wz_radps : g_chassis_control_command.wz_radps;
        const float rotation = g_chassis_control_debug.rotation_radius_m *
                               commanded_wz;

        /* 电机顺序：0x201 右后、0x202 左后、0x203 左前、0x204 右前。 */
        wheel_mps[0] = vx - vy + rotation;
        wheel_mps[1] = vx + vy - rotation;
        wheel_mps[2] = vx - vy - rotation;
        wheel_mps[3] = vx + vy + rotation;

        for (index = 0U; index < 4U; index++)
        {
            wheel_rpm[index] = wheel_mps[index] *
                               g_chassis_control_debug.rpm_per_mps *
                               (float)g_chassis_control_debug.motor_direction[index];
            value = wheel_rpm[index] < 0.0F ? -wheel_rpm[index] : wheel_rpm[index];
            if (value > maximum)
            {
                maximum = value;
            }
        }

        /* 统一等比缩放四轮，保留原始运动方向，避免单轮截断导致轨迹畸变。 */
        if (maximum > (float)CHASSIS_TARGET_RPM_LIMIT)
        {
            scale = (float)CHASSIS_TARGET_RPM_LIMIT / maximum;
        }

        for (index = 0U; index < 4U; index++)
        {
            target[index] = (int16_t)(wheel_rpm[index] * scale);
        }
    }
    else
    {
        for (index = 0U; index < 4U; index++)
        {
            target[index] = chassis_limit_rpm(
                g_chassis_control_command.target_rpm[index]);
        }
    }
}

/**
 * @brief 将底盘输出归零并清空所有车轮 PID。
 * @note 任一底盘电机离线或底盘未使能时执行。
 */
static void chassis_stop_and_reset(void)
{
    uint32_t index;

    for (index = 0U; index < 4U; index++)
    {
        pid_reset(&wheel_speed_pid[index]);
        g_chassis_control_status.output_current[index] = 0;
    }

    g_chassis_control_status.running = false;
    g_chassis_control_status.measured_wz_radps = 0.0F;
    robot_chassis_set_currents(0, 0, 0, 0);
}

/**
 * @brief 根据四轮反馈转速反解底盘角速度，供 Yaw 对地稳定前馈使用。
 * @param measured_rpm 四个底盘电机的实际转速，顺序为 0x201~0x204。
 * @return 轮速估算的底盘角速度，单位 rad/s。
 * @note 该结果能反映电机实际响应，但无法识别轮胎打滑。
 */
static float chassis_calculate_measured_wz(
    const volatile int16_t measured_rpm[4])
{
    float wheel_mps[4];
    uint32_t index;

    if ((g_chassis_control_debug.rpm_per_mps <= 0.0F) ||
        (g_chassis_control_debug.rotation_radius_m <= 0.0F))
    {
        return 0.0F;
    }

    for (index = 0U; index < 4U; index++)
    {
        wheel_mps[index] =
            (float)measured_rpm[index] /
            (g_chassis_control_debug.rpm_per_mps *
             (float)g_chassis_control_debug.motor_direction[index]);
    }

    return (wheel_mps[0] - wheel_mps[1] -
            wheel_mps[2] + wheel_mps[3]) /
           (4.0F * g_chassis_control_debug.rotation_radius_m);
}

/**
 * @brief 底盘 5 ms 周期控制任务。
 * @param argument RTOS 任务参数，当前未使用。
 * @details 每周期获取电机快照、更新在线状态、解算轮速、运行速度 PID，最后只写入统一电机命令缓存。
 */
static void chassis_control_task(void *argument)
{
    const float dt_seconds =
        (float)CHASSIS_CONTROL_PERIOD_MS / 1000.0F;
    robot_motor_state_t motors;
    int16_t output[4];
    int16_t target[4];
    uint32_t current_tick;
    uint32_t index;
    bool all_online;
    float spin_wz_radps;
    TickType_t last_wake_tick;

    (void)argument;
    last_wake_tick = xTaskGetTickCount();

    for (;;)
    {
        robot_motors_get_snapshot(&motors);
        current_tick = osKernelGetTickCount();
        all_online = true;

        if (g_chassis_control_debug.apply)
        {
            chassis_apply_debug_parameters();
        }
        if (g_chassis_control_debug.reset)
        {
            chassis_reset_all_pid();
            g_chassis_control_debug.reset = false;
        }

        spin_wz_radps = chassis_spin_update(current_tick);
        chassis_calculate_targets(target, spin_wz_radps);

        for (index = 0U; index < 4U; index++)
        {
            g_chassis_control_status.target_rpm[index] = target[index];
            g_chassis_control_status.measured_rpm[index] =
                motors.chassis.wheel[index].speed_rpm;
            g_chassis_control_status.online[index] =
                dji_motor_feedback_is_online(
                    &motors.chassis.wheel[index],
                    current_tick,
                    pdMS_TO_TICKS(CHASSIS_CONTROL_TIMEOUT_MS));
            all_online = all_online &&
                         g_chassis_control_status.online[index];
        }
        g_chassis_control_status.measured_wz_radps = all_online ?
            chassis_calculate_measured_wz(
                g_chassis_control_status.measured_rpm) :
            0.0F;

        /* 底盘使能和四轮在线缺一不可，任一异常都不允许部分车轮继续输出。 */
        if ((!stair_owned && !g_chassis_control_command.enabled) || !all_online ||
            (stair_owned && stair_vx == 0.0F) ||
            (stair_owned && (uint32_t)(current_tick - stair_tick) >
             pdMS_TO_TICKS(CHASSIS_CONTROL_TIMEOUT_MS)))
        {
            chassis_stop_and_reset();
        }
        else
        {
            for (index = 0U; index < 4U; index++)
            {
                output[index] = chassis_float_to_current(pid_calculate(
                    &wheel_speed_pid[index],
                    (float)target[index],
                    (float)motors.chassis.wheel[index].speed_rpm,
                    dt_seconds));
                g_chassis_control_status.output_current[index] =
                    output[index];
                g_chassis_control_status.error[index] =
                    wheel_speed_pid[index].error;
                g_chassis_control_status.p_output[index] =
                    wheel_speed_pid[index].p_output;
                g_chassis_control_status.i_output[index] =
                    wheel_speed_pid[index].i_output;
                g_chassis_control_status.d_output[index] =
                    wheel_speed_pid[index].d_output;
            }

            robot_chassis_set_currents(
                output[0], output[1], output[2], output[3]);
            g_chassis_control_status.running = true;
        }

        g_chassis_control_status.update_count++;
        vTaskDelayUntil(
            &last_wake_tick,
            pdMS_TO_TICKS(CHASSIS_CONTROL_PERIOD_MS));
    }
}

/**
 * @brief 初始化底盘命令、状态、Debug 默认值和四个速度 PID。
 */
void chassis_control_init(void)
{
    const pid_config_t config = {
        .kp = CHASSIS_PID_KP,
        .ki = CHASSIS_PID_KI,
        .kd = CHASSIS_PID_KD,
        .integral_limit = CHASSIS_PID_INTEGRAL_LIMIT,
        .output_limit = CHASSIS_PID_OUTPUT_LIMIT
    };
    uint32_t index;

    stair_owned = false;
    stair_vx = 0.0F;
    stair_tick = 0U;

    (void)memset(&g_chassis_control_command, 0,
                 sizeof(g_chassis_control_command));
    (void)memset(&g_chassis_control_status, 0,
                 sizeof(g_chassis_control_status));
    (void)memset(&g_chassis_control_debug, 0,
                 sizeof(g_chassis_control_debug));

    g_chassis_control_debug.kp = CHASSIS_PID_KP;
    g_chassis_control_debug.ki = CHASSIS_PID_KI;
    g_chassis_control_debug.kd = CHASSIS_PID_KD;
    g_chassis_control_debug.integral_limit = CHASSIS_PID_INTEGRAL_LIMIT;
    g_chassis_control_debug.output_limit = CHASSIS_PID_OUTPUT_LIMIT;
    g_chassis_control_debug.rpm_per_mps = CHASSIS_RPM_PER_MPS;
    g_chassis_control_debug.rotation_radius_m = CHASSIS_ROTATION_RADIUS_M;
    g_chassis_control_debug.motor_direction[0] = 1;
    g_chassis_control_debug.motor_direction[1] = -1;
    g_chassis_control_debug.motor_direction[2] = -1;
    g_chassis_control_debug.motor_direction[3] = 1;
    chassis_spin_init();

    for (index = 0U; index < 4U; index++)
    {
        pid_init(&wheel_speed_pid[index], &config);
    }
}

/**
 * @brief 使用静态内存创建底盘控制任务。
 * @return 创建成功或任务已存在返回 true。
 */
bool chassis_control_task_create(void)
{
    const osThreadAttr_t attributes = {
        .name = "chassis_ctrl",
        .cb_mem = &chassis_control_task_control,
        .cb_size = sizeof(chassis_control_task_control),
        .stack_mem = chassis_control_task_stack,
        .stack_size = sizeof(chassis_control_task_stack),
        .priority = osPriorityNormal
    };

    if (chassis_control_task_handle != NULL)
    {
        return true;
    }

    chassis_control_task_handle = osThreadNew(
        chassis_control_task, NULL, &attributes);
    return chassis_control_task_handle != NULL;
}

/**
 * @brief 设置底盘总使能。
 * @param enable true 允许闭环输出，false 使四轮电流归零。
 */
void chassis_control_enable(bool enable)
{
    g_chassis_control_command.enabled = enable;
}

/**
 * @brief 设置四个车轮的直接 RPM 目标，并切换到直接轮速模式。
 */
void chassis_control_set_wheel_rpm(
    int16_t wheel1,
    int16_t wheel2,
    int16_t wheel3,
    int16_t wheel4)
{
    g_chassis_control_command.target_rpm[0] = chassis_limit_rpm(wheel1);
    g_chassis_control_command.target_rpm[1] = chassis_limit_rpm(wheel2);
    g_chassis_control_command.target_rpm[2] = chassis_limit_rpm(wheel3);
    g_chassis_control_command.target_rpm[3] = chassis_limit_rpm(wheel4);
    g_chassis_control_command.mode = CHASSIS_CONTROL_MODE_DIRECT_RPM;
}

/**
 * @brief 设置车体速度目标，并切换到麦克纳姆解算模式。
 * @param vx_mps 车体前后速度，向前为正，单位 m/s。
 * @param vy_mps 车体横移速度，单位 m/s。
 * @param wz_radps 车体角速度，单位 rad/s。
 */
void chassis_control_set_velocity(float vx_mps, float vy_mps, float wz_radps)
{
    g_chassis_control_command.vx_mps = vx_mps;
    g_chassis_control_command.vy_mps = vy_mps;
    g_chassis_control_command.wz_radps = wz_radps;
    g_chassis_control_command.mode = CHASSIS_CONTROL_MODE_MECANUM;
}

void chassis_control_stair_velocity(float vx_mps)
{
    taskENTER_CRITICAL();
    stair_vx = vx_mps;
    stair_tick = osKernelGetTickCount();
    stair_owned = true;
    taskEXIT_CRITICAL();
}

void chassis_control_stair_release(void)
{
    taskENTER_CRITICAL();
    g_chassis_control_command.enabled = false;
    g_chassis_control_command.vx_mps = 0.0F;
    g_chassis_control_command.vy_mps = 0.0F;
    g_chassis_control_command.wz_radps = 0.0F;
    stair_owned = false;
    taskEXIT_CRITICAL();
}
