#include "remote_control_task.h"

#include "chassis_control.h"
#include "chassis_spin.h"
#include "cmsis_os2.h"
#include "dbus_remote.h"
#include "FreeRTOS.h"
#include "pitch_control.h"
#include "shooter_control.h"
#include "task.h"
#include "yaw_control.h"

#include <math.h>
#include <string.h>

#define REMOTE_TASK_STACK_WORDS       512U
#define REMOTE_PERIOD_MS              10U
#define REMOTE_TIMEOUT_MS             100U
#define REMOTE_DEG_TO_RAD             0.01745329252F
#define REMOTE_RADPS_TO_RPM           9.549296586F
#define REMOTE_DEFAULT_FIRE_THRESHOLD 200
#define REMOTE_DEFAULT_LONG_FIRE_MS   400U
#define REMOTE_PITCH_LOWER_TIMEOUT_MS 10000U
#define REMOTE_PITCH_STOP_RATE_DEG_S  2.0F

remote_mapping_debug_t g_remote_mapping_debug;
remote_mapping_status_t g_remote_mapping_status;

static osThreadId_t remote_task_handle;
static StaticTask_t remote_task_control;
static StackType_t remote_task_stack[REMOTE_TASK_STACK_WORDS];
static float remote_pitch_target_angle_deg;
static bool remote_pitch_target_valid;
static float remote_pitch_filtered_rate_deg_s;
static float remote_pitch_ramped_rate_deg_s;
static bool remote_pitch_powerdown_active;
static bool remote_pitch_powerdown_complete;
static uint32_t remote_pitch_powerdown_start_tick;

/** 将浮点数限制在给定闭区间内。 */
static float remote_clamp(float value, float minimum, float maximum)
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
 * @brief 把角度折返到 -180~180 度，避免跨圈时产生错误的大角度差。
 */
static float remote_wrap_degrees(float angle_deg)
{
    while (angle_deg > 180.0F)
    {
        angle_deg -= 360.0F;
    }
    while (angle_deg < -180.0F)
    {
        angle_deg += 360.0F;
    }
    return angle_deg;
}

/**
 * @brief 应用安装方向和车头校准偏置，得到以车头为 0 度的 Yaw 相对角度。
 */
static float remote_get_calibrated_gimbal_angle(float measured_angle_deg)
{
    return remote_wrap_degrees(
        (float)g_remote_mapping_debug.gimbal_frame_direction *
            measured_angle_deg +
        g_remote_mapping_debug.gimbal_frame_offset_deg);
}

/**
 * @brief 对输入执行一阶低通滤波，削弱遥控摇杆抖动和采样噪声。
 * @param previous 上一周期滤波输出。
 * @param input 当前周期原始输入。
 * @param tau_s 滤波时间常数；越大越平滑，但响应也越慢。
 * @param dt_s 本次计算距离上次计算的时间。
 */
static float remote_low_pass(
    float previous,
    float input,
    float tau_s,
    float dt_s)
{
    float alpha;

    if ((tau_s <= 0.0F) || (dt_s <= 0.0F))
    {
        return input;
    }
    alpha = dt_s / (tau_s + dt_s);
    return previous + alpha * (input - previous);
}

/**
 * @brief 限制信号每秒最大变化量，形成带加减速过程的斜坡输出。
 * @param previous 上一周期斜坡输出。
 * @param target 本周期希望达到的目标。
 * @param rate_limit_per_s 每秒允许变化的最大绝对值。
 * @param dt_s 本次计算距离上次计算的时间。
 */
static float remote_slew_rate_limit(
    float previous,
    float target,
    float rate_limit_per_s,
    float dt_s)
{
    const float maximum_step = rate_limit_per_s * dt_s;
    float difference = target - previous;

    if ((rate_limit_per_s <= 0.0F) || (dt_s <= 0.0F))
    {
        return target;
    }
    if (difference > maximum_step)
    {
        difference = maximum_step;
    }
    else if (difference < -maximum_step)
    {
        difference = -maximum_step;
    }
    return previous + difference;
}

/**
 * @brief 根据距离边界的剩余角度限制速度，使 Pitch 在限位前开始平滑减速。
 * @param requested_rate_deg_s 滤波后的期望角速度，正值朝最大角度运动。
 * @param target_angle_deg 当前积分得到的目标角度。
 * @param minimum_angle_deg 机械允许的最小逻辑角度。
 * @param maximum_angle_deg 机械允许的最大逻辑角度。
 * @param deceleration_deg_s2 用于停车的角减速度绝对值。
 * @return 满足 v^2 <= 2*a*s 的角速度；真正到达边界时返回 0。
 */
static float remote_limit_pitch_rate_near_boundary(
    float requested_rate_deg_s,
    float target_angle_deg,
    float minimum_angle_deg,
    float maximum_angle_deg,
    float deceleration_deg_s2)
{
    float remaining_angle_deg;
    float maximum_rate_deg_s;

    if (deceleration_deg_s2 <= 0.0F)
    {
        return requested_rate_deg_s;
    }

    if (requested_rate_deg_s > 0.0F)
    {
        remaining_angle_deg = maximum_angle_deg - target_angle_deg;
        if (remaining_angle_deg <= 0.0F)
        {
            return 0.0F;
        }
        maximum_rate_deg_s = sqrtf(
            2.0F * deceleration_deg_s2 * remaining_angle_deg);
        return remote_clamp(
            requested_rate_deg_s,
            0.0F,
            maximum_rate_deg_s);
    }

    if (requested_rate_deg_s < 0.0F)
    {
        remaining_angle_deg = target_angle_deg - minimum_angle_deg;
        if (remaining_angle_deg <= 0.0F)
        {
            return 0.0F;
        }
        maximum_rate_deg_s = sqrtf(
            2.0F * deceleration_deg_s2 * remaining_angle_deg);
        return remote_clamp(
            requested_rate_deg_s,
            -maximum_rate_deg_s,
            0.0F);
    }

    return 0.0F;
}

/**
 * @brief 对摇杆通道执行中位死区处理。
 * @return 死区内返回 0，死区外保留原始值。
 */
static int16_t remote_deadband(int16_t value)
{
    if ((value < g_remote_mapping_debug.deadband) &&
        (value > -g_remote_mapping_debug.deadband))
    {
        return 0;
    }
    return value;
}

/**
 * @brief 遥控失联或退出遥控接管时，撤销底盘、云台和发射权限。
 */
static void remote_stop_all(void)
{
    g_chassis_control_command.enabled = false;
    g_chassis_spin_command.enabled = false;
    g_yaw_control_command.mode = YAW_MODE_DISABLED;
    g_pitch_control_command.mode = PITCH_MODE_DISABLED;
    g_shooter_control_command.armed = false;
    g_shooter_control_command.fire = false;
    remote_pitch_target_valid = false;
    remote_pitch_filtered_rate_deg_s = 0.0F;
    remote_pitch_ramped_rate_deg_s = 0.0F;
    remote_pitch_powerdown_active = false;
    remote_pitch_powerdown_complete = false;

    g_remote_mapping_status.controlling = false;
    g_remote_mapping_status.chassis_powered = false;
    g_remote_mapping_status.gimbal_powered = false;
    g_remote_mapping_status.gimbal_lowering = false;
    g_remote_mapping_status.spin_active = false;
    g_remote_mapping_status.chassis_follow_active = false;
    g_remote_mapping_status.chassis_follow_wz_radps = 0.0F;
    g_remote_mapping_status.yaw_chassis_compensation_rpm = 0.0F;
    g_remote_mapping_status.shooter_armed = false;
    g_remote_mapping_status.fire_active = false;
}

/**
 * @brief 根据 Yaw 相对底盘的角度计算底盘跟随角速度。
 * @param gimbal_angle_deg Yaw 相对底盘角度，单位度。
 * @return 底盘角速度命令，单位 rad/s。
 */
static float remote_calculate_chassis_follow_wz(float gimbal_angle_deg)
{
    const float calibrated_angle_deg =
        remote_get_calibrated_gimbal_angle(gimbal_angle_deg);
    float angle_rad;
    float maximum_wz_radps =
        g_remote_mapping_debug.chassis_follow_max_wz_radps;

    if (fabsf(calibrated_angle_deg) <=
        fabsf(g_remote_mapping_debug.chassis_follow_deadband_deg))
    {
        return 0.0F;
    }
    if (maximum_wz_radps < 0.0F)
    {
        maximum_wz_radps = -maximum_wz_radps;
    }

    angle_rad = calibrated_angle_deg * REMOTE_DEG_TO_RAD;
    return remote_clamp(
        g_remote_mapping_debug.chassis_follow_kp * angle_rad,
        -maximum_wz_radps,
        maximum_wz_radps);
}

/**
 * @brief 云台切换到无力档时，先让 Pitch 缓慢下降到逻辑最低限位，再关闭电机。
 * @param current_tick 当前系统节拍，用于限制缓降最长持续时间。
 * @note Yaw 和发射机构由调用者立即关闭；失联保护不会调用本函数。
 */
static void remote_lower_pitch_before_powerdown(uint32_t current_tick)
{
    const float dt_s = (float)REMOTE_PERIOD_MS / 1000.0F;
    float minimum_angle_deg =
        g_pitch_control_status.effective_min_angle_deg;
    float lowering_rate_deg_s =
        g_remote_mapping_debug.pitch_powerdown_rate_deg_s;
    float lowering_accel_deg_s2 =
        g_remote_mapping_debug.pitch_powerdown_accel_deg_s2;
    float tolerance_deg =
        g_remote_mapping_debug.pitch_powerdown_tolerance_deg;
    float limited_rate_deg_s;
    bool reached_bottom;
    bool timed_out;

    if (remote_pitch_powerdown_complete)
    {
        g_pitch_control_command.mode = PITCH_MODE_DISABLED;
        g_remote_mapping_status.gimbal_powered = false;
        g_remote_mapping_status.gimbal_lowering = false;
        return;
    }

    if (!g_pitch_control_status.online ||
        !g_pitch_control_status.zero_valid)
    {
        g_pitch_control_command.mode = PITCH_MODE_DISABLED;
        remote_pitch_powerdown_active = false;
        remote_pitch_powerdown_complete = true;
        g_remote_mapping_status.gimbal_powered = false;
        g_remote_mapping_status.gimbal_lowering = false;
        return;
    }

    if (minimum_angle_deg > g_pitch_control_status.effective_max_angle_deg)
    {
        minimum_angle_deg = g_pitch_control_debug.min_angle_deg;
    }
    if (lowering_rate_deg_s < 0.0F)
    {
        lowering_rate_deg_s = -lowering_rate_deg_s;
    }
    if (tolerance_deg < 0.0F)
    {
        tolerance_deg = -tolerance_deg;
    }

    if (!remote_pitch_powerdown_active)
    {
        remote_pitch_target_angle_deg =
            g_pitch_control_status.measured_angle_deg;
        remote_pitch_filtered_rate_deg_s = 0.0F;
        remote_pitch_ramped_rate_deg_s = 0.0F;
        remote_pitch_powerdown_start_tick = current_tick;
        remote_pitch_powerdown_active = true;
    }

    limited_rate_deg_s = remote_limit_pitch_rate_near_boundary(
        -lowering_rate_deg_s,
        remote_pitch_target_angle_deg,
        minimum_angle_deg,
        g_pitch_control_status.effective_max_angle_deg,
        lowering_accel_deg_s2);
    remote_pitch_ramped_rate_deg_s = remote_slew_rate_limit(
        remote_pitch_ramped_rate_deg_s,
        limited_rate_deg_s,
        lowering_accel_deg_s2,
        dt_s);
    remote_pitch_target_angle_deg +=
        remote_pitch_ramped_rate_deg_s * dt_s;
    remote_pitch_target_angle_deg = remote_clamp(
        remote_pitch_target_angle_deg,
        minimum_angle_deg,
        g_pitch_control_status.effective_max_angle_deg);

    reached_bottom =
        (g_pitch_control_status.measured_angle_deg <=
         minimum_angle_deg + tolerance_deg) &&
        (fabsf(g_pitch_control_status.measured_velocity_deg_s) <=
         REMOTE_PITCH_STOP_RATE_DEG_S);
    timed_out =
        (uint32_t)(current_tick - remote_pitch_powerdown_start_tick) >=
        pdMS_TO_TICKS(REMOTE_PITCH_LOWER_TIMEOUT_MS);

    if (reached_bottom || timed_out)
    {
        g_pitch_control_command.mode = PITCH_MODE_DISABLED;
        remote_pitch_powerdown_active = false;
        remote_pitch_powerdown_complete = true;
        remote_pitch_target_valid = false;
        g_remote_mapping_status.gimbal_powered = false;
        g_remote_mapping_status.gimbal_lowering = false;
        return;
    }

    g_pitch_control_command.target_angle_deg =
        remote_pitch_target_angle_deg;
    g_pitch_control_command.target_speed_deg_s =
        remote_pitch_ramped_rate_deg_s;
    g_pitch_control_command.mode = PITCH_MODE_POSITION;
    remote_pitch_target_valid = false;
    g_remote_mapping_status.gimbal_powered = true;
    g_remote_mapping_status.gimbal_lowering = true;
}

/**
 * @brief 将左摇杆的云台坐标系平移命令转换为底盘坐标系速度。
 * @param remote 本周期的完整 DBUS 快照。
 * @param gimbal_angle_deg 云台相对底盘的 Yaw 角度。
 */
static void remote_set_chassis(
    const dbus_remote_state_t *remote,
    float gimbal_angle_deg,
    float wz_radps)
{
    const float forward =
        (float)remote_deadband(remote->channel[3]) *
        g_remote_mapping_debug.chassis_vx_scale;
    const float lateral =
        (float)remote_deadband(remote->channel[2]) *
        g_remote_mapping_debug.chassis_vy_scale;
    const float transform_angle =
        remote_get_calibrated_gimbal_angle(gimbal_angle_deg) *
        REMOTE_DEG_TO_RAD;
    const float cosine = cosf(transform_angle);
    const float sine = sinf(transform_angle);
    /* 旋转坐标后，摇杆“向前”始终对应当前枪口方向。 */
    const float chassis_vx = forward * cosine - lateral * sine;
    const float chassis_vy = forward * sine + lateral * cosine;

    chassis_control_set_velocity(chassis_vx, chassis_vy, wz_radps);
    chassis_control_enable(true);
    g_remote_mapping_status.chassis_vx_mps = chassis_vx;
    g_remote_mapping_status.chassis_vy_mps = chassis_vy;
    g_remote_mapping_status.chassis_powered = true;
}

/** @brief 将右摇杆通道映射为 Yaw/Pitch 速度命令。 */
static void remote_set_gimbal(
    const dbus_remote_state_t *remote,
    float chassis_wz_radps)
{
    const float dt_s = (float)REMOTE_PERIOD_MS / 1000.0F;
    const float pitch_raw_rate_deg_s =
        (float)remote_deadband(remote->channel[1]) *
        g_remote_mapping_debug.pitch_speed_scale;
    float effective_minimum_deg =
        g_pitch_control_status.effective_min_angle_deg;
    float effective_maximum_deg =
        g_pitch_control_status.effective_max_angle_deg;
    float mechanical_minimum_deg;
    float mechanical_maximum_deg;
    float tracking_error_deg =
        g_remote_mapping_debug.pitch_max_tracking_error_deg;
    float limited_rate_deg_s;

    const float yaw_chassis_compensation_rpm =
        (float)g_remote_mapping_debug.yaw_chassis_compensation_direction *
        g_remote_mapping_debug.yaw_chassis_compensation_gain *
        chassis_wz_radps * REMOTE_RADPS_TO_RPM;

    g_yaw_control_command.target_speed_rpm =
        (float)remote_deadband(remote->channel[0]) *
            g_remote_mapping_debug.yaw_speed_scale +
        yaw_chassis_compensation_rpm;
    g_yaw_control_command.mode = YAW_MODE_SPEED;
    g_remote_mapping_status.yaw_chassis_compensation_rpm =
        yaw_chassis_compensation_rpm;

    if (!remote_pitch_target_valid)
    {
        /* 从当前位置接管，避免云台使能瞬间跳向上一次或默认目标。 */
        remote_pitch_target_angle_deg =
            g_pitch_control_status.measured_angle_deg;
        remote_pitch_filtered_rate_deg_s = 0.0F;
        remote_pitch_ramped_rate_deg_s = 0.0F;
        remote_pitch_target_valid = true;
    }

    /* 先滤掉摇杆毛刺，再限制加减速度，避免启动和停止时产生突变。 */
    remote_pitch_filtered_rate_deg_s = remote_low_pass(
        remote_pitch_filtered_rate_deg_s,
        pitch_raw_rate_deg_s,
        g_remote_mapping_debug.pitch_filter_tau_s,
        dt_s);
    if (!g_pitch_control_status.zero_valid ||
        (effective_minimum_deg > effective_maximum_deg))
    {
        effective_minimum_deg = g_pitch_control_debug.min_angle_deg;
        effective_maximum_deg = g_pitch_control_debug.max_angle_deg;
    }
    if (tracking_error_deg < 0.0F)
    {
        tracking_error_deg = -tracking_error_deg;
    }

    mechanical_minimum_deg = effective_minimum_deg;
    mechanical_maximum_deg = effective_maximum_deg;

    /*
     * 先按停车距离压低目标速度，再交给原有斜坡限制器。
     * 靠近机械限位时会提前减速，避免在边界处从高速瞬间切到 0。
     */
    limited_rate_deg_s = remote_limit_pitch_rate_near_boundary(
        remote_pitch_filtered_rate_deg_s,
        remote_pitch_target_angle_deg,
        mechanical_minimum_deg,
        mechanical_maximum_deg,
        g_remote_mapping_debug.pitch_accel_limit_deg_s2);
    remote_pitch_ramped_rate_deg_s = remote_slew_rate_limit(
        remote_pitch_ramped_rate_deg_s,
        limited_rate_deg_s,
        g_remote_mapping_debug.pitch_accel_limit_deg_s2,
        dt_s);

    /* 将平滑后的角速度积分为位置目标，同时作为 MIT 速度前馈。 */
    remote_pitch_target_angle_deg +=
        remote_pitch_ramped_rate_deg_s * dt_s;

    /* 目标既不能超过真实机械范围，也不能长期领先实际位置。 */
    effective_minimum_deg = remote_clamp(
        g_pitch_control_status.measured_angle_deg - tracking_error_deg,
        effective_minimum_deg,
        effective_maximum_deg);
    effective_maximum_deg = remote_clamp(
        g_pitch_control_status.measured_angle_deg + tracking_error_deg,
        effective_minimum_deg,
        effective_maximum_deg);
    remote_pitch_target_angle_deg = remote_clamp(
        remote_pitch_target_angle_deg,
        effective_minimum_deg,
        effective_maximum_deg);

    /*
     * 只有真实机械边界才清除速度状态。5度跟踪窗口只限制位置领先量，
     * 若在跟踪窗口边缘清零，会造成速度前馈在0和目标值之间反复跳变。
     */
    if (((remote_pitch_target_angle_deg <= mechanical_minimum_deg) &&
         (remote_pitch_ramped_rate_deg_s < 0.0F)) ||
        ((remote_pitch_target_angle_deg >= mechanical_maximum_deg) &&
         (remote_pitch_ramped_rate_deg_s > 0.0F)))
    {
        remote_pitch_filtered_rate_deg_s = 0.0F;
        remote_pitch_ramped_rate_deg_s = 0.0F;
    }

    g_pitch_control_command.target_angle_deg =
        remote_pitch_target_angle_deg;
    g_pitch_control_command.target_speed_deg_s =
        remote_pitch_ramped_rate_deg_s;
    g_pitch_control_command.mode = PITCH_MODE_POSITION;
    g_remote_mapping_status.pitch_raw_rate_deg_s =
        pitch_raw_rate_deg_s;
    g_remote_mapping_status.pitch_filtered_rate_deg_s =
        remote_pitch_filtered_rate_deg_s;
    g_remote_mapping_status.pitch_ramped_rate_deg_s =
        remote_pitch_ramped_rate_deg_s;
    g_remote_mapping_status.gimbal_powered = true;
    g_remote_mapping_status.gimbal_lowering = false;
}

/**
 * @brief 根据拨轮保持时间生成单发或连发命令。
 * @note 刚越过阈值时是单发，保持超过配置时间后切换为连发。
 */
static void remote_set_shooter(
    const dbus_remote_state_t *remote,
    uint32_t current_tick,
    uint32_t *fire_start_tick,
    bool *fire_was_active)
{
    const bool fire_active =
        remote->channel[4] > g_remote_mapping_debug.fire_dial_threshold;

    g_shooter_control_command.armed = true;
    g_remote_mapping_status.shooter_armed = true;
    g_remote_mapping_status.fire_active = fire_active;

    if (fire_active && !*fire_was_active)
    {
        *fire_start_tick = current_tick;
    }

    g_shooter_control_command.fire = fire_active;
    g_shooter_control_command.fire_mode =
        fire_active &&
        ((uint32_t)(current_tick - *fire_start_tick) >=
         pdMS_TO_TICKS(g_remote_mapping_debug.continuous_fire_delay_ms)) ?
            SHOOTER_FIRE_CONTINUOUS : SHOOTER_FIRE_SINGLE;
    *fire_was_active = fire_active;
}

/**
 * @brief 10 ms 遥控映射任务。
 * @details 左拨杆管理云台/发射，右拨杆管理底盘/小陀螺，失联 100 ms 立即进入保护。
 */
static void remote_task(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t fire_start_tick = 0U;
    bool fire_was_active = false;

    (void)argument;

    for (;;)
    {
        dbus_remote_state_t remote;
        const uint32_t current_tick = osKernelGetTickCount();
        float chassis_wz_for_yaw_compensation = 0.0F;

        dbus_remote_get_snapshot(&remote);
        g_remote_mapping_status.online =
            dbus_remote_is_online(REMOTE_TIMEOUT_MS);
        g_remote_mapping_status.left_switch = remote.switch_left;
        g_remote_mapping_status.right_switch = remote.switch_right;
        g_remote_mapping_status.gimbal_relative_angle_deg =
            g_yaw_control_status.front_angle_deg;
        g_remote_mapping_status.chassis_follow_error_deg =
            remote_get_calibrated_gimbal_angle(
                g_yaw_control_status.front_angle_deg);

        if (!g_remote_mapping_debug.enabled)
        {
            if (g_remote_mapping_status.controlling)
            {
                remote_stop_all();
                fire_was_active = false;
            }
            /* At boot, leave intentional Debug commands untouched. */
        }
        else if (!g_remote_mapping_status.online)
        {
            remote_stop_all();
            fire_was_active = false;
            g_remote_mapping_status.failsafe_count++;
        }
        else
        {
            g_remote_mapping_status.controlling = true;

            /* 右拨杆：下档底盘无力，中档普通行驶，上档变速小陀螺。 */
            if (remote.switch_right == DBUS_SWITCH_DOWN)
            {
                g_chassis_control_command.enabled = false;
                g_chassis_spin_command.enabled = false;
                g_remote_mapping_status.chassis_powered = false;
                g_remote_mapping_status.spin_active = false;
                g_remote_mapping_status.chassis_follow_active = false;
                g_remote_mapping_status.chassis_follow_wz_radps = 0.0F;
            }
            else
            {
                if (!g_yaw_control_status.online)
                {
                    g_chassis_control_command.enabled = false;
                    g_chassis_spin_command.enabled = false;
                    g_remote_mapping_status.chassis_powered = false;
                    g_remote_mapping_status.spin_active = false;
                    g_remote_mapping_status.chassis_follow_active = false;
                    g_remote_mapping_status.chassis_follow_wz_radps = 0.0F;
                }
                else
                {
                    const bool follow_enabled =
                        (remote.switch_right == DBUS_SWITCH_MID) &&
                        (remote.switch_left != DBUS_SWITCH_DOWN) &&
                        g_yaw_control_debug.front_ecd_valid;
                    const float follow_wz_radps = follow_enabled ?
                        remote_calculate_chassis_follow_wz(
                            g_yaw_control_status.front_angle_deg) : 0.0F;

                    remote_set_chassis(
                        &remote,
                        g_yaw_control_status.front_angle_deg,
                        follow_wz_radps);
                    g_chassis_spin_command.enabled =
                        remote.switch_right == DBUS_SWITCH_UP;
                    g_chassis_spin_command.variable_speed = true;
                    g_remote_mapping_status.spin_active =
                        g_chassis_spin_command.enabled;
                    g_remote_mapping_status.chassis_follow_active =
                        follow_enabled;
                    g_remote_mapping_status.chassis_follow_wz_radps =
                        follow_wz_radps;
                    /* 使用四轮反馈反解的实际角速度，而不是控制目标。 */
                    chassis_wz_for_yaw_compensation =
                        g_chassis_control_status.measured_wz_radps;
                }
            }

            /* 左拨杆：下档云台无力，中档云台可控，上档额外开放发射权限。 */
            if (remote.switch_left == DBUS_SWITCH_DOWN)
            {
                g_yaw_control_command.mode = YAW_MODE_DISABLED;
                g_shooter_control_command.armed = false;
                g_shooter_control_command.fire = false;
                remote_pitch_target_valid = false;
                g_remote_mapping_status.shooter_armed = false;
                g_remote_mapping_status.fire_active = false;
                g_remote_mapping_status.yaw_chassis_compensation_rpm = 0.0F;
                fire_was_active = false;
                remote_lower_pitch_before_powerdown(current_tick);
            }
            else
            {
                remote_pitch_powerdown_active = false;
                remote_pitch_powerdown_complete = false;
                remote_set_gimbal(
                    &remote,
                    chassis_wz_for_yaw_compensation);
                if (remote.switch_left == DBUS_SWITCH_UP)
                {
                    remote_set_shooter(
                        &remote,
                        current_tick,
                        &fire_start_tick,
                        &fire_was_active);
                }
                else
                {
                    g_shooter_control_command.armed = false;
                    g_shooter_control_command.fire = false;
                    g_remote_mapping_status.shooter_armed = false;
                    g_remote_mapping_status.fire_active = false;
                    fire_was_active = false;
                }
            }
        }

        g_remote_mapping_status.update_count++;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(REMOTE_PERIOD_MS));
    }
}

/** @brief 初始化遥控比例、死区、坐标方向和发射阈值。 */
void remote_mapping_init(void)
{
    (void)memset(
        &g_remote_mapping_debug, 0, sizeof(g_remote_mapping_debug));
    (void)memset(
        &g_remote_mapping_status, 0, sizeof(g_remote_mapping_status));

    /* 默认由遥控器接管整车；调试时仍可手动清零该开关暂停遥控映射。 */
    g_remote_mapping_debug.enabled = true;
    g_remote_mapping_debug.deadband = 10;
    /* 实车方向校正：反转前后和左右两个平移通道。 */
    g_remote_mapping_debug.chassis_vx_scale = -0.001F;
    g_remote_mapping_debug.chassis_vy_scale = 0.001F;
    /* 沿用旧代码的云台跟随比例、最大角速度和约 1.7 度死区。 */
    g_remote_mapping_debug.chassis_follow_kp = 6.0F;
    g_remote_mapping_debug.chassis_follow_max_wz_radps = 8.0F;
    g_remote_mapping_debug.chassis_follow_deadband_deg = 1.7F;
    /* 车头零点改由 GM6020 单圈绝对编码器标定，这里不再补偿上电相对角。 */
    g_remote_mapping_debug.gimbal_frame_offset_deg = 0.0F;
    g_remote_mapping_debug.yaw_speed_scale = 1.0F;
    g_remote_mapping_debug.yaw_chassis_compensation_gain = 1.0F;
    g_remote_mapping_debug.yaw_chassis_compensation_direction = -1;
    g_remote_mapping_debug.pitch_speed_scale = 0.1F;
    g_remote_mapping_debug.pitch_filter_tau_s = 0.04F;
    g_remote_mapping_debug.pitch_accel_limit_deg_s2 = 200.0F;
    g_remote_mapping_debug.pitch_max_tracking_error_deg = 5.0F;
    g_remote_mapping_debug.pitch_powerdown_rate_deg_s = 15.0F;
    g_remote_mapping_debug.pitch_powerdown_accel_deg_s2 = 60.0F;
    g_remote_mapping_debug.pitch_powerdown_tolerance_deg = 0.8F;
    g_remote_mapping_debug.gimbal_frame_direction = 1;
    g_remote_mapping_debug.fire_dial_threshold =
        REMOTE_DEFAULT_FIRE_THRESHOLD;
    g_remote_mapping_debug.continuous_fire_delay_ms =
        REMOTE_DEFAULT_LONG_FIRE_MS;
    remote_pitch_target_angle_deg = 0.0F;
    remote_pitch_target_valid = false;
    remote_pitch_filtered_rate_deg_s = 0.0F;
    remote_pitch_ramped_rate_deg_s = 0.0F;
    remote_pitch_powerdown_active = false;
    remote_pitch_powerdown_complete = false;
    remote_pitch_powerdown_start_tick = 0U;
}

/** @brief 使用静态内存创建遥控映射任务。 */
bool remote_mapping_task_create(void)
{
    const osThreadAttr_t attributes = {
        .name = "remote_map",
        .cb_mem = &remote_task_control,
        .cb_size = sizeof(remote_task_control),
        .stack_mem = remote_task_stack,
        .stack_size = sizeof(remote_task_stack),
        .priority = osPriorityNormal
    };

    if (remote_task_handle != NULL)
    {
        return true;
    }

    remote_task_handle = osThreadNew(remote_task, NULL, &attributes);
    return remote_task_handle != NULL;
}
