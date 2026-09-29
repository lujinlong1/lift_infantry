#include "pid.h"

#include <stddef.h>

/** 按正负对称限幅约束 PID 中的积分或最终输出。 */
static float pid_clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

/** 写入 PID 参数并清空控制器历史状态。 */
void pid_init(pid_controller_t *pid, const pid_config_t *config)
{
    if ((pid == NULL) || (config == NULL))
    {
        return;
    }

    pid->config = *config;
    pid_reset(pid);
}

/** 清空积分、上次误差和各分量输出。 */
void pid_reset(pid_controller_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral = 0.0F;
    pid->previous_error = 0.0F;
    pid->error = 0.0F;
    pid->p_output = 0.0F;
    pid->i_output = 0.0F;
    pid->d_output = 0.0F;
    pid->output = 0.0F;
    pid->initialized = false;
}

/**
 * @brief 计算一次位置式 PID。
 * @note 积分项和最终输出分别限幅；第一次计算不使用微分项，避免初始冲击。
 */
float pid_calculate(
    pid_controller_t *pid,
    float target,
    float measurement,
    float dt_seconds)
{
    float error;
    float derivative = 0.0F;

    if ((pid == NULL) || (dt_seconds <= 0.0F))
    {
        return 0.0F;
    }

    error = target - measurement;
    pid->error = error;
    pid->integral += error * dt_seconds;
    pid->integral = pid_clamp(
        pid->integral, pid->config.integral_limit);

    if (pid->initialized)
    {
        derivative = (error - pid->previous_error) / dt_seconds;
    }

    pid->p_output = pid->config.kp * error;
    pid->i_output = pid->config.ki * pid->integral;
    pid->d_output = pid->config.kd * derivative;
    pid->output = pid->p_output + pid->i_output + pid->d_output;
    pid->output = pid_clamp(pid->output, pid->config.output_limit);
    pid->previous_error = error;
    pid->initialized = true;

    return pid->output;
}
