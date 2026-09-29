#include "chassis_spin.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

#define CHASSIS_SPIN_DEFAULT_FIXED_WZ       6.0F
#define CHASSIS_SPIN_DEFAULT_MINIMUM_WZ     4.0F
#define CHASSIS_SPIN_DEFAULT_MAXIMUM_WZ    10.0F
#define CHASSIS_SPIN_DEFAULT_MINIMUM_STEP   0.8F
#define CHASSIS_SPIN_DEFAULT_TRANSITION_MS  500U
#define CHASSIS_SPIN_MAXIMUM_WZ            12.0F

chassis_spin_command_t g_chassis_spin_command;
chassis_spin_debug_t g_chassis_spin_debug;
chassis_spin_status_t g_chassis_spin_status;

static bool transition_initialized;
static float transition_start_wz;
static uint32_t transition_start_tick;
static uint32_t random_state = 0x6D2B79F5U;

/** @brief 返回浮点数绝对值，避免为简单运算引入额外数学库依赖。 */
static float chassis_spin_abs(float value)
{
    return value < 0.0F ? -value : value;
}

/** @brief 将浮点数限制在指定上下限之间。 */
static float chassis_spin_clamp(float value, float minimum, float maximum)
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
 * @brief 使用 xorshift32 生成轻量伪随机数。
 * @return 新的 32 位伪随机值。
 * @note 该随机性只用于改变小陀螺转速，不用于密码学。
 */
static uint32_t chassis_spin_random(void)
{
    uint32_t value = random_state;

    value ^= value << 13U;
    value ^= value >> 17U;
    value ^= value << 5U;
    random_state = value;
    return value;
}

/** @brief 将伪随机数归一化为 0~1 浮点数。 */
static float chassis_spin_random_unit(void)
{
    return (float)(chassis_spin_random() & 0x00FFFFFFU) /
           16777215.0F;
}

/**
 * @brief 校验小陀螺 Debug 参数，防止负速度上限、反向范围或无效方向。
 */
static void chassis_spin_validate_debug(void)
{
    float temporary;

    g_chassis_spin_debug.fixed_wz_radps = chassis_spin_clamp(
        chassis_spin_abs(g_chassis_spin_debug.fixed_wz_radps),
        0.0F,
        CHASSIS_SPIN_MAXIMUM_WZ);
    g_chassis_spin_debug.minimum_wz_radps = chassis_spin_clamp(
        chassis_spin_abs(g_chassis_spin_debug.minimum_wz_radps),
        0.0F,
        CHASSIS_SPIN_MAXIMUM_WZ);
    g_chassis_spin_debug.maximum_wz_radps = chassis_spin_clamp(
        chassis_spin_abs(g_chassis_spin_debug.maximum_wz_radps),
        0.0F,
        CHASSIS_SPIN_MAXIMUM_WZ);

    if (g_chassis_spin_debug.minimum_wz_radps >
        g_chassis_spin_debug.maximum_wz_radps)
    {
        temporary = g_chassis_spin_debug.minimum_wz_radps;
        g_chassis_spin_debug.minimum_wz_radps =
            g_chassis_spin_debug.maximum_wz_radps;
        g_chassis_spin_debug.maximum_wz_radps = temporary;
    }

    g_chassis_spin_debug.minimum_step_radps = chassis_spin_clamp(
        chassis_spin_abs(g_chassis_spin_debug.minimum_step_radps),
        0.0F,
        g_chassis_spin_debug.maximum_wz_radps -
            g_chassis_spin_debug.minimum_wz_radps);
    if (g_chassis_spin_debug.transition_ms < 100U)
    {
        g_chassis_spin_debug.transition_ms = 100U;
    }
    if (g_chassis_spin_debug.transition_ms > 5000U)
    {
        g_chassis_spin_debug.transition_ms = 5000U;
    }

    if ((g_chassis_spin_command.direction != 1) &&
        (g_chassis_spin_command.direction != -1))
    {
        g_chassis_spin_command.direction = 1;
    }

    g_chassis_spin_debug.apply = false;
}

/**
 * @brief 在配置范围内挑选新的随机角速度目标。
 * @param previous_target 上一个目标，用于尽量保证两次变速幅度足够明显。
 */
static float chassis_spin_choose_target(float previous_target)
{
    float candidate = previous_target;
    uint32_t attempt;

    for (attempt = 0U; attempt < 8U; attempt++)
    {
        candidate = g_chassis_spin_debug.minimum_wz_radps +
                    chassis_spin_random_unit() *
                        (g_chassis_spin_debug.maximum_wz_radps -
                         g_chassis_spin_debug.minimum_wz_radps);
        if (chassis_spin_abs(candidate - previous_target) >=
            g_chassis_spin_debug.minimum_step_radps)
        {
            break;
        }
    }

    return candidate;
}

/**
 * @brief 保存本次过渡的固定起点，并生成下一个角速度目标。
 */
static void chassis_spin_begin_transition(uint32_t current_tick)
{
    transition_start_wz = g_chassis_spin_status.current_wz_radps;
    g_chassis_spin_status.target_wz_radps = chassis_spin_choose_target(
        g_chassis_spin_status.target_wz_radps);
    transition_start_tick = current_tick;
    transition_initialized = true;
    g_chassis_spin_status.target_change_count++;
}

/** @brief 初始化小陀螺命令、参数、状态和伪随机种子。 */
void chassis_spin_init(void)
{
    taskENTER_CRITICAL();
    (void)memset(&g_chassis_spin_command, 0, sizeof(g_chassis_spin_command));
    (void)memset(&g_chassis_spin_debug, 0, sizeof(g_chassis_spin_debug));
    (void)memset(&g_chassis_spin_status, 0, sizeof(g_chassis_spin_status));
    g_chassis_spin_command.direction = 1;
    g_chassis_spin_debug.fixed_wz_radps = CHASSIS_SPIN_DEFAULT_FIXED_WZ;
    g_chassis_spin_debug.minimum_wz_radps =
        CHASSIS_SPIN_DEFAULT_MINIMUM_WZ;
    g_chassis_spin_debug.maximum_wz_radps =
        CHASSIS_SPIN_DEFAULT_MAXIMUM_WZ;
    g_chassis_spin_debug.minimum_step_radps =
        CHASSIS_SPIN_DEFAULT_MINIMUM_STEP;
    g_chassis_spin_debug.transition_ms =
        CHASSIS_SPIN_DEFAULT_TRANSITION_MS;
    transition_initialized = false;
    random_state ^= xTaskGetTickCount();
    taskEXIT_CRITICAL();
}

/**
 * @brief 更新小陀螺角速度生成器。
 * @param current_tick 当前 RTOS 时基。
 * @return 带旋转方向的底盘角速度，单位 rad/s；未使能时返回 0。
 */
float chassis_spin_update(uint32_t current_tick)
{
    float progress;
    float smooth_progress;
    uint32_t elapsed_ticks;
    uint32_t duration_ticks;

    if (g_chassis_spin_debug.apply)
    {
        chassis_spin_validate_debug();
        transition_initialized = false;
    }
    if (g_chassis_spin_debug.reset)
    {
        transition_initialized = false;
        g_chassis_spin_debug.reset = false;
    }

    if (!g_chassis_spin_command.enabled)
    {
        transition_initialized = false;
        g_chassis_spin_status.active = false;
        g_chassis_spin_status.current_wz_radps = 0.0F;
        g_chassis_spin_status.target_wz_radps = 0.0F;
        g_chassis_spin_status.transition_elapsed_ms = 0U;
        return 0.0F;
    }

    chassis_spin_validate_debug();
    g_chassis_spin_status.active = true;

    if (!g_chassis_spin_command.variable_speed)
    {
        g_chassis_spin_status.current_wz_radps =
            g_chassis_spin_debug.fixed_wz_radps;
        g_chassis_spin_status.target_wz_radps =
            g_chassis_spin_debug.fixed_wz_radps;
        g_chassis_spin_status.transition_elapsed_ms = 0U;
    }
    else
    {
        if (!transition_initialized)
        {
            g_chassis_spin_status.current_wz_radps =
                g_chassis_spin_debug.minimum_wz_radps;
            g_chassis_spin_status.target_wz_radps =
                g_chassis_spin_debug.minimum_wz_radps;
            chassis_spin_begin_transition(current_tick);
        }

        elapsed_ticks = (uint32_t)(current_tick - transition_start_tick);
        duration_ticks = pdMS_TO_TICKS(g_chassis_spin_debug.transition_ms);
        if (duration_ticks == 0U)
        {
            duration_ticks = 1U;
        }

        if (elapsed_ticks >= duration_ticks)
        {
            g_chassis_spin_status.current_wz_radps =
                g_chassis_spin_status.target_wz_radps;
            chassis_spin_begin_transition(current_tick);
            elapsed_ticks = 0U;
        }

        progress = (float)elapsed_ticks / (float)duration_ticks;
        /* smoothstep 在起点和终点斜率都为 0，变速时不会产生阶跃。 */
        smooth_progress = progress * progress * (3.0F - 2.0F * progress);
        g_chassis_spin_status.current_wz_radps =
            transition_start_wz +
            (g_chassis_spin_status.target_wz_radps - transition_start_wz) *
                smooth_progress;
        g_chassis_spin_status.transition_elapsed_ms =
            elapsed_ticks * portTICK_PERIOD_MS;
    }

    return (float)g_chassis_spin_command.direction *
           g_chassis_spin_status.current_wz_radps;
}
