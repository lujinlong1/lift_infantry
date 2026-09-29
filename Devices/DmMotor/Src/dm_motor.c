#include "dm_motor.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>
#include <string.h>

#define DM_MIT_POSITION_MIN  (-12.5F)
#define DM_MIT_POSITION_MAX  12.5F
#define DM_MIT_VELOCITY_MIN  (-45.0F)
#define DM_MIT_VELOCITY_MAX  45.0F
#define DM_MIT_TORQUE_MIN    (-18.0F)
#define DM_MIT_TORQUE_MAX    18.0F
#define DM_MIT_KP_MIN        0.0F
#define DM_MIT_KP_MAX        500.0F
#define DM_MIT_KD_MIN        0.0F
#define DM_MIT_KD_MAX        5.0F

/** 将协议中的无符号整数按位宽线性映射为物理量。 */
static float dm_uint_to_float(
    uint32_t value,
    float minimum,
    float maximum,
    uint32_t bits)
{
    const uint32_t maximum_value = (1UL << bits) - 1UL;
    return ((float)value * (maximum - minimum) / (float)maximum_value) + minimum;
}

/** 将浮点数限制在协议允许范围内。 */
static float dm_clampf(float value, float minimum, float maximum)
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

/** 将物理量限制并线性量化为指定位宽的无符号整数。 */
static uint16_t dm_float_to_uint(
    float value,
    float minimum,
    float maximum,
    uint32_t bits)
{
    const uint32_t maximum_value = (1UL << bits) - 1UL;
    const float limited = dm_clampf(value, minimum, maximum);
    return (uint16_t)((limited - minimum) * (float)maximum_value /
                      (maximum - minimum));
}

/** 封装达妙电机全 0xFF 前缀的使能、失能或保存零点特殊命令。 */
static void dm_motor_pack_special_command(uint8_t data[8], uint8_t command)
{
    uint32_t index;

    if (data == NULL)
    {
        return;
    }

    for (index = 0U; index < 7U; index++)
    {
        data[index] = 0xFFU;
    }
    data[7] = command;
}

/** 在线程临界区内清空一个达妙电机反馈对象。 */
void dm_motor_feedback_reset(dm_motor_feedback_t *motor)
{
    if (motor == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    (void)memset(motor, 0, sizeof(*motor));
    taskEXIT_CRITICAL();
}

/** 解码达妙 MIT 反馈，并将位置、速度和力矩恢复为物理量。 */
bool dm_motor_decode_mit_feedback(
    dm_motor_feedback_t *motor,
    uint8_t expected_motor_id,
    const uint8_t data[8],
    uint8_t dlc,
    uint32_t current_tick)
{
    uint8_t motor_id;
    uint16_t position_raw;
    uint16_t velocity_raw;
    uint16_t torque_raw;

    if ((motor == NULL) || (data == NULL) || (dlc != 8U))
    {
        return false;
    }

    motor_id = data[0] & 0x0FU;
    if (motor_id != expected_motor_id)
    {
        return false;
    }

    position_raw = (uint16_t)(((uint16_t)data[1] << 8U) |
                              (uint16_t)data[2]);
    velocity_raw = (uint16_t)(((uint16_t)data[3] << 4U) |
                              ((uint16_t)data[4] >> 4U));
    torque_raw = (uint16_t)((((uint16_t)data[4] & 0x0FU) << 8U) |
                            (uint16_t)data[5]);

    taskENTER_CRITICAL();
    motor->motor_id = motor_id;
    motor->error_code = data[0] >> 4U;
    motor->position = dm_uint_to_float(
        position_raw, DM_MIT_POSITION_MIN, DM_MIT_POSITION_MAX, 16U);
    motor->velocity = dm_uint_to_float(
        velocity_raw, DM_MIT_VELOCITY_MIN, DM_MIT_VELOCITY_MAX, 12U);
    motor->torque = dm_uint_to_float(
        torque_raw, DM_MIT_TORQUE_MIN, DM_MIT_TORQUE_MAX, 12U);
    motor->mos_temperature = data[6];
    motor->rotor_temperature = data[7];
    motor->last_update_tick = current_tick;
    motor->update_count++;
    motor->initialized = true;
    taskEXIT_CRITICAL();

    return true;
}

/** 根据初始化标志和最后更新时间判断达妙电机是否在线。 */
bool dm_motor_feedback_is_online(
    const dm_motor_feedback_t *motor,
    uint32_t current_tick,
    uint32_t timeout_ticks)
{
    if ((motor == NULL) || !motor->initialized)
    {
        return false;
    }

    return (uint32_t)(current_tick - motor->last_update_tick) <= timeout_ticks;
}

/** 将位置、速度、刚度、阻尼和力矩封装为达妙 MIT 控制帧。 */
void dm_motor_pack_mit_command(
    uint8_t data[8],
    float position,
    float velocity,
    float kp,
    float kd,
    float torque)
{
    uint16_t position_raw;
    uint16_t velocity_raw;
    uint16_t kp_raw;
    uint16_t kd_raw;
    uint16_t torque_raw;

    if (data == NULL)
    {
        return;
    }

    position_raw = dm_float_to_uint(
        position, DM_MIT_POSITION_MIN, DM_MIT_POSITION_MAX, 16U);
    velocity_raw = dm_float_to_uint(
        velocity, DM_MIT_VELOCITY_MIN, DM_MIT_VELOCITY_MAX, 12U);
    kp_raw = dm_float_to_uint(kp, DM_MIT_KP_MIN, DM_MIT_KP_MAX, 12U);
    kd_raw = dm_float_to_uint(kd, DM_MIT_KD_MIN, DM_MIT_KD_MAX, 12U);
    torque_raw = dm_float_to_uint(
        torque, DM_MIT_TORQUE_MIN, DM_MIT_TORQUE_MAX, 12U);

    data[0] = (uint8_t)(position_raw >> 8U);
    data[1] = (uint8_t)position_raw;
    data[2] = (uint8_t)(velocity_raw >> 4U);
    data[3] = (uint8_t)((velocity_raw << 4U) | (kp_raw >> 8U));
    data[4] = (uint8_t)kp_raw;
    data[5] = (uint8_t)(kd_raw >> 4U);
    data[6] = (uint8_t)((kd_raw << 4U) | (torque_raw >> 8U));
    data[7] = (uint8_t)torque_raw;
}

/** 封装达妙电机使能命令。 */
void dm_motor_pack_enable_command(uint8_t data[8])
{
    dm_motor_pack_special_command(data, 0xFCU);
}

/** 封装达妙电机失能命令。 */
void dm_motor_pack_disable_command(uint8_t data[8])
{
    dm_motor_pack_special_command(data, 0xFDU);
}

/** 封装达妙电机保存当前位置为零点的命令。 */
void dm_motor_pack_save_zero_command(uint8_t data[8])
{
    dm_motor_pack_special_command(data, 0xFEU);
}
