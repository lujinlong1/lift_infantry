#include "dji_motor.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>
#include <string.h>

#define DJI_MOTOR_ECD_MAX         8191U
#define DJI_MOTOR_ECD_RANGE       8192
#define DJI_MOTOR_ECD_HALF_RANGE  4096

/** 在线程临界区内清空一个 DJI 电机反馈对象。 */
void dji_motor_feedback_reset(dji_motor_feedback_t *motor)
{
    if (motor == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    (void)memset(motor, 0, sizeof(*motor));
    taskEXIT_CRITICAL();
}

/**
 * @brief 解码 DJI 电机 8 字节反馈，并展开为连续累计编码器值。
 * @note 编码器跨越 0/8191 时用半圈阈值修正增量，避免累计角度跳变一整圈。
 */
bool dji_motor_decode_feedback(
    dji_motor_feedback_t *motor,
    const uint8_t data[8],
    uint8_t dlc,
    uint32_t current_tick)
{
    uint16_t new_ecd;
    int32_t ecd_delta;

    if ((motor == NULL) || (data == NULL) || (dlc != 8U))
    {
        return false;
    }

    new_ecd = (uint16_t)(((uint16_t)data[0] << 8U) |
                         (uint16_t)data[1]);
    if (new_ecd > DJI_MOTOR_ECD_MAX)
    {
        return false;
    }

    taskENTER_CRITICAL();

    if (!motor->initialized)
    {
        motor->ecd = new_ecd;
        motor->last_ecd = new_ecd;
        motor->total_ecd = 0;
        motor->initialized = true;
    }
    else
    {
        motor->last_ecd = motor->ecd;
        ecd_delta = (int32_t)new_ecd - (int32_t)motor->last_ecd;

        if (ecd_delta < -DJI_MOTOR_ECD_HALF_RANGE)
        {
            ecd_delta += DJI_MOTOR_ECD_RANGE;
        }
        else if (ecd_delta > DJI_MOTOR_ECD_HALF_RANGE)
        {
            ecd_delta -= DJI_MOTOR_ECD_RANGE;
        }
        else
        {
        }

        motor->ecd = new_ecd;
        motor->total_ecd += ecd_delta;
    }

    motor->speed_rpm =
        (int16_t)(((uint16_t)data[2] << 8U) | (uint16_t)data[3]);
    motor->given_current =
        (int16_t)(((uint16_t)data[4] << 8U) | (uint16_t)data[5]);
    motor->temperature = data[6];
    motor->last_update_tick = current_tick;
    motor->update_count++;

    taskEXIT_CRITICAL();
    return true;
}

/** 根据是否收到过反馈以及最后更新时间判断 DJI 电机在线状态。 */
bool dji_motor_feedback_is_online(
    const dji_motor_feedback_t *motor,
    uint32_t current_tick,
    uint32_t timeout_ticks)
{
    if ((motor == NULL) || !motor->initialized)
    {
        return false;
    }

    return (uint32_t)(current_tick - motor->last_update_tick) <= timeout_ticks;
}

/** 将四路有符号电流按大端序封装为 DJI 电机组命令。 */
void dji_motor_pack_group_command(
    uint8_t data[8],
    int16_t motor1,
    int16_t motor2,
    int16_t motor3,
    int16_t motor4)
{
    const int16_t currents[4] = {motor1, motor2, motor3, motor4};
    uint32_t index;

    if (data == NULL)
    {
        return;
    }

    for (index = 0U; index < 4U; index++)
    {
        data[index * 2U] = (uint8_t)((uint16_t)currents[index] >> 8U);
        data[index * 2U + 1U] = (uint8_t)currents[index];
    }
}
