#include "dji_motor.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>
#include <string.h>

#define DJI_MOTOR_ECD_MAX         8191U
/*
 * 相邻两帧间隔的预警阈值(ms)。超过它记一次"长间隔"，并记录最大值。
 *
 * 多圈展开的唯一假设是"一帧内位移不超过半圈(4096 计数)"，而这个假设
 * 靠【采样够快】保证：最快速 15468 计数/秒时，1 ms 只走约 15 个计数，
 * 要逼近 4096 需要约 265 ms 的间隔。所以 20 ms 只是早期预警。
 */
#define DJI_MOTOR_GAP_WARN_MS     20U
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

        /*
         * 诊断 ①：跨圈计数 + 跨圈时的最大原始差。
         *
         * ⚠️ 注意这是【正常事件】不是错误：每转一整圈必然跨过 0/8191 边界一次，
         *    而跨越那一刻"新单圈值 − 旧单圈值"本来就接近 ±8192。
         *    （曾经把这个当成"跳变错误"来判读，是错的。）
         */
        {
            const int32_t raw_abs = (ecd_delta < 0) ? -ecd_delta : ecd_delta;

            if (raw_abs > DJI_MOTOR_ECD_HALF_RANGE)
            {
                motor->ecd_wrap_count++;
            }
            if ((uint32_t)raw_abs > motor->ecd_wrap_max_raw_delta)
            {
                motor->ecd_wrap_max_raw_delta = (uint32_t)raw_abs;
            }
        }

        /*
         * 诊断 ②：帧间隔。这才是【真正值得警惕】的量。
         *
         * 多圈展开的唯一假设是"一帧内位移不超过半圈"，靠采样够快保证。
         * 帧间隔一长，位移就可能逼近半圈边界 → 补圈方向变成猜测。
         * 用 tick 差算间隔（此处 last_update_tick 还是上一帧的时间）。
         */
        {
            const uint32_t gap_ms =
                (uint32_t)(current_tick - motor->last_update_tick) *
                (uint32_t)portTICK_PERIOD_MS;

            if (gap_ms > (uint32_t)DJI_MOTOR_GAP_WARN_MS)
            {
                motor->ecd_long_gap_count++;
            }
            if (gap_ms > motor->ecd_long_gap_max_ms)
            {
                motor->ecd_long_gap_max_ms = gap_ms;
            }
        }

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

        /*
         * ---- 为什么【不】按转速限制单帧位移（踩过的坑）----------------
         *
         * 曾经在这里加过"用 speed_rpm 算物理上限、超过就丢弃该帧位移"的
         * 保护。它是错的，而且后果比原问题更严重：
         *
         *   speed_rpm 上报为 0（或 ±3 这类粗值）时，算出的上限只有 2 个计数，
         *   而静置时编码器抖动本来就有 ±1~4 计数/帧 —— 于是**正常帧被大批丢弃**。
         *
         * 更糟的是丢弃方式：last_ecd 照样更新、ecd 照样前进，只把 delta 清零，
         * 结果下一帧的差突然变大、又被丢弃 …… **位置直接冻死**。
         * 实测现象：那个计数器涨到 72，而"最大单帧差"只有 4 个计数。
         *
         * 所以这里保持原样：**只按 ±半圈 补圈，不做任何速率限制。**
         * 真要监控多圈展开的健康度：
         *   - ecd_wrap_count / ecd_wrap_max_raw_delta  跨圈统计（大是正常的）
         *   - ecd_long_gap_count / ecd_long_gap_max_ms 帧间隔（这才值得警惕）
         *   - 功能判据：往返推回同一位置，total_ecd 要回到同一个值
         */

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
