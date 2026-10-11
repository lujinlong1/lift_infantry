#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t ecd;
    uint16_t last_ecd;
    int16_t speed_rpm;
    int16_t given_current;
    int32_t total_ecd;
    uint32_t update_count;
    uint32_t last_update_tick;
    uint8_t temperature;
    bool initialized;
    /*
     * ---- 多圈累加诊断（三个量的含义完全不同，别混）-------------------
     *
     * total_ecd 用"相邻两帧的单圈编码器差"累加，靠"差接近一整圈就认为
     * 跨了 0/8191 边界、补一个 ∓8192"来展开多圈。
     *
     * ⚠️ 踩过的坑：曾经把"跨圈"当成"跳变错误"来计数。**跨圈是正常事件**，
     *    每转一整圈必然发生一次，而且跨越那一刻的原始差本来就接近 ±8192。
     *    所以"这个数很大"不代表出错，只代表"转过圈了"。
     *
     *   ecd_wrap_count          跨越 0/8191 边界的次数。**正常事件**，
     *                           转一圈 +1。它的用途是确认"确实转过圈了"
     *                           （如果推了好几圈它却一直是 0，那才反常）。
     *   ecd_wrap_max_raw_delta  跨越时见过的最大原始差。**≈8192/8160 是正常的**，
     *                           因为它就是"跨界那一刻的单圈值差"。
     *
     * 下面这个才是【真正值得警惕】的：
     *   ecd_long_gap_count / ecd_long_gap_max_ms
     *       相邻两帧间隔超过 DJI_MOTOR_GAP_WARN_MS 的次数与最大间隔。
     *
     *       为什么盯间隔：多圈展开的唯一假设是"一帧内位移不超过半圈"。
     *       这个假设靠【采样够快】保证。转速再快也只要 1 ms 就走十几个计数，
     *       但**帧间隔一长**，位移就可能逼近半圈边界 —— 间隔才是风险来源。
     *       按最快速 15468 计数/秒算，要逼近 4096 边界需要约 265 ms 的间隔，
     *       所以阈值取 20 ms 是"早期预警"，不是"已经出事"。
     */
    uint32_t ecd_wrap_count;
    uint32_t ecd_wrap_max_raw_delta;
    uint32_t ecd_long_gap_count;
    uint32_t ecd_long_gap_max_ms;
} dji_motor_feedback_t;

void dji_motor_feedback_reset(dji_motor_feedback_t *motor);

bool dji_motor_decode_feedback(
    dji_motor_feedback_t *motor,
    const uint8_t data[8],
    uint8_t dlc,
    uint32_t current_tick);

bool dji_motor_feedback_is_online(
    const dji_motor_feedback_t *motor,
    uint32_t current_tick,
    uint32_t timeout_ticks);

void dji_motor_pack_group_command(
    uint8_t data[8],
    int16_t motor1,
    int16_t motor2,
    int16_t motor3,
    int16_t motor4);

#endif
