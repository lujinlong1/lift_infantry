#ifndef PITCH_CONTROL_H
#define PITCH_CONTROL_H

/*
 * Pitch 轴控制（GM6020，标准 DJI 帧）。
 *
 * 与英雄版的根本差别：达妙 DM6220 用的是 MIT 私有帧，增益 kp/kd 和力矩下发后
 * 由【电机内部】闭环；GM6020 只能收到一个电压量，闭环必须搬到【MCU 里】。
 *
 * 因此本模块改为和 yaw_control.c 同构的串级结构：
 *
 *      角度环（输出 = 目标转速）
 *            |
 *      速度环（输出 = 电压量）
 *            |
 *      robot_gimbal_set_pitch_current()  ->  CAN1 0x1FF 槽 1
 *
 * 接口稳定性约束（S0-3 的验收点）：remote_control_task.c 与 telemetry_task.c
 * 直接读写下面三个结构体的字段。本模块可以改内部实现，但【字段名必须原样保留】，
 * 否则要连带修改那两个文件——那正是这次改造要避免的事。
 */

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    PITCH_MODE_DISABLED = 0,
    PITCH_MODE_POSITION,
    PITCH_MODE_SPEED
} pitch_mode_t;

typedef struct
{
    volatile uint8_t mode;
    volatile float target_angle_deg;
    volatile float target_speed_deg_s;
} pitch_command_t;

typedef struct
{
    /* 角度环 = 串级外环；速度环 = 串级内环。 */
    volatile float angle_kp;
    volatile float angle_ki;
    volatile float angle_kd;
    volatile float angle_integral_limit;
    volatile float speed_kp;
    volatile float speed_ki;
    volatile float speed_kd;
    volatile float speed_integral_limit;

    /* 外环输出的上限：即允许的最大目标转速（deg/s）。 */
    volatile float max_speed_deg_s;
    /* 内环输出的上限：即允许的最大下发电压量（对应 GM6020 的 ±30000）。 */
    volatile float max_current;
    /* 重力前馈：固定叠加在内环输出上，抵消枪管自重。 */
    volatile float gravity_torque;

    volatile float max_target_rate_deg_s;
    volatile float min_angle_deg;
    volatile float max_angle_deg;

    /*
     * 绝对位置硬限位。GM6020 只有【单圈绝对】编码器，没有多圈绝对位置，
     * 所以这道保护只能做成"单圈 ecd 窗口"，且必须先标定机械死点处的 ecd。
     * 标定完成前保持 false —— 不拿猜测值当保护。
     */
    volatile bool absolute_limit_enabled;
    volatile uint16_t absolute_min_ecd;
    volatile uint16_t absolute_max_ecd;

    volatile int8_t direction;
    volatile float gear_ratio;
    volatile bool set_zero;
    volatile bool apply;
    volatile bool reset;
} pitch_debug_t;

typedef struct
{
    volatile bool online;
    volatile bool zero_valid;
    volatile bool absolute_limit_active;
    volatile bool at_absolute_minimum;
    volatile bool at_absolute_maximum;
    volatile uint8_t active_mode;
    volatile uint16_t raw_ecd;
    volatile int32_t zero_total_ecd;
    volatile float measured_angle_deg;
    volatile float target_angle_deg;
    volatile float commanded_angle_deg;
    volatile float measured_velocity_deg_s;
    volatile float commanded_velocity_deg_s;
    volatile float angle_error_deg;
    volatile float speed_error_deg_s;
    volatile float angle_p_output;
    volatile float angle_i_output;
    volatile float angle_d_output;
    volatile float speed_p_output;
    volatile float speed_i_output;
    volatile float speed_d_output;
    volatile float commanded_torque;
    volatile float effective_min_angle_deg;
    volatile float effective_max_angle_deg;
    volatile int16_t output_current;
    volatile uint32_t update_count;
    volatile uint32_t parameter_apply_count;
} pitch_status_t;

extern pitch_command_t g_pitch_control_command;
extern pitch_debug_t g_pitch_control_debug;
extern pitch_status_t g_pitch_control_status;

void pitch_control_init(void);
bool pitch_control_task_create(void);

#endif
