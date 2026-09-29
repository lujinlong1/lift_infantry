#ifndef REMOTE_CONTROL_TASK_H
#define REMOTE_CONTROL_TASK_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    volatile bool enabled;
    volatile int16_t deadband;
    volatile float chassis_vx_scale;
    volatile float chassis_vy_scale;
    volatile float chassis_follow_kp;
    volatile float chassis_follow_max_wz_radps;
    volatile float chassis_follow_deadband_deg;
    volatile float yaw_speed_scale;
    volatile float yaw_chassis_compensation_gain;
    volatile int8_t yaw_chassis_compensation_direction;
    volatile float pitch_speed_scale;
    volatile float pitch_filter_tau_s;
    volatile float pitch_accel_limit_deg_s2;
    volatile float pitch_max_tracking_error_deg;
    volatile float pitch_powerdown_rate_deg_s;
    volatile float pitch_powerdown_accel_deg_s2;
    volatile float pitch_powerdown_tolerance_deg;
    volatile float gimbal_frame_offset_deg;
    volatile int8_t gimbal_frame_direction;
    volatile int16_t fire_dial_threshold;
    volatile uint32_t continuous_fire_delay_ms;
} remote_mapping_debug_t;

typedef struct
{
    volatile bool online;
    volatile bool controlling;
    volatile bool chassis_powered;
    volatile bool gimbal_powered;
    volatile bool gimbal_lowering;
    volatile bool spin_active;
    volatile bool chassis_follow_active;
    volatile bool shooter_armed;
    volatile bool fire_active;
    volatile uint8_t left_switch;
    volatile uint8_t right_switch;
    volatile float chassis_vx_mps;
    volatile float chassis_vy_mps;
    volatile float chassis_follow_wz_radps;
    volatile float yaw_chassis_compensation_rpm;
    volatile float gimbal_relative_angle_deg;
    volatile float chassis_follow_error_deg;
    volatile float pitch_raw_rate_deg_s;
    volatile float pitch_filtered_rate_deg_s;
    volatile float pitch_ramped_rate_deg_s;
    volatile uint32_t update_count;
    volatile uint32_t failsafe_count;
} remote_mapping_status_t;

extern remote_mapping_debug_t g_remote_mapping_debug;
extern remote_mapping_status_t g_remote_mapping_status;

void remote_mapping_init(void);
bool remote_mapping_task_create(void);

#endif
