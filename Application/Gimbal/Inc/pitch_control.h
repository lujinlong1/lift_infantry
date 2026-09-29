#ifndef PITCH_CONTROL_H
#define PITCH_CONTROL_H

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
    volatile float kp;
    volatile float kd;
    volatile float torque;
    volatile float gravity_torque;
    volatile float max_target_rate_deg_s;
    volatile float min_angle_deg;
    volatile float max_angle_deg;
    volatile bool absolute_limit_enabled;
    volatile float absolute_min_position_rad;
    volatile float absolute_max_position_rad;
    volatile int8_t direction;
    volatile bool set_zero;
} pitch_debug_t;

typedef struct
{
    volatile bool online;
    volatile bool zero_valid;
    volatile bool absolute_limit_active;
    volatile bool at_absolute_minimum;
    volatile bool at_absolute_maximum;
    volatile uint8_t active_mode;
    volatile float measured_angle_deg;
    volatile float measured_position_rad;
    volatile float target_angle_deg;
    volatile float commanded_angle_deg;
    volatile float measured_velocity_deg_s;
    volatile float commanded_velocity_deg_s;
    volatile float commanded_torque;
    volatile float zero_position_rad;
    volatile float effective_min_angle_deg;
    volatile float effective_max_angle_deg;
    volatile uint32_t update_count;
} pitch_status_t;

extern pitch_command_t g_pitch_control_command;
extern pitch_debug_t g_pitch_control_debug;
extern pitch_status_t g_pitch_control_status;

void pitch_control_init(void);
bool pitch_control_task_create(void);

#endif
