#ifndef YAW_CONTROL_H
#define YAW_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    YAW_MODE_DISABLED = 0,
    YAW_MODE_SPEED,
    YAW_MODE_ANGLE
} yaw_control_mode_t;

typedef struct
{
    volatile uint8_t mode;
    volatile float target_angle_deg;
    volatile float target_speed_rpm;
} yaw_control_command_t;

typedef struct
{
    volatile float angle_kp;
    volatile float angle_ki;
    volatile float angle_kd;
    volatile float angle_integral_limit;
    volatile float max_speed_rpm;
    volatile float speed_kp;
    volatile float speed_ki;
    volatile float speed_kd;
    volatile float speed_integral_limit;
    volatile float max_current;
    volatile float gear_ratio;
    volatile int8_t motor_direction;
    volatile uint16_t front_ecd;
    volatile bool front_ecd_valid;
    volatile bool capture_front_ecd;
    volatile bool apply;
    volatile bool reset;
    volatile bool set_zero;
} yaw_control_debug_t;

typedef struct
{
    volatile bool online;
    volatile bool zero_valid;
    volatile uint8_t active_mode;
    volatile int32_t zero_total_ecd;
    volatile uint16_t raw_ecd;
    volatile float front_angle_deg;
    volatile float target_angle_deg;
    volatile float measured_angle_deg;
    volatile float angle_error_deg;
    volatile float target_speed_rpm;
    volatile float measured_speed_rpm;
    volatile float speed_error_rpm;
    volatile float angle_p_output;
    volatile float angle_i_output;
    volatile float angle_d_output;
    volatile float speed_p_output;
    volatile float speed_i_output;
    volatile float speed_d_output;
    volatile int16_t output_current;
    volatile uint32_t update_count;
    volatile uint32_t parameter_apply_count;
} yaw_control_status_t;

extern yaw_control_command_t g_yaw_control_command;
extern yaw_control_debug_t g_yaw_control_debug;
extern yaw_control_status_t g_yaw_control_status;

void yaw_control_init(void);
bool yaw_control_task_create(void);

#endif
