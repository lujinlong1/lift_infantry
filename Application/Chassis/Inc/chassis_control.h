#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    volatile uint8_t mode;
    volatile bool enabled;
    volatile int16_t target_rpm[4];
    volatile float vx_mps;
    volatile float vy_mps;
    volatile float wz_radps;
} chassis_control_command_t;

typedef struct
{
    volatile float kp;
    volatile float ki;
    volatile float kd;
    volatile float integral_limit;
    volatile float output_limit;
    volatile float rpm_per_mps;
    volatile float rotation_radius_m;
    volatile int8_t motor_direction[4];
    volatile bool apply;
    volatile bool reset;
} chassis_control_debug_t;

typedef struct
{
    volatile bool running;
    volatile bool online[4];
    volatile int16_t target_rpm[4];
    volatile int16_t measured_rpm[4];
    volatile float measured_wz_radps;
    volatile int16_t output_current[4];
    volatile float error[4];
    volatile float p_output[4];
    volatile float i_output[4];
    volatile float d_output[4];
    volatile uint32_t parameter_apply_count;
    volatile uint32_t update_count;
} chassis_control_status_t;

extern chassis_control_command_t g_chassis_control_command;
extern chassis_control_status_t g_chassis_control_status;
extern chassis_control_debug_t g_chassis_control_debug;

#define CHASSIS_CONTROL_MODE_DIRECT_RPM 0U
#define CHASSIS_CONTROL_MODE_MECANUM    1U

void chassis_control_init(void);
bool chassis_control_task_create(void);
void chassis_control_enable(bool enable);
void chassis_control_set_wheel_rpm(
    int16_t wheel1,
    int16_t wheel2,
    int16_t wheel3,
    int16_t wheel4);
void chassis_control_set_velocity(float vx_mps, float vy_mps, float wz_radps);

#endif
