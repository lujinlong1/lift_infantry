#ifndef FRICTION_CONTROL_H
#define FRICTION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    volatile bool enabled;
    volatile int16_t target_rpm[4];
} friction_control_command_t;

typedef struct
{
    volatile bool running;
    volatile bool online[4];
    volatile int16_t target_rpm[4];
    volatile int16_t measured_rpm[4];
    volatile int16_t output_current[4];
    volatile float error[4];
    volatile float p_output[4];
    volatile float i_output[4];
    volatile float d_output[4];
    volatile uint32_t update_count;
} friction_control_status_t;

typedef struct
{
    volatile float kp;
    volatile float ki;
    volatile float kd;
    volatile float integral_limit;
    volatile float output_limit;
    volatile bool apply;
    volatile bool reset;
} friction_control_debug_t;

extern friction_control_command_t g_friction_control_command;
extern friction_control_status_t g_friction_control_status;
extern friction_control_debug_t g_friction_control_debug;

void friction_control_init(void);
bool friction_control_task_create(void);

#endif
