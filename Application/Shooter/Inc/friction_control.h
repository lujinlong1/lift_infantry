#ifndef FRICTION_CONTROL_H
#define FRICTION_CONTROL_H

#include "robot_motor_config.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    volatile bool enabled;
    volatile int16_t target_rpm[ROBOT_FRICTION_MOTOR_COUNT];
} friction_control_command_t;

typedef struct
{
    volatile bool running;
    volatile bool online[ROBOT_FRICTION_MOTOR_COUNT];
    volatile int16_t target_rpm[ROBOT_FRICTION_MOTOR_COUNT];
    volatile int16_t measured_rpm[ROBOT_FRICTION_MOTOR_COUNT];
    volatile int16_t output_current[ROBOT_FRICTION_MOTOR_COUNT];
    volatile float error[ROBOT_FRICTION_MOTOR_COUNT];
    volatile float p_output[ROBOT_FRICTION_MOTOR_COUNT];
    volatile float i_output[ROBOT_FRICTION_MOTOR_COUNT];
    volatile float d_output[ROBOT_FRICTION_MOTOR_COUNT];
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
