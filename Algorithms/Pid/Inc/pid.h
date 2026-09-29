#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
} pid_config_t;

typedef struct
{
    pid_config_t config;
    float integral;
    float previous_error;
    float error;
    float p_output;
    float i_output;
    float d_output;
    float output;
    bool initialized;
} pid_controller_t;

void pid_init(pid_controller_t *pid, const pid_config_t *config);
void pid_reset(pid_controller_t *pid);
float pid_calculate(
    pid_controller_t *pid,
    float target,
    float measurement,
    float dt_seconds);

#endif
