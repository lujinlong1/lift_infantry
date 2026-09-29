#ifndef CHASSIS_SPIN_H
#define CHASSIS_SPIN_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    volatile bool enabled;
    volatile bool variable_speed;
    volatile int8_t direction;
} chassis_spin_command_t;

typedef struct
{
    volatile float fixed_wz_radps;
    volatile float minimum_wz_radps;
    volatile float maximum_wz_radps;
    volatile float minimum_step_radps;
    volatile uint32_t transition_ms;
    volatile bool apply;
    volatile bool reset;
} chassis_spin_debug_t;

typedef struct
{
    volatile bool active;
    volatile float current_wz_radps;
    volatile float target_wz_radps;
    volatile uint32_t transition_elapsed_ms;
    volatile uint32_t target_change_count;
} chassis_spin_status_t;

extern chassis_spin_command_t g_chassis_spin_command;
extern chassis_spin_debug_t g_chassis_spin_debug;
extern chassis_spin_status_t g_chassis_spin_status;

void chassis_spin_init(void);
float chassis_spin_update(uint32_t current_tick);

#endif
