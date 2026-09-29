#ifndef SHOOTER_CONTROL_H
#define SHOOTER_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    SHOOTER_STATE_STOPPED = 0,
    SHOOTER_STATE_SPINUP,
    SHOOTER_STATE_READY,
    SHOOTER_STATE_FIRING_SINGLE,
    SHOOTER_STATE_FIRING_CONTINUOUS,
    SHOOTER_STATE_REVERSING,
    SHOOTER_STATE_FAULT
} shooter_state_t;

typedef enum
{
    SHOOTER_FIRE_SINGLE = 0,
    SHOOTER_FIRE_CONTINUOUS
} shooter_fire_mode_t;

typedef struct
{
    volatile bool armed;
    volatile bool fire;
    volatile uint8_t fire_mode;
} shooter_control_command_t;

typedef struct
{
    volatile int16_t friction_rpm[4];
    volatile int16_t projectile_feeder_rpm;
    volatile int16_t trigger_rpm;
    volatile int8_t feed_direction;
    volatile int32_t trigger_ecd_per_shot;
    volatile int16_t friction_ready_tolerance_rpm;
    volatile int16_t jam_speed_threshold_rpm;
    volatile uint32_t ready_stable_ms;
    volatile uint32_t spinup_timeout_ms;
    volatile uint32_t jam_timeout_ms;
    volatile uint32_t reverse_ms;
    volatile uint8_t max_jam_retries;
} shooter_control_debug_t;

typedef struct
{
    volatile uint8_t state;
    volatile bool ready;
    volatile bool friction_stable;
    volatile uint8_t jam_retries;
    volatile uint32_t shot_count;
    volatile uint32_t jam_count;
    volatile uint32_t state_elapsed_ms;
    volatile int32_t trigger_current_ecd;
    volatile int32_t trigger_target_ecd;
    volatile uint8_t fault_code;
    volatile uint32_t update_count;
} shooter_control_status_t;

extern shooter_control_command_t g_shooter_control_command;
extern shooter_control_debug_t g_shooter_control_debug;
extern shooter_control_status_t g_shooter_control_status;

void shooter_control_init(void);
bool shooter_control_task_create(void);

#endif
