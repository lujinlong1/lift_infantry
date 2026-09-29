#ifndef ROBOT_MOTOR_COMMANDS_H
#define ROBOT_MOTOR_COMMANDS_H

#include "robot_motor_config.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int16_t wheel[4];
    uint32_t update_tick;
} chassis_motor_commands_t;

typedef struct
{
    float pitch_position;
    float pitch_velocity;
    float pitch_kp;
    float pitch_kd;
    float pitch_torque;
    uint32_t pitch_update_tick;
    int16_t yaw_current;
    uint32_t yaw_update_tick;
    bool pitch_enable_requested;
} gimbal_motor_commands_t;

typedef struct
{
    int16_t friction[ROBOT_FRICTION_MOTOR_COUNT];
    int16_t trigger_current;
    uint32_t friction_update_tick;
    uint32_t trigger_update_tick;
} shooter_motor_commands_t;

typedef struct
{
    chassis_motor_commands_t chassis;
    gimbal_motor_commands_t gimbal;
    shooter_motor_commands_t shooter;
} robot_motor_commands_t;

typedef struct
{
    volatile bool enabled;
    volatile uint8_t can_bus;
    volatile uint16_t feedback_id;
    volatile int16_t test_current;
    volatile uint32_t duration_ms;
} dji_can_test_command_t;

typedef struct
{
    volatile bool running;
    volatile bool zero_holding;
    volatile bool valid;
    volatile uint8_t resolved_bus;
    volatile uint8_t motor_slot;
    volatile uint16_t command_id;
    volatile int16_t applied_current;
    volatile uint32_t elapsed_ms;
    volatile uint8_t last_send_status;
} dji_can_test_status_t;

extern dji_can_test_command_t g_dji_can_test_command;
extern dji_can_test_status_t g_dji_can_test_status;

void robot_motor_commands_init(void);
bool robot_motor_tx_task_create(void);

void robot_chassis_set_currents(
    int16_t wheel1,
    int16_t wheel2,
    int16_t wheel3,
    int16_t wheel4);

void robot_gimbal_set_yaw_current(int16_t current);

void robot_gimbal_set_pitch_mit(
    float position,
    float velocity,
    float kp,
    float kd,
    float torque);

void robot_gimbal_request_pitch_enable(bool enable);

void robot_shooter_set_command(
    int16_t friction1,
    int16_t friction2,
    int16_t trigger_current);

void robot_shooter_set_friction_currents(
    int16_t friction1,
    int16_t friction2);

void robot_shooter_set_feed_currents(int16_t trigger_current);

void robot_motor_commands_get_snapshot(robot_motor_commands_t *snapshot);

#endif
