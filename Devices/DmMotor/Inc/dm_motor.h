#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float position;
    float velocity;
    float torque;
    uint32_t update_count;
    uint32_t last_update_tick;
    uint8_t motor_id;
    uint8_t error_code;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
    bool initialized;
} dm_motor_feedback_t;

void dm_motor_feedback_reset(dm_motor_feedback_t *motor);

bool dm_motor_decode_mit_feedback(
    dm_motor_feedback_t *motor,
    uint8_t expected_motor_id,
    const uint8_t data[8],
    uint8_t dlc,
    uint32_t current_tick);

bool dm_motor_feedback_is_online(
    const dm_motor_feedback_t *motor,
    uint32_t current_tick,
    uint32_t timeout_ticks);

void dm_motor_pack_mit_command(
    uint8_t data[8],
    float position,
    float velocity,
    float kp,
    float kd,
    float torque);

void dm_motor_pack_enable_command(uint8_t data[8]);
void dm_motor_pack_disable_command(uint8_t data[8]);
void dm_motor_pack_save_zero_command(uint8_t data[8]);

#endif
