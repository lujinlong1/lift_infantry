#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t ecd;
    uint16_t last_ecd;
    int16_t speed_rpm;
    int16_t given_current;
    int32_t total_ecd;
    uint32_t update_count;
    uint32_t last_update_tick;
    uint8_t temperature;
    bool initialized;
} dji_motor_feedback_t;

void dji_motor_feedback_reset(dji_motor_feedback_t *motor);

bool dji_motor_decode_feedback(
    dji_motor_feedback_t *motor,
    const uint8_t data[8],
    uint8_t dlc,
    uint32_t current_tick);

bool dji_motor_feedback_is_online(
    const dji_motor_feedback_t *motor,
    uint32_t current_tick,
    uint32_t timeout_ticks);

void dji_motor_pack_group_command(
    uint8_t data[8],
    int16_t motor1,
    int16_t motor2,
    int16_t motor3,
    int16_t motor4);

#endif
