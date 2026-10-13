#ifndef DSH_FWCHECK_DJI_MOTOR_H
#define DSH_FWCHECK_DJI_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Host compile-check stub of Devices/DjiMotor/Inc/dji_motor.h.
 *
 * ⚠️ Field-for-field copy of the firmware's feedback struct. If the firmware
 *    adds a field, add it here too -- otherwise this check would happily accept
 *    code that reads a field Keil does not have (which is the exact mistake the
 *    check exists to catch).
 */

typedef struct
{
    /* raw single-turn rotor encoder, 0..8191 */
    uint16_t ecd;
    /* previous frame's ecd */
    uint16_t last_ecd;
    /* multi-turn rotor-side accumulator (int32) */
    int32_t  total_ecd;
    /* raw int16 rotor-side RPM as reported by the C620 (no gearbox applied) */
    int16_t  speed_rpm;
    /* raw int16: what the ESC says it is actually delivering */
    int16_t  given_current;
    /* raw uint8, degrees C */
    uint8_t  temperature;
    /* true once at least one valid 8-byte frame has been decoded */
    bool     initialized;
    /* last feedback tick + counters (diagnostics only) */
    uint32_t last_update_tick;
    uint32_t update_count;
    uint32_t ecd_wrap_count;
    uint32_t ecd_wrap_max_raw_delta;
    uint32_t ecd_long_gap_count;
    uint32_t ecd_long_gap_max_ms;
} dji_motor_feedback_t;

bool dji_motor_feedback_is_online(
    const dji_motor_feedback_t *feedback,
    uint32_t now_ms,
    uint32_t timeout_ticks);

#endif
