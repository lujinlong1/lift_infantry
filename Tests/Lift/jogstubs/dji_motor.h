#ifndef DSH_TEST_STUB_DJI_MOTOR_H
#define DSH_TEST_STUB_DJI_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

/* Host-test stub of the firmware dji_motor feedback type (only what
 * lift_jog_task.c touches is declared). */

typedef struct
{
    bool     initialized;
    int32_t  total_ecd;
    int16_t  speed_rpm;
    int16_t  given_current;
    uint8_t  temperature;
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
