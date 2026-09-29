#ifndef PITCH_TEST_TASK_H
#define PITCH_TEST_TASK_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    volatile bool enabled;
    volatile uint32_t duration_ms;
} pitch_test_command_t;

typedef struct
{
    volatile bool running;
    volatile uint32_t elapsed_ms;
    volatile uint32_t raw_id_001_count;
    volatile uint32_t raw_id_011_count;
    volatile uint32_t feedback_update_count;
    volatile float position;
    volatile float velocity;
    volatile float torque;
    volatile uint8_t error_code;
    volatile uint8_t mos_temperature;
    volatile uint8_t rotor_temperature;
    volatile bool online;
} pitch_test_status_t;

extern pitch_test_command_t g_pitch_test_command;
extern pitch_test_status_t g_pitch_test_status;

bool pitch_test_task_create(void);

#endif

