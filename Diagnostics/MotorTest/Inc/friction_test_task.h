#ifndef FRICTION_TEST_TASK_H
#define FRICTION_TEST_TASK_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    volatile bool enabled;
    volatile uint8_t motor_index;
    volatile int16_t test_current;
    volatile uint32_t duration_ms;
} friction_test_command_t;

typedef struct
{
    volatile bool running;
    volatile uint8_t active_motor_index;
    volatile int16_t applied_current;
    volatile uint32_t elapsed_ms;

    volatile uint16_t ecd[4];
    volatile int16_t speed_rpm[4];
    volatile int16_t feedback_current[4];
    volatile uint8_t temperature[4];
    volatile uint32_t update_count[4];
    volatile bool online[4];

    volatile uint32_t can_rx_count;
    volatile uint32_t can_rx_dropped;
    volatile uint32_t can_tx_count;
    volatile uint32_t can_tx_busy;
    volatile uint32_t can_tx_errors;
} friction_test_status_t;

extern friction_test_command_t g_friction_test_command;
extern friction_test_status_t g_friction_test_status;

bool friction_test_task_create(void);

#endif

