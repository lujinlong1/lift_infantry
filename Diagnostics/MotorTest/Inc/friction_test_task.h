#ifndef FRICTION_TEST_TASK_H
#define FRICTION_TEST_TASK_H

#include "robot_motor_config.h"

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

    volatile uint16_t ecd[ROBOT_FRICTION_MOTOR_COUNT];
    volatile int16_t speed_rpm[ROBOT_FRICTION_MOTOR_COUNT];
    volatile int16_t feedback_current[ROBOT_FRICTION_MOTOR_COUNT];
    volatile uint8_t temperature[ROBOT_FRICTION_MOTOR_COUNT];
    volatile uint32_t update_count[ROBOT_FRICTION_MOTOR_COUNT];
    volatile bool online[ROBOT_FRICTION_MOTOR_COUNT];

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

