#ifndef DSH_TEST_STUB_ROBOT_MOTORS_H
#define DSH_TEST_STUB_ROBOT_MOTORS_H

#include "dji_motor.h"

#include <stdbool.h>

/* Host-test stub: only the lift group is needed by lift_jog_task.c. */

typedef struct
{
    dji_motor_feedback_t motor[4];
} robot_lift_motors_t;

typedef struct
{
    robot_lift_motors_t lift;
} robot_motor_state_t;

void robot_motors_get_snapshot(robot_motor_state_t *state);

#endif
