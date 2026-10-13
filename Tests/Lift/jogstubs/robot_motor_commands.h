#ifndef DSH_TEST_STUB_ROBOT_MOTOR_COMMANDS_H
#define DSH_TEST_STUB_ROBOT_MOTOR_COMMANDS_H

#include <stdbool.h>
#include <stdint.h>

/* Host-test stub: what lift_jog_task.c sends to the motors. */
void robot_lift_set_currents(int16_t c0, int16_t c1, int16_t c2, int16_t c3);

#endif
