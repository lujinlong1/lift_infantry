#ifndef DSH_FWCHECK_ROBOT_MOTOR_COMMANDS_H
#define DSH_FWCHECK_ROBOT_MOTOR_COMMANDS_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Host compile-check stub. Mirrors the shape of the firmware header
 * (Devices/RobotMotors/Inc/robot_motor_commands.h) for exactly what
 * lift_control_task.c touches. Values do not matter here -- nothing runs.
 */

typedef struct
{
    volatile bool running;
    volatile bool zero_holding;
} dji_can_test_status_t;

extern dji_can_test_status_t g_dji_can_test_status;

void robot_lift_set_currents(int16_t c0, int16_t c1, int16_t c2, int16_t c3);
void robot_chassis_set_currents(int16_t c0, int16_t c1, int16_t c2, int16_t c3);

#endif
