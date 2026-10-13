#ifndef DSH_FWCHECK_ROBOT_MOTORS_H
#define DSH_FWCHECK_ROBOT_MOTORS_H

#include "dji_motor.h"
#include "robot_motor_config.h"

#include <stdbool.h>

/*
 * Host compile-check stub of Devices/RobotMotors/Inc/robot_motors.h.
 * Same field names as the firmware so lift_control_task.c compiles verbatim.
 */

typedef struct
{
    dji_motor_feedback_t wheel[4];
} chassis_motors_t;

typedef struct
{
    dji_motor_feedback_t motor[ROBOT_LIFT_MOTOR_COUNT];
} lift_motors_t;

typedef struct
{
    dji_motor_feedback_t pitch;
    dji_motor_feedback_t yaw;
} gimbal_motors_t;

typedef struct
{
    dji_motor_feedback_t friction[ROBOT_FRICTION_MOTOR_COUNT];
    dji_motor_feedback_t trigger;
} shooter_motors_t;

typedef struct
{
    chassis_motors_t chassis;
    lift_motors_t lift;
    gimbal_motors_t gimbal;
    shooter_motors_t shooter;
} robot_motor_state_t;

void robot_motors_get_snapshot(robot_motor_state_t *snapshot);

#endif
