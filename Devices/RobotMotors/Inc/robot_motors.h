#ifndef ROBOT_MOTORS_H
#define ROBOT_MOTORS_H

#include "can_rx_service.h"
#include "dji_motor.h"
#include "robot_motor_config.h"

#include <stdbool.h>

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

/* Read directly only in the debugger. Tasks should use the snapshot API. */
extern robot_motor_state_t g_robot_motors;

void robot_motors_init(void);
bool robot_motors_process_frame(const can_rx_frame_t *frame);
void robot_motors_get_snapshot(robot_motor_state_t *snapshot);

#endif
