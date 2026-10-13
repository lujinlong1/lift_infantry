#ifndef DSH_FWCHECK_ROBOT_MOTOR_CONFIG_H
#define DSH_FWCHECK_ROBOT_MOTOR_CONFIG_H

/*
 * Host compile-check stub of the firmware's motor configuration.
 *
 * WHY THIS FILE EXISTS
 *   `lift_control_task.c` is the only place that drives the whole lift chain,
 *   and until now nothing on the host ever compiled it: Keil was the first
 *   thing that saw it. That means a typo in a struct field or a missing
 *   include could only be found by opening the IDE.
 *
 *   `compile_check.ps1` compiles the REAL firmware source against these
 *   stubs with -Wall -Wextra -Wpedantic -Werror. It does not link and it does
 *   not run anything -- it only proves the file compiles, and it catches
 *   exactly the class of mistake Keil used to catch first.
 *
 * The values here must stay in sync with Config/Inc/robot_motor_config.h.
 * If they drift, this check becomes noise rather than signal.
 */

#define ROBOT_LIFT_MOTOR_COUNT        4U
#define ROBOT_LIFT_FRONT_MASK         0x03U
#define ROBOT_LIFT_REAR_MASK          0x0CU
#define ROBOT_FRICTION_MOTOR_COUNT    2U

#define ROBOT_MOTOR_TX_PERIOD_MS      2U
#define ROBOT_MOTOR_COMMAND_TIMEOUT_MS 100U

#define ROBOT_DJI_COMMAND_ID_1_TO_4   0x200U
#define ROBOT_DJI_COMMAND_ID_5_TO_8   0x1FFU

#define ROBOT_3508_CURRENT_LIMIT      16384
#define ROBOT_2006_CURRENT_LIMIT      10000
#define ROBOT_6020_CURRENT_LIMIT      30000

#endif
