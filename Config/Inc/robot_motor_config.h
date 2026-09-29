#ifndef ROBOT_MOTOR_CONFIG_H
#define ROBOT_MOTOR_CONFIG_H

/* CAN1: pitch and shooter motors. */
#define ROBOT_DM_PITCH_TX_ID              0x001U
#define ROBOT_DM_PITCH_RX_ID              0x011U
#define ROBOT_DM_PITCH_MOTOR_ID           0x01U

#define ROBOT_FRICTION_MOTOR_1_ID         0x204U
#define ROBOT_PROJECTILE_FEEDER_ID        0x205U
#define ROBOT_FRICTION_MOTOR_2_ID         0x206U
#define ROBOT_FRICTION_MOTOR_3_ID         0x207U
#define ROBOT_FRICTION_MOTOR_4_ID         0x208U

/* CAN2: chassis, trigger and yaw motors. */
#define ROBOT_CHASSIS_WHEEL_1_ID           0x201U
#define ROBOT_CHASSIS_WHEEL_2_ID           0x202U
#define ROBOT_CHASSIS_WHEEL_3_ID           0x203U
#define ROBOT_CHASSIS_WHEEL_4_ID           0x204U
#define ROBOT_TRIGGER_MOTOR_ID             0x207U
#define ROBOT_YAW_MOTOR_ID                 0x208U

#define ROBOT_DJI_COMMAND_ID_1_TO_4        0x200U
#define ROBOT_DJI_COMMAND_ID_5_TO_8        0x1FFU

#define ROBOT_MOTOR_TX_PERIOD_MS           2U
#define ROBOT_MOTOR_COMMAND_TIMEOUT_MS     100U
#define ROBOT_DM_ENABLE_RETRY_MS            100U
#define ROBOT_DJI_TEST_ZERO_HOLD_MS         500U

#define ROBOT_3508_CURRENT_LIMIT           16384
#define ROBOT_2006_CURRENT_LIMIT           10000
#define ROBOT_6020_CURRENT_LIMIT           30000

#endif
