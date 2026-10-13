#ifndef ROBOT_MOTOR_CONFIG_H
#define ROBOT_MOTOR_CONFIG_H

/*
 * 本工程为四丝杆抬升步兵。CAN 电机编号的唯一权威来源是
 * 讨论池/电机映射表.md，改 ID 时先改该表再改本文件。
 *
 * CAN1 组帧：
 *   0x200 = { 抬升1, 抬升2, 抬升3, 抬升4 }     四个 M3508 抬升组
 *   0x1FF = { 空闲, 摩擦轮1, 摩擦轮2, 空闲 }   槽1 预留 Pitch(GM6020)
 * CAN2 组帧：
 *   0x200 = { 底盘1, 底盘2, 底盘3, 底盘4 }
 *   0x1FF = { 空闲, 空闲, 拨弹盘, Yaw }
 *
 * 注意：同一条帧内 GM6020 收电压(±30000)，C620/C610 收电流(±16384/±10000)；
 *       同总线内 ESC_ID 不可重复。
 */

/* 本车机械配置数量。改变数量只需改这里，数组长度随之变化。 */
#define ROBOT_FRICTION_MOTOR_COUNT         2U
#define ROBOT_FEED_MOTOR_COUNT             1U

/* CAN1: four lift motors, pitch, and friction wheels. */
#define ROBOT_LIFT_MOTOR_COUNT            4U
#define ROBOT_LIFT_MOTOR_1_ID             0x201U
#define ROBOT_LIFT_MOTOR_2_ID             0x202U
#define ROBOT_LIFT_MOTOR_3_ID             0x203U
#define ROBOT_LIFT_MOTOR_4_ID             0x204U
/* 2026-10-13 实车确认：ch0/ch1 前辅助轮，ch2/ch3 后麦轮。 */
#define ROBOT_LIFT_FRONT_MASK             0x03U
#define ROBOT_LIFT_REAR_MASK              0x0CU

/* Pitch 改为 GM6020，走标准 DJI 帧：反馈 0x205，命令 0x1FF 槽 1（电压量纲）。 */
#define ROBOT_PITCH_MOTOR_ID               0x205U

#define ROBOT_FRICTION_MOTOR_1_ID         0x206U
#define ROBOT_FRICTION_MOTOR_2_ID         0x207U

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
#define ROBOT_DJI_TEST_ZERO_HOLD_MS         500U

#define ROBOT_3508_CURRENT_LIMIT           16384
#define ROBOT_2006_CURRENT_LIMIT           10000
#define ROBOT_6020_CURRENT_LIMIT           30000
/* 拨弹盘由 M3508 改为 M2006，必须用 C610 的量程，不能沿用电调满量程。 */
#define ROBOT_TRIGGER_CURRENT_LIMIT        ROBOT_2006_CURRENT_LIMIT

#endif
