#ifndef FEED_CONTROL_H
#define FEED_CONTROL_H

#include "robot_motor_config.h"

#include <stdbool.h>
#include <stdint.h>

/* 本车只有一个拨弹盘（M2006），送弹轮已取消。 */
#define FEED_MOTOR_TRIGGER    0U
#define FEED_MOTOR_COUNT      ROBOT_FEED_MOTOR_COUNT

typedef struct
{
    volatile bool enabled[FEED_MOTOR_COUNT];
    volatile int16_t target_rpm[FEED_MOTOR_COUNT];
} feed_control_command_t;

typedef struct
{
    volatile bool running[FEED_MOTOR_COUNT];
    volatile bool online[FEED_MOTOR_COUNT];
    volatile int16_t target_rpm[FEED_MOTOR_COUNT];
    volatile int16_t measured_rpm[FEED_MOTOR_COUNT];
    volatile int16_t output_current[FEED_MOTOR_COUNT];
    volatile float error[FEED_MOTOR_COUNT];
    volatile float p_output[FEED_MOTOR_COUNT];
    volatile float i_output[FEED_MOTOR_COUNT];
    volatile float d_output[FEED_MOTOR_COUNT];
    volatile uint32_t update_count;
} feed_control_status_t;

typedef struct
{
    volatile float kp[FEED_MOTOR_COUNT];
    volatile float ki[FEED_MOTOR_COUNT];
    volatile float kd[FEED_MOTOR_COUNT];
    volatile float integral_limit[FEED_MOTOR_COUNT];
    volatile float output_limit[FEED_MOTOR_COUNT];
    volatile bool apply;
    volatile bool reset;
} feed_control_debug_t;

extern feed_control_command_t g_feed_control_command;
extern feed_control_status_t g_feed_control_status;
extern feed_control_debug_t g_feed_control_debug;

void feed_control_init(void);
bool feed_control_task_create(void);

#endif
