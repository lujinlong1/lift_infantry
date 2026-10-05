#ifndef PITCH_TEST_TASK_H
#define PITCH_TEST_TASK_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Pitch（GM6020）诊断点动。
 *
 * 与英雄版的关键差别：达妙电机必须"先使能才有反馈"，所以旧版测试的一大半
 * 逻辑是使能握手。GM6020 上电即有反馈，使能那套整条删掉。
 *
 * 本任务【故意绕过 pitch_control 任务】直接写设备层电压，用来单独验证
 * "反馈能收、命令能发"。它与 pitch_control 任务互斥，同一时刻只能有一个在写。
 * 因为它只发恒定电压、没有角度闭环，只能用于点动测向，不能长时间通电。
 */

typedef struct
{
    volatile bool enabled;
    volatile int16_t test_current;
    volatile uint32_t duration_ms;
} pitch_test_command_t;

typedef struct
{
    volatile bool running;
    volatile uint32_t elapsed_ms;
    volatile uint32_t feedback_update_count;
    volatile uint16_t ecd;
    volatile int32_t total_ecd;
    volatile int16_t speed_rpm;
    volatile int16_t given_current;
    volatile uint8_t temperature;
    volatile bool online;
} pitch_test_status_t;

extern pitch_test_command_t g_pitch_test_command;
extern pitch_test_status_t g_pitch_test_status;

bool pitch_test_task_create(void);

#endif
