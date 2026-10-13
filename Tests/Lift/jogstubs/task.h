#ifndef DSH_TEST_STUB_TASK_H
#define DSH_TEST_STUB_TASK_H

#include "FreeRTOS.h"

/* Host-test stub: vTaskDelayUntil just records how many times it was called. */

extern uint32_t g_stub_delay_calls;

void vTaskDelayUntil(TickType_t *previous_wake, TickType_t increment);
TickType_t xTaskGetTickCount(void);

#endif
