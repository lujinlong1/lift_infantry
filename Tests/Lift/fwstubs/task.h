#ifndef DSH_FWCHECK_TASK_H
#define DSH_FWCHECK_TASK_H

#include "FreeRTOS.h"

/* Host compile-check stub of the FreeRTOS task API used by the lift task. */

void vTaskDelayUntil(TickType_t *previous_wake, TickType_t increment);
TickType_t xTaskGetTickCount(void);

#endif
